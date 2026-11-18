"""Simulation only: the D435i accelerometer computed from the physics, for OpenVINS.

Gazebo's IMU sensor reports wrong linear accelerations while the drone rotates (measured in the
greenhouse: 0.2-0.6 m/s^2 sideways in yaw turns, while the gyro and the camera images agree with
the true motion). OpenVINS integrates that into a false velocity and diverges in every turn.

This node keeps Gazebo's gyro and replaces the accelerometer: the second difference of the sensor
link's position (published by Gazebo's PosePublisher at 1 kHz, noise-free physics; the sensor link's
origin is the IMU, see sensors.py), minus gravity,
in the IMU frame, plus BMI055-like white noise and bias random walk. It republishes the IMU on
<prefix>/d435i_imu_kin, which the VIO bridge reads instead of <prefix>/d435i_imu.

    python -m rowpilot.imu_synth [--model gh_iris]
"""

from __future__ import annotations

import argparse
import bisect
import math
import threading
import time
from collections import deque

import numpy as np

from .sensors import D435i
from .vio_link import quat_to_rot

GRAVITY = np.array([0.0, 0.0, -9.8])  # Gazebo's default world gravity
DIFF_STEP_S = 0.005


class PoseBuffer:
    """Recent (t, position, rotation) samples of one link, in time order."""

    def __init__(self, keep_s: float = 0.5):
        self.t: list[float] = []
        self.p: list[np.ndarray] = []
        self.R: list[np.ndarray] = []
        self.keep_s = keep_s

    def add(self, t: float, p: np.ndarray, R: np.ndarray) -> None:
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t)
        self.p.append(p)
        self.R.append(R)
        if self.t[0] < t - 2 * self.keep_s:  # trim now and then, not on every sample
            k = bisect.bisect_left(self.t, t - self.keep_s)
            del self.t[:k], self.p[:k], self.R[:k]

    def newest(self) -> float:
        return self.t[-1] if self.t else -math.inf

    def _index(self, t: float) -> int | None:
        """i with t[i-1] < t <= t[i], or None outside the buffer."""
        if not self.t or t < self.t[0] or t > self.t[-1]:
            return None
        return bisect.bisect_left(self.t, t)

    def position(self, t: float) -> np.ndarray | None:
        i = self._index(t)
        if i is None:
            return None
        if i == 0 or self.t[i] == t:
            return self.p[i]
        a = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return (1 - a) * self.p[i - 1] + a * self.p[i]

    def rotation(self, t: float) -> np.ndarray | None:
        i = self._index(t)
        return None if i is None else self.R[i]


def specific_force(buf: PoseBuffer, t: float, h: float = DIFF_STEP_S) -> np.ndarray | None:
    """What an ideal accelerometer fixed to the link measures at time t, in the link frame."""
    p0, p1, p2 = buf.position(t - h), buf.position(t), buf.position(t + h)
    R = buf.rotation(t)
    if p0 is None or p1 is None or p2 is None or R is None:
        return None
    a = (p2 - 2 * p1 + p0) / (h * h)
    return R.T @ (a - GRAVITY)


class AccelNoise:
    """White noise and bias random walk (continuous-time densities, as in Kalibr/OpenVINS configs)."""

    def __init__(self, d: D435i, seed: int = 0):
        self.rng = np.random.default_rng(seed)
        self.nd, self.rw = d.accel_noise_density, d.accel_random_walk
        self.bias = np.zeros(3)

    def __call__(self, f: np.ndarray, dt: float) -> np.ndarray:
        if dt > 0:
            self.bias += self.rw * math.sqrt(dt) * self.rng.normal(size=3)
            white = self.nd / math.sqrt(dt) * self.rng.normal(size=3)
        else:
            white = np.zeros(3)
        return f + self.bias + white


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", default="gh_iris")
    ap.add_argument("--link", default="sensor_link")
    a = ap.parse_args(argv)

    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    node = Node()
    pub = node.advertise(f"/{a.model}/d435i_imu_kin", IMU)
    buf, noise = PoseBuffer(), AccelNoise(D435i())
    pending: deque = deque()
    lock = threading.Lock()
    state = {"last_t": None, "count": 0}

    link_name = f"{a.model}::{a.link}"

    def on_pose(m) -> None:
        # The model's pose is in the world frame, its links' poses relative to the model.
        model = link = None
        for p in m.pose:
            if p.name == a.model:
                model = p
            elif p.name == link_name:
                link = p
        if model is None or link is None:
            return
        t = model.header.stamp.sec + 1e-9 * model.header.stamp.nsec
        qm, ql = model.orientation, link.orientation
        Rm, Rl = quat_to_rot(qm.w, qm.x, qm.y, qm.z), quat_to_rot(ql.w, ql.x, ql.y, ql.z)
        pm = np.array([model.position.x, model.position.y, model.position.z])
        pl = np.array([link.position.x, link.position.y, link.position.z])
        with lock:
            buf.add(t, pm + Rm @ pl, Rm @ Rl)
        flush()

    def on_imu(m) -> None:
        with lock:
            pending.append(m)
        flush()

    def flush() -> None:
        out = []
        with lock:
            while pending:
                m = pending[0]
                t = m.header.stamp.sec + 1e-9 * m.header.stamp.nsec
                if buf.newest() < t + DIFF_STEP_S:
                    break  # wait for the poses after this sample
                pending.popleft()
                f = specific_force(buf, t)
                if f is None:
                    continue
                dt = 0.0 if state["last_t"] is None else t - state["last_t"]
                state["last_t"] = t
                f = noise(f, dt)
                o = IMU()
                o.CopyFrom(m)
                o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z = (float(v) for v in f)
                out.append(o)
        for o in out:
            pub.publish(o)
            state["count"] += 1

    node.subscribe(Pose_V, f"/model/{a.model}/pose", on_pose)
    node.subscribe(IMU, f"/{a.model}/d435i_imu", on_imu)
    print(f"[imu_synth] /{a.model}/d435i_imu + /model/{a.model}/pose -> /{a.model}/d435i_imu_kin", flush=True)
    try:
        while True:
            time.sleep(5.0)
            print(f"[imu_synth] {state['count']} samples", flush=True)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
