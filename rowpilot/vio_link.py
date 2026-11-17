"""From OpenVINS's estimate to the drone's pose in the greenhouse (map) frame.

OpenVINS reports the D435i IMU frame I in its global frame G: gravity-aligned (z up), origin and
heading fixed when it initialised. The mission and the flight controller need the drone body frame
B in the map frame M (ENU: x along the rows). Since the drone initialises standing at the known
take-off spot, the fixed transform M <- G is computed once, from the first estimate:

    T_M_G = T_M_B(home) * T_B_I * inv(T_G_I(first))

after which every estimate maps as T_M_B = T_M_G * T_G_I * inv(T_B_I).
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import numpy as np

from .sensors import D435i, _rot


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """(w, x, y, z), Hamilton."""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def make_T(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ p)


@dataclass(frozen=True)
class BodyPose:
    t: float  # VIO timestamp (simulation time)
    p: np.ndarray  # body position in map ENU [m]
    R: np.ndarray  # body (FLU) -> map (ENU) rotation
    v: np.ndarray  # body velocity in map ENU [m/s]
    wall: float  # local time it arrived, for freshness checks

    @property
    def yaw(self) -> float:
        return math.atan2(self.R[1, 0], self.R[0, 0])


class VioAligner:
    """Turns OpenVINS IMU poses (in G) into body poses in the map frame."""

    def __init__(
        self, home_xy: tuple[float, float], home_yaw: float = 0.0, home_z: float = 0.0, d435i: D435i | None = None
    ):
        d = d435i or D435i()
        self.T_B_I = make_T(_rot(0.0, d.tilt, 0.0), [d.mount_x, 0.0, d.mount_z])  # IMU in body
        self.T_M_Bhome = make_T(_rot(0.0, 0.0, home_yaw), [home_xy[0], home_xy[1], home_z])
        self.T_M_G: np.ndarray | None = None
        self.latest: BodyPose | None = None
        self._lock = threading.Lock()

    def update(self, t: float, p_IinG, q_ItoG_wxyz, v_IinG) -> BodyPose:
        T_G_I = make_T(quat_to_rot(*q_ItoG_wxyz), p_IinG)
        if self.T_M_G is None:
            # Align once: at this moment the drone stands at home, level apart from its legs.
            T_G_B = T_G_I @ inv_T(self.T_B_I)
            yaw_G = math.atan2(T_G_B[1, 0], T_G_B[0, 0])
            T_G_Bflat = make_T(_rot(0.0, 0.0, yaw_G), T_G_B[:3, 3])  # keep only heading for the alignment
            self.T_M_G = self.T_M_Bhome @ inv_T(T_G_Bflat)
        T_M_B = self.T_M_G @ T_G_I @ inv_T(self.T_B_I)
        v = self.T_M_G[:3, :3] @ np.asarray(v_IinG, float)  # IMU velocity ~ body velocity (rigid, slow rotation)
        pose = BodyPose(t, T_M_B[:3, 3], T_M_B[:3, :3], v, time.monotonic())
        with self._lock:
            self.latest = pose
        return pose

    def get(self) -> BodyPose | None:
        with self._lock:
            return self.latest


def subscribe_vio(aligner: VioAligner, topic: str = "/rowpilot/vio/odometry", on_pose=None):
    """Subscribe to the VIO bridge's gz topic; returns the gz Node (keep it alive)."""
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.transport13 import Node

    def cb(m) -> None:
        if not m.header.data or m.header.data[0].value[0] != "initialized":
            return
        q, p, v = m.pose.orientation, m.pose.position, m.twist.linear
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nsec
        pose = aligner.update(t, (p.x, p.y, p.z), (q.w, q.x, q.y, q.z), (v.x, v.y, v.z))
        if on_pose:
            on_pose(pose)

    node = Node()
    if not node.subscribe(Odometry, topic, cb):
        raise RuntimeError(f"cannot subscribe to {topic}")
    return node
