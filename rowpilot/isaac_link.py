"""Isaac Sim backend, the parts that do not need Isaac Sim (so they are tested like the rest).

With SIM=isaac, Isaac Sim stands in for Gazebo and everything else stays as it is:

    ArduCopter SITL <-- JSON lock-step (UDP 9002) --> Isaac Sim (isaac/greenhouse_isaac.py)
                                                         | cameras, IMU, ground truth, clock
                                                         v  TCP 127.0.0.1:9100 (this file's framing)
                                                       rowpilot.isaac_relay -> gz-transport topics
                                                         v
                         vio_bridge (OpenVINS), rowpilot ardupilot (the tree), as with Gazebo

Here:
  - ArduPilot's JSON SITL protocol (libraries/SITL/SIM_JSON.cpp): the servo packet ArduPilot sends
    each physics step and the JSON state the physics replies with. ardupilot_gazebo's plugin speaks
    the same protocol, which is why the ArduCopter parameters carry over.
  - RotorThrust: the FireFly's four rotors as forces and torques on the body, sized like the Gazebo
    model (airframe.py) so the rate gains tuned there still fit.
  - ImuSampler: the D435i IMU from the physics, as rowpilot.imu_synth computes it for Gazebo
    (accelerometer from the sensor link's poses), with the gyro from the body's angular velocity.
  - Frame packing for the Isaac -> relay stream: Isaac Sim ships its own Python, which cannot load
    the system's gz-transport bindings, so a relay in .venv-sim publishes for it.

Frames: Isaac's world is the map frame (ENU, x along the rows), as Gazebo's. ArduPilot wants NED
positions and velocities (north = map y, east = map x, as in ardupilot.py) and FRD body rates.
"""

from __future__ import annotations

import json
import math
import struct
from collections import deque
from dataclasses import dataclass

import numpy as np

from .airframe import MAX_ROTOR_SPEED, ROTOR_IZZ, ROTOR_P_GAIN, FireFly
from .imu_synth import DIFF_STEP_S, GRAVITY, AccelNoise, PoseBuffer, specific_force
from .sensors import D435i
from .vio_link import rot_to_quat

# ---- ArduPilot JSON SITL protocol ------------------------------------------------------------

SERVO_MAGIC_16 = 18458  # servo_packet_16: magic, frame_rate, frame_count, pwm[16]
SERVO_MAGIC_32 = 29569  # servo_packet_32 (SIM_JSON with 32 channels)
_SERVO_16 = struct.Struct("<HHI16H")
_SERVO_32 = struct.Struct("<HHI32H")

ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


@dataclass(frozen=True)
class ServoPacket:
    frame_rate: int  # Hz ArduPilot wants the physics to run at
    frame_count: int  # increments by one per physics step; repeats when ArduPilot resends
    pwm: tuple[int, ...]  # microseconds, channel 0 first


def parse_servo_packet(data: bytes) -> ServoPacket | None:
    """ArduPilot's servo output packet, or None if it is not one."""
    for magic, st in ((SERVO_MAGIC_16, _SERVO_16), (SERVO_MAGIC_32, _SERVO_32)):
        if len(data) == st.size:
            v = st.unpack(data)
            if v[0] == magic:
                return ServoPacket(v[1], v[2], tuple(v[3:]))
    return None


def servo_packet(frame_rate: int, frame_count: int, pwm) -> bytes:
    """The packet ArduPilot sends (for tests and tools)."""
    pwm = list(pwm) + [0] * (16 - len(pwm))
    return _SERVO_16.pack(SERVO_MAGIC_16, frame_rate, frame_count, *pwm[:16])


def enu_flu_to_ned_frd(R_enu_flu: np.ndarray) -> np.ndarray:
    """Attitude of an FLU body in the ENU world -> attitude of the FRD body in the NED world."""
    return ENU_TO_NED @ R_enu_flu @ FLU_TO_FRD


def fdm_reply(
    t: float,
    p_enu: np.ndarray,
    R_enu_flu: np.ndarray,
    v_enu: np.ndarray,
    gyro_flu: np.ndarray,
    accel_flu: np.ndarray,
) -> bytes:
    """The physics' state for ArduPilot: JSON, one line. gyro and accel (specific force: +9.8 up at
    rest) are in the body frame; position, velocity and attitude in the world frame."""
    q = rot_to_quat(enu_flu_to_ned_frd(R_enu_flu))
    msg = {
        "timestamp": round(float(t), 6),
        "imu": {
            "gyro": [round(float(x), 6) for x in FLU_TO_FRD @ gyro_flu],
            "accel_body": [round(float(x), 5) for x in FLU_TO_FRD @ accel_flu],
        },
        "position": [round(float(x), 5) for x in ENU_TO_NED @ p_enu],
        "quaternion": [round(float(x), 7) for x in q],
        "velocity": [round(float(x), 5) for x in ENU_TO_NED @ v_enu],
    }
    return ("\n" + json.dumps(msg, separators=(",", ":")) + "\n").encode()


# ---- rotors ----------------------------------------------------------------------------------

SERVO_MIN, SERVO_MAX = 1100, 1900  # as the Gazebo model's <control> (airframe.py)
# Rotor speed response, as the Gazebo model's joint controller (airframe.py), which ArduCopter's FireFly
# gains were tuned on: a P controller on a light rotor (time constant inertia / gain = 0.8 ms) whose
# torque saturates at 2.5 N m, so large speed changes slew at 2.5 / inertia rad/s^2 (~13 ms to full speed).
# A plain 10 ms lag here made the roll loop oscillate at ~±130 deg/s (first Isaac Sim flight).
MOTOR_TAU_S = ROTOR_IZZ / ROTOR_P_GAIN
MOTOR_SLEW = 2.5 / ROTOR_IZZ  # rad/s^2
# Yaw reaction torque per newton of thrust: the Gazebo blades' drag/lift (cda/cla = 0.10/4.25) at
# their centre of pressure. Small next to a real prop, but it is what ArduCopter's yaw gains were tuned on.
DRAG_LIFT_RATIO = 0.10 / 4.25
BODY_DRAG = 0.05  # N per m/s: a little air drag on the body (Gazebo's model has none besides the blades)


class RotorThrust:
    """Four rotors: speed from ArduPilot's PWM (as ardupilot_gazebo: (pwm - min) / (max - min) x the
    multiplier), a first-order speed response with a slew limit (as Gazebo's), thrust ~ speed^2
    (max_thrust_n at full speed) along body +z, and the yaw reaction of each prop (a counter-clockwise
    prop yaws the body clockwise)."""

    def __init__(self, ff: FireFly | None = None, tau_s: float = MOTOR_TAU_S, slew: float = MOTOR_SLEW):
        self.ff = ff or FireFly()
        self.tau, self.slew = tau_s, slew
        self.speed = np.zeros(len(self.ff.rotors))  # rad/s
        self.k_yaw = self.ff.blade_cp * DRAG_LIFT_RATIO
        com = np.array(self.ff.com)
        self.arms = np.array([np.array(r.xyz) - com for r in self.ff.rotors])
        self.spin = np.array([1.0 if r.ccw else -1.0 for r in self.ff.rotors])
        self.channels = [r.channel for r in self.ff.rotors]

    def command(self, pwm) -> np.ndarray:
        """Commanded rotor speeds (rad/s) for ArduPilot's PWM outputs."""
        raw = np.array([pwm[c] if c < len(pwm) else 0 for c in self.channels], float)
        return np.clip((raw - SERVO_MIN) / (SERVO_MAX - SERVO_MIN), 0.0, 1.0) * MAX_ROTOR_SPEED

    def thrusts(self) -> np.ndarray:
        return self.ff.max_thrust_n * (self.speed / MAX_ROTOR_SPEED) ** 2

    def step(self, pwm, dt: float, v_body: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Advance the rotors by dt; returns (force, torque about the centre of mass), body FLU."""
        target = self.command(pwm)
        a = 1.0 - math.exp(-dt / self.tau) if self.tau > 0 else 1.0
        change = a * (target - self.speed)
        if self.slew > 0:
            change = np.clip(change, -self.slew * dt, self.slew * dt)
        self.speed += change
        T = self.thrusts()
        force = np.array([0.0, 0.0, T.sum()])
        torque = np.zeros(3)
        for arm, t_i, s in zip(self.arms, T, self.spin, strict=True):
            torque += np.cross(arm, [0.0, 0.0, t_i])
            torque[2] -= s * self.k_yaw * t_i
        if v_body is not None:
            force -= BODY_DRAG * np.asarray(v_body, float)
        return force, torque


# ---- D435i IMU from the physics --------------------------------------------------------------


class GyroNoise:
    """Gyro white noise and bias random walk (continuous-time densities, as AccelNoise)."""

    def __init__(self, d: D435i, seed: int = 1):
        self.rng = np.random.default_rng(seed)
        self.nd, self.rw = d.gyro_noise_density, d.gyro_random_walk
        self.bias = np.zeros(3)

    def __call__(self, w: np.ndarray, dt: float) -> np.ndarray:
        if dt <= 0:
            return w + self.bias
        self.bias += self.rw * math.sqrt(dt) * self.rng.normal(size=3)
        return w + self.bias + self.nd / math.sqrt(dt) * self.rng.normal(size=3)


class ImuSampler:
    """Feed it the sensor link's world pose and angular velocity every physics step; it returns the
    D435i IMU samples (t, gyro, accel in the link frame) at imu_rate as soon as the poses DIFF_STEP_S
    after each sample are in (the accelerometer is a second difference, see imu_synth)."""

    def __init__(self, d: D435i | None = None, seed: int = 0, noise: bool = True):
        d = d or D435i()
        self.period = 1.0 / d.imu_rate
        self.buf = PoseBuffer()
        self.gyro: deque[tuple[float, np.ndarray]] = deque(maxlen=2000)
        self.acc_noise = AccelNoise(d, seed) if noise else None
        self.gyro_noise = GyroNoise(d, seed + 1) if noise else None
        self.next_t: float | None = None
        self.last_t: float | None = None

    def add(
        self, t: float, p: np.ndarray, R: np.ndarray, w_world: np.ndarray
    ) -> list[tuple[float, np.ndarray, np.ndarray]]:
        self.buf.add(t, np.asarray(p, float), np.asarray(R, float))
        self.gyro.append((t, R.T @ np.asarray(w_world, float)))
        if self.next_t is None:
            self.next_t = t + DIFF_STEP_S
        out = []
        while self.buf.newest() >= self.next_t + DIFF_STEP_S:
            ts = self.next_t
            self.next_t = round((self.next_t + self.period) * 1e6) / 1e6
            f = specific_force(self.buf, ts)
            if f is None:
                continue
            w = min(self.gyro, key=lambda e: abs(e[0] - ts))[1]
            dt = 0.0 if self.last_t is None else ts - self.last_t
            self.last_t = ts
            if self.acc_noise:
                f = self.acc_noise(f, dt)
                w = self.gyro_noise(w, dt)
            out.append((ts, w, f))
        return out


def body_specific_force(v_prev: np.ndarray, v: np.ndarray, dt: float, R_enu_flu: np.ndarray) -> np.ndarray:
    """ArduPilot's accelerometer: the body's acceleration minus gravity, in the body frame."""
    return R_enu_flu.T @ ((np.asarray(v) - np.asarray(v_prev)) / dt - GRAVITY)


# ---- Isaac -> relay stream -------------------------------------------------------------------
# One frame: <u32 header length><u32 payload length><header JSON><payload bytes>.
# Headers: {"k": "img", "topic", "t", "w", "h", "fmt": "L8" | "RGB8" | "F32"}   payload = pixels
#          {"k": "imu", "topic", "t", "gyro": [3], "acc": [3]}
#          {"k": "pose", "topic", "t", "poses": [[name, x, y, z, qw, qx, qy, qz], ...]}
#          {"k": "clock", "topic", "t"}

RELAY_PORT = 9100
_LEN = struct.Struct("<II")
PIXEL_BYTES = {"L8": 1, "RGB8": 3, "F32": 4}


def pack_frame(header: dict, payload: bytes = b"") -> bytes:
    h = json.dumps(header, separators=(",", ":")).encode()
    return _LEN.pack(len(h), len(payload)) + h + payload


class FrameReader:
    """Splits a byte stream into (header, payload) frames."""

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[dict, bytes]]:
        self.buf += data
        out = []
        while len(self.buf) >= _LEN.size:
            hl, pl = _LEN.unpack_from(self.buf)
            end = _LEN.size + hl + pl
            if len(self.buf) < end:
                break
            header = json.loads(bytes(self.buf[_LEN.size : _LEN.size + hl]))
            out.append((header, bytes(self.buf[_LEN.size + hl : end])))
            del self.buf[:end]
        return out


def image_frame(topic: str, t: float, img: np.ndarray, fmt: str) -> bytes:
    h, w = img.shape[:2]
    dtype = np.float32 if fmt == "F32" else np.uint8
    data = np.ascontiguousarray(img, dtype=dtype).tobytes()
    if len(data) != w * h * PIXEL_BYTES[fmt]:
        raise ValueError(f"{fmt} image of shape {img.shape} has the wrong number of channels")
    return pack_frame({"k": "img", "topic": topic, "t": t, "w": w, "h": h, "fmt": fmt}, data)


def to_gray(rgba: np.ndarray) -> np.ndarray:
    """An infrared imager's L8 image from a rendered RGB(A) one (Rec. 601 luma)."""
    rgb = rgba[..., :3].astype(np.float32)
    return np.clip(rgb @ np.array([0.299, 0.587, 0.114], np.float32) + 0.5, 0, 255).astype(np.uint8)
