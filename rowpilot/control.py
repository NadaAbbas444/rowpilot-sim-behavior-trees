"""Controllers that turn errors into body-frame velocity commands (what PX4 offboard velocity mode takes)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .perception import LaneEstimate


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class Command:
    """Body-frame velocity setpoint: vx forward, vy left, vz up (m/s), yaw_rate CCW (rad/s)."""

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, i_limit: float = 0.5):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.reset()

    def reset(self) -> None:
        self._i = 0.0
        self._prev: float | None = None

    def step(self, err: float, dt: float) -> float:
        self._i = clamp(self._i + err * dt, -self.i_limit, self.i_limit)
        d = 0.0 if self._prev is None or dt <= 0 else (err - self._prev) / dt
        self._prev = err
        return self.kp * err + self.ki * self._i + self.kd * d


def altitude_hold(z: float, z_ref: float, k: float = 1.2, v_max: float = 0.8) -> float:
    return clamp(k * (z_ref - z), -v_max, v_max)


class LaneFollower:
    """Keeps the drone on the lane centre line while moving forward at a set speed."""

    # Tuned for the flight controller's measured response (ArduCopter GUIDED: 0.5 s dead time, 0.3 s lag,
    # see SimConfig): the earlier k_lat 2.0 / k_yaw 1.5 made the drone swing across the lane in ArduPilot.
    def __init__(
        self, k_lat: float = 0.6, ki_lat: float = 0.0, k_yaw: float = 0.8, vy_max: float = 0.8, kd_lat: float = 0.2
    ):
        self.lat = PID(k_lat, ki_lat, kd_lat)
        self.k_yaw, self.vy_max = k_yaw, vy_max

    def reset(self) -> None:
        self.lat.reset()

    def step(self, est: LaneEstimate, forward_speed: float, dt: float) -> Command:
        vy = clamp(self.lat.step(est.lateral_offset, dt), -self.vy_max, self.vy_max)
        yaw_rate = clamp(self.k_yaw * est.heading_error, -0.8, 0.8)
        # Slow down when badly off-centre or misaligned, so the correction happens before the crop does.
        slow = clamp(1.0 - 1.5 * abs(est.lateral_offset) - 1.0 * abs(est.heading_error), 0.2, 1.0)
        return Command(vx=forward_speed * slow, vy=vy, yaw_rate=yaw_rate)


def goto_command(
    x: float,
    y: float,
    yaw: float,
    tx: float,
    ty: float,
    final_yaw: float | None,
    start: tuple[float, float] | None = None,
    v_max: float = 1.0,
    k_pos: float = 0.8,
    k_cross: float = 0.9,  # cross-track and yaw gains tuned for the flight controller's delay (see SimConfig)
    k_yaw: float = 0.8,
    tol: float = 0.15,
) -> tuple[Command, bool]:
    """Fly to (tx, ty) in the map frame, along the straight line from `start` if given.

    Following the line (not just aiming at the target) matters in a lane: a 40 m leg aimed at its
    end point hardly reacts to a 0.4 m sideways drift, which is already close to the crop.
    Long legs turn to face the direction of travel first, so the forward cameras see ahead.
    Returns (command, arrived)."""
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < tol:
        if final_yaw is None:
            return Command(), True
        e = wrap_angle(final_yaw - yaw)
        return Command(yaw_rate=clamp(k_yaw * e, -0.8, 0.8)), abs(e) < math.radians(4)

    ux, uy = dx / dist, dy / dist  # towards the target
    along = dist  # signed distance still to go along the line (negative once past the target)
    cross_x = cross_y = 0.0
    if start is not None and (seg := math.hypot(tx - start[0], ty - start[1])) > 0.5:
        ux, uy = (tx - start[0]) / seg, (ty - start[1]) / seg
        e_cross = -uy * (x - start[0]) + ux * (y - start[1])  # signed distance left of the line
        cross_x, cross_y = k_cross * e_cross * uy, -k_cross * e_cross * ux  # push back towards it
        along = ux * dx + uy * dy

    long_leg = along > 0.6
    desired_yaw = math.atan2(uy, ux) if long_leg else (final_yaw if final_yaw is not None else yaw)
    e_yaw = wrap_angle(desired_yaw - yaw)
    speed = clamp(k_pos * along, -v_max, v_max)  # backs up if it overshot
    if abs(e_yaw) > math.radians(30) and long_leg:
        speed = 0.0  # turn on the spot first
    vxm = clamp(speed * ux + cross_x, -v_max, v_max)
    vym = clamp(speed * uy + cross_y, -v_max, v_max)
    c, s = math.cos(yaw), math.sin(yaw)
    return Command(vx=c * vxm + s * vym, vy=-s * vxm + c * vym, yaw_rate=clamp(k_yaw * e_yaw, -0.8, 0.8)), False
