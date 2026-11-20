"""A pipe-rail trolley: the greenhouse's ground robot, simulated, and its mission as a behaviour tree.

Pipe-rail trolleys are the work trolleys of tomato greenhouses: a long, low chassis with flanged wheels
that ride on the two heating pipes in each lane, and small wheels for the concrete path in the headland.
Here it is autonomous, carrying the same RealSense D435i at the front, and driven by plain motor control:
the tree sends a forward speed and a turn rate (differential drive); there is no flight controller.

How it moves (TrolleySim):
  - on the concrete: drives and turns on the spot, a first-order motor lag;
  - mounting: when its front reaches the rail start (x = 0) lined up with a lane's rails (within
    rail_tol_m across and rail_tol_deg in heading), it is on the rails; arriving badly lined up is a
    collision with the rail ends;
  - on the rails: forward and back only, no steering, it cannot leave the lane; past the rail end it
    derails (a crash); reversing until its front is off the rail start puts it back on the concrete.
How it knows where it is: on the rails, the rails fix its position across the lane and its heading, and
the wheels measure how far along it is; on the concrete, its wheel odometry, from where it left the rails
(or the dock): a short trip (WheelOdometry). Visual odometry (OpenVINS) runs too, as an independent check:
lost for long on the concrete, the trolley stops (it cannot tell whether its wheels slip). In Gazebo,
OpenVINS on the trolley was 10-20 cm off after every start and stop, which is why the wheels lead.

The mission (build_trolley_tree), per lane: GoToRailStart -> MountRail -> DriveAlongRail (to the rail
end, stopping for anything in the way) -> ReverseOut (back off the rails; the lane is done), then home to
the dock. The safety guard is the drone's, with stopping and docking in place of hovering and landing:

Root (Sequence, re-evaluated every tick)
├── SafetyGuard (Selector)
│   ├── AllHealthy: NotAborted, BatteryAboveReserve, LocalizationOK (visual odometry, or on the rails)
│   └── Failsafe
│       ├── PoseGlitch: VOLostBriefly -> Stop             pause on the concrete, then resume
│       ├── EmergencyStop: MustStopNow -> StopHere       battery critical / lost on the concrete
│       └── ReturnAndDock: MotorsOn -> ReverseOut -> ReturnToDock -> Dock   low battery / a failed step
└── Mission: Preflight, Lane 1 .. Lane n, ReturnToDock, Dock
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
import py_trees
from py_trees.common import Status

from .control import Command, clamp
from .layout import GreenhouseLayout
from .mission import (
    BatteryAboveReserve,
    Condition,
    MissionContext,
    Node,
    NotAborted,
    Once,
    Preflight,
    VOLostBriefly,
)
from .sim import SimConfig, SimVehicle
from .vehicle import VehicleState


@dataclass
class TrolleyConfig:
    length: float = 1.5  # m, chassis (as the pipe-rail trolleys in Dutch-style greenhouses)
    width: float = 0.6  # m; the wheels sit on the pipes, 0.55 m apart
    camera_height: float = 0.8  # m, the D435i on the front of the chassis
    max_speed: float = 1.0  # m/s
    max_turn_rate: float = 0.8  # rad/s, turning on the spot on the concrete
    motor_lag_s: float = 0.25
    rail_tol_m: float = 0.06  # lined up this well with the rails to mount (the rail starts' lead-in guides it)
    rail_tol_deg: float = 4.0
    odometry_noise_per_sqrt_m: float = 0.002  # wheel odometry along the rail
    # wheel odometry on the concrete: a scale error per run (tyres), noise per metre and per radian turned
    wheel_scale_err: float = 0.005
    wheel_turn_err: float = 0.01
    wheel_noise_per_sqrt_m: float = 0.003
    wheel_turn_noise_per_sqrt_rad: float = 0.005
    idle_drain_pct_s: float = 0.01  # electronics and sensors
    move_drain_pct_per_m: float = 0.04
    # Things in the way, (x, y, radius, from s, until s): a worker, a harvest trolley in a lane.
    obstacles: list[tuple[float, float, float, float, float]] = field(default_factory=list)


def headland_x(layout: GreenhouseLayout, cfg: TrolleyConfig) -> float:
    """The line in the headland the trolley drives along between lanes: its front 1.5 m short of the
    rail starts, a straight run to line up on, and it turns on the spot clear of the crop rows and of
    the facade behind."""
    return -cfg.length / 2 - 1.5


def rail_start_approach(layout: GreenhouseLayout, cfg: TrolleyConfig, lane: int) -> tuple[float, float]:
    """Where the trolley lines up for a lane's rails (its centre), facing along the rows."""
    return (headland_x(layout, cfg), layout.lane_center_y(lane))


def rail_end_stop(layout: GreenhouseLayout, cfg: TrolleyConfig, clearance: float = 0.5) -> float:
    """The trolley's centre x where it stops at the far end of the rails, its front clearance short."""
    return layout.row_length - clearance - cfg.length / 2


def dock_xy(layout: GreenhouseLayout) -> tuple[float, float]:
    return layout.home


def planned_route(layout: GreenhouseLayout, cfg: TrolleyConfig | None = None) -> list[tuple[float, float]]:
    """The trolley's intended path (its centre): along the headland line, up each lane's rails to the stop
    short of the rail end and back out, then home to the dock."""
    cfg = cfg or TrolleyConfig()
    hx = headland_x(layout, cfg)
    route = [dock_xy(layout)]
    for i in range(layout.n_lanes):
        y = layout.lane_center_y(i)
        route += [(hx, y), (rail_end_stop(layout, cfg), y), (hx, y)]
    return route + [dock_xy(layout)]


# ---- the rails' rules: shared by the Python simulator (TrolleySim) and Gazebo (trolley_gazebo.py) ----------


def footprint(x: float, y: float, yaw: float, cfg: TrolleyConfig) -> np.ndarray:
    """Corners and edge midpoints of the chassis, map frame."""
    h, w = cfg.length / 2, cfg.width / 2
    pts = np.array([[a * h, b * w] for a in (-1, 0, 1) for b in (-1, 0, 1) if (a, b) != (0, 0)])
    c, s = math.cos(yaw), math.sin(yaw)
    return pts @ np.array([[c, s], [-s, c]]) + [x, y]


def front(x: float, y: float, yaw: float, cfg: TrolleyConfig) -> tuple[float, float]:
    h = cfg.length / 2
    return x + h * math.cos(yaw), y + h * math.sin(yaw)


def near_wall_x(layout: GreenhouseLayout) -> float:
    """The near facade (gazebo_world.infrastructure)."""
    return layout.home[0] - 2.0


def mount(layout: GreenhouseLayout, cfg: TrolleyConfig, x: float, y: float, yaw: float) -> tuple[int | None, str]:
    """The chassis has just reached the rail start (x = 0) driving forwards: (the lane whose rails it is
    now on, "") if lined up with them; (None, crash reason) if it hit the rail start badly lined up;
    (None, "") if it is not at a lane (the floor collision check then decides)."""
    fy = front(x, y, yaw, cfg)[1]
    lane = layout.nearest_lane(fy)
    if abs(fy - layout.lane_center_y(lane)) > layout.path_width / 2:
        return None, ""
    lined_up = abs(y - layout.lane_center_y(lane)) <= cfg.rail_tol_m and abs(yaw) <= math.radians(cfg.rail_tol_deg)
    return (lane, "") if lined_up else (None, "hit the rail start badly lined up")


def floor_collision(layout: GreenhouseLayout, cfg: TrolleyConfig, x: float, y: float, yaw: float) -> str:
    """Why the chassis, on the concrete, is touching something ("" if it is not)."""
    y0, y1 = -layout.pitch - 1.0, layout.n_lanes * layout.pitch + 1.0
    for px, py in footprint(x, y, yaw, cfg):
        if layout.in_crop(px, py):
            return "drove into a crop row"
        if px < near_wall_x(layout) or not (y0 < py < y1):
            return "drove into the greenhouse wall"
        if px > 0.0:  # only on the rails past the rail start
            return "drove onto the rails off the concrete"
    return ""


def derailed(layout: GreenhouseLayout, cfg: TrolleyConfig, x: float) -> bool:
    """On the rails (heading along +x), past the end of them."""
    return x + cfg.length / 2 > layout.row_length - 0.05


def off_rail_start(cfg: TrolleyConfig, x: float) -> bool:
    """On the rails, reversed until the front is off the rail start: back on the concrete."""
    return x + cfg.length / 2 < 0.0


class WheelOdometry:
    """Dead reckoning from the wheel encoders on the concrete, from the last known pose (the dock, or where
    the trolley left the rails). Errors: a scale error per run on distance and on turning (tyre wear and
    pressure, turning on the spot), and noise per metre and per radian (slip)."""

    def __init__(self, cfg: TrolleyConfig, rng: np.random.Generator, x: float, y: float, yaw: float):
        self.cfg, self.rng = cfg, rng
        self.scale = 1.0 + rng.normal(0, cfg.wheel_scale_err)
        self.turn_scale = 1.0 + rng.normal(0, cfg.wheel_turn_err)
        self.reset(x, y, yaw)

    def reset(self, x: float, y: float, yaw: float) -> None:
        self.x, self.y, self.yaw = x, y, yaw

    def update(self, v: float, w: float, dt: float) -> None:
        c = self.cfg
        ds, dyaw = v * dt * self.scale, w * dt * self.turn_scale
        if ds:
            ds += self.rng.normal(0, c.wheel_noise_per_sqrt_m * math.sqrt(abs(v) * dt))
        if dyaw:
            dyaw += self.rng.normal(0, c.wheel_turn_noise_per_sqrt_rad * math.sqrt(abs(w) * dt))
        mid = self.yaw + dyaw / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = (self.yaw + dyaw + math.pi) % (2 * math.pi) - math.pi


class TrolleySim(SimVehicle):
    """The pipe-rail trolley in the greenhouse; the Vehicle interface, as SimVehicle for the drone."""

    robot = "trolley"

    def __init__(self, layout: GreenhouseLayout, cfg: SimConfig | None = None, trolley: TrolleyConfig | None = None):
        super().__init__(layout, cfg)
        self.tc = trolley or TrolleyConfig()
        self.x, self.y = dock_xy(layout)
        self.z = 0.0
        self.v = self.w = 0.0  # forward speed, turn rate
        self.rail: int | None = None
        self.crash_reason: str | None = None
        self.odo_err = 0.0
        self.wheels = WheelOdometry(self.tc, np.random.default_rng(7000 + self.cfg.seed), self.x, self.y, self.yaw)

    # ---- geometry --------------------------------------------------------------------------
    def _front(self) -> tuple[float, float]:
        return front(self.x, self.y, self.yaw, self.tc)

    def _footprint(self) -> np.ndarray:
        return footprint(self.x, self.y, self.yaw, self.tc)

    def _camera_xy(self) -> tuple[float, float]:
        return self._front()

    def _scan_blocked(self) -> bool:
        return False  # the camera is always below the canopy

    def _active_obstacles(self) -> list[tuple[float, float, float]]:
        return [(x, y, r) for x, y, r, t0, t1 in self.tc.obstacles if t0 <= self.t < t1]

    def _extra_hits(self, x0: float, y0: float, th: np.ndarray) -> np.ndarray:
        t = np.full(len(th), np.inf)
        d = np.stack([np.cos(th), np.sin(th)], 1)
        for ox, oy, r in self._active_obstacles():
            o = np.array([ox - x0, oy - y0])
            along = d @ o
            miss2 = o @ o - along**2
            hit = (along > 0) & (miss2 <= r * r)
            t = np.where(hit, np.minimum(t, along - np.sqrt(np.maximum(r * r - miss2, 0))), t)
        return t

    # ---- Vehicle interface -------------------------------------------------------------------
    @property
    def landed(self) -> bool:
        """Standing still."""
        return abs(self.v) < 0.01 and abs(self.w) < 0.01

    @property
    def at_rest(self) -> bool:
        """Finished: motors off. (Standing still is not enough: it stands still waiting for a worker.)"""
        return not self.armed and self.landed

    def state(self) -> VehicleState:
        if self.rail is not None:  # rails: across-lane position and heading fixed, distance from the wheels
            ex, ey, eyaw = self.x + self.odo_err, self.y, 0.0
        else:  # the concrete: wheel odometry since the rails or the dock
            ex, ey, eyaw = self.wheels.x, self.wheels.y, self.wheels.yaw
        return VehicleState(
            self.t, ex, ey, 0.0, eyaw, self.vo_healthy(), self.battery, self.armed, self.landed, self.rail
        )

    def disarm(self) -> None:
        if self.landed:
            self.armed = False
            self.cmd = Command()

    def land(self) -> None:
        """Stop where it is."""
        self.cmd = Command()

    # ---- simulation ------------------------------------------------------------------------
    def _crash(self, reason: str) -> None:
        self.crashed, self.crash_reason = True, reason
        self.v = self.w = 0.0

    def step(self, dt: float) -> None:
        tc = self.tc
        self.t += dt
        self.battery = max(0.0, self.battery - tc.idle_drain_pct_s * dt)
        if not self.armed or self.crashed or self.battery <= 0:
            self.v = self.w = 0.0
            return
        k = min(1.0, dt / tc.motor_lag_s)
        self.v += (clamp(self.cmd.vx, -tc.max_speed, tc.max_speed) - self.v) * k
        w_target = 0.0 if self.rail is not None else clamp(self.cmd.yaw_rate, -tc.max_turn_rate, tc.max_turn_rate)
        self.w += (w_target - self.w) * k
        dist = abs(self.v) * dt
        lead_before = self._footprint()[:, 0].max()
        if self.rail is not None:
            self.x += self.v * dt
            self.odo_err += self.rng.normal(0, tc.odometry_noise_per_sqrt_m * math.sqrt(dist)) if dist else 0.0
            if derailed(self.layout, tc, self.x):
                self._crash("ran off the end of the rails")
            elif off_rail_start(tc, self.x):  # leaves the rails knowing where it is: the wheels take over
                self.wheels.reset(self.x + self.odo_err, self.layout.lane_center_y(self.rail), 0.0)
                self.rail = None
        else:
            self.yaw = (self.yaw + self.w * dt + math.pi) % (2 * math.pi) - math.pi
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.wheels.update(self.v, self.w, dt)
            if lead_before < 0.0 <= self._footprint()[:, 0].max() and self.v > 0:  # reaches the rail start
                lane, why = mount(self.layout, tc, self.x, self.y, self.yaw)
                if why:
                    self._crash(why)
                elif lane is not None:
                    self.rail = lane
                    self.y, self.yaw, self.w = self.layout.lane_center_y(lane), 0.0, 0.0
                    self.odo_err = 0.0
            if not self.crashed and self.rail is None:
                why = floor_collision(self.layout, tc, self.x, self.y, self.yaw)
                if why:
                    self._crash(why)
        if not self.crashed:
            for ox, oy, r in self._active_obstacles():
                if np.min(np.hypot(*(self._footprint() - [ox, oy]).T)) < r:
                    self._crash("ran into an obstacle")
        self.battery = max(0.0, self.battery - tc.move_drain_pct_per_m * dist)


# ---- motor control ----------------------------------------------------------------------------


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def drive_to(
    x: float, y: float, yaw: float, tx: float, ty: float, final_yaw: float | None, v_max: float = 0.6, tol: float = 0.04
) -> tuple[Command, bool]:
    """Differential drive to a point: turn on the spot towards it, drive with heading correction, then
    turn on the spot to the final heading. Returns (command, arrived)."""
    dx, dy = tx - x, ty - y
    d = math.hypot(dx, dy)
    if d > tol:
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        if d < 0.7 and abs(along) > 0.85 * d:  # close and (nearly) in line: straight there, forwards or back
            return Command(vx=clamp(1.0 * along, -0.3, 0.3), yaw_rate=0.0), False
        e = wrap(math.atan2(dy, dx) - yaw)
        if abs(e) > math.radians(20):  # turn on the spot first
            return Command(yaw_rate=clamp(1.5 * e, -0.8, 0.8)), False
        return Command(vx=min(v_max, 0.8 * d + 0.08), yaw_rate=clamp(2.0 * e, -0.8, 0.8)), False
    if final_yaw is not None:
        e = wrap(final_yaw - yaw)
        if abs(e) > math.radians(1.0):
            return Command(yaw_rate=math.copysign(max(0.05, min(0.6, 1.5 * abs(e))), e)), False
    return Command(), True


def follow_line(y: float, yaw: float, line_y: float, speed: float) -> Command:
    """Drive along +x keeping on y = line_y: heading and offset feedback (a Stanley-like law)."""
    return Command(vx=speed, yaw_rate=clamp(-1.6 * wrap(yaw) - 2.2 * (y - line_y), -0.5, 0.5))


# ---- conditions -------------------------------------------------------------------------------


class LocalizationOK(Condition):
    """Visual odometry is tracking, or the trolley is on the rails (where it does not need it)."""

    def check(self) -> bool:
        return self.ctx.s.vo_healthy or self.ctx.s.on_rail is not None


class OnRail(Condition):
    def check(self) -> bool:
        return self.ctx.s.on_rail is not None


class MotorsOn(Condition):
    def check(self) -> bool:
        return self.ctx.s.armed


class MustStopNow(Condition):
    """Battery critical, or visual odometry lost too long on the concrete: stop where it is."""

    def check(self) -> bool:
        c, p = self.ctx, self.ctx.params
        if c.s.battery_pct <= p.critical_battery_pct:
            c.abort("battery critical")
        elif not c.s.vo_healthy and c.s.on_rail is None and c.vo_lost_for() >= p.vo_glitch_timeout_s:
            c.abort("visual odometry lost")
        return c.abort_reason in ("battery critical", "visual odometry lost")


# ---- actions ----------------------------------------------------------------------------------


def trolley_config(ctx: MissionContext) -> TrolleyConfig:
    return getattr(ctx.vehicle, "tc", None) or TrolleyConfig()


class Stop(Node):
    """Stand still (always RUNNING): wait out a short loss of visual odometry."""

    def initialise(self) -> None:
        self.log("stopped, waiting")

    def update(self) -> Status:
        self.ctx.vehicle.send(Command())
        return Status.RUNNING


class StopHere(Node):
    """Stop where it is and switch the motors off."""

    def initialise(self) -> None:
        self.log("stopping")

    def update(self) -> Status:
        self.ctx.vehicle.send(Command())
        if self.ctx.s.landed:
            self.ctx.vehicle.disarm()
            self.log("stopped, motors off")
            return Status.SUCCESS
        return Status.RUNNING


class DriveTo(Once):
    """Drive on the concrete through map-frame waypoints (differential drive), computed at the start."""

    def __init__(self, name, ctx, waypoints: Callable[[], list[tuple[float, float]]], final_yaw: float | None = None):
        super().__init__(name, ctx)
        self.waypoints_fn, self.final_yaw = waypoints, final_yaw

    def start(self) -> None:
        if self.ctx.s.on_rail is not None:
            return
        self.wps = list(self.waypoints_fn())
        self.log("waypoints " + ", ".join(f"({x:.1f}, {y:.1f})" for x, y in self.wps))

    def run(self) -> Status:
        s = self.ctx.s
        if s.on_rail is not None:
            return self.fail("on the rails: reverse out first")
        while self.wps:
            last = len(self.wps) == 1
            cmd, arrived = drive_to(
                s.x, s.y, s.yaw, *self.wps[0], self.final_yaw if last else None, v_max=self.ctx.params.transit_speed
            )
            if not arrived:
                self.ctx.vehicle.send(cmd)
                return Status.RUNNING
            self.wps.pop(0)
        self.ctx.vehicle.send(Command())
        return Status.SUCCESS


class GoToRailStart(DriveTo):
    """Along the headland to the start of a lane's rails, lined up to drive onto them."""

    def __init__(self, name, ctx, lane: int):
        self.lane_idx = lane
        tc = trolley_config(ctx)
        target = rail_start_approach(ctx.layout, tc, lane)

        def route():
            s = ctx.s
            return [(headland_x(ctx.layout, tc), s.y), target]

        super().__init__(name, ctx, route, final_yaw=0.0)


class MountRail(Once):
    """Drive slowly onto the rails until the rail contact says so, on the lane's centre line, steering on
    visual odometry (re-localized at the previous rail start, so it has drifted over a few metres only).
    Just before the rail start it checks: not lined up, it backs up and tries again (MAX_TRIES).

    The depth camera is not used here: at the row ends it sees the ragged ends of the crop rows, not the
    lane's faces, and mixing it in made the line-up worse (measured: 2.8 cm error instead of 1.8 cm)."""

    SPEED = 0.25
    CHECK_AT = -0.25  # m: front this far before the rail start: lined up, or back up
    BACK_TO = -1.0  # m: front back to here for another try
    TOL_M, TOL_DEG, MAX_TRIES = 0.03, 2.5, 3

    def __init__(self, name, ctx, lane: int):
        super().__init__(name, ctx)
        self.lane_idx = lane

    def start(self) -> None:
        self.tries, self.backing = 1, False

    def errors(self) -> tuple[float, float]:
        """(offset of the trolley's centre from the lane centre line, heading off it), > 0 to the left."""
        s = self.ctx.s
        return s.y - self.ctx.layout.lane_center_y(self.lane_idx), wrap(s.yaw)

    def run(self) -> Status:
        c, s = self.ctx, self.ctx.s
        if s.on_rail == self.lane_idx:
            c.vehicle.send(Command())
            self.log(f"on the rails of lane {self.lane_idx + 1}")
            return Status.SUCCESS
        if s.on_rail is not None:
            return self.fail(f"on the rails of lane {s.on_rail + 1}, not {self.lane_idx + 1}")
        front_x = s.x + trolley_config(c).length / 2 * math.cos(s.yaw)
        ey, eh = self.errors()
        if self.backing:
            if front_x > self.BACK_TO:
                c.vehicle.send(Command(vx=-self.SPEED))
                return Status.RUNNING
            self.backing, self.tries = False, self.tries + 1
        if front_x > self.CHECK_AT and (abs(ey) > self.TOL_M or abs(eh) > math.radians(self.TOL_DEG)):
            if self.tries >= self.MAX_TRIES:
                return self.fail(f"could not line up with the rails in {self.MAX_TRIES} tries")
            self.log(f"not lined up ({ey:+.2f} m, {math.degrees(eh):+.1f} deg): backing up to try again")
            self.backing = True
            c.vehicle.send(Command(vx=-self.SPEED))
            return Status.RUNNING
        if front_x > 0.15:
            return self.fail("no rail contact past the rail start")
        c.vehicle.send(follow_line(ey, eh, 0.0, self.SPEED))
        return Status.RUNNING


def obstacle_ahead(scan: np.ndarray, half_width: float = 0.4) -> float:
    """Distance to the nearest depth point straight ahead, within the trolley's width (inf: clear)."""
    if not len(scan):
        return math.inf
    ahead = scan[(scan[:, 0] > 0) & (np.abs(scan[:, 1]) < half_width)]
    return float(ahead[:, 0].min()) if len(ahead) else math.inf


class DriveAlongRail(Once):
    """Along the rails to the far end (the wheels say how far), slowing down before the end, and
    stopping for anything in the way; waits for it to go, gives up after obstacle_timeout_s."""

    STOP_AT, SLOW_AT = 0.45, 1.5  # m: depth distance to an obstacle

    def __init__(self, name, ctx, lane: int, obstacle_timeout_s: float = 60.0):
        super().__init__(name, ctx)
        self.lane_idx, self.timeout = lane, obstacle_timeout_s
        self.end_x = rail_end_stop(ctx.layout, trolley_config(ctx))

    def start(self) -> None:
        self.blocked_since: float | None = None

    def run(self) -> Status:
        c, s = self.ctx, self.ctx.s
        if s.on_rail != self.lane_idx:
            return self.fail(f"not on the rails of lane {self.lane_idx + 1}")
        left = self.end_x - s.x
        if left <= 0.03:
            c.vehicle.send(Command())
            self.log(f"at the end of lane {self.lane_idx + 1}")
            return Status.SUCCESS
        speed = clamp(0.7 * left, 0.08, c.params.sweep_speed)
        gap = obstacle_ahead(c.vehicle.depth_scan())
        if gap < self.STOP_AT:
            if self.blocked_since is None:
                self.blocked_since = s.t
                self.log(f"something {gap:.2f} m ahead: waiting")
            if s.t - self.blocked_since > self.timeout:
                return self.fail("lane blocked")
            c.vehicle.send(Command())
            return Status.RUNNING
        if self.blocked_since is not None:
            self.log(f"clear after {s.t - self.blocked_since:.0f}s, carrying on")
            self.blocked_since = None
        if gap < self.SLOW_AT:
            speed = min(speed, 0.15)
        c.vehicle.send(Command(vx=speed))
        return Status.RUNNING


class ReverseOut(Once):
    """Back along the rails and off them, onto the concrete; the lane is done when final."""

    def __init__(self, name, ctx, lane: int | None = None, final: bool = True):
        super().__init__(name, ctx)
        self.lane_idx, self.final = lane, final

    def run(self) -> Status:
        c, s = self.ctx, self.ctx.s
        stop_x = headland_x(c.layout, trolley_config(c)) + 0.5
        if s.on_rail is None and s.x <= stop_x:
            c.vehicle.send(Command())
            if self.final and self.lane_idx is not None:
                c.lanes_completed.append(self.lane_idx)
                self.log(f"lane {self.lane_idx + 1} done")
            return Status.SUCCESS
        if s.on_rail is None and s.x > 0.0:
            return self.fail("not on the rails")
        c.vehicle.send(Command(vx=-clamp(0.8 * (s.x - stop_x) + 0.1, 0.1, c.params.sweep_speed)))
        return Status.RUNNING


class FailsafeReverseOut(ReverseOut):
    """ReverseOut as the start of a failsafe return (marks the mission aborted: battery low); does
    nothing if the trolley is already on the concrete."""

    def __init__(self, name, ctx):
        super().__init__(name, ctx, None, final=False)

    def start(self) -> None:
        self.ctx.abort("battery low")

    def run(self) -> Status:
        if self.ctx.s.on_rail is None and self.ctx.s.x < 0:
            return Status.SUCCESS
        return super().run()


def dock_route(ctx: MissionContext) -> list[tuple[float, float]]:
    """Along the headland line, never between the crop rows, then to the dock."""
    tc = trolley_config(ctx)
    return [(headland_x(ctx.layout, tc), ctx.s.y), dock_xy(ctx.layout)]


class ReturnToDock(DriveTo):
    def __init__(self, name, ctx):
        super().__init__(name, ctx, lambda: dock_route(ctx), final_yaw=0.0)


class Dock(Once):
    """At the dock: stop and switch the motors off (stays done)."""

    def start(self) -> None:
        self.log("docking")

    def run(self) -> Status:
        self.ctx.vehicle.send(Command())
        if self.ctx.s.landed:
            self.ctx.vehicle.disarm()
            return Status.SUCCESS
        return Status.RUNNING


# ---- tree -------------------------------------------------------------------------------------


def trolley_lane(ctx: MissionContext, i: int) -> py_trees.behaviour.Behaviour:
    n = f"Lane {i + 1}"
    return py_trees.composites.Sequence(
        n,
        memory=False,
        children=[
            GoToRailStart(f"{n}: GoToRailStart", ctx, i),
            MountRail(f"{n}: MountRail", ctx, i),
            DriveAlongRail(f"{n}: DriveAlongRail", ctx, i),
            ReverseOut(f"{n}: ReverseOut", ctx, i),
        ],
    )


def build_trolley_tree(ctx: MissionContext) -> py_trees.behaviour.Behaviour:
    Seq, Sel = py_trees.composites.Sequence, py_trees.composites.Selector
    guard = Sel(
        "SafetyGuard",
        memory=False,
        children=[
            Seq(
                "AllHealthy",
                memory=False,
                children=[
                    NotAborted("NotAborted", ctx),
                    BatteryAboveReserve("BatteryAboveReserve", ctx),
                    LocalizationOK("LocalizationOK", ctx),
                ],
            ),
            py_trees.decorators.SuccessIsFailure(
                "Failsafe",
                Sel(
                    "FailsafeChoice",
                    memory=False,
                    children=[
                        Seq(
                            "PoseGlitch",
                            memory=False,
                            children=[VOLostBriefly("VOLostBriefly", ctx), Stop("Stop", ctx)],
                        ),
                        Seq(
                            "EmergencyStop",
                            memory=False,
                            children=[MustStopNow("MustStopNow", ctx), StopHere("StopHere", ctx)],
                        ),
                        Seq(
                            "ReturnAndDock",
                            memory=False,
                            children=[
                                MotorsOn("MotorsOn", ctx),
                                FailsafeReverseOut("Failsafe: ReverseOut", ctx),
                                ReturnToDock("Failsafe: ReturnToDock", ctx),
                                Dock("Failsafe: Dock", ctx),
                            ],
                        ),
                    ],
                ),
            ),
        ],
    )
    mission = Seq(
        "Mission",
        memory=False,
        children=[
            Preflight("Preflight", ctx),
            *[trolley_lane(ctx, i) for i in range(ctx.layout.n_lanes)],
            ReturnToDock("ReturnToDock", ctx),
            Dock("Dock", ctx),
        ],
    )
    return Seq("Root", memory=False, children=[guard, mission])
