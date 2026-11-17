"""The sweep mission as a behaviour tree (py_trees).

Root (Sequence, re-evaluated every tick)
├── SafetyGuard (Selector)
│   ├── AllHealthy (Sequence): NotAborted, BatteryAboveReserve, VOHealthy
│   └── Failsafe -> reported as FAILURE when it finishes, which ends the run
│       ├── VOGlitch (Sequence): VOLostBriefly, Hover        <- pauses the mission, then resumes
│       ├── EmergencyLand (Sequence): MustLandNow, LandHere  <- battery critical or VO lost too long
│       └── ReturnAndLand (Sequence): Airborne, ReturnHome, Land  <- low battery or a failed step
└── Mission (Sequence)
    ├── Preflight, Takeoff
    ├── Lane 1 (Sequence): GoToEntry, Align, Sweep, Exit
    ├── ...
    └── ReturnHome, Land

Because the guard sits in front of the mission and both are re-ticked every tick, a failing check
interrupts whatever mission node is running. Mission nodes are therefore written to be re-entrant:
each one remembers when it has finished (ctx.done), so after a VO glitch the mission picks up at
the node that was running instead of starting over.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
import py_trees
from py_trees.common import Status

from .control import Command, LaneFollower, altitude_hold, goto_command
from .layout import GreenhouseLayout
from .perception import LaneEstimate, estimate_lane
from .vehicle import Vehicle, VehicleState


@dataclass
class MissionParams:
    altitude: float = 1.5
    sweep_speed: float = 0.8
    transit_speed: float = 1.0
    min_start_battery_pct: float = 40.0
    reserve_battery_pct: float = 25.0  # below this: stop sweeping and fly home
    critical_battery_pct: float = 12.0  # below this: land where you are
    vo_glitch_timeout_s: float = 3.0  # hover this long waiting for VO, then land
    align_timeout_s: float = 20.0
    lane_lost_timeout_s: float = 5.0
    lane_fix_gain: float = 0.05  # how strongly lane perception corrects the VO cross-track drift


@dataclass
class MissionContext:
    vehicle: Vehicle
    layout: GreenhouseLayout
    params: MissionParams = field(default_factory=MissionParams)
    dt: float = 0.1
    log_fn: Callable[[str, str], None] | None = None
    done: set[str] = field(default_factory=set)
    abort_reason: str | None = None
    vo_lost_since: float | None = None
    y_bias: float = 0.0  # lane-centric localization: correction added to the VO y estimate
    lane: LaneEstimate = field(default_factory=LaneEstimate)
    lanes_completed: list[int] = field(default_factory=list)
    s: VehicleState | None = None

    def pre_tick(self) -> None:
        """Read sensors once per tick, so every node in the tick sees the same snapshot."""
        raw = self.vehicle.state()
        self.s = VehicleState(
            raw.t, raw.x, raw.y + self.y_bias, raw.z, raw.yaw, raw.vo_healthy, raw.battery_pct, raw.armed, raw.landed
        )
        if raw.vo_healthy:
            self.vo_lost_since = None
        elif self.vo_lost_since is None:
            self.vo_lost_since = raw.t
        self.lane = estimate_lane(self.vehicle.depth_scan(), self.layout.path_width)
        self._lane_fix()

    def _lane_fix(self) -> None:
        """Lane-centric localization. Inside a lane, the lane centre's map y is known, so the perceived
        offset tells us the drone's true y. Nudge the VO estimate towards it to cancel cross-track drift."""
        s, lay = self.s, self.layout
        if not (s.vo_healthy and self.lane.valid and 0.5 < s.x < lay.row_length - 0.5):
            return
        if abs(math.sin(s.yaw)) > math.sin(math.radians(20)):  # only when pointing along the rows
            return
        lane_y = lay.lane_center_y(lay.nearest_lane(s.y))
        implied_y = lane_y - self.lane.lateral_offset * math.copysign(1.0, math.cos(s.yaw))
        self.y_bias += self.params.lane_fix_gain * (implied_y - s.y)
        self.s = VehicleState(
            s.t,
            s.x,
            s.y + self.params.lane_fix_gain * (implied_y - s.y),
            s.z,
            s.yaw,
            s.vo_healthy,
            s.battery_pct,
            s.armed,
            s.landed,
        )

    def vo_lost_for(self) -> float:
        return 0.0 if self.vo_lost_since is None else self.s.t - self.vo_lost_since

    def abort(self, reason: str) -> None:
        if self.abort_reason is None:
            self.abort_reason = reason

    def log(self, node: str, msg: str) -> None:
        if self.log_fn:
            self.log_fn(node, msg)

    def send(self, cmd: Command) -> None:
        cmd.vz = cmd.vz or altitude_hold(self.s.z, self.params.altitude)
        self.vehicle.send(cmd)


# ---- base classes ----------------------------------------------------------------------------
class Node(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, ctx: MissionContext):
        super().__init__(name)
        self.ctx = ctx

    def log(self, msg: str) -> None:
        self.feedback_message = msg
        self.ctx.log(self.name, msg)


class Condition(Node):
    def check(self) -> bool:
        raise NotImplementedError

    def update(self) -> Status:
        return Status.SUCCESS if self.check() else Status.FAILURE


class Once(Node):
    """A mission step that succeeds once and then stays done, even if the tree is interrupted."""

    def initialise(self) -> None:
        if self.name not in self.ctx.done:
            self.t0 = self.ctx.s.t
            self.start()

    def update(self) -> Status:
        if self.name in self.ctx.done:
            return Status.SUCCESS
        status = self.run()
        if status == Status.SUCCESS:
            self.ctx.done.add(self.name)
            self.log(f"done in {self.ctx.s.t - self.t0:.1f}s")
        return status

    def start(self) -> None:
        pass

    def run(self) -> Status:
        raise NotImplementedError

    def fail(self, reason: str) -> Status:
        self.log(f"failed: {reason}")
        self.ctx.abort(f"{reason} ({self.name})")
        return Status.FAILURE


# ---- conditions ------------------------------------------------------------------------------
class NotAborted(Condition):
    def check(self) -> bool:
        return self.ctx.abort_reason is None


class BatteryAboveReserve(Condition):
    def check(self) -> bool:
        return self.ctx.s.battery_pct > self.ctx.params.reserve_battery_pct


class VOHealthy(Condition):
    def check(self) -> bool:
        return self.ctx.s.vo_healthy


class VOLostBriefly(Condition):
    def check(self) -> bool:
        return not self.ctx.s.vo_healthy and self.ctx.vo_lost_for() < self.ctx.params.vo_glitch_timeout_s


class MustLandNow(Condition):
    def check(self) -> bool:
        c, p = self.ctx, self.ctx.params
        if c.s.battery_pct <= p.critical_battery_pct:
            c.abort("battery critical")
        elif not c.s.vo_healthy and c.vo_lost_for() >= p.vo_glitch_timeout_s:
            c.abort("visual odometry lost")
        return c.abort_reason in ("battery critical", "visual odometry lost")


# ---- actions ---------------------------------------------------------------------------------
class Hover(Node):
    def initialise(self) -> None:
        self.log("holding position")

    def update(self) -> Status:
        self.ctx.send(Command())
        return Status.RUNNING


class Preflight(Once):
    def run(self) -> Status:
        s, p = self.ctx.s, self.ctx.params
        if s.battery_pct < p.min_start_battery_pct:
            return self.fail(f"battery {s.battery_pct:.0f}% below {p.min_start_battery_pct:.0f}%")
        if not s.vo_healthy:
            return self.fail("visual odometry not tracking")
        if not self.ctx.vehicle.arm():
            return self.fail("arming refused")
        self.log(f"armed, battery {s.battery_pct:.0f}%")
        return Status.SUCCESS


class Takeoff(Once):
    def run(self) -> Status:
        target = self.ctx.params.altitude
        self.ctx.send(Command(vz=altitude_hold(self.ctx.s.z, target)))
        return Status.SUCCESS if abs(self.ctx.s.z - target) < 0.1 else Status.RUNNING


class GoTo(Once):
    """Fly through map-frame waypoints using the VO pose. Waypoints are computed when the node starts."""

    def __init__(self, name, ctx, waypoints: Callable[[], list[tuple[float, float]]], final_yaw: float | None = None):
        super().__init__(name, ctx)
        self.waypoints_fn, self.final_yaw = waypoints, final_yaw

    def start(self) -> None:
        if getattr(self, "wps", None):  # interrupted earlier (e.g. a VO glitch): carry on, don't re-plan
            self.leg_start = (self.ctx.s.x, self.ctx.s.y)
            self.log("resuming towards " + ", ".join(f"({x:.1f}, {y:.1f})" for x, y in self.wps))
            return
        self.wps = list(self.waypoints_fn())
        self.leg_start = (self.ctx.s.x, self.ctx.s.y)
        self.log("waypoints " + ", ".join(f"({x:.1f}, {y:.1f})" for x, y in self.wps))

    def run(self) -> Status:
        s = self.ctx.s
        while self.wps:
            last = len(self.wps) == 1
            cmd, arrived = goto_command(
                s.x,
                s.y,
                s.yaw,
                *self.wps[0],
                self.final_yaw if last else None,
                start=self.leg_start,
                v_max=self.ctx.params.transit_speed,
            )
            if not arrived:
                self.ctx.send(cmd)
                return Status.RUNNING
            self.leg_start = self.wps.pop(0)
        self.ctx.send(Command())
        return Status.SUCCESS


class AlignToLane(Once):
    """Creep into the lane until perception says the drone is centred and pointing down it."""

    def __init__(self, name, ctx, lane: int):
        super().__init__(name, ctx)
        self.lane_idx, self.follower = lane, LaneFollower()

    def start(self) -> None:
        self.follower.reset()
        self.good_since: float | None = None

    def run(self) -> Status:
        c, est = self.ctx, self.ctx.lane
        if c.s.t - self.t0 > c.params.align_timeout_s:
            return self.fail("could not align")
        if not est.valid:
            c.send(Command(vx=0.15))  # creep forward until the crop faces come into view
            return Status.RUNNING
        c.send(self.follower.step(est, 0.15, c.dt))
        if abs(est.lateral_offset) < 0.08 and abs(est.heading_error) < math.radians(4):
            self.good_since = self.good_since if self.good_since is not None else c.s.t
            if c.s.t - self.good_since >= 1.0:
                self.log(
                    f"centred: offset {est.lateral_offset:+.2f} m, heading {math.degrees(est.heading_error):+.1f} deg"
                )
                return Status.SUCCESS
        else:
            self.good_since = None
        return Status.RUNNING


class SweepLane(Once):
    """Fly the length of the lane on the perceived centre line; hold it on VO if the faces drop out."""

    def __init__(self, name, ctx, lane: int):
        super().__init__(name, ctx)
        self.lane_idx, self.follower = lane, LaneFollower()

    def start(self) -> None:
        self.follower.reset()
        self.lost_since: float | None = None
        if not hasattr(self, "offsets"):  # keep the statistics across interruptions
            self.offsets: list[float] = []

    def run(self) -> Status:
        c, est, lay = self.ctx, self.ctx.lane, self.ctx.layout
        progress = lay.progress_along_lane(self.lane_idx, c.s.x)
        if progress >= lay.row_length - 0.2:
            c.lanes_completed.append(self.lane_idx)
            if self.offsets:
                rms = float(np.sqrt(np.mean(np.square(self.offsets))))
                self.log(f"lane {self.lane_idx + 1} swept, perceived RMS offset {rms:.3f} m")
            return Status.SUCCESS
        if est.valid:
            self.lost_since = None
            self.offsets.append(est.lateral_offset)
            c.send(self.follower.step(est, c.params.sweep_speed, c.dt))
            return Status.RUNNING
        # Crop faces not visible: hold the lane centre by VO. Near the row end that is expected (the
        # faces run out of view); mid-row it means something is wrong, so slow down and give up after a while.
        near_end = progress > lay.row_length - 3.0
        if not near_end:
            self.lost_since = self.lost_since if self.lost_since is not None else c.s.t
            if c.s.t - self.lost_since > c.params.lane_lost_timeout_s:
                return self.fail("lane lost")
        d, lane_y = lay.lane_direction(self.lane_idx), lay.lane_center_y(self.lane_idx)
        cmd, _ = goto_command(
            c.s.x,
            c.s.y,
            c.s.yaw,
            c.s.x + 2.0 * d,
            lane_y,
            None,
            start=(c.s.x - d, lane_y),
            v_max=c.params.sweep_speed if near_end else 0.3,
        )
        c.send(cmd)
        return Status.RUNNING


def land_step(ctx: MissionContext) -> bool:
    """Hand the touchdown to the flight controller (it detects ground contact and disarms), then
    make sure the vehicle is disarmed. Returns True once landed."""
    if ctx.s.landed:
        ctx.vehicle.disarm()
        return True
    ctx.vehicle.land()
    return False


class Land(Node):
    def initialise(self) -> None:
        self.log("landing")

    def update(self) -> Status:
        if land_step(self.ctx):
            self.log("landed and disarmed")
            return Status.SUCCESS
        return Status.RUNNING


class MissionLand(Once):
    def start(self) -> None:
        self.log("landing")

    def run(self) -> Status:
        return Status.SUCCESS if land_step(self.ctx) else Status.RUNNING


class Airborne(Condition):
    def check(self) -> bool:
        return not self.ctx.s.landed


def home_route(ctx: MissionContext) -> list[tuple[float, float]]:
    """Back to the take-off spot without cutting through crop rows: leave the lane along its axis
    on the near headland side, then cross the headland."""
    lay, s = ctx.layout, ctx.s
    hx, hy = lay.home
    if s.x > 0:  # inside the rows or in the far headland
        lane_y = lay.lane_center_y(lay.nearest_lane(s.y))
        return [(min(s.x, lay.row_length + lay.headland), lane_y), (-lay.headland, lane_y), (hx, hy)]
    return [(hx, hy)]


class ReturnHome(GoTo):
    def __init__(self, name, ctx):
        super().__init__(name, ctx, lambda: home_route(ctx), final_yaw=None)


class FailsafeReturnHome(ReturnHome):
    """Same route as ReturnHome. Being a Once node, it keeps its progress if a VO glitch interrupts it
    and re-plans from wherever the drone is when it resumes."""

    def start(self) -> None:
        self.ctx.abort("battery low")
        super().start()


# ---- tree ------------------------------------------------------------------------------------
def build_tree(ctx: MissionContext) -> py_trees.behaviour.Behaviour:
    Seq, Sel = py_trees.composites.Sequence, py_trees.composites.Selector
    lay = ctx.layout

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
                    VOHealthy("VOHealthy", ctx),
                ],
            ),
            py_trees.decorators.SuccessIsFailure(
                "Failsafe",
                Sel(
                    "FailsafeChoice",
                    memory=False,
                    children=[
                        Seq(
                            "VOGlitch",
                            memory=False,
                            children=[VOLostBriefly("VOLostBriefly", ctx), Hover("Hover", ctx)],
                        ),
                        Seq(
                            "EmergencyLand",
                            memory=False,
                            children=[MustLandNow("MustLandNow", ctx), Land("LandHere", ctx)],
                        ),
                        Seq(
                            "ReturnAndLand",
                            memory=False,
                            children=[
                                Airborne("Airborne", ctx),
                                FailsafeReturnHome("Failsafe: ReturnHome", ctx),
                                Land("Failsafe: Land", ctx),
                            ],
                        ),
                    ],
                ),
            ),
        ],
    )

    lanes = []
    for i in range(lay.n_lanes):
        n = f"Lane {i + 1}"
        prev_exit = lay.lane_exit(i - 1) if i > 0 else None
        entry = lay.lane_entry(i)

        def entry_route(prev=prev_exit, entry=entry):
            return [entry] if prev is None else [prev, entry]

        lanes.append(
            Seq(
                n,
                memory=False,
                children=[
                    GoTo(f"{n}: GoToEntry", ctx, entry_route, final_yaw=lay.lane_yaw(i)),
                    AlignToLane(f"{n}: Align", ctx, i),
                    SweepLane(f"{n}: Sweep", ctx, i),
                    GoTo(f"{n}: Exit", ctx, lambda i=i: [lay.lane_exit(i)], final_yaw=None),
                ],
            )
        )

    mission = Seq(
        "Mission",
        memory=False,
        children=[
            Preflight("Preflight", ctx),
            Takeoff("Takeoff", ctx),
            *lanes,
            ReturnHome("ReturnHome", ctx),
            MissionLand("Land", ctx),
        ],
    )
    return Seq("Root", memory=False, children=[guard, mission])


def tree_to_dict(root: py_trees.behaviour.Behaviour) -> dict:
    """Nodes and edges, for drawing the tree in a node editor."""
    nodes, edges = [], []
    for node in root.iterate():
        nodes.append({"id": str(node.id), "name": node.name, "type": type(node).__name__})
        edges += [{"source": str(node.id), "target": str(ch.id)} for ch in node.children]
    return {"nodes": nodes, "edges": edges}
