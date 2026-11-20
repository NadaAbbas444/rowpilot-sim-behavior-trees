"""Run a mission in the simulator and record what every node did.

The event stream (status changes and node log lines) is what a node editor would show live, one
log per node. Pass on_event to get the events as they happen, and on_tick to see every tick's state
(return False from it to stop the run). tree_factory builds a different tree, e.g. one drawn in the
editor; nodes that carry a graph_id have it in their events.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import asdict, dataclass, field

import py_trees
from py_trees.common import Status

from .layout import GreenhouseLayout
from .mission import MissionContext, MissionParams, SweepLane, build_tree
from .sim import SimConfig, SimVehicle
from .trolley import TrolleyConfig, TrolleySim, build_trolley_tree

ROBOTS = ("drone", "trolley")  # the FireFly 2 drone; the pipe-rail trolley (trolley.py)


@dataclass
class RunConfig:
    layout: GreenhouseLayout = field(default_factory=GreenhouseLayout)
    sim: SimConfig = field(default_factory=SimConfig)
    mission: MissionParams = field(default_factory=MissionParams)
    sim_dt: float = 0.02
    tick_hz: float = 10.0
    max_time_s: float = 1800.0
    robot: str = "drone"
    trolley: TrolleyConfig = field(default_factory=TrolleyConfig)


@dataclass
class RunResult:
    outcome: str  # "completed", "aborted: <reason>", "crashed", "timeout", "stopped"
    duration_s: float
    battery_end_pct: float
    lanes_completed: list[int]
    lane_rms_error_m: dict[int, float]  # true distance from lane centre while sweeping
    max_lane_error_m: float
    events: list[dict]
    trace: list[dict]

    def summary(self) -> dict:
        d = asdict(self)
        d.pop("events")
        d.pop("trace")
        return d


def run_mission(
    cfg: RunConfig | None = None,
    on_event: Callable[[dict], None] | None = None,
    *,
    tree_factory: Callable[[MissionContext], py_trees.behaviour.Behaviour] | None = None,
    on_tick: Callable[[dict], bool | None] | None = None,
) -> RunResult:
    cfg = cfg or RunConfig()
    lay = cfg.layout
    if cfg.robot not in ROBOTS:
        raise ValueError(f"robot must be one of {', '.join(ROBOTS)}")
    trolley = cfg.robot == "trolley"
    sim = TrolleySim(lay, cfg.sim, cfg.trolley) if trolley else SimVehicle(lay, cfg.sim)
    tree_factory = tree_factory or (build_trolley_tree if trolley else build_tree)
    events: list[dict] = []

    ids: dict[str, str] = {}  # node name -> graph id (trees built from an editor graph)

    def emit(ev: dict) -> None:
        if ev["node"] in ids:
            ev["id"] = ids[ev["node"]]
        events.append(ev)
        if on_event:
            on_event(ev)

    ctx = MissionContext(
        sim,
        lay,
        cfg.mission,
        dt=1.0 / cfg.tick_hz,
        log_fn=lambda node, msg: emit({"t": round(sim.t, 2), "type": "log", "node": node, "msg": msg}),
    )
    root = tree_factory(ctx)
    ids.update({n.name: n.graph_id for n in root.iterate() if getattr(n, "graph_id", None) is not None})
    last_status: dict[str, Status] = {}
    trace: list[dict] = []
    lane_sq: dict[int, list[float]] = {}
    steps_per_tick = max(1, round(1.0 / (cfg.tick_hz * cfg.sim_dt)))
    outcome = "timeout"

    while sim.t < cfg.max_time_s:
        ctx.pre_tick()
        root.tick_once()
        for node in root.iterate():
            if last_status.get(node.name) != node.status:
                last_status[node.name] = node.status
                emit({"t": round(sim.t, 2), "type": "status", "node": node.name, "status": node.status.value})

        tip = root.tip()
        est = ctx.s
        trace.append(
            {
                "t": round(sim.t, 2),
                "x": sim.x,
                "y": sim.y,
                "z": sim.z,
                "yaw": sim.yaw,
                "est_x": est.x,
                "est_y": est.y,
                "battery": sim.battery,
                "vo_ok": est.vo_healthy,
                "node": tip.name if tip else "",
            }
        )
        if isinstance(tip, SweepLane) and 0 < lay.progress_along_lane(tip.lane_idx, sim.x) < lay.row_length:
            lane_sq.setdefault(tip.lane_idx, []).append((sim.y - lay.lane_center_y(tip.lane_idx)) ** 2)

        if sim.crashed:
            outcome = "crashed"
            if getattr(sim, "crash_reason", None):
                emit({"t": round(sim.t, 2), "type": "log", "node": "sim", "msg": f"crash: {sim.crash_reason}"})
            break
        if root.status != Status.RUNNING and sim.at_rest:
            outcome = (
                "completed"
                if root.status == Status.SUCCESS
                else (f"aborted: {ctx.abort_reason or 'safety check failed before take-off'}")
            )
            break
        if on_tick and on_tick(trace[-1]) is False:
            outcome = "stopped"
            break
        for _ in range(steps_per_tick):
            sim.step(cfg.sim_dt)

    rms = {k: math.sqrt(sum(v) / len(v)) for k, v in sorted(lane_sq.items())}
    max_err = max((math.sqrt(max(v)) for v in lane_sq.values()), default=0.0)
    emit({"t": round(sim.t, 2), "type": "end", "node": root.name, "msg": outcome})
    return RunResult(outcome, round(sim.t, 2), round(sim.battery, 1), ctx.lanes_completed, rms, max_err, events, trace)
