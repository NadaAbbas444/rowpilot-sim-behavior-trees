"""Behaviour trees as graphs: the node types the editor offers, and graph <-> py_trees.

A graph is plain JSON, the format the editor saves and the server runs:

    {"version": 1,
     "nodes": [{"id": "n1", "type": "Sequence", "name": "Root", "params": {}, "position": {"x": 0, "y": 0}}, ...],
     "edges": [{"source": "n1", "target": "n2"}, ...]}

Each edge goes from a parent to a child. A parent ticks its children in the order they are drawn,
top to bottom (by position.y; nodes without a position keep the order of their edges). Node names
must be unique: mission steps remember by name that they are done (see mission.Once), and the run's
events name the node they come from.

Lanes are numbered from 1 here, as in the node names ("Lane 1"); the mission code counts from 0.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import py_trees
from py_trees.common import ParallelPolicy, Status

from ..control import Command
from ..layout import GreenhouseLayout
from ..mission import (
    Airborne,
    AlignToLane,
    BatteryAboveReserve,
    Condition,
    FailsafeReturnHome,
    GoTo,
    Hover,
    Land,
    MissionContext,
    MissionLand,
    MustLandNow,
    NotAborted,
    Once,
    Preflight,
    ReturnHome,
    SweepLane,
    Takeoff,
    VOHealthy,
    VOLostBriefly,
)
from ..trolley import (
    Dock,
    DriveAlongRail,
    FailsafeReverseOut,
    GoToRailStart,
    LocalizationOK,
    MotorsOn,
    MountRail,
    MustStopNow,
    OnRail,
    ReturnToDock,
    ReverseOut,
    Stop,
    StopHere,
)

GRAPH_VERSION = 1

COMPOSITE, DECORATOR, CONDITION, ACTION = "composite", "decorator", "condition", "action"
ROBOTS = ("drone", "trolley")
DRONE, TROLLEY = ("drone",), ("trolley",)
ROBOT_NAMES = {"drone": "the drone", "trolley": "the pipe-rail trolley"}


def robot_of(ctx: MissionContext) -> str:
    """The robot a mission context drives: its vehicle says (TrolleySim and TrolleyGazebo: "trolley")."""
    return getattr(ctx.vehicle, "robot", "drone")


@dataclass(frozen=True)
class Param:
    name: str
    kind: str  # "int", "float", "bool", "choice", "lane"
    default: Any
    help: str = ""
    choices: tuple[str, ...] = ()
    min: float | None = None
    max: float | None = None

    def coerce(self, value: Any, n_lanes: int) -> Any:
        """Check one parameter value; raise ValueError with a readable reason."""
        if self.kind == "bool":
            if not isinstance(value, bool):
                raise ValueError("must be true or false")
            return value
        if self.kind == "choice":
            if value not in self.choices:
                raise ValueError(f"must be one of {', '.join(self.choices)}")
            return value
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ValueError("must be a number")
        if self.kind in ("int", "lane"):
            if value != int(value):
                raise ValueError("must be a whole number")
            value = int(value)
        value = float(value) if self.kind == "float" else value
        lo, hi = self.min, self.max
        if self.kind == "lane" and not 1 <= value <= n_lanes:
            raise ValueError(f"{value} does not exist: the greenhouse has {n_lanes} lane{'s' * (n_lanes != 1)}")
        if lo is not None and value < lo:
            raise ValueError(f"must be at least {lo:g}")
        if hi is not None and value > hi:
            raise ValueError(f"must be at most {hi:g}")
        return value

    def describe(self) -> dict:
        d = {"name": self.name, "kind": self.kind, "default": self.default, "help": self.help}
        if self.choices:
            d["choices"] = list(self.choices)
        if self.min is not None:
            d["min"] = self.min
        if self.max is not None:
            d["max"] = self.max
        return d


Builder = Callable[[str, dict, MissionContext, list], py_trees.behaviour.Behaviour]


@dataclass(frozen=True)
class NodeType:
    type: str
    category: str
    help: str
    build: Builder
    params: tuple[Param, ...] = ()
    group: str = ""
    robots: tuple[str, ...] = ROBOTS  # the robots it makes sense for

    @property
    def children(self) -> tuple[int, int | None]:
        """(min, max) number of children."""
        return {COMPOSITE: (1, None), DECORATOR: (1, 1)}.get(self.category, (0, 0))

    def describe(self) -> dict:
        lo, hi = self.children
        return {
            "type": self.type,
            "category": self.category,
            "group": self.group or self.category,
            "help": self.help,
            "params": [p.describe() for p in self.params],
            "min_children": lo,
            "max_children": hi,
            "robots": list(self.robots),
        }


# ---- extra nodes, only offered by the editor ----------------------------------------------------
class BatteryAbove(Condition):
    def __init__(self, name, ctx, pct: float):
        super().__init__(name, ctx)
        self.pct = pct

    def check(self) -> bool:
        return self.ctx.s.battery_pct > self.pct


class Wait(Once):
    """Hold position for a number of (simulated) seconds."""

    def __init__(self, name, ctx, seconds: float):
        super().__init__(name, ctx)
        self.seconds = seconds

    def start(self) -> None:
        self.log(f"holding for {self.seconds:g} s")

    def run(self) -> Status:
        self.ctx.send(Command())
        return Status.SUCCESS if self.ctx.s.t - self.t0 >= self.seconds - 1e-6 else Status.RUNNING


class SimTimeout(py_trees.decorators.Decorator):
    """FAILURE if the child is still running after a number of simulated seconds (py_trees' own
    Timeout counts wall-clock time, which is meaningless when the simulation runs faster)."""

    def __init__(self, name, child, ctx: MissionContext, seconds: float):
        super().__init__(name=name, child=child)
        self.ctx, self.seconds = ctx, seconds

    def initialise(self) -> None:
        self.t0 = self.ctx.s.t

    def update(self) -> Status:
        if self.decorated.status == Status.RUNNING and self.ctx.s.t - self.t0 > self.seconds:
            self.feedback_message = f"timed out after {self.seconds:g} s"
            self.ctx.log(self.name, self.feedback_message)
            return Status.FAILURE
        return self.decorated.status


# ---- builders -----------------------------------------------------------------------------------
def _seq(name, p, ctx, ch):
    return py_trees.composites.Sequence(name, memory=p["memory"], children=ch)


def _sel(name, p, ctx, ch):
    return py_trees.composites.Selector(name, memory=p["memory"], children=ch)


def _par(name, p, ctx, ch):
    policy = ParallelPolicy.SuccessOnAll(synchronise=False) if p["policy"] == "all" else ParallelPolicy.SuccessOnOne()
    return py_trees.composites.Parallel(name, policy=policy, children=ch)


def _deco(cls):
    return lambda name, p, ctx, ch: cls(name=name, child=ch[0])


def _leaf(cls):
    return lambda name, p, ctx, ch: cls(name, ctx)


def _goto_entry(name, p, ctx, ch):
    lay, lane = ctx.layout, p["lane"] - 1
    entry, frm = lay.lane_entry(lane), p["from_lane"] - 1
    prev = lay.lane_exit(frm) if frm >= 0 else None

    def route():
        return [entry] if prev is None else [prev, entry]

    return GoTo(name, ctx, route, final_yaw=lay.lane_yaw(lane))


def _sweep(name, p, ctx, ch):
    lay, lane = ctx.layout, p["lane"] - 1
    direction = {"auto": None, "+x": 1, "-x": -1}[p["direction"]]
    end_x = {"auto": None, "turn point": lay.turn_x(), "row start": 0.0, "row end": lay.row_length}[p["end"]]
    return SweepLane(name, ctx, lane, direction, end_x=end_x, stop_at_end=p["stop_at_end"], final=p["final"])


def _turn(name, p, ctx, ch):
    turn = (ctx.layout.turn_x(), ctx.layout.lane_center_y(p["lane"] - 1))
    return GoTo(name, ctx, lambda: [turn], final_yaw=math.pi)


def _exit(name, p, ctx, ch):
    target = ctx.layout.lane_exit(p["lane"] - 1)
    return GoTo(name, ctx, lambda: [target], final_yaw=None)


def _goto_point(name, p, ctx, ch):
    yaw = math.radians(p["yaw_deg"]) if p["set_yaw"] else None
    return GoTo(name, ctx, lambda: [(p["x"], p["y"])], final_yaw=yaw)


MEMORY = Param(
    "memory",
    "bool",
    False,
    "off: start from the first child every tick (a failing check interrupts whatever runs after it); "
    "on: resume at the child that was running",
)
LANE = Param("lane", "lane", 1, "lane number, 1 = next to home", min=1)

NODE_TYPES: dict[str, NodeType] = {
    t.type: t
    for t in [
        # composites
        NodeType("Sequence", COMPOSITE, "Runs its children in order; fails as soon as one fails.", _seq, (MEMORY,)),
        NodeType(
            "Selector",
            COMPOSITE,
            "Tries its children in order until one succeeds or is running (a fallback).",
            _sel,
            (MEMORY,),
        ),
        NodeType(
            "Parallel",
            COMPOSITE,
            "Ticks all children every tick.",
            _par,
            (Param("policy", "choice", "all", "succeed when all children / one child succeed", ("all", "one")),),
        ),
        # decorators
        NodeType("Inverter", DECORATOR, "Swaps SUCCESS and FAILURE.", _deco(py_trees.decorators.Inverter)),
        NodeType(
            "SuccessIsFailure",
            DECORATOR,
            "Reports a child's SUCCESS as FAILURE (a failsafe that finishes ends the mission).",
            _deco(py_trees.decorators.SuccessIsFailure),
        ),
        NodeType(
            "FailureIsSuccess",
            DECORATOR,
            "Reports a child's FAILURE as SUCCESS (an optional step).",
            _deco(py_trees.decorators.FailureIsSuccess),
        ),
        NodeType(
            "Timeout",
            DECORATOR,
            "Fails the child if it runs longer than this (simulated time).",
            lambda name, p, ctx, ch: SimTimeout(name, ch[0], ctx, p["seconds"]),
            (Param("seconds", "float", 30.0, "time limit (s)", min=0.1),),
        ),
        # conditions
        NodeType("NotAborted", CONDITION, "No step has failed and no failsafe has started.", _leaf(NotAborted)),
        NodeType(
            "BatteryAboveReserve",
            CONDITION,
            "Battery above the reserve (mission setting, 25 %): enough to fly home.",
            _leaf(BatteryAboveReserve),
        ),
        NodeType(
            "BatteryAbove",
            CONDITION,
            "Battery above a given level.",
            lambda name, p, ctx, ch: BatteryAbove(name, ctx, p["pct"]),
            (Param("pct", "float", 50.0, "battery level (%)", min=0, max=100),),
        ),
        NodeType("VOHealthy", CONDITION, "Visual odometry is tracking.", _leaf(VOHealthy)),
        NodeType(
            "VOLostBriefly",
            CONDITION,
            "Visual odometry lost for less than the glitch timeout (3 s): worth waiting for.",
            _leaf(VOLostBriefly),
        ),
        NodeType(
            "MustLandNow",
            CONDITION,
            "Battery critical, or visual odometry lost too long: land where you are.",
            _leaf(MustLandNow),
        ),
        NodeType("Airborne", CONDITION, "The drone is in the air.", _leaf(Airborne)),
        # actions: flight
        NodeType(
            "Preflight", ACTION, "Checks battery and visual odometry, then arms.", _leaf(Preflight), group="flight"
        ),
        NodeType("Takeoff", ACTION, "Climbs to the mission altitude.", _leaf(Takeoff), group="flight"),
        NodeType("Hover", ACTION, "Holds position (always RUNNING).", _leaf(Hover), group="flight"),
        NodeType(
            "Wait",
            ACTION,
            "Holds position for a while, then succeeds.",
            lambda name, p, ctx, ch: Wait(name, ctx, p["seconds"]),
            (Param("seconds", "float", 2.0, "how long (s)", min=0),),
            group="flight",
        ),
        NodeType(
            "GoToPoint",
            ACTION,
            "Flies to a point in the map frame (x along the rows, y across, from the start of lane 1).",
            _goto_point,
            (
                Param("x", "float", -2.5, "m", min=-50, max=200),
                Param("y", "float", 0.0, "m", min=-50, max=200),
                Param("set_yaw", "bool", False, "turn to a heading on arrival"),
                Param("yaw_deg", "float", 0.0, "heading on arrival (deg, 0 = along the rows)", min=-180, max=180),
            ),
            group="flight",
        ),
        NodeType("ReturnHome", ACTION, "Flies home without crossing the crop rows.", _leaf(ReturnHome), group="flight"),
        NodeType(
            "FailsafeReturnHome",
            ACTION,
            "ReturnHome as a failsafe: marks the mission aborted (battery low).",
            _leaf(FailsafeReturnHome),
            group="flight",
        ),
        NodeType("Land", ACTION, "Lands where it is and disarms.", _leaf(Land), group="flight"),
        NodeType(
            "MissionLand",
            ACTION,
            "Lands at the end of the mission; stays done once landed.",
            _leaf(MissionLand),
            group="flight",
        ),
        # actions: lanes
        NodeType(
            "GoToLaneEntry",
            ACTION,
            "Flies to the start of a lane and turns to face down it.",
            _goto_entry,
            (LANE, Param("from_lane", "int", 0, "first fly out through this lane's exit (0: straight there)", min=0)),
            group="lane",
        ),
        NodeType(
            "AlignToLane",
            ACTION,
            "Creeps into the lane until the depth camera says it is centred and straight.",
            lambda name, p, ctx, ch: AlignToLane(name, ctx, p["lane"] - 1),
            (LANE,),
            group="lane",
        ),
        NodeType(
            "SweepLane",
            ACTION,
            "Flies along the lane on the centre line the depth camera sees.",
            _sweep,
            (
                LANE,
                Param("direction", "choice", "auto", "+x: into the greenhouse, -x: out", ("auto", "+x", "-x")),
                Param(
                    "end",
                    "choice",
                    "auto",
                    "where the sweep ends (auto: end of the rows in the sweep direction)",
                    ("auto", "turn point", "row start", "row end"),
                ),
                Param("stop_at_end", "bool", False, "slow down to stop at the end (a dead end ahead)"),
                Param("final", "bool", True, "the lane counts as swept when this sweep ends"),
            ),
            group="lane",
        ),
        NodeType(
            "TurnAround",
            ACTION,
            "At the turning point of a dead-end lane: turn to face back out.",
            _turn,
            (LANE,),
            group="lane",
        ),
        NodeType("ExitLane", ACTION, "Flies out of the lane to its exit.", _exit, (LANE,), group="lane"),
    ]
}


# The drone's flying and lane-sweeping nodes; the rest so far suit both robots.
_DRONE_ONLY = {
    "MustLandNow", "Airborne", "Takeoff", "Hover", "GoToPoint", "ReturnHome", "FailsafeReturnHome", "Land",
    "MissionLand", "GoToLaneEntry", "AlignToLane", "SweepLane", "TurnAround", "ExitLane",
}  # fmt: skip
NODE_TYPES = {k: dataclasses.replace(t, robots=DRONE) if k in _DRONE_ONLY else t for k, t in NODE_TYPES.items()}

# ---- the pipe-rail trolley's nodes (trolley.py) ----------------------------------------------------
NODE_TYPES.update(
    {
        t.type: t
        for t in [
            NodeType(
                "LocalizationOK", CONDITION, "Visual odometry is tracking, or the trolley is on the rails.",
                _leaf(LocalizationOK), robots=TROLLEY,
            ),
            NodeType("OnRail", CONDITION, "The trolley is on a lane's rails (rail contact).", _leaf(OnRail),
                     robots=TROLLEY),
            NodeType("MotorsOn", CONDITION, "The motors are on (not docked or stopped for good).", _leaf(MotorsOn),
                     robots=TROLLEY),
            NodeType(
                "MustStopNow", CONDITION,
                "Battery critical, or visual odometry lost too long on the concrete: stop where it is.",
                _leaf(MustStopNow), robots=TROLLEY,
            ),
            NodeType("Stop", ACTION, "Stands still (always RUNNING).", _leaf(Stop), group="drive", robots=TROLLEY),
            NodeType("StopHere", ACTION, "Stops where it is and switches the motors off.", _leaf(StopHere),
                     group="drive", robots=TROLLEY),
            NodeType("ReturnToDock", ACTION, "Drives along the headland to the dock.", _leaf(ReturnToDock),
                     group="drive", robots=TROLLEY),
            NodeType("Dock", ACTION, "At the dock: stops and switches the motors off; stays done.", _leaf(Dock),
                     group="drive", robots=TROLLEY),
            NodeType(
                "GoToRailStart", ACTION, "Drives along the headland to a lane's rail start, lined up to mount.",
                lambda name, p, ctx, ch: GoToRailStart(name, ctx, p["lane"] - 1), (LANE,), group="rail",
                robots=TROLLEY,
            ),
            NodeType(
                "MountRail", ACTION,
                "Drives onto the lane's rails; backs up and tries again if not lined up (3 tries).",
                lambda name, p, ctx, ch: MountRail(name, ctx, p["lane"] - 1), (LANE,), group="rail", robots=TROLLEY,
            ),
            NodeType(
                "DriveAlongRail", ACTION,
                "Along the rails to the far end; stops for anything in the way and waits for it to go.",
                lambda name, p, ctx, ch: DriveAlongRail(name, ctx, p["lane"] - 1, p["obstacle_timeout_s"]),
                (LANE, Param("obstacle_timeout_s", "float", 60.0, "give up waiting after (s)", min=1)),
                group="rail", robots=TROLLEY,
            ),
            NodeType(
                "ReverseOut", ACTION, "Back along the rails and off them, onto the concrete.",
                lambda name, p, ctx, ch: ReverseOut(name, ctx, p["lane"] - 1, final=p["final"]),
                (LANE, Param("final", "bool", True, "the lane counts as done when out")),
                group="rail", robots=TROLLEY,
            ),
            NodeType(
                "FailsafeReverseOut", ACTION,
                "ReverseOut as a failsafe: marks the mission aborted (battery low); nothing to do off the rails.",
                _leaf(FailsafeReverseOut), group="rail", robots=TROLLEY,
            ),
        ]
    }
)  # fmt: skip


def node_types() -> list[dict]:
    return [t.describe() for t in NODE_TYPES.values()]


# ---- graph -> tree ------------------------------------------------------------------------------
class GraphError(ValueError):
    def __init__(self, errors: list[dict]):
        super().__init__("; ".join(e["message"] for e in errors))
        self.errors = errors


@dataclass
class _Checked:
    root: str
    nodes: dict[str, dict]
    children: dict[str, list[str]]
    params: dict[str, dict] = field(default_factory=dict)


def _check(graph: dict, n_lanes: int, robot: str = "drone") -> _Checked:
    errors: list[dict] = []

    def err(node: str | None, msg: str) -> None:
        errors.append({"node": node, "message": msg})

    if graph.get("version", GRAPH_VERSION) != GRAPH_VERSION:
        raise GraphError([{"node": None, "message": f"unsupported graph version {graph.get('version')}"}])
    nodes: dict[str, dict] = {}
    for n in graph.get("nodes", []):
        nid = str(n.get("id", ""))
        if not nid or nid in nodes:
            err(nid or None, f"node id {nid!r} is missing or used twice")
            continue
        nodes[nid] = n
    if not nodes:
        raise GraphError([{"node": None, "message": "the tree is empty"}])

    names: dict[str, str] = {}
    params: dict[str, dict] = {}
    for nid, n in nodes.items():
        name = str(n.get("name", "")).strip()
        if not name:
            err(nid, "node has no name")
        elif name in names:
            err(nid, f"name {name!r} is used twice: mission steps remember by name that they are done")
        else:
            names[name] = nid
        t = NODE_TYPES.get(n.get("type"))
        if t is None:
            err(nid, f"unknown node type {n.get('type')!r}")
            continue
        if robot not in t.robots:
            err(
                nid,
                f"{name}: {t.type} is for {' and '.join(ROBOT_NAMES[r] for r in t.robots)}, not {ROBOT_NAMES[robot]}",
            )
        given = dict(n.get("params") or {})
        p = {}
        for spec in t.params:
            try:
                p[spec.name] = spec.coerce(given.pop(spec.name, spec.default), n_lanes)
            except ValueError as e:
                err(nid, f"{name}: {spec.name} {e}")
        for extra in given:
            err(nid, f"{name}: {t.type} has no parameter {extra!r}")
        params[nid] = p

    children: dict[str, list[tuple[float, int, str]]] = {nid: [] for nid in nodes}
    parent: dict[str, str] = {}
    for i, e in enumerate(graph.get("edges", [])):
        s, t = str(e.get("source")), str(e.get("target"))
        if s not in nodes or t not in nodes:
            err(None, f"edge {s} -> {t} connects a node that does not exist")
            continue
        if s == t:
            err(s, "a node cannot be its own child")
            continue
        if t in parent:
            err(t, f"{nodes[t].get('name')} has two parents")
            continue
        parent[t] = s
        y = (nodes[t].get("position") or {}).get("y")
        children[s].append((float(y) if y is not None else math.inf, i, t))
    ordered = {nid: [c for _, _, c in sorted(ch)] for nid, ch in children.items()}

    roots = [nid for nid in nodes if nid not in parent]
    if not roots:
        err(None, "there is no root: every node has a parent (a cycle)")
    elif len(roots) > 1:
        # The root is the parentless node with the biggest subtree; the others are loose.
        def size(nid: str) -> int:
            return 1 + sum(size(c) for c in ordered[nid])

        main = max(roots, key=size)
        for r in roots:
            if r != main:
                err(r, f"{nodes[r].get('name')} is not connected to the tree")
        roots = [main]
    root = roots[0] if roots else ""

    if root:  # anything not reachable from the root sits in a cycle
        seen, todo = set(), [root]
        while todo:
            nid = todo.pop()
            seen.add(nid)
            todo += [c for c in ordered[nid] if c not in seen]
        for nid in nodes:
            if nid not in seen and nid in parent:
                err(nid, f"{nodes[nid].get('name')} is part of a cycle")

    for nid, n in nodes.items():
        t = NODE_TYPES.get(n.get("type"))
        if t is None:
            continue
        lo, hi = t.children
        k = len(ordered[nid])
        if k < lo:
            err(nid, f"{n.get('name')}: a {t.category} needs {'a child' if lo == 1 else f'{lo} children'}")
        if hi is not None and k > hi:
            what = "cannot have children" if hi == 0 else f"takes only {hi} child"
            err(nid, f"{n.get('name')}: a {t.category} {what}")

    if errors:
        raise GraphError(errors)
    return _Checked(root, nodes, ordered, params)


def validate(graph: dict, layout: GreenhouseLayout, robot: str = "drone") -> list[dict]:
    """Problems that would stop the graph from running, as [{"node": id or None, "message": ...}]."""
    try:
        _check(graph, layout.n_lanes, robot)
    except GraphError as e:
        return e.errors
    return []


def build_from_graph(graph: dict, ctx: MissionContext) -> py_trees.behaviour.Behaviour:
    """The py_trees tree for a graph. Every behaviour gets graph_id, the id of its graph node."""
    g = _check(graph, ctx.layout.n_lanes, robot_of(ctx))

    def make(nid: str) -> py_trees.behaviour.Behaviour:
        n = g.nodes[nid]
        kids = [make(c) for c in g.children[nid]]
        b = NODE_TYPES[n["type"]].build(str(n["name"]).strip(), g.params[nid], ctx, kids)
        b.graph_id = nid
        return b

    return make(g.root)


# ---- the mission tree as a graph ------------------------------------------------------------------
def _add_lane(add, k: int, closed_end: bool) -> str:
    """Lane k's subtree (k from 1), built with default_graph's add(); returns the lane's id. A dead-end
    lane: in to the turning point, turn round, back out; an open lane: one sweep through."""
    n = f"Lane {k}"
    steps = [
        add("GoToLaneEntry", f"{n}: GoToEntry", lane=k, from_lane=k - 1),
        add("AlignToLane", f"{n}: Align", lane=k),
    ]
    if closed_end:
        steps += [
            add("SweepLane", f"{n}: Sweep in", lane=k, direction="+x", end="turn point", stop_at_end=True, final=False),
            add("TurnAround", f"{n}: TurnAround", lane=k),
            add("SweepLane", f"{n}: Sweep out", lane=k, direction="-x", end="row start"),
        ]
    else:
        steps.append(add("SweepLane", f"{n}: Sweep", lane=k))
    steps.append(add("ExitLane", f"{n}: Exit", lane=k))
    return add("Sequence", n, steps)


def _add_trolley_lane(add, k: int) -> str:
    """The trolley's lane k (from 1): to the rail start, onto the rails, along them, back out."""
    n = f"Lane {k}"
    return add(
        "Sequence",
        n,
        [
            add("GoToRailStart", f"{n}: GoToRailStart", lane=k),
            add("MountRail", f"{n}: MountRail", lane=k),
            add("DriveAlongRail", f"{n}: DriveAlongRail", lane=k),
            add("ReverseOut", f"{n}: ReverseOut", lane=k),
        ],
    )


def lane_graph(lane: int, closed_end: bool = True, robot: str = "drone") -> dict:
    """One lane's subtree as a graph, as in the mission tree: for the editor's Add lane.
    "root" is the lane's Sequence; ids are local (the editor renumbers them)."""
    nodes: list[dict] = []
    edges: list[dict] = []

    def add(type_: str, name: str, children: list[str] = (), **params) -> str:
        nid = f"n{len(nodes) + 1}"
        nodes.append({"id": nid, "type": type_, "name": name, "params": params})
        edges.extend({"source": nid, "target": c} for c in children)
        return nid

    root = _add_trolley_lane(add, lane) if robot == "trolley" else _add_lane(add, lane, closed_end)
    graph = {"version": GRAPH_VERSION, "nodes": nodes, "edges": edges, "root": root}
    tidy_layout(graph)
    return graph


def default_graph(layout: GreenhouseLayout, robot: str = "drone") -> dict:
    """mission.build_tree (the drone) or trolley.build_trolley_tree as a graph: the starting point in the
    editor. Building it gives the same tree (tests/test_editor.py runs both and compares)."""
    nodes: list[dict] = []
    edges: list[dict] = []

    def add(type_: str, name: str, children: list[str] = (), **params) -> str:
        nid = f"n{len(nodes) + 1}"
        nodes.append({"id": nid, "type": type_, "name": name, "params": params})
        edges.extend({"source": nid, "target": c} for c in children)
        return nid

    if robot == "trolley":
        return _trolley_graph(add, nodes, edges, layout)

    # Children are added before their parent, so ids grow bottom-up; tidy_layout puts them in order.
    healthy = add(
        "Sequence",
        "AllHealthy",
        [
            add("NotAborted", "NotAborted"),
            add("BatteryAboveReserve", "BatteryAboveReserve"),
            add("VOHealthy", "VOHealthy"),
        ],
    )
    glitch = add("Sequence", "VOGlitch", [add("VOLostBriefly", "VOLostBriefly"), add("Hover", "Hover")])
    emergency = add("Sequence", "EmergencyLand", [add("MustLandNow", "MustLandNow"), add("Land", "LandHere")])
    rtl = add(
        "Sequence",
        "ReturnAndLand",
        [
            add("Airborne", "Airborne"),
            add("FailsafeReturnHome", "Failsafe: ReturnHome"),
            add("Land", "Failsafe: Land"),
        ],
    )
    choice = add("Selector", "FailsafeChoice", [glitch, emergency, rtl])
    guard = add("Selector", "SafetyGuard", [healthy, add("SuccessIsFailure", "Failsafe", [choice])])

    lanes = [_add_lane(add, k, layout.closed_end) for k in range(1, layout.n_lanes + 1)]

    mission = add(
        "Sequence",
        "Mission",
        [
            add("Preflight", "Preflight"),
            add("Takeoff", "Takeoff"),
            *lanes,
            add("ReturnHome", "ReturnHome"),
            add("MissionLand", "Land"),
        ],
    )
    add("Sequence", "Root", [guard, mission])
    graph = {"version": GRAPH_VERSION, "nodes": nodes, "edges": edges}
    tidy_layout(graph)
    return graph


def _trolley_graph(add, nodes: list, edges: list, layout: GreenhouseLayout) -> dict:
    healthy = add(
        "Sequence",
        "AllHealthy",
        [
            add("NotAborted", "NotAborted"),
            add("BatteryAboveReserve", "BatteryAboveReserve"),
            add("LocalizationOK", "LocalizationOK"),
        ],
    )
    glitch = add("Sequence", "PoseGlitch", [add("VOLostBriefly", "VOLostBriefly"), add("Stop", "Stop")])
    emergency = add("Sequence", "EmergencyStop", [add("MustStopNow", "MustStopNow"), add("StopHere", "StopHere")])
    home = add(
        "Sequence",
        "ReturnAndDock",
        [
            add("MotorsOn", "MotorsOn"),
            add("FailsafeReverseOut", "Failsafe: ReverseOut"),
            add("ReturnToDock", "Failsafe: ReturnToDock"),
            add("Dock", "Failsafe: Dock"),
        ],
    )
    choice = add("Selector", "FailsafeChoice", [glitch, emergency, home])
    guard = add("Selector", "SafetyGuard", [healthy, add("SuccessIsFailure", "Failsafe", [choice])])
    lanes = [_add_trolley_lane(add, k) for k in range(1, layout.n_lanes + 1)]
    mission = add(
        "Sequence",
        "Mission",
        [add("Preflight", "Preflight"), *lanes, add("ReturnToDock", "ReturnToDock"), add("Dock", "Dock")],
    )
    add("Sequence", "Root", [guard, mission])
    graph = {"version": GRAPH_VERSION, "nodes": nodes, "edges": edges}
    tidy_layout(graph)
    return graph


def tidy_layout(graph: dict, dx: float = 260.0, dy: float = 84.0) -> dict:
    """Left-to-right tree layout: depth along x, leaves one row apart, each parent level with the
    middle of its children. Keeps the order children are ticked in (edge order here)."""
    kids: dict[str, list[str]] = {n["id"]: [] for n in graph["nodes"]}
    has_parent = set()
    for e in graph["edges"]:
        kids[e["source"]].append(e["target"])
        has_parent.add(e["target"])
    row = 0

    def place(nid: str, depth: int) -> float:
        nonlocal row
        node = by_id[nid]
        if kids[nid]:
            ys = [place(c, depth + 1) for c in kids[nid]]
            y = (ys[0] + ys[-1]) / 2
        else:
            y = row * dy
            row += 1
        node["position"] = {"x": depth * dx, "y": y}
        return y

    by_id = {n["id"]: n for n in graph["nodes"]}
    for n in graph["nodes"]:
        if n["id"] not in has_parent:
            place(n["id"], 0)
    return graph
