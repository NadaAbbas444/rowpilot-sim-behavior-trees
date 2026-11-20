"""Command line: run a mission, print the tree.

python -m rowpilot run --out runs/nominal
python -m rowpilot run --battery 45 --out runs/low-battery
python -m rowpilot run --vo-dropout 30:1.5 --vo-dropout 120:10 --out runs/vo
python -m rowpilot run --robot trolley --out runs/trolley   # the pipe-rail trolley instead of the drone
python -m rowpilot tree [--robot trolley]
python -m rowpilot trolley-gazebo  # the trolley in a running Gazebo (VEHICLE=trolley scripts/run_ardupilot_sim.sh)
python -m rowpilot editor          # the behaviour tree editor at http://127.0.0.1:8000
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import py_trees

from .layout import GreenhouseLayout
from .mission import MissionContext, MissionParams, build_tree, tree_to_dict
from .runner import ROBOTS, RunConfig, run_mission
from .sim import SimConfig, SimVehicle
from .trolley import TrolleySim, build_trolley_tree


def _dropout(s: str) -> tuple[float, float]:
    t0, d = s.split(":")
    return float(t0), float(d)


def _layout(a) -> GreenhouseLayout:
    return GreenhouseLayout(n_lanes=a.lanes, row_length=a.length, path_width=a.path_width, closed_end=not a.open_ends)


def _stream(kind: str, **data) -> None:
    """One message for the editor (rowpilot/editor/runs.py reads these from the run's output)."""
    print("@rowpilot " + json.dumps({"kind": kind, **data}, default=float), flush=True)


def _stream_hooks() -> dict:
    """run_ardupilot callbacks that stream to the editor, and 'stop' on stdin to end the run."""
    import _thread
    import threading

    def watch_stdin() -> None:
        for line in sys.stdin:
            if line.strip() == "stop":
                _stream("event", type="system", source="editor", node="editor", msg="stop requested: landing")
                _thread.interrupt_main()  # KeyboardInterrupt: land in place, end the run
                return

    threading.Thread(target=watch_stdin, daemon=True).start()
    return {
        "on_event": lambda ev: _stream("event", **ev),
        "on_tick": lambda row: _stream("tick", **row),
        "say": lambda source, msg, t=None: _stream("event", t=t, type="system", source=source, node=source, msg=msg),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="rowpilot", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("run", "tree", "world", "px4", "ardupilot", "trolley-gazebo"):
        sp = sub.add_parser(name)
        sp.add_argument("--lanes", type=int, default=6)
        sp.add_argument("--length", type=float, default=GreenhouseLayout.row_length, help="row length (m)")
        sp.add_argument("--path-width", type=float, default=1.6, help="free lane width (m)")
        sp.add_argument(
            "--open-ends",
            action="store_true",
            help="lanes open at the far end too (default: dead ends, as in the greenhouse)",
        )
    for name in ("run", "tree"):
        sub.choices[name].add_argument(
            "--robot", choices=ROBOTS, default="drone", help="the FireFly 2 drone, or the pipe-rail trolley"
        )
    r = sub.choices["run"]
    r.add_argument("--seed", type=int, default=0)
    r.add_argument("--battery", type=float, default=100.0, help="start battery (%%)")
    r.add_argument("--wind", type=float, default=0.3, help="gust strength (m/s)")
    r.add_argument("--vo-dropout", type=_dropout, action="append", default=[], metavar="START:SECONDS")
    r.add_argument("--altitude", type=float, default=1.5)
    r.add_argument("--speed", type=float, default=0.8, help="sweep speed (m/s)")
    r.add_argument("--out", type=Path, help="directory for events.jsonl, trace.csv, summary.json, path.png")
    r.add_argument("--quiet", action="store_true")
    sub.choices["tree"].add_argument("--json", action="store_true", help="nodes and edges for a node editor")
    w = sub.choices["world"]
    w.add_argument("--template", type=Path, required=True, help="PX4 Tools/simulation/gz/worlds/default.sdf")
    w.add_argument("--out", type=Path, required=True)
    w.add_argument("--assets-dir", type=Path, help="Gazebo model folder for plant meshes; omit for plain boxes")
    w.add_argument("--vehicle", help="model URI to place at home, e.g. model://gh_iris (ArduPilot)")
    w.add_argument("--vehicle-z", type=float, default=0.2, help="its height at home (m): 0 for the trolley")
    x = sub.choices["px4"]
    x.add_argument("--url", default="udpin://0.0.0.0:14540", help="MAVLink connection to PX4")
    x.add_argument("--world", default="greenhouse")
    x.add_argument("--model", default="gh_x500_0")
    x.add_argument("--max-time", type=float, default=900.0)
    x.add_argument("--out", type=Path)
    ap = sub.choices["ardupilot"]
    ap.add_argument("--url", default="tcp:127.0.0.1:5760", help="MAVLink connection to ArduPilot SITL")
    ap.add_argument("--max-sim-time", type=float, default=900.0)
    ap.add_argument("--vehicle", choices=("firefly", "iris"), default="firefly", help="simulated drone")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--tree", type=Path, help="fly this tree (a graph saved by the editor) instead of the mission")
    ap.add_argument(
        "--stream",
        action="store_true",
        help="print every event, tick and flight-stack message as a JSON line for the editor; "
        "a line 'stop' on stdin lands the drone and ends the run",
    )
    tg = sub.choices["trolley-gazebo"]
    tg.add_argument("--max-sim-time", type=float, default=1200.0)
    tg.add_argument("--out", type=Path)
    tg.add_argument("--tree", type=Path, help="drive this tree (a graph saved by the editor) instead of the mission")
    tg.add_argument(
        "--stream",
        action="store_true",
        help="print every event, tick and message as a JSON line for the editor; 'stop' on stdin stops it",
    )
    ed = sub.add_parser("editor", help="the behaviour tree editor (web page + API)")
    ed.add_argument("--host", default="127.0.0.1")
    ed.add_argument("--port", type=int, default=8000)
    ed.add_argument("--trees", type=Path, default=Path("trees"), help="folder for saved trees")
    ed.add_argument("--ardupilot", type=Path, default=Path.home() / "ardupilot", help="for Gazebo runs")
    ed.add_argument("--ardupilot-gazebo", type=Path, default=Path.home() / "ardupilot_gazebo", help="for Gazebo runs")
    a = p.parse_args(argv)

    if a.cmd == "editor":
        try:
            import uvicorn

            from .editor.runs import GazeboSetup
            from .editor.server import WEB_DIST, create_app
        except ImportError:
            print('the editor needs its extras: pip install -e ".[editor]"', file=sys.stderr)
            return 1
        if not (WEB_DIST / "index.html").is_file():
            print(f"web page not built ({WEB_DIST}): cd web && npm ci && npm run build; API only", file=sys.stderr)
        gazebo = GazeboSetup(ardupilot=a.ardupilot, ardupilot_gazebo=a.ardupilot_gazebo)
        for sim, name in (("gazebo", "Gazebo"), ("isaac", "Isaac Sim")):
            missing = gazebo.missing(sim)
            print(f"{name} runs: " + ("ready" if not missing else "not available, missing " + "; ".join(missing)))
        print(f"rowpilot editor on http://{a.host}:{a.port}")
        uvicorn.run(create_app(a.trees, gazebo=gazebo), host=a.host, port=a.port, log_level="warning")
        return 0

    layout = _layout(a)

    if a.cmd == "world":
        from .gazebo_world import spawn_pose, write_world

        write_world(layout, a.template, a.out, assets_dir=a.assets_dir, vehicle=a.vehicle, vehicle_z=a.vehicle_z)
        print(f"wrote {a.out}; spawn with PX4_GZ_MODEL_POSE={spawn_pose(layout)}")
        return 0

    if a.cmd == "px4":
        from .px4 import run_px4

        res = run_px4(layout, MissionParams(), a.url, a.world, a.model, a.max_time, a.out)
        print(json.dumps(res, indent=1))
        return 0 if res["outcome"] != "crashed" else 1

    if a.cmd == "trolley-gazebo":
        from .trolley_gazebo import run_trolley_gazebo

        hooks = {}
        if a.tree:
            from .editor.nodes import build_from_graph

            graph = json.loads(a.tree.read_text())
            hooks["tree_factory"] = lambda ctx: build_from_graph(graph, ctx)
        if a.stream:
            hooks.update(_stream_hooks())
        res = run_trolley_gazebo(layout, MissionParams(), max_sim_time_s=a.max_sim_time, out=a.out, **hooks)
        if a.stream:
            _stream("summary", **res)
        else:
            print(json.dumps(res, indent=1))
        return 0

    if a.cmd == "ardupilot":
        from .airframe import VEHICLES
        from .ardupilot import run_ardupilot

        v = VEHICLES[a.vehicle]
        hooks: dict = {}
        if a.tree:
            from .editor.nodes import build_from_graph

            graph = json.loads(a.tree.read_text())
            hooks["tree_factory"] = lambda ctx: build_from_graph(graph, ctx)
        if a.stream:
            hooks.update(_stream_hooks())
        res = run_ardupilot(
            layout,
            MissionParams(),
            a.url,
            model=v.model,
            max_sim_time_s=a.max_sim_time,
            out=a.out,
            home_z=v.home_z,
            rig=v.rig,
            **hooks,
        )
        if a.stream:
            _stream("summary", **res)
        else:
            print(json.dumps(res, indent=1))
        return 0

    if a.cmd == "tree":
        if a.robot == "trolley":
            root = build_trolley_tree(MissionContext(TrolleySim(layout), layout))
        else:
            root = build_tree(MissionContext(SimVehicle(layout), layout))
        print(json.dumps(tree_to_dict(root), indent=1) if a.json else py_trees.display.unicode_tree(root))
        return 0

    cfg = RunConfig(
        robot=a.robot,
        layout=layout,
        sim=SimConfig(seed=a.seed, start_battery_pct=a.battery, wind_sigma=a.wind, vo_dropouts=a.vo_dropout),
        mission=MissionParams(altitude=a.altitude, sweep_speed=a.speed),
    )

    def show(ev: dict) -> None:
        if ev["type"] in ("log", "end"):
            print(f"{ev['t']:7.1f}s  {ev['node']:<24} {ev['msg']}")

    res = run_mission(cfg, on_event=None if a.quiet else show)
    print(json.dumps(res.summary(), indent=1))
    if a.out:
        a.out.mkdir(parents=True, exist_ok=True)
        with open(a.out / "events.jsonl", "w") as f:
            f.writelines(json.dumps(e) + "\n" for e in res.events)
        with open(a.out / "trace.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(res.trace[0]))
            w.writeheader()
            w.writerows(
                {k: (round(float(v), 4) if isinstance(v, float) else v) for k, v in t.items()} for t in res.trace
            )
        (a.out / "summary.json").write_text(json.dumps(res.summary(), indent=1))
        try:
            from .plot import plot_run

            plot_run(res, layout, str(a.out / "path.png"), robot=a.robot)
        except ImportError:
            print("matplotlib not installed, skipping path.png", file=sys.stderr)
    return 0 if res.outcome != "crashed" else 1


if __name__ == "__main__":
    sys.exit(main())
