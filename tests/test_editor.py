import copy
import json
import time
from pathlib import Path

import pytest

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from rowpilot.editor.nodes import NODE_TYPES, build_from_graph, default_graph, tidy_layout, validate  # noqa: E402
from rowpilot.editor.server import create_app  # noqa: E402
from rowpilot.layout import GreenhouseLayout  # noqa: E402
from rowpilot.mission import MissionContext, build_tree, tree_to_dict  # noqa: E402
from rowpilot.runner import RunConfig, run_mission  # noqa: E402
from rowpilot.sim import SimVehicle  # noqa: E402

FAST = {"lanes": 1, "speed": 0}


def names_types(root):
    return [(n["name"], n["type"]) for n in tree_to_dict(root)["nodes"]]


@pytest.mark.parametrize("closed", [True, False])
def test_default_graph_builds_the_mission_tree(closed):
    lay = GreenhouseLayout(n_lanes=3, closed_end=closed)
    ctx = MissionContext(SimVehicle(lay), lay)
    assert names_types(build_from_graph(default_graph(lay), ctx)) == names_types(build_tree(ctx))


def test_default_graph_flies_exactly_like_the_mission_tree():
    lay = GreenhouseLayout(n_lanes=2)
    graph = default_graph(lay)
    a = run_mission(RunConfig(layout=lay))
    b = run_mission(RunConfig(layout=lay), tree_factory=lambda ctx: build_from_graph(graph, ctx))
    assert a.outcome == b.outcome == "completed"
    assert [{k: v for k, v in e.items() if k != "id"} for e in b.events] == a.events
    assert all("id" in e for e in b.events)  # every event names its graph node


def test_children_run_in_drawn_order_top_to_bottom():
    lay = GreenhouseLayout(n_lanes=1)
    g = default_graph(lay)
    root = next(n for n in g["nodes"] if n["name"] == "Mission")
    kids = [e["target"] for e in g["edges"] if e["source"] == root["id"]]
    by_id = {n["id"]: n for n in g["nodes"]}
    by_id[kids[0]]["position"]["y"], by_id[kids[1]]["position"]["y"] = (
        by_id[kids[1]]["position"]["y"],
        by_id[kids[0]]["position"]["y"],
    )  # Takeoff drawn above Preflight
    ctx = MissionContext(SimVehicle(lay), lay)
    mission = next(n for n in build_from_graph(g, ctx).iterate() if n.name == "Mission")
    assert [c.name for c in mission.children[:2]] == ["Takeoff", "Preflight"]


def small(*nodes, edges=()):
    return {"version": 1, "nodes": list(nodes), "edges": [{"source": s, "target": t} for s, t in edges]}


def node(nid, type_, name=None, **params):
    return {"id": nid, "type": type_, "name": name or nid, "params": params}


LAY = GreenhouseLayout(n_lanes=2)


def messages(graph):
    return [e["message"] for e in validate(graph, LAY)]


def test_validation_finds_what_is_wrong():
    assert validate(small(node("a", "Sequence"), node("b", "Takeoff"), edges=[("a", "b")]), LAY) == []
    assert any("needs a child" in m for m in messages(small(node("a", "Sequence"))))
    assert any(
        "cannot have children" in m
        for m in messages(small(node("a", "Takeoff"), node("b", "Land"), edges=[("a", "b")]))
    )
    assert any(
        "takes only 1 child" in m
        for m in messages(
            small(node("a", "Inverter"), node("b", "Land"), node("c", "Hover"), edges=[("a", "b"), ("a", "c")])
        )
    )
    assert any(
        "used twice" in m
        for m in messages(
            small(
                node("a", "Sequence"), node("b", "Land", "x"), node("c", "Hover", "x"), edges=[("a", "b"), ("a", "c")]
            )
        )
    )
    assert any("unknown node type" in m for m in messages(small(node("a", "Teleport"))))
    assert any(
        "not connected" in m
        for m in messages(small(node("a", "Sequence"), node("b", "Land"), node("c", "Hover"), edges=[("a", "b")]))
    )
    assert any(
        "two parents" in m
        for m in messages(
            small(
                node("a", "Sequence"),
                node("b", "Sequence"),
                node("c", "Land"),
                edges=[("a", "b"), ("a", "c"), ("b", "c")],
            )
        )
    )
    assert any(
        "cycle" in m
        for m in messages(
            small(
                node("a", "Sequence"),
                node("b", "Sequence"),
                node("c", "Land"),
                edges=[("a", "c"), ("b", "a"), ("a", "b")],
            )
        )
    )


def test_parameters_are_checked():
    lane3 = small(node("a", "Sequence"), node("b", "AlignToLane", lane=3), edges=[("a", "b")])
    (e,) = validate(lane3, LAY)
    assert e["node"] == "b" and "lane 3 does not exist: the greenhouse has 2 lanes" in e["message"]
    bad = small(node("a", "Sequence"), node("b", "SweepLane", lane=1, direction="up", speed=2), edges=[("a", "b")])
    msgs = messages(bad)
    assert any("direction must be one of" in m for m in msgs) and any("no parameter 'speed'" in m for m in msgs)
    assert any("must be a number" in m for m in messages(small(node("a", "Wait", seconds="x"))))


def test_every_node_type_builds_with_its_defaults():
    from rowpilot.trolley import TrolleySim

    lay = GreenhouseLayout(n_lanes=2)
    ctxs = {"drone": MissionContext(SimVehicle(lay), lay), "trolley": MissionContext(TrolleySim(lay), lay)}
    for t in NODE_TYPES.values():
        robot = t.robots[0]  # build it for a robot it is for
        ctx, leaf = ctxs[robot], {"drone": "Hover", "trolley": "Stop"}[robot]
        lo, _ = t.children
        nodes = [node("p", t.type)] + [node(f"c{i}", leaf) for i in range(lo)]
        g = small(*nodes, edges=[("p", f"c{i}") for i in range(lo)])
        assert build_from_graph(g, ctx).graph_id == "p", t.type


def test_a_hand_made_tree_flies():
    """Take off, wait, fly to a point, land: the editor's own nodes, in a tree the mission never had."""
    g = small(
        node("r", "Sequence", "Root"),
        node("a", "Preflight"),
        node("b", "Takeoff"),
        node("w", "Wait", seconds=3.0),
        node("t", "Timeout", "Limit", seconds=60.0),
        node("g", "GoToPoint", "Hop", x=-2.5, y=2.0),
        node("l", "MissionLand", "Land"),
        edges=[("r", "a"), ("r", "b"), ("r", "w"), ("r", "t"), ("t", "g"), ("r", "l")],
    )
    res = run_mission(RunConfig(layout=LAY), tree_factory=lambda ctx: build_from_graph(g, ctx))
    assert res.outcome == "completed"
    assert abs(res.trace[-1]["y"] - 2.0) < 0.3
    logs = [e for e in res.events if e["type"] == "log" and e["id"] == "w"]
    assert logs and "holding for 3 s" in logs[0]["msg"]


def test_timeout_fails_a_slow_child():
    g = small(
        node("r", "Sequence", "Root"),
        node("a", "Preflight"),
        node("b", "Takeoff"),
        node("t", "Timeout", "Limit", seconds=2.0),
        node("h", "Hover"),
        edges=[("r", "a"), ("r", "b"), ("r", "t"), ("t", "h")],
    )
    res = run_mission(RunConfig(layout=LAY, max_time_s=60), tree_factory=lambda ctx: build_from_graph(g, ctx))
    assert any(e.get("id") == "t" and e.get("msg", "").startswith("timed out") for e in res.events)


def test_tidy_layout_keeps_child_order():
    g = tidy_layout(small(node("a", "Sequence"), node("b", "Land"), node("c", "Hover"), edges=[("a", "c"), ("a", "b")]))
    pos = {n["id"]: n["position"] for n in g["nodes"]}
    assert pos["c"]["y"] < pos["b"]["y"] and pos["a"]["x"] < pos["b"]["x"]


# ---- the API ------------------------------------------------------------------------------------
@pytest.fixture
def client(tmp_path):
    app = create_app(trees_dir=tmp_path / "trees")
    with TestClient(app) as c:
        yield c


def test_node_types_and_default_tree(client):
    types = client.get("/api/node-types").json()
    assert {t["type"] for t in types} == set(NODE_TYPES)
    sweep = next(t for t in types if t["type"] == "SweepLane")
    assert sweep["category"] == "action" and sweep["group"] == "lane" and sweep["max_children"] == 0
    g = client.get("/api/default-tree", params={"lanes": 2}).json()
    assert {"Lane 1: Sweep in", "Lane 2: Exit"} <= {n["name"] for n in g["nodes"]}
    assert client.post("/api/validate", json={"graph": g, "settings": {"lanes": 2}}).json() == {"errors": []}
    errs = client.post("/api/validate", json={"graph": g, "settings": {"lanes": 1}}).json()["errors"]
    assert errs and all("has 1 lane" in e["message"] for e in errs)


def test_layout(client):
    lay = client.get("/api/layout", params={"lanes": 3}).json()
    assert lay["n_lanes"] == 3 and len(lay["crop_rows_y"]) == 4 and lay["closed_end"]
    assert lay["planned_route"][0] == list(lay["home"]) == lay["planned_route"][-1]


def test_run_streams_status_logs_ticks_and_the_end(client):
    g = client.get("/api/default-tree", params={"lanes": 1}).json()
    run = client.post("/api/runs", json={"graph": g, "settings": FAST}).json()
    assert run["status"] == "running"
    with client.websocket_connect(f"/api/runs/{run['id']}/stream") as ws:
        msgs = []
        while not msgs or msgs[-1]["kind"] != "end":
            msgs += ws.receive_json()
    end = msgs[-1]
    assert end["status"] == "finished" and end["summary"]["outcome"] == "completed"
    kinds = {m["kind"] for m in msgs}
    assert kinds == {"event", "tick", "end"}
    ids = {n["id"] for n in g["nodes"]}
    assert {m["id"] for m in msgs if m["kind"] == "event" and m["type"] == "status"} <= ids
    sweep = next(n["id"] for n in g["nodes"] if n["name"] == "Lane 1: Sweep in")
    assert any(m["kind"] == "event" and m["type"] == "log" and m["id"] == sweep for m in msgs)
    # a late viewer gets the whole run, from any point
    with client.websocket_connect(f"/api/runs/{run['id']}/stream?start=5") as ws:
        again = []
        while not again or again[-1]["kind"] != "end":
            again += ws.receive_json()
    assert again == msgs[5:]
    assert client.get(f"/api/runs/{run['id']}").json()["summary"]["outcome"] == "completed"


def test_stop_a_run(client):
    g = client.get("/api/default-tree", params={"lanes": 1}).json()
    run = client.post("/api/runs", json={"graph": g, "settings": {"lanes": 1, "speed": 5}}).json()
    client.post(f"/api/runs/{run['id']}/stop")
    with client.websocket_connect(f"/api/runs/{run['id']}/stream") as ws:
        msgs = []
        while not msgs or msgs[-1]["kind"] != "end":
            msgs += ws.receive_json()
    assert msgs[-1]["status"] == "stopped" and msgs[-1]["summary"]["outcome"] == "stopped"


def test_a_bad_graph_is_not_run(client):
    r = client.post("/api/runs", json={"graph": small(node("a", "Sequence")), "settings": FAST})
    assert r.status_code == 422 and "needs a child" in r.json()["detail"]["errors"][0]["message"]
    assert client.post("/api/runs", json={"graph": {"nodes": "x"}}).status_code == 422
    assert client.get("/api/runs/nope").status_code == 404


def test_a_node_that_raises_ends_the_run_with_an_error(client, monkeypatch):
    from rowpilot.mission import Takeoff

    monkeypatch.setattr(Takeoff, "run", lambda self: 1 / 0)
    g = client.get("/api/default-tree", params={"lanes": 1}).json()
    run = client.post("/api/runs", json={"graph": g, "settings": FAST}).json()
    with client.websocket_connect(f"/api/runs/{run['id']}/stream") as ws:
        msgs = []
        while not msgs or msgs[-1]["kind"] != "end":
            msgs += ws.receive_json()
    assert msgs[-1]["status"] == "error" and "ZeroDivisionError" in msgs[-1]["error"]


def test_save_load_delete_trees(client):
    g = copy.deepcopy(client.get("/api/default-tree", params={"lanes": 1}).json())
    assert client.get("/api/trees").json() == []
    assert client.put("/api/trees/one-lane", json=g).status_code == 200
    assert [t["name"] for t in client.get("/api/trees").json()] == ["one-lane"]
    assert client.get("/api/trees/one-lane").json()["nodes"] == g["nodes"]
    assert client.put("/api/trees/..%2Fescape", json=g).status_code in (400, 404, 405)
    assert client.put("/api/trees/bad name", json=g).status_code == 400
    assert client.delete("/api/trees/one-lane").status_code == 204
    assert client.get("/api/trees/one-lane").status_code == 404


def test_web_page_and_unknown_api_paths(client):
    from rowpilot.editor.server import WEB_DIST

    if (WEB_DIST / "index.html").is_file():  # only once the page has been built (npm run build)
        assert '<div id="root">' in client.get("/").text
        assert client.get("/some/route").status_code == 200  # the page handles its own routes
        assert client.get("/../pyproject.toml").text == client.get("/").text
    assert client.get("/api/nothing-here").status_code == 404


# ---- Gazebo runs (with a stand-in for scripts/run_ardupilot_sim.sh) -----------------------------------
FAKE_SIM = Path(__file__).parent / "data" / "fake_run_ardupilot_sim.sh"  # stands in for the run script


@pytest.fixture
def gz_client(tmp_path):
    from rowpilot.editor.runs import GazeboSetup

    setup = GazeboSetup(script=FAKE_SIM, runs_dir=tmp_path / "runs", check_tools=False)
    with TestClient(create_app(trees_dir=tmp_path / "trees", gazebo=setup)) as c:
        yield c


def stream_all(client, run_id):
    with client.websocket_connect(f"/api/runs/{run_id}/stream") as ws:
        msgs = []
        while not msgs or msgs[-1]["kind"] != "end":
            msgs += ws.receive_json()
    return msgs


def test_gazebo_run_streams_the_flight_stack_and_the_tree(gz_client):
    assert gz_client.get("/api/simulators").json()["gazebo"]["available"]
    g = gz_client.get("/api/default-tree", params={"lanes": 2}).json()
    settings = {"simulator": "gazebo", "lanes": 2, "gazebo_window": False, "vehicle": "iris"}
    run = gz_client.post("/api/runs", json={"graph": g, "settings": settings}).json()
    assert run["simulator"] == "gazebo"
    msgs = stream_all(gz_client, run["id"])
    system = [(m["source"], m["msg"]) for m in msgs if m["kind"] == "event" and m["type"] == "system"]
    texts = " | ".join(f"{s}: {t}" for s, t in system)
    assert "setup: Starting Gazebo" in texts
    assert "--lanes 2" in texts and "window: 1" in texts and "vehicle: iris" in texts  # settings reach the script
    assert "vio: OpenVINS initialised" in texts
    assert texts.count("OpenVINS waiting to initialise") == 1  # repeated log lines shown once
    assert "mavlink: GUIDED mode, motors armed" in texts and "ekf: EKF3 has a position" in texts
    takeoff = next(n["id"] for n in g["nodes"] if n["name"] == "Takeoff")
    assert any(
        m["kind"] == "event" and m.get("id") == takeoff and m["status"] == "RUNNING" for m in msgs if "status" in m
    )
    assert sum(m["kind"] == "tick" for m in msgs) == 3
    assert msgs[-1]["status"] == "finished" and msgs[-1]["summary"]["duration_s"] == 20.5
    # a system line after the ticks carries the simulation time
    assert next(m for m in msgs if m.get("source") == "setup" and m["msg"].startswith("Results"))["t"] == 18.3


def test_gazebo_run_stops_by_landing(gz_client, monkeypatch):
    monkeypatch.setenv("FAKE_WAIT_FOR_STOP", "1")
    g = gz_client.get("/api/default-tree", params={"lanes": 1}).json()
    run = gz_client.post("/api/runs", json={"graph": g, "settings": {"simulator": "gazebo", "lanes": 1}}).json()
    with gz_client.websocket_connect(f"/api/runs/{run['id']}/stream") as ws:
        msgs = []
        while not any(m["kind"] == "tick" for m in msgs):
            msgs += ws.receive_json()
        # one Gazebo run at a time
        assert (
            gz_client.post("/api/runs", json={"graph": g, "settings": {"simulator": "gazebo", "lanes": 1}}).status_code
            == 429
        )
        gz_client.post(f"/api/runs/{run['id']}/stop")
        while msgs[-1]["kind"] != "end":
            msgs += ws.receive_json()
    assert any(m.get("msg") == "stop requested: landing" for m in msgs)
    assert msgs[-1]["status"] == "stopped" and msgs[-1]["summary"]["outcome"].startswith("stopped")


def test_gazebo_needs_its_setup(tmp_path):
    from rowpilot.editor.runs import GazeboSetup

    setup = GazeboSetup(ardupilot=tmp_path / "none", ardupilot_gazebo=tmp_path / "none2")
    with TestClient(create_app(trees_dir=tmp_path / "trees", gazebo=setup)) as c:
        sims = c.get("/api/simulators").json()
        assert sims["python"]["available"] and not sims["gazebo"]["available"]
        assert any("ArduCopter" in m for m in sims["gazebo"]["missing"])
        g = c.get("/api/default-tree", params={"lanes": 1}).json()
        r = c.post("/api/runs", json={"graph": g, "settings": {"simulator": "gazebo", "lanes": 1}})
        assert r.status_code == 409 and "not set up" in r.json()["detail"]


def test_isaac_run_uses_the_same_script_with_sim_isaac(gz_client):
    sims = gz_client.get("/api/simulators").json()
    assert sims["isaac"]["available"]
    g = gz_client.get("/api/default-tree", params={"lanes": 1}).json()
    run = gz_client.post("/api/runs", json={"graph": g, "settings": {"simulator": "isaac", "lanes": 1}}).json()
    assert run["simulator"] == "isaac"
    msgs = stream_all(gz_client, run["id"])
    texts = " | ".join(m["msg"] for m in msgs if m["kind"] == "event" and m["type"] == "system")
    assert "full simulation in Isaac Sim (firefly)" in texts and "sim: isaac" in texts
    assert msgs[-1]["status"] == "finished"
    r = gz_client.post("/api/runs", json={"graph": g, "settings": {"simulator": "isaac", "vehicle": "iris"}})
    assert r.status_code == 409 and "FireFly only" in r.json()["detail"]


def test_isaac_needs_isaac_sim_but_not_gazebo(tmp_path, monkeypatch):
    from rowpilot.editor import runs

    ap = tmp_path / "ardupilot" / "build/sitl/bin"
    ap.mkdir(parents=True)
    (ap / "arducopter").touch()
    monkeypatch.setattr(runs, "REPO", tmp_path)
    for f in (".venv-sim/bin/python", "build/vio/vio_bridge"):
        (tmp_path / f).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / f).touch()
    setup = runs.GazeboSetup(
        ardupilot=tmp_path / "ardupilot",
        ardupilot_gazebo=tmp_path / "none",
        script=FAKE_SIM,
        isaac_python=tmp_path / "no-isaac",
    )
    monkeypatch.setenv("PATH", str(tmp_path))  # no gz command
    assert setup.missing("isaac") == ["Isaac Sim (scripts/setup_isaac_sim.sh)"]
    assert any("ardupilot_gazebo" in m for m in setup.missing("gazebo"))
    assert "Gazebo Harmonic (the gz command)" in setup.missing("gazebo")
    (tmp_path / "no-isaac").touch()
    assert setup.missing("isaac") == []


def test_python_runs_say_which_simulator_they_use(client):
    g = client.get("/api/default-tree", params={"lanes": 1}).json()
    run = client.post("/api/runs", json={"graph": g, "settings": FAST}).json()
    msgs = stream_all(client, run["id"])
    first = msgs[0]
    assert first["type"] == "system" and first["source"] == "sim" and "no Gazebo" in first["msg"]
    takeoff = [m["msg"] for m in msgs if m["kind"] == "event" and m["node"] == "Takeoff" and m["type"] == "log"]
    assert takeoff[0] == "climbing to 1.5 m"


def test_openvins_log_lines():
    from rowpilot.editor.runs import openvins_line

    assert openvins_line("\x1b[0m\x1b[33m[init]: successful initialization in 0.0002 seconds") == "OpenVINS initialised"
    assert openvins_line("[ZUPT]: accepted |v_IinG|") is None
    assert openvins_line("bg = 0.1,0.2,0.3") is None


def test_gazebo_run_stopped_while_starting_ends_at_once(gz_client, monkeypatch):
    monkeypatch.setenv("FAKE_SLOW_START", "1")  # the stand-in hangs like a Gazebo that never loads
    g = gz_client.get("/api/default-tree", params={"lanes": 1}).json()
    run = gz_client.post("/api/runs", json={"graph": g, "settings": {"simulator": "gazebo", "lanes": 1}}).json()
    with gz_client.websocket_connect(f"/api/runs/{run['id']}/stream") as ws:
        msgs = ws.receive_json()
        t0 = time.monotonic()
        gz_client.post(f"/api/runs/{run['id']}/stop")
        while msgs[-1]["kind"] != "end":
            msgs += ws.receive_json()
    assert time.monotonic() - t0 < 10
    assert msgs[-1]["status"] == "stopped"
    assert any("stopped while starting" in m.get("msg", "") for m in msgs)


@pytest.mark.parametrize("open_ends", [False, True])
def test_one_lane_is_the_mission_trees_lane(client, open_ends):
    mission = client.get("/api/default-tree", params={"lanes": 3, "open_ends": open_ends}).json()
    lane = client.get("/api/lane", params={"lane": 3, "open_ends": open_ends}).json()
    want = sorted(
        (n["type"], n["name"], json.dumps(n["params"], sort_keys=True))
        for n in mission["nodes"]
        if n["name"].startswith("Lane 3")
    )
    got = sorted((n["type"], n["name"], json.dumps(n["params"], sort_keys=True)) for n in lane["nodes"])
    assert got == want
    root = next(n for n in lane["nodes"] if n["id"] == lane["root"])
    assert root["name"] == "Lane 3" and all("position" in n for n in lane["nodes"])
    assert client.get("/api/lane", params={"lane": 0}).status_code == 422


# ---- the pipe-rail trolley ------------------------------------------------------------------------
def test_trolley_default_graph_builds_and_flies_exactly_like_its_tree():
    from rowpilot.trolley import TrolleySim, build_trolley_tree

    lay = GreenhouseLayout(n_lanes=2)
    graph = default_graph(lay, "trolley")
    ctx = MissionContext(TrolleySim(lay), lay)
    assert names_types(build_from_graph(graph, ctx)) == names_types(build_trolley_tree(ctx))
    a = run_mission(RunConfig(layout=lay, robot="trolley"))
    b = run_mission(RunConfig(layout=lay, robot="trolley"), tree_factory=lambda ctx: build_from_graph(graph, ctx))
    assert a.outcome == b.outcome == "completed"
    assert [{k: v for k, v in e.items() if k != "id"} for e in b.events] == a.events


def test_each_robot_gets_its_own_nodes():
    lay = GreenhouseLayout(n_lanes=1)
    assert validate(default_graph(lay, "trolley"), lay, "trolley") == []
    wrong = validate(default_graph(lay), lay, "trolley")
    assert any("Takeoff is for the drone, not the pipe-rail trolley" in e["message"] for e in wrong)
    wrong = validate(default_graph(lay, "trolley"), lay, "drone")
    assert any("MountRail is for the pipe-rail trolley, not the drone" in e["message"] for e in wrong)
    shared = {t.type for t in NODE_TYPES.values() if set(t.robots) == {"drone", "trolley"}}
    assert {"Sequence", "Selector", "NotAborted", "BatteryAboveReserve", "Preflight", "Wait"} <= shared


def test_trolley_runs_from_the_editor_in_the_python_simulator(client):
    g = client.get("/api/default-tree", params={"lanes": 1, "robot": "trolley"}).json()
    assert any(n["type"] == "MountRail" for n in g["nodes"])
    lane = client.get("/api/lane", params={"lane": 2, "robot": "trolley"}).json()
    assert {n["type"] for n in lane["nodes"]} >= {"GoToRailStart", "MountRail", "DriveAlongRail", "ReverseOut"}
    types = {t["type"]: t for t in client.get("/api/node-types").json()}
    assert types["MountRail"]["robots"] == ["trolley"] and types["Takeoff"]["robots"] == ["drone"]
    run = client.post("/api/runs", json={"graph": g, "settings": {**FAST, "robot": "trolley"}}).json()
    msgs = stream_all(client, run["id"])
    assert msgs[-1]["status"] == "finished" and msgs[-1]["summary"]["outcome"] == "completed"
    assert any("pipe-rail trolley" in m.get("msg", "") for m in msgs if m["kind"] == "event")


def test_trolley_runs_in_gazebo_without_ardupilot(gz_client, tmp_path):
    from rowpilot.editor.runs import GazeboSetup

    assert gz_client.get("/api/simulators").json()["gazebo_trolley"]["available"]
    g = gz_client.get("/api/default-tree", params={"lanes": 1, "robot": "trolley"}).json()
    settings = {"simulator": "gazebo", "robot": "trolley", "lanes": 1}
    run = gz_client.post("/api/runs", json={"graph": g, "settings": settings}).json()
    msgs = stream_all(gz_client, run["id"])
    texts = " | ".join(m["msg"] for m in msgs if m["kind"] == "event" and m["type"] == "system")
    assert "vehicle: trolley" in texts and "the pipe-rail trolley, no ArduPilot" in texts
    # the trolley needs Gazebo and OpenVINS, not ArduPilot
    setup = GazeboSetup(ardupilot=tmp_path / "none", ardupilot_gazebo=tmp_path / "none2")
    assert any("ArduCopter" in m for m in setup.missing(robot="drone"))
    assert not any("ArduCopter" in m or "ardupilot_gazebo" in m for m in setup.missing(robot="trolley"))


def test_a_tree_from_the_editor_builds_for_any_trolley_vehicle():
    """The Gazebo trolley is not TrolleySim: the vehicle's robot attribute decides, not its class."""
    from types import SimpleNamespace

    from rowpilot.trolley import TrolleySim

    lay = GreenhouseLayout(n_lanes=2)
    graph = default_graph(lay, "trolley")
    gazebo_like = SimpleNamespace(robot="trolley")  # what TrolleyGazebo says it is
    build_from_graph(graph, MissionContext(gazebo_like, lay))
    build_from_graph(graph, MissionContext(TrolleySim(lay), lay))
    with pytest.raises(ValueError, match="for the pipe-rail trolley, not the drone"):
        build_from_graph(graph, MissionContext(SimVehicle(lay), lay))


def test_isaac_sim_is_for_the_drone_only(gz_client):
    g = gz_client.get("/api/default-tree", params={"lanes": 1, "robot": "trolley"}).json()
    r = gz_client.post("/api/runs", json={"graph": g, "settings": {"simulator": "isaac", "robot": "trolley"}})
    assert r.status_code == 409 and "FireFly only" in r.json()["detail"]
