"""Whole missions in the simulator, judged against ground truth."""

import pytest

from rowpilot.layout import GreenhouseLayout
from rowpilot.mission import MissionContext, build_tree, tree_to_dict
from rowpilot.runner import RunConfig, run_mission
from rowpilot.sim import SimConfig, SimVehicle

SMALL = GreenhouseLayout(n_lanes=3, row_length=15.0)


def run(layout=SMALL, **sim):
    return run_mission(RunConfig(layout=layout, sim=SimConfig(**sim)))


def statuses(result, node):
    return [e["status"] for e in result.events if e["type"] == "status" and e["node"] == node]


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_nominal_mission_sweeps_every_lane_and_lands(seed):
    r = run(seed=seed)
    assert r.outcome == "completed"
    assert r.lanes_completed == [0, 1, 2]
    assert r.max_lane_error_m < 0.45  # 0.8 m half-lane minus the drone radius is 0.55 m
    assert max(r.lane_rms_error_m.values()) < 0.2
    assert statuses(r, "Land")[-1] == "SUCCESS"


def test_full_greenhouse_on_one_battery():
    r = run(layout=GreenhouseLayout())
    assert r.outcome == "completed", r.outcome
    assert r.lanes_completed == list(range(6))
    assert r.battery_end_pct > 20


@pytest.mark.parametrize("seed", [3, 4])
def test_strong_gusts_still_safe(seed):
    # Gusts of 0.5 m/s rms (peaks ~1.5 m/s), against the default 0.3. With the flight controller's real
    # response (0.5 s dead time + 0.3 s lag, measured on ArduCopter SITL) this is about the limit: 20/20
    # seeds complete at 0.5 m/s; at 0.8 m/s most hit the crop, whatever the gains (the old 0.8 m/s test
    # passed only because the simulator assumed a flight controller without delay).
    r = run(seed=seed, wind_sigma=0.5, wind_tau_s=1.0)
    assert r.outcome != "crashed"
    assert r.max_lane_error_m < 0.55


def test_low_battery_returns_home_and_lands():
    r = run(layout=GreenhouseLayout(), start_battery_pct=45)
    assert r.outcome == "aborted: battery low"
    assert 0 < len(r.lanes_completed) < 6
    assert r.trace[-1]["z"] < 0.05
    hx, hy = GreenhouseLayout().home
    assert abs(r.trace[-1]["x"] - hx) < 0.6 and abs(r.trace[-1]["y"] - hy) < 0.6


def test_short_vo_dropout_hovers_then_resumes():
    r = run(vo_dropouts=[(25.0, 1.5)])
    assert r.outcome == "completed"
    assert "RUNNING" in statuses(r, "Hover")
    # The interrupted mission resumed where it was instead of starting over.
    assert statuses(r, "Takeoff").count("SUCCESS") >= 1
    assert sum(1 for e in r.events if e["type"] == "log" and e["node"] == "Takeoff" and "done" in e["msg"]) == 1


def test_long_vo_dropout_lands_in_place():
    r = run(vo_dropouts=[(25.0, 30.0)])
    assert r.outcome == "aborted: visual odometry lost"
    assert "SUCCESS" in statuses(r, "LandHere")
    assert r.duration_s < 25.0 + 3.0 + 10.0  # waited out the glitch window, then came straight down


def test_refuses_to_start_on_low_battery():
    r = run(start_battery_pct=30)
    assert r.outcome.startswith("aborted")
    assert max(t["z"] for t in r.trace) == 0.0


def test_tree_export_for_editor():
    ctx = MissionContext(SimVehicle(SMALL), SMALL)
    d = tree_to_dict(build_tree(ctx))
    names = {n["name"] for n in d["nodes"]}
    lane3 = {"Lane 3: Sweep in", "Lane 3: TurnAround", "Lane 3: Sweep out"}
    assert {"SafetyGuard", "Mission", "Failsafe: ReturnHome", *lane3} <= names
    assert len(d["edges"]) == len(d["nodes"]) - 1  # a tree


def test_events_stream_live():
    seen = []
    run_mission(RunConfig(layout=GreenhouseLayout(n_lanes=1, row_length=8.0)), on_event=seen.append)
    assert {"status", "log"} <= {e["type"] for e in seen}
    assert seen[-1] == {"t": seen[-1]["t"], "type": "end", "node": "Root", "msg": "completed"}


def test_frequent_vo_glitches_resume_without_replanning():
    """Glitches in the middle of legs must not send the drone back to the start of the leg."""
    glitches = [(t, 0.4) for t in range(8, 200, 7)]
    r = run(vo_dropouts=glitches)
    assert r.outcome == "completed", r.outcome
    assert r.lanes_completed == [0, 1, 2]
    for node in ("Lane 2: GoToEntry", "Lane 3: GoToEntry", "ReturnHome"):
        plans = [e for e in r.events if e["type"] == "log" and e["node"] == node and e["msg"].startswith("waypoints")]
        assert len(plans) == 1, node
    assert all(any(e["msg"].startswith(f"lane {i} swept") for e in r.events if e["type"] == "log") for i in (1, 2, 3))


def test_closed_lanes_are_flown_in_and_back_out():
    """Dead-end lanes (the real greenhouse): every lane is entered and left from the near headland,
    with a turn short of the end wall, and the drone never gets close to the wall."""
    r = run()
    assert r.outcome == "completed", r.outcome
    for i in (1, 2, 3):
        assert statuses(r, f"Lane {i}: TurnAround")[-1] == "SUCCESS"
        assert statuses(r, f"Lane {i}: Sweep out")[-1] == "SUCCESS"
    assert max(t["x"] for t in r.trace) < SMALL.row_length - 0.5  # wall at row_length
    assert max(t["x"] for t in r.trace) > SMALL.turn_x() - 0.3  # but it did sweep to the end


def test_open_lanes_are_still_swept_serpentine():
    lay = GreenhouseLayout(n_lanes=2, row_length=10.0, closed_end=False)
    r = run(layout=lay)
    assert r.outcome == "completed", r.outcome
    assert r.lanes_completed == [0, 1]
    assert max(t["x"] for t in r.trace) > lay.row_length  # turned in the far headland


def test_planned_route_visits_every_lane_and_never_crosses_the_crop():
    for lay in (SMALL, GreenhouseLayout(n_lanes=3, row_length=10.0, closed_end=False)):
        route = lay.planned_route()
        assert route[0] == route[-1] == lay.home
        for i in range(lay.n_lanes):
            assert any(abs(y - lay.lane_center_y(i)) < 1e-9 and x > lay.row_length / 2 for x, y in route)
        for (x0, y0), (x1, y1) in zip(route, route[1:], strict=False):
            if abs(y1 - y0) > 1e-9:  # every move across the rows happens in a headland
                assert (x0 < 0 and x1 < 0) or (x0 > lay.row_length and x1 > lay.row_length)
