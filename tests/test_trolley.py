"""The pipe-rail trolley in the Python simulator: its motion, the rails, and its mission tree."""

import math

import numpy as np
import pytest

from rowpilot.control import Command
from rowpilot.layout import GreenhouseLayout
from rowpilot.runner import RunConfig, run_mission
from rowpilot.sim import SimConfig
from rowpilot.trolley import TrolleyConfig, TrolleySim, drive_to, obstacle_ahead, rail_start_approach


def trolley_run(**kw):
    sim = kw.pop("sim", SimConfig())
    tc = kw.pop("trolley", TrolleyConfig())
    events = []
    res = run_mission(RunConfig(robot="trolley", sim=sim, trolley=tc, **kw), on_event=events.append)
    return res, events


def logs(events, word):
    return [e for e in events if e["type"] == "log" and word in e["msg"]]


def at_dock(res, layout=None):
    layout = layout or GreenhouseLayout()
    end = res.trace[-1]
    return math.hypot(end["x"] - layout.home[0], end["y"] - layout.home[1]) < 0.15


# ---- the simulated trolley ----------------------------------------------------------------------


def drive(sim, cmd, seconds, dt=0.02):
    for _ in range(int(seconds / dt)):
        sim.send(cmd)
        sim.step(dt)


def lined_up_sim(dy=0.0, dyaw_deg=0.0):
    lay, tc = GreenhouseLayout(), TrolleyConfig()
    sim = TrolleySim(lay, SimConfig(), tc)
    sim.x, sim.y = rail_start_approach(lay, tc, 1)
    sim.y += dy
    sim.yaw = math.radians(dyaw_deg)
    sim.arm()
    return sim, lay


def test_mounts_when_lined_up_and_is_then_held_by_the_rails():
    sim, lay = lined_up_sim(dy=0.03, dyaw_deg=-2.0)
    drive(sim, Command(vx=0.3), 8)
    assert sim.rail == 1 and not sim.crashed
    assert sim.y == lay.lane_center_y(1) and sim.yaw == 0.0  # the rails set y and heading
    drive(sim, Command(vx=0.3, vy=0.5, yaw_rate=0.8), 5)  # no steering, no sideways on the rails
    assert sim.y == lay.lane_center_y(1) and sim.yaw == 0.0 and sim.state().on_rail == 1


def test_badly_lined_up_hits_the_rail_start():
    sim, _ = lined_up_sim(dy=0.12)
    drive(sim, Command(vx=0.3), 8)
    assert sim.crashed and "lined up" in sim.crash_reason


def test_reversing_off_the_rail_start_puts_it_back_on_the_concrete():
    sim, _ = lined_up_sim()
    drive(sim, Command(vx=0.3), 8)
    assert sim.rail == 1
    drive(sim, Command(vx=-0.3), 10)
    assert sim.rail is None and not sim.crashed and sim.state().on_rail is None


def test_runs_off_the_end_of_the_rails():
    sim, lay = lined_up_sim()
    drive(sim, Command(vx=0.3), 8)
    drive(sim, Command(vx=1.0), 30)
    assert sim.crashed and "end of the rails" in sim.crash_reason


def test_on_the_rails_the_estimate_needs_no_visual_odometry():
    sim, lay = lined_up_sim()
    sim.cfg.vo_dropouts = [(0.0, 1e9)]
    drive(sim, Command(vx=0.3), 8)
    drive(sim, Command(vx=0.5), 10)
    s = sim.state()
    assert not s.vo_healthy and s.on_rail == 1
    assert abs(s.x - sim.x) < 0.02 and s.y == lay.lane_center_y(1)  # wheel odometry along, rails across


def test_drive_to_turns_on_the_spot_then_drives_and_reverses_short_hops():
    cmd, done = drive_to(0, 0, 0, 0, 2, None)
    assert cmd.vx == 0 and cmd.yaw_rate > 0 and not done  # target to the left: turn first
    cmd, _ = drive_to(0, 0, 0, -0.4, 0, None)
    assert cmd.vx < 0 and cmd.yaw_rate == 0  # just behind: back straight up
    assert drive_to(0, 0, 0, 0.02, 0, None)[1]


def test_obstacle_ahead_looks_only_within_the_trolleys_width():
    scan = np.array([[1.0, 0.7], [2.0, 0.1], [-1.0, 0.0]])
    assert obstacle_ahead(scan) == 2.0
    assert obstacle_ahead(np.empty((0, 2))) == math.inf


# ---- missions -----------------------------------------------------------------------------------


@pytest.mark.parametrize("closed", [True, False])
def test_sweeps_every_lane_and_docks(closed):
    lay = GreenhouseLayout(closed_end=closed)
    res, events = trolley_run(layout=lay)
    assert res.outcome == "completed"
    assert res.lanes_completed == list(range(lay.n_lanes))
    assert at_dock(res, lay)
    assert len(logs(events, "on the rails of lane")) == lay.n_lanes
    assert not logs(events, "backing up")  # lined up first time, every lane


def test_never_drives_between_the_crop_rows_off_the_rails():
    res, _ = trolley_run()
    lay = GreenhouseLayout()
    between_rows = [r for r in res.trace if r["x"] > 0.0]
    assert between_rows
    assert all(abs(r["y"] - lay.lane_center_y(lay.nearest_lane(r["y"]))) < 1e-9 for r in between_rows)


def test_several_seeds_complete():
    for seed in range(1, 6):
        res, _ = trolley_run(sim=SimConfig(seed=seed))
        assert res.outcome == "completed", f"seed {seed}: {res.outcome}"


def test_waits_for_a_worker_in_the_lane_then_carries_on():
    res, events = trolley_run(trolley=TrolleyConfig(obstacles=[(7.0, 4.0, 0.25, 0.0, 150.0)]))
    assert res.outcome == "completed" and len(res.lanes_completed) == 6
    assert logs(events, "waiting") and logs(events, "clear after")
    assert min(r["x"] for r in res.trace if abs(r["y"] - 4.0) < 0.01 and 100 < r["t"] < 150) < 7.0 - 0.25 - 0.75


def test_a_blocked_lane_reverses_out_and_docks():
    res, events = trolley_run(trolley=TrolleyConfig(obstacles=[(9.0, 2.0, 0.25, 0.0, 1e9)]))
    assert res.outcome.startswith("aborted: lane blocked")
    assert res.lanes_completed == [0]
    assert logs(events, "docking") and at_dock(res)


def test_returns_to_the_dock_on_a_low_battery():
    res, events = trolley_run(trolley=TrolleyConfig(move_drain_pct_per_m=0.5))
    assert res.outcome == "aborted: battery low"
    assert 0 < len(res.lanes_completed) < 6 and at_dock(res)
    assert res.battery_end_pct > 12


def test_short_loss_of_visual_odometry_on_the_concrete_pauses():
    res, events = trolley_run(sim=SimConfig(vo_dropouts=[(50.0, 1.5)]))
    assert res.outcome == "completed" and logs(events, "stopped, waiting")


def test_long_loss_of_visual_odometry_on_the_concrete_stops_it():
    res, events = trolley_run(sim=SimConfig(vo_dropouts=[(50.0, 10.0)]))
    assert res.outcome == "aborted: visual odometry lost"
    assert logs(events, "motors off")


def test_loss_of_visual_odometry_on_the_rails_does_not_matter():
    res, events = trolley_run(sim=SimConfig(vo_dropouts=[(12.0, 12.0)]))  # lane 1, on the rails
    assert res.outcome == "completed" and not logs(events, "waiting")


def test_refuses_to_start_on_a_low_battery():
    res, _ = trolley_run(sim=SimConfig(start_battery_pct=30))
    assert res.outcome.startswith("aborted: battery") and res.lanes_completed == []
