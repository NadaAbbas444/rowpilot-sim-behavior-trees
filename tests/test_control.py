import math

import pytest

from ghdrone.control import LaneFollower, goto_command, wrap_angle
from ghdrone.perception import LaneEstimate


def test_wrap_angle():
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(-0.1) == pytest.approx(-0.1)


def test_lane_follower_steers_towards_centre_and_aligns():
    cmd = LaneFollower().step(LaneEstimate(0.3, 0.1, valid=True), 0.8, 0.1)
    assert cmd.vy > 0  # centre is to the left -> move left
    assert cmd.yaw_rate > 0  # lane runs to the left of the nose -> yaw left
    assert 0 < cmd.vx < 0.8  # slows down while off-centre


def test_goto_arrives_and_turns_to_final_yaw():
    cmd, arrived = goto_command(0, 0, 0.0, 0.05, 0, final_yaw=math.pi / 2)
    assert not arrived and cmd.yaw_rate > 0
    _, arrived = goto_command(0, 0, math.pi / 2, 0.05, 0, final_yaw=math.pi / 2)
    assert arrived


def test_goto_turns_before_long_leg():
    cmd, _ = goto_command(0, 0, 0.0, 0, 10, final_yaw=None)
    assert cmd.vx == pytest.approx(0) and cmd.vy == pytest.approx(0) and cmd.yaw_rate > 0


def test_goto_corrects_cross_track_on_long_leg():
    # Flying +x along y=0 from (0,0) to (40,0), drifted 0.4 m left: must push right (negative vy).
    cmd, _ = goto_command(10, 0.4, 0.0, 40, 0, None, start=(0, 0))
    assert cmd.vy < -0.3


def test_goto_backs_up_after_overshoot():
    cmd, arrived = goto_command(0.5, 0, 0.0, 0, 0, None, start=(-5, 0))
    assert not arrived and cmd.vx < 0
