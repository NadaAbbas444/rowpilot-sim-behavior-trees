import math

import numpy as np
import pytest

from rowpilot.layout import GreenhouseLayout
from rowpilot.perception import estimate_lane
from rowpilot.sim import SimConfig, SimVehicle

LAY = GreenhouseLayout()
OPEN = GreenhouseLayout(closed_end=False)  # lanes open at both ends


def scan_at(x, y, yaw, noise=0.01, seed=0, foliage=0.0, layout=LAY):
    """Geometry tests use clean crop faces (foliage=0); the foliage tests below use leafy ones."""
    v = SimVehicle(layout, SimConfig(depth_noise_m=noise, seed=seed, foliage_m=foliage, depth_dropout=0.0))
    v.x, v.y, v.yaw, v.z = x, y, yaw, 1.5
    return v.depth_scan()


def true_offset(lane, y, yaw):
    """Perpendicular distance to the lane centre, positive when the centre is on the drone's left."""
    return (LAY.lane_center_y(lane) - y) * math.copysign(1, math.cos(yaw))


@pytest.mark.parametrize("lane", [0, 1, 3])
@pytest.mark.parametrize("dy", [-0.4, -0.1, 0.0, 0.25])
@pytest.mark.parametrize("dyaw", [-0.2, 0.0, 0.15])
def test_mid_lane_estimate_matches_truth(lane, dy, dyaw):
    yaw = LAY.lane_yaw(lane) + dyaw
    y = LAY.lane_center_y(lane) + dy
    est = estimate_lane(scan_at(LAY.row_length / 2, y, yaw), LAY.path_width)
    assert est.valid
    assert est.lateral_offset == pytest.approx(true_offset(lane, y, yaw), abs=0.04)
    assert est.heading_error == pytest.approx(-dyaw, abs=0.03)


def test_both_faces_seen_in_mid_lane():
    est = estimate_lane(scan_at(LAY.row_length / 2, 0.0, 0.0), LAY.path_width)
    assert est.n_left > 10 and est.n_right > 10


@pytest.mark.parametrize("x,lane", [(-1.5, 0), (OPEN.row_length + 1.5, 1)])
def test_lane_entry_from_headland(x, lane):
    """At a lane entry, the end faces and other rows are visible too; the estimate must still be right.
    (Entering from the far headland only happens with open lanes.)"""
    yaw = OPEN.lane_yaw(lane) + 0.05
    y = OPEN.lane_center_y(lane) + 0.1
    est = estimate_lane(scan_at(x, y, yaw, layout=OPEN), OPEN.path_width)
    assert est.valid
    assert est.lateral_offset == pytest.approx(true_offset(lane, y, yaw), abs=0.05)


@pytest.mark.parametrize("seed", range(5))
@pytest.mark.parametrize(
    "x,y,yaw",
    [(-1.5, 0.3, 0.25), (-1.0, -0.4, -0.3), (LAY.row_length - 0.5, 0.1, 0.0), (LAY.row_length / 2, 0.4, -0.35)],
)
def test_never_confidently_wrong(seed, x, y, yaw):
    """Hard poses may give no estimate, but a valid estimate has to be close to the truth."""
    est = estimate_lane(scan_at(x, y, yaw, seed=seed), LAY.path_width)
    if est.valid:
        assert est.lateral_offset == pytest.approx(true_offset(0, y, yaw), abs=0.08)


def test_empty_scan_is_invalid():
    assert not estimate_lane(np.empty((0, 2)), LAY.path_width).valid
    assert not estimate_lane(None, LAY.path_width).valid


def test_leafy_rows_valid_and_close_to_truth():
    """Leaves stick into the lane by up to 14 cm, unevenly along the row. Single frames are noisier
    than with clean faces, but the estimate must stay valid and unbiased."""
    errs, valid, n = [], 0, 0
    for x in np.linspace(1, LAY.row_length - 1, 25):
        for dy in (-0.3, 0.0, 0.3):
            for dyaw in (-0.15, 0.0, 0.15):
                n += 1
                yaw = LAY.lane_yaw(0) + dyaw
                est = estimate_lane(scan_at(x, dy, yaw, seed=int(x * 10), foliage=0.14), LAY.path_width)
                if est.valid:
                    valid += 1
                    errs.append(est.lateral_offset - true_offset(0, dy, yaw))
    errs = np.asarray(errs)
    assert valid / n > 0.95
    assert abs(errs.mean()) < 0.03  # unbiased
    assert np.percentile(np.abs(errs), 95) < 0.16
