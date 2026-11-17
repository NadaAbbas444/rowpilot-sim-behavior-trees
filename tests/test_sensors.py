import math

import numpy as np
import pytest

from rowpilot.perception import LaneEstimate, LaneFilter, estimate_lane
from rowpilot.sensors import D435i, _rot, depth_to_lane_scan, gh_x500_sdf


def synthetic_depth(cam, altitude, lane_y=0.0, half_width=0.8, yaw=0.0):
    """Depth image of a lane between two vertical walls (y = lane_y +- half_width) over a floor."""
    v, u = np.mgrid[0 : cam.height, 0 : cam.width]
    ray_cam = np.stack([np.ones_like(u, float), -(u - cam.cx) / cam.fx, -(v - cam.cy) / cam.fx])
    x, y, z, r, p, yw = cam.pose
    rb = np.einsum("ij,jhw->ihw", _rot(r, p, yw), ray_cam)
    rw = np.einsum("ij,jhw->ihw", _rot(0, 0, yaw), rb)
    origin = np.array([0, 0, altitude]) + _rot(0, 0, yaw) @ np.array([x, y, z])
    t = np.full(u.shape, np.inf)
    for wall in (lane_y + half_width, lane_y - half_width):
        with np.errstate(divide="ignore", invalid="ignore"):
            tw = (wall - origin[1]) / rw[1]
        t = np.where((tw > 0) & (tw < t), tw, t)
    with np.errstate(divide="ignore", invalid="ignore"):
        tf = -origin[2] / rw[2]
    t = np.where((tf > 0) & (tf < t), tf, t)
    return (t * ray_cam[0]).astype(np.float32)  # Gazebo depth: distance along the optical axis


@pytest.mark.parametrize("dy,yaw", [(0.0, 0.0), (0.3, 0.0), (-0.25, 0.1), (0.1, -0.12)])
def test_depth_image_to_lane_estimate(dy, yaw):
    cam = D435i().depth()
    depth = synthetic_depth(cam, altitude=1.5, lane_y=-dy, yaw=yaw)  # drone dy left of the centre
    pts = depth_to_lane_scan(depth, cam, altitude=1.5)
    assert len(pts) > 20
    est = estimate_lane(pts, 1.6)
    assert est.valid
    assert est.lateral_offset == pytest.approx(-dy * math.cos(yaw), abs=0.04)
    assert est.heading_error == pytest.approx(-yaw, abs=math.radians(2))


def test_floor_is_not_a_lane_wall():
    cam = D435i().depth()
    depth = synthetic_depth(cam, altitude=1.5, half_width=50.0)  # floor only, walls far away
    assert len(depth_to_lane_scan(depth, cam, altitude=1.5)) == 0


def test_lane_filter_smooths_heading_not_offset_and_passes_gaps():
    f = LaneFilter()
    f.update(LaneEstimate(0.0, 0.0, valid=True), 0.0)
    out = f.update(LaneEstimate(0.2, 0.2, valid=True), 0.1)
    assert out.lateral_offset == pytest.approx(0.2)  # offset is not smoothed (fast in gusts)
    assert 0 < out.heading_error < 0.1  # heading is
    assert not f.update(LaneEstimate(), 0.2).valid  # no estimate is passed through, never a stale one
    restarted = f.update(LaneEstimate(0.0, -0.3, valid=True), 2.0)  # after a long gap: restart
    assert restarted.heading_error == pytest.approx(-0.3)


def test_drone_model_has_the_real_sensors():
    sdf = gh_x500_sdf()
    for name in ("infra1", "infra2", "depth", "picam", "d435i_imu"):
        assert f'name="{name}"' in sdf
    assert 'relative_to="base_link"' in sdf  # sensors ride on the body, not at the model origin
