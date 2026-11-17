import xml.etree.ElementTree as ET

import pytest

from rowpilot.gazebo_world import make_world, spawn_pose
from rowpilot.layout import GreenhouseLayout

TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode"><max_step_size>0.004</max_step_size></physics>
    <model name="ground_plane"><static>true</static></model>
  </world>
</sdf>
"""


def test_world_has_one_box_per_crop_row_matching_the_layout():
    lay = GreenhouseLayout(n_lanes=4, row_length=30.0)
    world = ET.fromstring(make_world(lay, TEMPLATE)).find("world")
    assert world.get("name") == "greenhouse"
    assert world.find("physics") is not None  # PX4's plugins and physics are kept
    rows = [m for m in world.findall("model") if m.get("name").startswith("crop_row_")]
    assert len(rows) == lay.n_lanes + 1
    for m, cy in zip(rows, lay.crop_row_centers_y(), strict=True):
        x, y, z = map(float, m.find("pose").text.split()[:3])
        size = list(map(float, m.find("link/collision/geometry/box/size").text.split()))
        assert (x, y, z) == pytest.approx((lay.row_length / 2, cy, lay.canopy_height / 2), abs=1e-3)
        assert size == pytest.approx([lay.row_length, lay.crop_width, lay.canopy_height], abs=1e-3)


def test_rejects_non_world_template():
    with pytest.raises(ValueError):
        make_world(GreenhouseLayout(), "<sdf/>")


def test_spawn_pose_is_home():
    assert spawn_pose(GreenhouseLayout()) == "-2.5,0.0,0.25,0,0,0"
