import xml.etree.ElementTree as ET

import pytest

from rowpilot.gazebo_world import PLANT_SPACING_M, make_world, spawn_pose
from rowpilot.greenhouse_assets import tomato_plant_mesh, write_plant_model
from rowpilot.layout import GreenhouseLayout

TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode"><max_step_size>0.004</max_step_size></physics>
    <model name="ground_plane"><static>true</static></model>
  </world>
</sdf>
"""
LAY = GreenhouseLayout(n_lanes=4)


def models(world, prefix):
    return [m for m in world.findall("model") if m.get("name").startswith(prefix)]


@pytest.mark.parametrize("plants", [None, ["tomato_0.dae", "tomato_1.dae"]])
def test_crop_rows_match_the_layout(plants):
    world = ET.fromstring(make_world(LAY, TEMPLATE, plants=plants)).find("world")
    assert world.get("name") == "greenhouse"
    assert world.find("physics") is not None  # PX4's physics and plugins are kept
    rows = models(world, "crop_row_")
    assert len(rows) == LAY.n_lanes + 1
    for m, cy in zip(rows, LAY.crop_row_centers_y(), strict=True):
        assert float(m.find("pose").text.split()[1]) == pytest.approx(cy, abs=1e-3)
        size = list(map(float, m.find("link/collision/geometry/box/size").text.split()))
        assert size == pytest.approx([LAY.row_length, LAY.crop_width, LAY.canopy_height], abs=1e-3)
        visuals = m.findall("link/visual")
        if plants:
            assert len(visuals) == pytest.approx(LAY.row_length / PLANT_SPACING_M, abs=1)
            assert all(v.find("geometry/mesh/uri").text.endswith(tuple(plants)) for v in visuals)
        else:
            assert len(visuals) == 1


def test_pipe_rails_in_every_lane_and_plants_stay_in_their_row():
    world = ET.fromstring(make_world(LAY, TEMPLATE, plants=["p.dae"])).find("world")
    assert len(models(world, "pipe_")) == 2 * LAY.n_lanes
    for row in models(world, "crop_row_"):
        for v in row.findall("link/visual"):
            x, y = map(float, v.find("pose").text.split()[:2])
            assert 0 < x < LAY.row_length and abs(y) < LAY.crop_width / 2


def test_plant_mesh_is_a_plausible_tomato_plant():
    mesh = tomato_plant_mesh(seed=3, height=2.6)
    v = mesh.v
    zs = [p[2] for p in v]
    assert max(zs) == pytest.approx(2.6, abs=0.15)
    assert {"stem", "fruit_red"} <= set(mesh.groups)
    reach = max((p[0] ** 2 + p[1] ** 2) ** 0.5 for p in v)
    assert 0.3 < reach < 0.75
    leaf_points = [p for p in v if p[2] < 0.5]
    assert all((p[0] ** 2 + p[1] ** 2) ** 0.5 < 0.05 for p in leaf_points)  # bottom is de-leafed


def test_write_plant_model(tmp_path):
    names = write_plant_model(tmp_path / "greenhouse_assets", variants=2)
    assert names == ["tomato_0.dae", "tomato_1.dae"]
    assert (tmp_path / "greenhouse_assets/meshes/floor.png").stat().st_size > 100_000
    dae = ET.parse(tmp_path / "greenhouse_assets/meshes/tomato_0.dae").getroot()  # valid XML
    ns = "{http://www.collada.org/2005/11/COLLADASchema}"
    colours = {m.get("id") for m in dae.iter(ns + "material")}
    assert {"stem-mat", "fruit_red-mat"} <= colours


def test_rejects_non_world_template():
    with pytest.raises(ValueError):
        make_world(GreenhouseLayout(), "<sdf/>")


def test_spawn_pose_is_home():
    assert spawn_pose(GreenhouseLayout()) == "-2.5,0.0,0.25,0,0,0"
