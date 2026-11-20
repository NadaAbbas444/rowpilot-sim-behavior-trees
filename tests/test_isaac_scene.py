import math
from pathlib import Path

import numpy as np

from rowpilot.gazebo_world import make_world
from rowpilot.greenhouse_assets import tomato_plant_mesh
from rowpilot.isaac_scene import (
    camera_apertures,
    camera_matrix,
    mat_to_usd,
    pose_matrix,
    read_dae,
    read_world,
    resolve_uri,
)
from rowpilot.layout import GreenhouseLayout
from rowpilot.sensors import D435i

TEMPLATE = (Path(__file__).parent.parent / "ardupilot" / "world_template.sdf").read_text()


def test_reads_the_generated_greenhouse_back():
    layout = GreenhouseLayout(n_lanes=3)
    w = read_world(make_world(layout, TEMPLATE, vehicle="model://gh_firefly"))
    assert w.name == "greenhouse"
    ((uri, T),) = w.includes  # the drone is Isaac's to build, at the pose the world gives it
    assert uri == "model://gh_firefly" and np.allclose(T[:2, 3], layout.home)
    rows = [s for s in w.colliders() if s.model.startswith("crop_row_")]
    assert len(rows) == layout.n_lanes + 1
    for s, cy in zip(sorted(rows, key=lambda s: s.T[1, 3]), layout.crop_row_centers_y(), strict=True):
        assert s.kind == "box"
        assert np.allclose(s.T[:3, 3], [layout.row_length / 2, cy, layout.canopy_height / 2])
        assert np.allclose(s.size, [layout.row_length, layout.crop_width, layout.canopy_height], atol=1e-4)
    pipes = [s for s in w.visuals() if s.model.startswith("pipe_")]
    assert len(pipes) == 2 * layout.n_lanes and pipes[0].kind == "cylinder"
    assert np.allclose(pipes[0].T[:3, :3] @ [0, 0, 1], [1, 0, 0], atol=1e-4)  # along the rows (4-decimal pose)
    assert any(s.model == "ground_plane" and s.kind == "plane" for s in w.colliders())
    assert w.sun_direction == (-0.5, 0.1, -0.9)


def test_textured_world_has_plant_meshes_and_floor_texture():
    w = read_world(make_world(GreenhouseLayout(n_lanes=1), TEMPLATE, plants=["tomato_0.dae", "tomato_1.dae"]))
    plants = [s for s in w.visuals() if s.kind == "mesh"]
    assert plants and plants[0].uri.endswith(".dae") and plants[0].scale[1] < plants[0].scale[0]
    assert any(s.texture.endswith("floor.png") for s in w.visuals())


def test_reads_our_collada_plants():
    mesh = tomato_plant_mesh(0)
    parts = read_dae(mesh.dae())
    assert {p.material for p in parts} == {k for k, v in mesh.groups.items() if v}
    n = sum(len(p.triangles) for p in parts)
    assert n == sum(len(v) for v in mesh.groups.values())
    for p in parts:
        assert p.triangles.max() < len(p.points)
    assert max(p.points[:, 2].max() for p in parts) > 2.0  # a 2.6 m plant


def test_pose_and_usd_matrix_conventions():
    T = pose_matrix("1 2 3 0 0 1.5707963")
    assert np.allclose(T[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-6)
    M = np.array(mat_to_usd(T))
    assert np.allclose(np.array([1, 0, 0, 1]) @ M, [1, 3, 3, 1], atol=1e-6)  # row vector times USD matrix


def test_usd_camera_looks_along_the_tilted_optical_axis():
    cam = D435i(tilt_deg=15.0).ir("left")
    T = camera_matrix(cam.pose)
    look = T[:3, :3] @ [0, 0, -1]
    up = T[:3, :3] @ [0, 1, 0]
    assert np.allclose(look, [math.cos(cam.pose[4]), 0, -math.sin(cam.pose[4])], atol=1e-9)
    assert up[2] > 0.9
    assert np.allclose(T[:3, 3], cam.pose[:3])
    h, v = camera_apertures(640, 480, 87.0, focal_mm=1.0)
    assert math.isclose(2 * math.degrees(math.atan(h / 2)), 87.0) and math.isclose(v / h, 0.75)


def test_model_uris_resolve_against_the_resource_folders(tmp_path):
    (tmp_path / "greenhouse_assets" / "meshes").mkdir(parents=True)
    f = tmp_path / "greenhouse_assets" / "meshes" / "floor.png"
    f.write_bytes(b"")
    assert resolve_uri("model://greenhouse_assets/meshes/floor.png", [tmp_path / "x", tmp_path]) == f
    assert resolve_uri("model://nothing/here", [tmp_path]) is None
