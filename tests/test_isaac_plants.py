import numpy as np

from rowpilot.gazebo_world import make_world
from rowpilot.greenhouse_assets import tomato_plant_mesh, tomato_plant_parts, write_plant_model
from rowpilot.isaac_plants import (
    detailed_plant,
    leaf_texture,
    leaflet,
    normal_map,
    read_plant_index,
    write_textures,
)
from rowpilot.isaac_scene import look_for, read_world
from rowpilot.layout import GreenhouseLayout
from tests.test_isaac_scene import TEMPLATE


def test_detailed_plant_has_the_gazebo_plants_parts_in_the_same_places():
    parts = tomato_plant_parts(3)
    plant = detailed_plant(3)
    n_leaflets = sum(p[0] == "leaflet" for p in parts)
    leaf_tris = sum(len(t) for k, m in plant.items() if k.startswith("leaf") for t in m.tris)
    assert leaf_tris == n_leaflets * 16  # a 5 x 3 card each
    gz = np.asarray(tomato_plant_mesh(3).v)
    pts = np.concatenate([m.arrays()[0] for m in plant.values()])
    assert np.allclose(pts.min(0), gz.min(0), atol=0.08) and np.allclose(pts.max(0), gz.max(0), atol=0.08)
    for m in plant.values():
        p, n, uv, tris = m.arrays()
        assert len(p) == len(n) == len(uv) and tris.max() < len(p)
        assert np.allclose(np.linalg.norm(n, axis=1), 1.0, atol=1e-4)


def test_leaflet_card_faces_up_and_droops_to_the_tip():
    p, n, uv, _ = leaflet(np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), 0.2, 0.1)
    assert (n[:, 2] > 0).all()
    base, tip = p[uv[:, 0] == 0], p[uv[:, 0] == 1]
    assert tip[:, 0].mean() > 0.19 and tip[:, 2].mean() < base[:, 2].mean() < 0.02


def test_leaf_texture_is_a_leaflet_cut_out_by_alpha():
    rgba, h = leaf_texture()
    alpha = rgba[..., 3]
    assert set(np.unique(alpha)) <= {0, 255}
    assert 0.35 < (alpha > 0).mean() < 0.75  # a leaflet, not the whole card
    assert alpha[128, 256] == 255 and alpha[0, 0] == 0 and alpha[255, 0] == 0  # the middle in, the corners out
    assert (alpha[:, 511] > 0).mean() < 0.05  # a pointed tip
    nm = normal_map(h)
    assert nm.shape == (*h.shape, 3) and nm[..., 2].min() > 128  # normals point out of the surface


def test_textures_and_plant_index_are_written(tmp_path):
    names = write_plant_model(tmp_path / "assets", variants=2, height=2.4, seed=1)
    idx = read_plant_index(tmp_path / "assets" / "meshes")
    assert idx == {names[0]: {"seed": 100, "height": 2.4}, names[1]: {"seed": 101, "height": 2.4}}
    files = write_textures(tmp_path / "tex", tmp_path / "assets" / "meshes" / "floor.png")
    assert {"leaf", "leaf_normal", "floor_normal"} <= set(files) and all(f.exists() for f in files.values())


def test_every_greenhouse_part_gets_an_isaac_material():
    w = read_world(make_world(GreenhouseLayout(n_lanes=2), TEMPLATE, plants=["tomato_0.dae"]))
    looks = {s.model.split("_")[0]: look_for(s, "floor_normal.png") for s in w.visuals() if s.kind != "mesh"}
    assert looks["pipe"].get("metallic_constant") == 1.0
    assert looks["post"].get("metallic_constant", 0) > 0.5
    assert looks["floor"].get("normalmap_texture") == "floor_normal.png"
    walls = {look_for(s).name for s in w.visuals() if s.model == "walls"}
    assert "greenhouse_glass" in walls and "concrete" in walls
    glass = next(look_for(s) for s in w.visuals() if s.model == "walls" and look_for(s).module == "OmniGlass")
    assert glass.get("thin_walled") is True
