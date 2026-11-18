import math
import re

import numpy as np
import pytest

from rowpilot.airframe import FIREFLY_RIG, MAX_ROTOR_SPEED, VEHICLES, FireFly, firefly_sdf, gh_firefly_sdf
from rowpilot.cad_import import read_stl, simplify, write_stl


def test_firefly_has_four_rotors_driven_by_ardupilot():
    sdf = firefly_sdf()
    assert sdf.count("gz::sim::systems::LiftDrag") == 8  # two blades per rotor
    assert sdf.count("ApplyJointForce") == 4
    channels = sorted(int(c) for c in re.findall(r'<control channel="(\d)">', sdf))
    assert channels == [0, 1, 2, 3]
    assert "<imuName>imu_link::imu_sensor</imuName>" in sdf


def test_quad_x_motor_order_and_spin():
    """ArduCopter quad X: 1 front right CCW, 2 back left CCW, 3 front left CW, 4 back right CW."""
    by_channel = {r.channel: r for r in FireFly().rotors}
    expect = {0: (1, -1, True), 1: (-1, 1, True), 2: (1, 1, False), 3: (-1, -1, False)}
    for ch, (sx, sy, ccw) in expect.items():
        r = by_channel[ch]
        assert (math.copysign(1, r.xyz[0]), math.copysign(1, r.xyz[1]), r.ccw) == (sx, sy, ccw)


def test_rotor_thrust_and_hover():
    ff = FireFly()
    v = MAX_ROTOR_SPEED * ff.blade_cp
    full = 2 * 0.5 * 1.2041 * v * v * ff.blade_area * ff.blade_cl
    assert full == pytest.approx(ff.max_thrust_n)
    assert 4 * ff.max_thrust_n / (ff.total_mass * 9.8) > 3  # thrust / weight
    assert 0.3 * MAX_ROTOR_SPEED < ff.hover_speed() < 0.7 * MAX_ROTOR_SPEED


def test_firefly_with_sensors():
    sdf = gh_firefly_sdf()
    for name in ("infra1", "infra2", "depth", "picam", "d435i_imu"):
        assert f'name="{name}"' in sdf
    assert "<uri>model://firefly</uri>" in sdf
    assert FIREFLY_RIG.picam.pose[1] > 0  # Pi camera on the left
    assert FIREFLY_RIG.d435i.tilt_deg == 15.0
    assert VEHICLES["firefly"].model == "gh_firefly"


def test_simplify_keeps_the_shape(tmp_path):
    # a finely tessellated 10 cm square plate (2 x 400 x 400 triangles)
    n, s = 400, 0.1
    g = np.linspace(0, s, n + 1)
    tris = []
    for i in range(n):
        for j in range(n):
            a, b, c, d = (g[i], g[j]), (g[i + 1], g[j]), (g[i + 1], g[j + 1]), (g[i], g[j + 1])
            tris += [[(*a, 0), (*b, 0), (*c, 0)], [(*a, 0), (*c, 0), (*d, 0)]]
    tris = np.array(tris)
    small = simplify(tris, 0.005)
    assert len(small) < len(tris) / 50
    lo, hi = small.reshape(-1, 3).min(0), small.reshape(-1, 3).max(0)
    assert np.allclose(lo[:2], 0, atol=0.003) and np.allclose(hi[:2], s, atol=0.003)
    write_stl(tmp_path / "plate.stl", small)
    assert np.allclose(read_stl(tmp_path / "plate.stl"), small, atol=1e-6)


def test_depth_view_colours_near_blue_far_red_and_missing_black():
    from rowpilot.depth_view import colourise

    d = np.array([[0.3, 6.0, np.inf, 0.0]], dtype=np.float32)
    rgb = colourise(d)
    assert rgb.shape == (1, 4, 3) and rgb.dtype == np.uint8
    assert rgb[0, 0, 2] == 255 and rgb[0, 0, 0] == 0  # near: blue
    assert rgb[0, 1, 0] == 255 and rgb[0, 1, 2] == 0  # far: red
    assert (rgb[0, 2:] == 0).all()  # no depth: black
