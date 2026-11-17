import math

import numpy as np
import pytest

from rowpilot.sensors import D435i, _rot
from rowpilot.vio_link import VioAligner, inv_T, make_T, quat_to_rot, rot_to_quat


def imu_pose_in_G(T_M_G, body_xyz, body_yaw, d=D435i()):
    """Where OpenVINS would report the IMU, for a body pose in the map, given a hidden M<-G transform."""
    T_M_B = make_T(_rot(0, 0, body_yaw), body_xyz)
    T_B_I = make_T(_rot(0, d.tilt, 0), [d.mount_x, 0, d.mount_z])
    T_G_I = inv_T(T_M_G) @ T_M_B @ T_B_I
    return T_G_I[:3, 3], rot_to_quat(T_G_I[:3, :3])


def test_quaternion_round_trip():
    R = _rot(0.1, -0.4, 2.5)
    assert np.allclose(quat_to_rot(*rot_to_quat(R)), R)


@pytest.mark.parametrize("g_yaw", [0.0, math.pi, -2.0])  # OpenVINS picks its global heading arbitrarily
def test_aligner_recovers_body_pose_in_map(g_yaw):
    T_M_G = make_T(_rot(0, 0, g_yaw), [3.0, -1.0, 0.5])
    home = (-2.5, 0.0)
    al = VioAligner(home, home_z=0.195)
    p, q = imu_pose_in_G(T_M_G, [home[0], home[1], 0.195], 0.0)
    first = al.update(0.0, p, q, (0, 0, 0))
    assert first.p == pytest.approx([home[0], home[1], 0.195], abs=1e-9)
    p, q = imu_pose_in_G(T_M_G, [4.0, 2.0, 1.5], math.pi / 2)
    later = al.update(10.0, p, q, (0, 0, 0))
    assert later.p == pytest.approx([4.0, 2.0, 1.5], abs=1e-9)
    assert later.yaw == pytest.approx(math.pi / 2, abs=1e-9)
