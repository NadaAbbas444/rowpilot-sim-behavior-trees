import math

import numpy as np

from rowpilot.imu_synth import GRAVITY, AccelNoise, PoseBuffer, specific_force
from rowpilot.sensors import D435i, _rot


def test_still_and_tilted_measures_gravity_in_its_own_frame():
    buf = PoseBuffer()
    R = _rot(0.0, math.radians(15), 0.0)  # tilted down like the D435i
    for k in range(100):
        buf.add(k * 0.001, np.array([1.0, 2.0, 1.5]), R)
    f = specific_force(buf, 0.05)
    assert np.allclose(f, R.T @ -GRAVITY, atol=1e-9)
    assert f[0] < -2.4  # gravity shows on the tilted x axis, as Gazebo's IMU does at rest


def test_circle_while_yawing_gives_centripetal_acceleration():
    """A sensor 0.2 m from the yaw axis of a drone turning at 0.8 rad/s on the spot."""
    w, r = 0.8, 0.2
    buf = PoseBuffer()
    for k in range(400):
        t = k * 0.001
        yaw = w * t
        R = _rot(0.0, 0.0, yaw)
        buf.add(t, R @ np.array([r, 0.0, 0.0]), R)
    f = specific_force(buf, 0.2)
    assert np.allclose(f, [-w * w * r, 0.0, 9.8], atol=1e-4)  # towards the axis, no sideways term


def test_outside_the_buffer_is_none():
    buf = PoseBuffer()
    buf.add(0.0, np.zeros(3), np.eye(3))
    buf.add(0.001, np.zeros(3), np.eye(3))
    assert specific_force(buf, 0.0005) is None  # needs poses 5 ms either side


def test_noise_has_the_configured_density():
    d = D435i()
    n = AccelNoise(d, seed=1)
    dt = 1 / d.imu_rate
    s = np.array([n(np.zeros(3), dt) for _ in range(4000)])
    assert abs(s[:, 0].std() - d.accel_noise_density / math.sqrt(dt)) < 0.004
