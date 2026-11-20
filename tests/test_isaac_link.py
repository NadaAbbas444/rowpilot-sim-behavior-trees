import json
import math

import numpy as np
import pytest

from rowpilot.airframe import MAX_ROTOR_SPEED, FireFly
from rowpilot.imu_synth import GRAVITY
from rowpilot.isaac_link import (
    FrameReader,
    ImuSampler,
    RotorThrust,
    body_specific_force,
    enu_flu_to_ned_frd,
    fdm_reply,
    image_frame,
    pack_frame,
    parse_servo_packet,
    servo_packet,
    to_gray,
)
from rowpilot.sensors import _rot
from rowpilot.vio_link import quat_to_rot


def test_servo_packet_round_trip_and_rejects_others():
    pkt = parse_servo_packet(servo_packet(1000, 42, [1500, 1600, 1700, 1800]))
    assert pkt.frame_rate == 1000 and pkt.frame_count == 42
    assert pkt.pwm[:5] == (1500, 1600, 1700, 1800, 0)
    assert parse_servo_packet(b"\x00" * 40) is None
    assert parse_servo_packet(b"hello") is None


def test_level_drone_facing_along_the_rows_faces_east_in_ned():
    R = enu_flu_to_ned_frd(np.eye(3))
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])  # body forward = east (map x)
    assert np.allclose(R @ [0, 0, 1], [0, 0, 1])  # body down = down
    assert np.allclose(R @ [0, 1, 0], [-1, 0, 0])  # body right = south (map -y)


def test_fdm_reply_at_rest_is_what_arducopter_expects():
    msg = json.loads(
        fdm_reply(
            1.5, np.array([2.0, 3.0, 1.0]), np.eye(3), np.array([0.1, 0.2, 0.3]), np.array([0.0, 0.0, 0.5]), -GRAVITY
        )
    )
    assert msg["timestamp"] == 1.5
    assert msg["position"] == [3.0, 2.0, -1.0]  # north = map y, east = map x, down = -z
    assert msg["velocity"] == [0.2, 0.1, -0.3]
    assert msg["imu"]["accel_body"] == [0.0, 0.0, -9.8]  # FRD specific force at rest
    assert msg["imu"]["gyro"] == [0.0, 0.0, -0.5]  # yaw left (FLU) = negative yaw rate in FRD
    w, x, y, z = msg["quaternion"]
    assert np.allclose(quat_to_rot(w, x, y, z), enu_flu_to_ned_frd(np.eye(3)), atol=1e-6)
    yaw_ned = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    assert math.isclose(yaw_ned, math.pi / 2, abs_tol=1e-6)  # yaw_NED = 90 deg - yaw_map


def test_rotors_hover_at_the_hover_speed_and_balance():
    ff = FireFly()
    r = RotorThrust(ff, tau_s=0.0, slew=0)
    pwm_hover = 1100 + 800 * ff.hover_speed() / MAX_ROTOR_SPEED
    f, tq = r.step([pwm_hover] * 4, 0.001)
    assert math.isclose(f[2], ff.total_mass * 9.8, rel_tol=1e-6)
    # Equal rotors: only the CoM's few-mm offset from the rotors' centre, which ArduCopter trims out.
    assert np.allclose(tq[:2], 0.0, atol=0.01) and abs(tq[2]) < 1e-12


def test_rotor_response_matches_the_gazebo_joint_controller():
    r = RotorThrust()
    for _ in range(30):  # spin up to near hover
        r.step([1520] * 4, 0.001)
    hover = r.speed.copy()
    for _ in range(5):  # a small change settles within a few ms (Gazebo: 0.8 ms time constant)
        r.step([1530] * 4, 0.001)
    assert np.allclose(r.speed, MAX_ROTOR_SPEED * 430 / 800, rtol=1e-3) and hover[0] < r.speed[0]
    r = RotorThrust()
    r.step([1900] * 4, 0.001)  # a jump to full speed slews: ~13 ms
    assert math.isclose(r.speed[0], 62.5, rel_tol=1e-6)
    for _ in range(20):
        r.step([1900] * 4, 0.001)
    assert math.isclose(r.speed[0], MAX_ROTOR_SPEED, rel_tol=1e-3)
    r = RotorThrust(tau_s=0.01, slew=0)
    r.step([1900] * 4, 0.01)
    assert 0.6 < r.speed[0] / MAX_ROTOR_SPEED < 0.65  # 1 - 1/e after one time constant
    assert np.allclose(r.command([900, 2100, 1100, 1500]), [0, MAX_ROTOR_SPEED, 0, MAX_ROTOR_SPEED / 2])


def test_rotor_torques_have_arducopter_quad_x_signs():
    ff = FireFly()
    r = RotorThrust(ff, tau_s=0.0, slew=0)
    ch = {rot.name: rot.channel for rot in ff.rotors}
    base = [1500] * 4

    def torque(name, d=200):
        pwm = list(base)
        pwm[ch[name]] += d
        return r.step(pwm, 0.001)[1] - r.step(base, 0.001)[1]

    # FLU: +x torque lifts the left side, +y torque pushes the nose down.
    assert torque("rotor_0")[0] < 0 and torque("rotor_0")[1] < 0  # front right lifts the right side and the nose
    assert torque("rotor_2")[0] > 0 and torque("rotor_2")[1] < 0  # front left lifts the left side and the nose
    assert torque("rotor_0")[2] < 0  # counter-clockwise prop yaws the body clockwise
    assert torque("rotor_2")[2] > 0


def test_body_specific_force_at_rest_and_in_free_fall():
    R = _rot(0.0, 0.0, 1.0)
    assert np.allclose(body_specific_force(np.zeros(3), np.zeros(3), 0.001, R), [0, 0, 9.8])
    assert np.allclose(body_specific_force(np.zeros(3), np.array([0, 0, -0.0098]), 0.001, R), 0.0, atol=1e-9)


def test_imu_sampler_rate_gravity_and_gyro_frame():
    s = ImuSampler(noise=False)
    R = _rot(0.0, math.radians(15), 0.0)  # the D435i, tilted down
    w_world = np.array([0.0, 0.0, 0.4])
    out = []
    for k in range(1000):
        out += s.add(k * 0.001, np.array([1.0, 2.0, 1.5]), R, w_world)
    ts = [t for t, _, _ in out]
    assert len(out) in (197, 198, 199)
    assert np.allclose(np.diff(ts), 0.005)
    t, w, f = out[50]
    assert np.allclose(f, R.T @ -GRAVITY, atol=1e-6)
    assert np.allclose(w, R.T @ w_world)


def test_frames_split_from_a_stream_in_any_chunking():
    img = np.arange(12, dtype=np.uint8).reshape(3, 4)
    stream = image_frame("/m/infra1", 1.25, img, "L8") + pack_frame({"k": "clock", "topic": "/c", "t": 2.0})
    rd, got = FrameReader(), []
    for i in range(0, len(stream), 7):
        got += rd.feed(stream[i : i + 7])
    assert [h["k"] for h, _ in got] == ["img", "clock"]
    h, payload = got[0]
    assert (h["w"], h["h"], h["fmt"], h["t"]) == (4, 3, "L8", 1.25)
    assert np.array_equal(np.frombuffer(payload, np.uint8).reshape(3, 4), img)
    with pytest.raises(ValueError):
        image_frame("/x", 0.0, np.zeros((3, 4, 4), np.uint8), "L8")


def test_to_gray():
    rgba = np.zeros((1, 3, 4), np.uint8)
    rgba[0, 0, :3] = 255
    rgba[0, 1, 1] = 255
    assert to_gray(rgba).tolist() == [[255, 150, 0]]


def test_viewer_turns_every_stream_into_rgb():
    from rowpilot.isaac_view import to_rgb

    assert to_rgb("infra1", 4, 3, bytes(range(12))).shape == (3, 4, 3)
    assert to_rgb("chase", 4, 3, bytes(36)).shape == (3, 4, 3)
    depth = np.full((3, 4), 2.0, np.float32)
    depth[0, 0] = np.inf
    assert to_rgb("depth", 4, 3, depth.tobytes()).shape == (3, 4, 3)
