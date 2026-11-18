"""The drone's real sensors, described once and used for both the Gazebo model and the processing.

Intel RealSense D435i, front, tilted down:
  depth       424x240 @ 15 Hz, 87 x 58 deg, 0.3 - 10 m    (lane perception)
  infra1/2    640x480 @ 30 Hz, global shutter, 50 mm baseline (OpenVINS stereo)
  imu         200 Hz, BMI055                               (OpenVINS)
Raspberry Pi Camera Module 3, left side, level, looking at the crop row (plant images).

Frames: body = FLU (x forward, y left, z up) at the drone's centre. Camera links are FLU too, with
the optical axis along +x, as Gazebo renders them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Camera:
    name: str
    width: int
    height: int
    hfov_deg: float
    rate_hz: float
    pose: tuple[float, float, float, float, float, float]  # x y z roll pitch yaw in body FLU (pitch>0: down)
    kind: str = "camera"  # camera | depth_camera
    pixel_format: str = "L8"
    near: float = 0.1
    far: float = 50.0
    noise_std: float = 0.0

    @property
    def fx(self) -> float:
        return (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    @property
    def cx(self) -> float:
        return (self.width - 1) / 2

    @property
    def cy(self) -> float:
        return (self.height - 1) / 2


@dataclass(frozen=True)
class D435i:
    mount_x: float = 0.20  # front edge of the x500 frame, clear of the body
    mount_z: float = 0.0
    tilt_deg: float = 15.0  # downwards; confirm on the real drone
    baseline: float = 0.050
    depth_size: tuple[int, int] = (424, 240)
    depth_rate: float = 15.0
    ir_size: tuple[int, int] = (640, 480)
    ir_rate: float = 30.0
    imu_rate: float = 200.0
    # BMI055 continuous-time noise (per sqrt(Hz)) and bias random walks, as used in OpenVINS configs
    accel_noise_density: float = 2.0e-3
    accel_random_walk: float = 3.0e-3
    gyro_noise_density: float = 1.6e-4
    gyro_random_walk: float = 2.0e-5

    @property
    def tilt(self) -> float:
        return math.radians(self.tilt_deg)

    def ir(self, side: str) -> Camera:
        y = self.baseline / 2 if side == "left" else -self.baseline / 2  # infra1 = left imager
        return Camera(
            f"infra{1 if side == 'left' else 2}",
            *self.ir_size,
            87.0,
            self.ir_rate,
            (self.mount_x, y, self.mount_z, 0.0, self.tilt, 0.0),
            near=0.1,
            far=50.0,
        )

    def depth(self) -> Camera:
        return Camera(
            "depth",
            *self.depth_size,
            87.0,
            self.depth_rate,
            (self.mount_x, self.baseline / 2, self.mount_z, 0.0, self.tilt, 0.0),
            kind="depth_camera",
            pixel_format="R_FLOAT32",
            near=0.3,
            far=10.0,
            noise_std=0.01,
        )


@dataclass(frozen=True)
class PiCamera3:
    """Camera Module 3 (standard lens): 66 x 41 deg."""

    pose: tuple[float, float, float, float, float, float] = (
        0.0,
        0.16,
        0.0,
        0.0,
        0.0,
        math.pi / 2,
    )  # between the left arms

    def camera(self) -> Camera:
        return Camera("picam", 1280, 720, 66.0, 2.0, self.pose, pixel_format="R8G8B8")


@dataclass(frozen=True)
class SensorRig:
    d435i: D435i = field(default_factory=D435i)
    picam: PiCamera3 = field(default_factory=PiCamera3)


def topic(model: str, cam: str) -> str:
    return f"/{model}/{cam}"


# ---- depth image -> lane scan ----------------------------------------------------------------


def _rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation of a frame with FLU-style roll/pitch/yaw (pitch > 0 tilts +x down)."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def depth_to_lane_scan(
    depth: np.ndarray,
    cam: Camera,
    altitude: float,
    roll: float = 0.0,
    pitch: float = 0.0,
    band: tuple[float, float] = (0.6, 2.4),
    stride: int = 2,
    n_dirs: int = 96,
) -> np.ndarray:
    """Turn a depth image into the horizontal 'lane scan' the lane perception uses.

    Every pixel becomes a 3D point (camera -> body -> gravity-levelled with the drone's roll and pitch).
    Only points at foliage height above the floor are kept (band, metres above ground): this drops the
    floor and the heating pipes, which are not lane walls. Returns (N, 2) points, x forward, y left,
    in the levelled body frame. Gazebo depth is the distance along the optical axis (+x)."""
    d = depth[::stride, ::stride].astype(float)
    v, u = np.mgrid[0 : depth.shape[0] : stride, 0 : depth.shape[1] : stride]
    ok = np.isfinite(d) & (d > cam.near) & (d < cam.far)
    d, u, v = d[ok], u[ok], v[ok]
    fy = cam.fx  # square pixels
    pc = np.stack([d, -(u - cam.cx) / cam.fx * d, -(v - cam.cy) / fy * d])  # camera FLU
    x, y, z, r, p, yw = cam.pose
    pb = _rot(r, p, yw) @ pc + np.array([[x], [y], [z]])  # body FLU
    pl = _rot(roll, pitch, 0.0) @ pb  # levelled
    h = altitude + pl[2]
    keep = (h > band[0]) & (h < band[1])
    xy = pl[:2, keep].T
    if not len(xy):
        return xy
    # Collapse to a horizontal scan: per direction, the nearest point at any height in the band
    # (the leaf that sticks out furthest is what the drone must keep clear of). This is the same
    # kind of scan the Python simulator produces, in angle order.
    az = np.arctan2(xy[:, 1], xy[:, 0])
    rng = np.hypot(xy[:, 0], xy[:, 1])
    half = math.radians(cam.hfov_deg) / 2 + 0.2
    bins = np.clip(((az + half) / (2 * half) * n_dirs).astype(int), 0, n_dirs - 1)
    order = np.lexsort((rng, bins))
    first = np.r_[True, bins[order][1:] != bins[order][:-1]]
    return xy[order][first]


# ---- Gazebo model ----------------------------------------------------------------------------


def _in_frame(frame: tuple, pose: tuple) -> tuple[float, float, float, float, float, float]:
    """A body-frame pose (x, y, z, roll, pitch, yaw) expressed in another body-frame pose's frame."""
    Rf, Rp = _rot(*frame[3:]), _rot(*pose[3:])
    p = Rf.T @ (np.array(pose[:3]) - np.array(frame[:3]))
    R = Rf.T @ Rp
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    return (*(float(v) for v in p), roll, pitch, yaw)


def _camera_sdf(cam: Camera, model: str, frame: tuple) -> str:
    pose = " ".join(f"{v:.4f}" for v in _in_frame(frame, cam.pose))
    noise = (
        f"<noise><type>gaussian</type><mean>0</mean><stddev>{cam.noise_std}</stddev></noise>" if cam.noise_std else ""
    )
    extra = (
        f"<depth_camera><clip><near>{cam.near}</near><far>{cam.far}</far></clip></depth_camera>"
        if cam.kind == "depth_camera"
        else ""
    )
    return (
        f'      <sensor name="{cam.name}" type="{cam.kind}">\n'
        f"        <pose>{pose}</pose><topic>{topic(model, cam.name)}</topic>\n"
        f"        <update_rate>{cam.rate_hz}</update_rate><always_on>1</always_on>\n"
        f"        <camera><horizontal_fov>{math.radians(cam.hfov_deg):.5f}</horizontal_fov>"
        f"<image><width>{cam.width}</width><height>{cam.height}</height><format>{cam.pixel_format}</format></image>"
        f"<clip><near>{cam.near}</near><far>{cam.far}</far></clip>{extra}{noise}</camera>\n"
        "      </sensor>\n"
    )


def _imu_sdf(d: D435i, model: str) -> str:
    def axis(sd: float, rw: float) -> str:
        return "".join(
            f'<{a}><noise type="gaussian"><mean>0</mean><stddev>{sd:.6f}</stddev>'
            f"<dynamic_bias_stddev>{rw:.6f}</dynamic_bias_stddev>"
            f"<dynamic_bias_correlation_time>400</dynamic_bias_correlation_time></noise></{a}>"
            for a in "xyz"
        )

    s = math.sqrt(d.imu_rate)  # continuous noise density -> per-sample standard deviation
    return (
        '      <sensor name="d435i_imu" type="imu">\n'
        f"        <pose>0 0 0 0 0 0</pose><topic>{topic(model, 'd435i_imu')}</topic>\n"
        f"        <update_rate>{d.imu_rate}</update_rate><always_on>1</always_on>\n"
        f"        <imu><angular_velocity>{axis(d.gyro_noise_density * s, d.gyro_random_walk)}</angular_velocity>"
        f"<linear_acceleration>{axis(d.accel_noise_density * s, d.accel_random_walk)}</linear_acceleration></imu>\n"
        "      </sensor>\n"
    )


def rig_model_sdf(model: str, base_uri: str, base_link: str, rig: SensorRig | None = None, note: str = "") -> str:
    """A drone model: an existing airframe model (merged in) plus the real drone's sensor rig,
    fixed to the airframe's body link. Sensor topics are /<model>/<sensor>."""
    rig = rig or SensorRig()
    d = rig.d435i
    # The sensor link's origin is the D435i (and its IMU), tilted with it, with the mass there too.
    # Gazebo's IMU gives wrong linear accelerations while the drone rotates if the IMU sits away from
    # its link's origin (measured: ~0.3 m/s^2 sideways per rad/s of yaw), which made OpenVINS diverge
    # in every turn. Sensor poses below are relative to this link.
    frame = (d.mount_x, 0.0, d.mount_z, 0.0, d.tilt, 0.0)
    cams = (d.ir("left"), d.ir("right"), d.depth(), rig.picam.camera())
    sensors = "".join(_camera_sdf(c, model, frame) for c in cams)
    sensors += _imu_sdf(d, model)
    link_pose = " ".join(f"{v:.5f}" for v in frame)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by rowpilot.sensors: edit sensors.py, not this file. {note} -->
<sdf version="1.9">
  <model name="{model}">
    <include merge="true">
      <uri>{base_uri}</uri>
    </include>
    <!-- Intel RealSense D435i (front, tilted {d.tilt_deg:g} deg down) and Raspberry Pi Camera Module 3 (left). -->
    <link name="sensor_link">
      <pose relative_to="{base_link}">{link_pose}</pose>
      <inertial><mass>0.1</mass>
        <inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <visual name="d435i">
        <geometry><box><size>0.025 0.09 0.025</size></box></geometry>
        <material><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>
{sensors}    </link>
    <joint name="sensor_joint" type="fixed">
      <parent>{base_link}</parent>
      <child>sensor_link</child>
    </joint>
    <!-- Exact sensor-link poses at 1 kHz: rowpilot.imu_synth computes the D435i accelerometer from them. -->
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <update_frequency>1000</update_frequency>
    </plugin>
  </model>
</sdf>
"""


def gh_x500_sdf(rig: SensorRig | None = None, model: str = "gh_x500") -> str:
    """PX4: the x500 with the real drone's sensors (position from VIO, no Gazebo odometry, no GPS)."""
    return rig_model_sdf(model, "x500", "base_link", rig, "PX4 SITL.")


def gh_iris_sdf(rig: SensorRig | None = None, model: str = "gh_iris") -> str:
    """ArduPilot: the ardupilot_gazebo Iris (ArduPilotPlugin, JSON lock-step) with the real drone's sensors."""
    return rig_model_sdf(model, "model://iris_with_ardupilot", "iris_with_standoffs::base_link", rig, "ArduPilot SITL.")
