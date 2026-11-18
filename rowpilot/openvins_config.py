"""OpenVINS configuration for the D435i, generated from the same sensor description as the drone model.

Writes the three files OpenVINS reads: estimator_config.yaml, kalibr_imu_chain.yaml and
kalibr_imucam_chain.yaml (Kalibr format). On the real drone, replace the camera intrinsics and
T_cam_imu with a Kalibr calibration of your D435i; the estimator settings stay.

Frames. IMU frame = the D435i link (FLU, tilted down with the camera). Camera optical frames:
x right, y down, z forward. T_cam_imu maps IMU coordinates into a camera's optical frame.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .sensors import Camera, D435i

# optical (x right, y down, z forward) <- link FLU (x forward, y left, z up)
R_OPT_FLU = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def t_cam_imu(cam: Camera, d: D435i) -> np.ndarray:
    """4x4 transform IMU -> camera optical frame. Cameras and IMU share the link orientation."""
    p_cam_in_imu = np.array([cam.pose[0] - d.mount_x, cam.pose[1], cam.pose[2] - d.mount_z])
    T = np.eye(4)
    T[:3, :3] = R_OPT_FLU
    T[:3, 3] = -R_OPT_FLU @ p_cam_in_imu
    return T


def _mat(rows: np.ndarray, indent: str = "    ") -> str:
    return "\n".join(f"{indent}- [{', '.join(f'{v:.6f}' for v in r)}]" for r in rows)


def imucam_yaml(d: D435i) -> str:
    out = ["%YAML:1.0", ""]
    for i, side in enumerate(("left", "right")):
        cam = d.ir(side)
        out += [
            f"cam{i}:",
            "  T_cam_imu:",
            _mat(t_cam_imu(cam, d)),
            f"  cam_overlaps: [{1 - i}]",
            "  camera_model: pinhole",
            "  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]",
            "  distortion_model: radtan",
            f"  intrinsics: [{cam.fx:.4f}, {cam.fx:.4f}, {cam.cx:.4f}, {cam.cy:.4f}]",
            f"  resolution: [{cam.width}, {cam.height}]",
            f"  rostopic: /gh_x500/{cam.name}",
            "  timeshift_cam_imu: 0.0",
            "",
        ]
    return "\n".join(out)


def imu_yaml(d: D435i) -> str:
    eye3 = _mat(np.eye(3))
    zero3 = _mat(np.zeros((3, 3)))
    return f"""%YAML:1.0

imu0:
  T_i_b:
{_mat(np.eye(4))}
  # BMI055-like noise, inflated as recommended for real sensors (white noise x2, random walk x10
  # would be the usual margin; here x1.5 / x2 since the simulated noise is exactly known)
  accelerometer_noise_density: {d.accel_noise_density * 1.5:.6f}
  accelerometer_random_walk: {d.accel_random_walk * 2:.6f}
  gyroscope_noise_density: {d.gyro_noise_density * 1.5:.7f}
  gyroscope_random_walk: {d.gyro_random_walk * 2:.7f}
  rostopic: /gh_x500/d435i_imu
  time_offset: 0.0
  update_rate: {d.imu_rate:.0f}
  model: "kalibr"
  Tw:
{eye3}
  R_IMUtoGYRO:
{eye3}
  Ta:
{eye3}
  R_IMUtoACC:
{eye3}
  Tg:
{zero3}
"""


ESTIMATOR = """%YAML:1.0

verbosity: "INFO"
use_fej: true
integration: "rk4"
use_stereo: true
max_cameras: 2
calib_cam_extrinsics: false
calib_cam_intrinsics: false
calib_cam_timeoffset: false
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false
max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1
gravity_mag: 9.80665
feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"

# Zero-velocity updates: the drone sits still on the ground before take-off. This also lets
# OpenVINS initialise while standing still (PX4/ArduPilot need a position before arming).
# Only until it first moves: hovering or creeping along a lane, the image hardly changes, the
# disparity test passes and ZUPT would clamp the velocity of a drone that is not standing still.
try_zupt: true
zupt_chi2_multipler: 0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5
zupt_only_at_beginning: true

init_window_time: 2.0
init_imu_thresh: 1.0
init_max_disparity: 2.0
init_max_features: 50
init_dyn_use: false
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 2
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0
init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12
init_dyn_bias_g: [0.0, 0.0, 0.0]
init_dyn_bias_a: [0.0, 0.0, 0.0]

record_timing_information: false
record_timing_filepath: "/tmp/ov_timing.txt"
save_total_state: false
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"

use_klt: true
num_pts: 150
fast_threshold: 20
grid_x: 5
grid_y: 5
min_px_dist: 15
knn_ratio: 0.70
track_frequency: 31.0
downsample_cameras: false
num_opencv_threads: 2
histogram_method: "HISTOGRAM"

use_aruco: false
num_aruco: 1024
downsize_aruco: true

up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1
up_slam_sigma_px: 1
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1

use_mask: false
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
"""


def write_openvins_config(out_dir: Path, d: D435i | None = None) -> Path:
    d = d or D435i()
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "estimator_config.yaml").write_text(ESTIMATOR)
    (out_dir / "kalibr_imu_chain.yaml").write_text(imu_yaml(d))
    (out_dir / "kalibr_imucam_chain.yaml").write_text(imucam_yaml(d))
    return out_dir / "estimator_config.yaml"
