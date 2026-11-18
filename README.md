# rowpilot-bt

[![Tests](https://github.com/NadaAbbas444/rowpilot-bt/actions/workflows/test.yml/badge.svg)](https://github.com/NadaAbbas444/rowpilot-bt/actions/workflows/test.yml)
![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-3776ab)
[![License: MIT](https://img.shields.io/badge/license-MIT-lightgrey)](LICENSE)

**A drone for GPS-denied greenhouses, flown by a behaviour tree.**

A drone that flies the lanes between greenhouse crop rows, where GPS is denied (glass, steel and dense crop). It localizes with visual odometry, keeps itself centred in each lane using the crop rows it sees in a depth camera, and runs the mission as a **behaviour tree** that handles a low battery and lost tracking safely.

Target hardware: a flight controller running ArduPilot (ArduCopter), a Raspberry Pi 5 companion computer on SkiffOS, an Intel RealSense D435i at the front tilted 15° down (depth for lane perception; stereo infrared + IMU for OpenVINS visual-inertial odometry), a Raspberry Pi Camera Module 3 on the left side for plant images, and a battery. No GPS, no compass.

The core of this repo is the part that does not depend on a simulator or the hardware: perception, control, the behaviour tree, and a lightweight simulator that sits behind the same `Vehicle` interface PX4 will use. It runs anywhere with Python 3.10+ and is covered by tests and CI.

![Low-battery run: three lanes swept, then back home along a lane and landed](docs/low-battery-run.png)

*`python -m rowpilot run --battery 45`: the reserve is reached at the start of lane 4, so the tree flies home along the lane (not through the crop) and lands. Blue is the true path; dashed orange is what visual odometry believed.*

## Run it

```bash
python3 -m venv .venv && source .venv/bin/activate   # Ubuntu: sudo apt install python3-venv if missing
pip install -U pip && pip install -e ".[dev]"
python -m rowpilot tree                                    # print the behaviour tree
python -m rowpilot run --out runs/nominal                  # full 6-lane greenhouse
python -m rowpilot run --battery 45 --out runs/low-battery # returns home on the reserve
python -m rowpilot run --vo-dropout 60:1.5 --vo-dropout 200:10 --out runs/vo
pytest -q
```

Each run writes `events.jsonl` (every node's status changes and log lines), `trace.csv` (true and estimated pose, battery and active node at 10 Hz), `summary.json` and `path.png`.

## How it works

| Piece | File | What it does |
|---|---|---|
| Layout | `rowpilot/layout.py` | Greenhouse geometry: lanes, crop rows, headlands, serpentine lane order, home |
| Lane perception | `rowpilot/perception.py` | Horizontal depth scan → distance from the lane centre and heading error. RANSAC line fit for each crop face, nearest face wins, both faces fitted together with a shared slope, sanity checks (parallel faces, correct lane width). Says "don't know" rather than guessing |
| Control | `rowpilot/control.py` | Lane follower (PID on the sideways offset, P on heading, slows down when off-centre); waypoint path following with cross-track correction |
| Mission | `rowpilot/mission.py` | The behaviour tree (py_trees), plus lane-centric localization: inside a lane, the perceived offset corrects the VO cross-track drift |
| Vehicle interface | `rowpilot/vehicle.py` | `state()`, `arm()`, `disarm()`, `send(velocity)`, `depth_scan()`, which is all the mission needs |
| Simulator | `rowpilot/sim.py` | Velocity-loop lag, gusts, drifting VO with dropouts, battery drain, RealSense-like depth rays hitting the crop faces and row ends. Keeps ground truth for tests |
| Runner | `rowpilot/runner.py` | Ticks the tree at 10 Hz and records per-node events (`on_event` streams them live) |

### The behaviour tree

```
Root (Sequence, re-evaluated every tick)
├── SafetyGuard (Selector)
│   ├── AllHealthy: NotAborted, BatteryAboveReserve, VOHealthy
│   └── Failsafe (ends the run when it finishes)
│       ├── VOGlitch: VOLostBriefly → Hover               pause, then resume the mission
│       ├── EmergencyLand: MustLandNow → LandHere         battery critical / VO lost > 3 s
│       └── ReturnAndLand: Airborne → ReturnHome → Land   low battery / a mission step failed
└── Mission (Sequence)
    ├── Preflight, Takeoff
    ├── Lane 1: GoToEntry → Align → Sweep → Exit
    ├── … one subtree per lane
    └── ReturnHome, Land
```

The guard is ticked before the mission on every tick, so a failing check interrupts whatever is running. That is why mission nodes are **re-entrant**: each remembers when it has finished, so after a VO glitch the mission continues from the node it was in instead of starting over. This is the main design rule to keep when adding nodes.

### What the tests check

- Perception matches the true offset and heading across positions and angles, including lane entries seen from the headland, and is never confidently wrong in hard poses.
- Full missions against ground truth: every lane swept and landed; the whole 6-lane greenhouse on one battery; strong gusts (0.5 m/s rms, with the flight controller's measured delay) without hitting the crop; return home on a low battery; hover and resume on a short VO dropout; land in place on a long one; refusing to take off on a low battery.

## ArduPilot SITL + Gazebo (the real drone's setup)

The behaviour tree flies ArduCopter SITL in the Gazebo greenhouse, with the real drone's sensors and estimator chain:
Gazebo D435i → OpenVINS (no ROS) → MAVLink ODOMETRY → ArduPilot EKF3 (ExternalNav, GPS-denied) → GUIDED velocity targets from the tree.

| Piece | File |
|---|---|
| `gh_iris`: ArduPilot Iris + RealSense D435i (front, 15° down) + Pi Camera Module 3 (left), generated from one sensor description | `rowpilot/sensors.py` |
| Greenhouse world: tomato rows, pipe rails, posts, sticky tape, ground cover, glass facades | `rowpilot/gazebo_world.py`, `rowpilot/greenhouse_assets.py` |
| OpenVINS config for the D435i | `rowpilot/openvins_config.py` |
| VIO bridge: D435i stereo IR + IMU → OpenVINS → odometry | `vio_bridge/` |
| OpenVINS pose → greenhouse frame | `rowpilot/vio_link.py` |
| D435i accelerometer computed from the physics (simulation only, see below) | `rowpilot/imu_synth.py` |
| `ArduPilotVehicle` (MAVLink: ODOMETRY to EKF3, GUIDED take-off and velocity targets, LAND mode) | `rowpilot/ardupilot.py` |
| ArduCopter parameters for GPS-denied flight | `ardupilot/gh_iris.parm` |

```bash
./scripts/setup_ardupilot_sim.sh                                 # once: ArduPilot SITL, ardupilot_gazebo, OpenVINS, the bridge, .venv-sim
./scripts/run_ardupilot_sim.sh ~/ardupilot ~/ardupilot_gazebo    # opens Gazebo and flies; results in runs/ardupilot-<time>/
```

**Result** (6 lanes, 13.3 m rows, OpenVINS → EKF3, scored against Gazebo ground truth): every lane swept, **48–112 mm RMS from the lane centre, 0.31 m worst case**, home through the headland, landed by ArduCopter's LAND mode, 239 s. OpenVINS + EKF3 position error in flight: 0.20 m mean, 0.54 m worst.

![ArduPilot SITL + OpenVINS in Gazebo: six lanes swept, back home and landed](docs/ardupilot-gazebo-run.png)

Each run folder has `gazebo.log`, `arducopter.log`, `openvins.log`, `imu_synth.log`, `events.jsonl`, `trace.json` (estimate, ground truth, lane estimate and commands at 10 Hz), `summary.json` and `path.png`. `HEADLESS=1` runs without the Gazebo window; `GPU=0` keeps the default OpenGL renderer.

Things learned getting it to fly, all handled in the scripts and code:
- **GPU rendering.** On a hybrid-graphics laptop OpenGL falls back to llvmpipe (CPU) and the five cameras slow Gazebo to 0.004× real time. The run script uses NVIDIA PRIME offload when there is an NVIDIA GPU: 1.0× real time.
- **1 kHz physics.** In SITL the IMU rate is the physics rate, and ArduCopter's pre-arm check needs it ≥ 1.8 × its 400 Hz loop: 400 Hz physics never arms.
- **gz-transport on loopback** (`GZ_IP=127.0.0.1`): with several network interfaces (Wi-Fi, docker0) the GUI's "starting world" message can miss the server, which then waits forever. Gazebo also sometimes segfaults at start-up (a getenv race in gz-transport); the run script restarts it.
- **Take-off.** A GUIDED velocity target cancels ArduCopter's NAV_TAKEOFF climb, and on the ground ArduCopter does not climb on velocity targets (it disarms). The vehicle holds velocity targets back until the take-off is airborne.
- **Gazebo's IMU accelerometer is wrong while the drone rotates** (0.2–0.6 m/s² sideways in yaw turns, while the gyro and the images agree with the true motion). OpenVINS integrated it into a false velocity and diverged in every turn. Gazebo's gyro is kept; the accelerometer is computed from the sensor link's physics pose (1 kHz), with BMI055-like noise and bias. With it, OpenVINS stays within 0.11 m of ground truth through 0.8 rad/s turns. This is a simulator fix only; the real D435i IMU goes straight to OpenVINS.
- **Visual features.** The floor is tiled ground cover with dirt and leaf litter, and glass facades with columns and glazing bars close the greenhouse, so the cameras have something to track in the headlands. SDF's default metalness (0.5) renders a floor dark and featureless; it is set to 0.
- **ZUPT only before take-off.** Hovering or creeping, the image barely changes and OpenVINS's zero-velocity test passes for a drone that is not standing still.
- **The flight controller's response.** ArduCopter follows GUIDED velocity targets with ~0.5 s dead time and a 0.3 s lag (its jerk and acceleration limits). The Python simulator now models that, and the lane and waypoint controllers are tuned for it: the old gains made the drone swing across the lane. With this delay, the drone holds the lanes in gusts up to 0.5 m/s rms (20/20 runs); at 0.8 m/s most runs touch the crop.

## PX4 SITL + Gazebo

The older PX4 path (kept, not the priority: the real drone runs ArduPilot). The same behaviour tree flies the PX4 flight stack in Gazebo, through a `Vehicle` implementation over MAVSDK. No GPS and no magnetometer: PX4's EKF2 fuses only the vision odometry, as on the real drone.

| Piece | File |
|---|---|
| Greenhouse world generated from the same `GreenhouseLayout` | `rowpilot/gazebo_world.py` |
| `gh_x500`: x500 with vision odometry and a forward horizontal lidar standing in for one row of the RealSense depth image (cropped to its 87° field of view) | `px4/models/gh_x500/` |
| Airframe for GPS-denied flight (no GPS, no compass, offboard only) | `px4/airframes/4099_gz_gh_x500` |
| Copies the above into a PX4-Autopilot checkout | `px4/install.sh` |
| `Px4Vehicle` (MAVSDK offboard velocity, PX4 Land mode for touchdown) and the live runner | `rowpilot/px4.py` |

**Result** (6 lanes, 40 m rows, scored against Gazebo ground truth): every lane swept, **5–8 mm RMS from the lane centre, 43 mm worst case**, home through the headland, and touchdown by PX4's Land mode.

![PX4 SITL + Gazebo run: six lanes swept, back home and landed](docs/gazebo-px4-run.png)

### Run it on your machine (Ubuntu 22.04 / 24.04)

The scripts repeat the steps used to fly it headless on Ubuntu 24.04; the Gazebo window itself has not been tested yet.

```bash
./scripts/setup_px4_sim.sh      # once: PX4, Gazebo Harmonic, the greenhouse world and drone (20-40 min)
./scripts/run_px4_sim.sh        # opens Gazebo and flies the mission; results in runs/px4-<time>/
```

Things learned getting PX4 to fly it, all handled in `rowpilot/px4.py`:
- PX4 refuses to arm indoors in its default Hold mode (it needs a global position), so it switches to offboard first, then arms.
- Without GPS, PX4 sends no global position, so MAVSDK's `heading()` is NaN; yaw comes from attitude.
- PX4's local frame shares the Gazebo world origin, because the vision odometry is reported in world coordinates.
- MAVSDK's local-position flag drops out for single messages, so it is debounced before the tree treats it as lost VO.
- In offboard velocity mode PX4's land detector never confirms touchdown; landing is handed to PX4's Land mode.

## Next steps

1. **Node editor**: React Flow frontend + FastAPI backend that runs the tree, streaming `on_event` over a WebSocket so each node lights up and shows its own log (`python -m rowpilot tree --json` already exports nodes and edges).
2. **Hardware**: RealSense driver and visual odometry (e.g. OpenVINS or VINS-Fusion) on the Pi 5, feeding PX4's EKF2 as external vision; one row of the depth image as the lane scan.
