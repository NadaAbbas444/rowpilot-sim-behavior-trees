# Setting up the simulation on your laptop

Step by step, from a fresh Ubuntu laptop to watching the FireFly fly the greenhouse mission in
Gazebo. Two levels:

- **Level 1, the Python simulator** (5 minutes, any OS with Python 3.10+): the behaviour tree,
  perception and control against a fast built-in simulator. Enough to work on the mission logic.
- **Level 2, ArduPilot SITL + Gazebo** (about 1 hour the first time, Ubuntu 22.04): the real drone's
  software chain: ArduCopter, OpenVINS on simulated D435i images, the FireFly model, live camera
  screens.

## What you need

| | Level 1 | Level 2 |
|---|---|---|
| OS | Linux, macOS or Windows | **Ubuntu 22.04** (tested on 22.04.5) |
| Python | 3.10 or newer | the system Python 3.10 |
| Disk | 200 MB | ~6 GB (ArduPilot 1.8 GB, OpenVINS 1.6 GB, build files) |
| GPU | – | **an NVIDIA GPU with the proprietary driver** strongly recommended (tested: RTX 4070, driver 580). Without one, Gazebo renders the five cameras on the CPU and runs ~100x slower than real time |
| RAM | – | 16 GB (8 GB works, building is slower) |
| Admin | – | `sudo`, once, for the system packages |

Check the NVIDIA driver with `nvidia-smi`: it should list the GPU. If it does not, install the driver
first (Ubuntu: *Software & Updates → Additional Drivers*), and reboot.

## Get the code

```bash
git clone https://github.com/NadaAbbas444/rowpilot-sim-behavior-trees.git
cd rowpilot-sim-behavior-trees
```

## Level 1: the Python simulator

```bash
python3 -m venv .venv && source .venv/bin/activate     # Ubuntu: sudo apt install python3-venv if missing
pip install -U pip && pip install -e ".[dev,plot]"
python -m rowpilot tree                                 # print the behaviour tree
python -m rowpilot run --out runs/nominal               # the 6-lane greenhouse, about 1 s
pytest -q                                               # all tests
```

`runs/nominal/path.png` shows the flight from above; `summary.json` says how it went. More runs:

```bash
python -m rowpilot run --battery 45 --out runs/low-battery    # flies home on the battery reserve
python -m rowpilot run --vo-dropout 60:1.5 --out runs/vo      # visual odometry lost for 1.5 s at t=60 s
python -m rowpilot run --open-ends --out runs/open            # lanes open at both ends: serpentine
python -m rowpilot run --help                                 # all options
```

### The behaviour tree editor (optional)

With Node.js 18+ as well (`node --version`):

```bash
cd web && npm ci && npm run build && cd ..
python -m rowpilot editor                                   # open http://127.0.0.1:8000
```

See [editor.md](editor.md).

## Level 2: ArduPilot SITL + Gazebo

### 1. Install everything (once)

```bash
./scripts/setup_ardupilot_sim.sh
```

It asks for your `sudo` password once (for the system packages) and then, without further questions:

1. installs Gazebo Harmonic and the build dependencies from apt (skipped if already installed);
2. creates `.venv-sim/`, a Python environment that can see Gazebo's Python bindings;
3. downloads and builds ArduPilot SITL (ArduCopter) into `~/ardupilot` (the longest step, 10–20 min);
4. downloads and builds the ardupilot_gazebo plugin into `~/ardupilot_gazebo`;
5. downloads and builds OpenVINS (without ROS) into `~/open_vins`;
6. builds rowpilot's VIO bridge into `build/vio/`.

ArduPilot, ardupilot_gazebo and OpenVINS are checked out at the exact commits the mission was tested
with (listed at the top of the script). To put them somewhere else than your home folder:
`./scripts/setup_ardupilot_sim.sh /path/to/folder`. Running the script again is safe: it skips what
is already there.

### 2. Fly

```bash
./scripts/run_ardupilot_sim.sh ~/ardupilot ~/ardupilot_gazebo
```

What happens (the terminal prints each step):

1. Gazebo opens with the greenhouse, the FireFly at home, and three live camera screens on the right
   (D435i infrared, D435i depth, Pi Camera). The 3D view follows the drone.
2. ArduCopter SITL starts, OpenVINS starts on the D435i images.
3. "waiting for EKF3 to use the visual odometry": ~30 s until ArduPilot's position estimate uses OpenVINS.
4. "Arm: Accels inconsistent" for a few seconds is normal (the drone settling after it is dropped
   into the world); then "Arming motors" and take-off.
5. The mission: six lanes, each flown in to the end wall, turned round and flown back out, then home and
   land. About 7 minutes. Each step is printed with its time.
6. The result is printed and saved in `runs/ardupilot-<date>-<time>/` (`path.png`, `summary.json`,
   logs). See [how-it-works.md §13](how-it-works.md#13-what-each-run-saves-and-where).

Stop any time with **Ctrl+C** in the terminal (it lands the drone and closes everything). Closing the
Gazebo window also ends the simulation.

Options (environment variables before the command):

| | |
|---|---|
| `HEADLESS=1` | no Gazebo window (faster, e.g. on a server) |
| `VEHICLE=iris` | fly the ArduPilot Iris instead of the FireFly |
| `GPU=0` | do not use the NVIDIA GPU (only if offload causes problems) |

### 3. Check it worked

A good run ends with `"outcome": "completed"`, `"lanes_completed": [0, 1, 2, 3, 4, 5]` and a lane-centre
error (`lane_rms_error_m`) under about 0.15 m. Three runs on the tested laptop (one from a fresh clone
set up with this guide): 37–117 mm RMS, worst 0.36–0.52 m, 371–393 s. In `path.png` the green line is
the planned path, blue where the drone really flew, dashed orange where it believed it was.

## Troubleshooting

| What you see | Why, and what to do |
|---|---|
| Gazebo is extremely slow (real-time factor below 0.1 in the bottom-right corner) | It renders on the CPU. Check `nvidia-smi` works; the run script then uses the NVIDIA GPU automatically (PRIME offload). |
| "Gazebo did not start (attempt 1), retrying ..." | Gazebo sometimes crashes while starting; the script retries up to 3 times. Only a failure after 3 attempts is a problem: see `runs/<run>/gazebo.log`. |
| Stuck at "waiting for EKF3 to use the visual odometry" for minutes | OpenVINS is not tracking: see `runs/<run>/openvins.log`. Usually the simulation runs too slowly (see the first row). |
| "Arm: Accels inconsistent" for more than a minute | The drone did not settle after spawning; stop (Ctrl+C) and start again. |
| `ModuleNotFoundError: gz` or Gazebo Python errors | Run through the scripts (they use `.venv-sim` and set `PYTHONNOUSERSITE=1`). Python packages installed with `pip install --user` into `~/.local` break Gazebo's bindings; the scripts hide them. |
| Gazebo hangs at start on a laptop with Wi-Fi and Docker | Handled: the scripts keep Gazebo's messaging on the local machine (`GZ_IP=127.0.0.1`). If you run `gz` commands by hand, set `export GZ_IP=127.0.0.1` too. |
| "ArduPilot disarmed in flight" | ArduCopter's crash check fired. Look at `runs/<run>/logs/*.BIN` with [UAV Log Viewer](https://plot.ardupilot.org) (attitude, vibration). |
| The build of ArduPilot fails | Delete `~/ardupilot` and run the setup script again (it fetches the tested commit). |

## Where things are

| | |
|---|---|
| [README.md](../README.md) | overview, results |
| [how-it-works.md](how-it-works.md) | the whole system: processes, the behaviour tree node by node, failsafes, perception, control, localization |
| [bt/](bt/) | behaviour tree diagrams (regenerate with `python scripts/bt_diagrams.py`) |
| [editor.md](editor.md) | the behaviour tree editor (web page): draw the tree, run it, watch every node |
| [deployment.md](deployment.md) | notes on running on the real drone |
| `rowpilot/` | the Python package: tree, perception, control, simulator, ArduPilot vehicle |
| `ardupilot/` | Gazebo models, world template, window layout, ArduCopter parameters |
| `vio_bridge/` | OpenVINS bridge (C++) |
| `scripts/` | setup and run scripts, camera views, diagrams |
| `tests/` | the tests (`pytest -q`) |
