# Setting up and running everything

Step by step, from a fresh Ubuntu laptop to flying the FireFly through the greenhouse in Gazebo,
from the terminal or from the behaviour tree editor. Three levels; each one adds to the one before:

| Level | What you get | Time the first time | Needs |
|---|---|---|---|
| **1. Python simulator** | the behaviour tree, lane perception and control against a fast built-in simulator; the tests | 5 minutes | Python 3.10+, any OS |
| **2. The editor** | the behaviour tree editor in the browser: draw the tree, run it, watch every node | +5 minutes | Node.js 18+ |
| **3. Gazebo + ArduPilot + OpenVINS** | the real drone's software chain: ArduCopter SITL, OpenVINS on simulated D435i images, the FireFly in Gazebo with live camera screens; from the terminal or the editor | +about 1 hour | Ubuntu 22.04, an NVIDIA GPU recommended |

Every command and option: [reference.md](reference.md). How it all works: [how-it-works.md](how-it-works.md).

## What you need

| | Level 1 | Level 2 | Level 3 |
|---|---|---|---|
| OS | Linux, macOS or Windows | same | **Ubuntu 22.04** (tested on 22.04.5) |
| Python | 3.10 or newer | same | the system Python 3.10 |
| Node.js | – | **18 or newer** (tested 22): `node --version` | same |
| Disk | 200 MB | +250 MB (`web/node_modules`) | ~6 GB (ArduPilot 1.8 GB, OpenVINS 1.6 GB, build files) |
| GPU | – | – | **an NVIDIA GPU with the proprietary driver** strongly recommended (tested: RTX 4070, driver 580). Without one, Gazebo renders the five cameras on the CPU and runs ~100x slower than real time |
| RAM | – | – | 16 GB (8 GB works, building is slower) |
| Admin | – | – | `sudo`, once, for the system packages |

For level 3, check the NVIDIA driver with `nvidia-smi`: it should list the GPU. If it does not,
install the driver first (Ubuntu: *Software & Updates → Additional Drivers*) and reboot.
Node.js, if you do not have it: `sudo apt install nodejs npm` gives an old version on Ubuntu 22.04;
use [nvm](https://github.com/nvm-sh/nvm) (`nvm install 22`) instead.

## Get the code

```bash
git clone https://github.com/NadaAbbas444/rowpilot-sim-behavior-trees.git
cd rowpilot-sim-behavior-trees
git checkout editor          # the branch with the editor; the simulator alone is on `simulator`
```

All commands below run from this folder.

---

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

(`source .venv/bin/activate` once per terminal; or call `.venv/bin/python` directly.)

---

## Level 2: the behaviour tree editor

### Build the page (once, and after pulling changes to `web/`)

```bash
cd web && npm ci && npm run build && cd ..
```

### Start it

```bash
python -m rowpilot editor
```

and open **http://127.0.0.1:8000**. The terminal prints whether Gazebo runs are ready (level 3) or
what is missing for them; the Python simulator always works. Stop the editor with **Ctrl+C**.

### First things to try

1. **▶ Run**: the mission flies in the Python simulator. Running nodes turn amber, finished green,
   failed red; the **Log** tab fills; the map (top right) draws the flight.
2. **Lanes − 6 of 6 +** (toolbar): press **−** twice, Run again: the drone sweeps lanes 1–4 and goes
   home. The greenhouse stays at 6 lanes; the tree chooses how many to sweep.
3. Click a node: the **Node** tab shows its parameters and its own log.
4. *Run settings* → *start battery* `45`, Run: the battery failsafe takes over in lane 3 and flies home.

Everything the page does: [editor.md](editor.md).

---

## Level 3: Gazebo + ArduPilot SITL + OpenVINS

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
`./scripts/setup_ardupilot_sim.sh /path/to/folder` (then pass that folder to the run script and the
editor, see [reference.md](reference.md)). Running the script again is safe: it skips what is there.

Note: the Gazebo here is **Gazebo Harmonic**, the `gz` command. If Ubuntu suggests
`sudo apt install gazebo` when you type `gazebo`, do not: that is the old Gazebo Classic.

### 2a. Fly from the terminal

```bash
./scripts/run_ardupilot_sim.sh
```

What happens (the terminal prints each step):

1. Gazebo opens with the greenhouse, the FireFly at home, and three live camera screens on the right
   (D435i infrared, D435i depth, Pi Camera). The 3D view follows the drone.
2. ArduCopter SITL starts, OpenVINS starts on the D435i images.
3. "waiting for EKF3 to use the visual odometry": ~10–30 s until ArduPilot's position estimate uses OpenVINS.
4. "Arm: Accels inconsistent" for a few seconds is normal (the drone settling after it is dropped
   into the world); then "Arming motors" and take-off.
5. The mission: six lanes, each flown in to the end wall, turned round and flown back out, then home
   and land. About 7 minutes. Each step is printed with its time.
6. The result is printed and saved in `runs/ardupilot-<date>-<time>/` (`path.png`, `summary.json`,
   logs). See [how-it-works.md §13](how-it-works.md#13-what-each-run-saves-and-where).

Stop with **Ctrl+C** in the terminal: the drone lands and everything closes.

Options go in front of the command, e.g. `HEADLESS=1 VEHICLE=iris ./scripts/run_ardupilot_sim.sh`:

| | |
|---|---|
| `HEADLESS=1` | no Gazebo window (faster, e.g. on a server) |
| `VEHICLE=iris` | fly the ArduPilot Iris instead of the FireFly |
| `GREENHOUSE_ARGS="--lanes 4"` | another greenhouse (world and mission) |
| `TREE=trees/mine.json` | fly a tree saved in the editor instead of the mission |
| `GPU=0` | do not use the NVIDIA GPU (only if offload causes problems) |

All of them: [reference.md](reference.md#scriptsrun_ardupilot_simsh-fly-the-mission-in-gazebo).

### 2b. Fly from the editor

1. Start the editor (`python -m rowpilot editor`); it should print **"Gazebo runs: ready"**.
2. In **Run settings**, choose **Gazebo + ArduPilot + OpenVINS** (the toolbar badge turns orange);
   pick the drone and whether to show the Gazebo window.
3. For a short first flight, use **Lanes −** to get *1 of 6* (about 2 minutes).
4. **▶ Run**. The Gazebo window opens; the **Log** tab shows the flight stack's messages, labelled
   *setup*, *VIO*, *EKF3*, *ArduPilot* and *MAVLink*, next to the tree's: OpenVINS initialising,
   "EKF3 IMU0 is using external nav data", arming, "take-off to 1.0 m accepted", "climbing to 1.5 m",
   the lane steps, LAND mode. The tree and the map follow the flight.
5. **■ Stop** at any time: in flight, the drone lands where it is and everything closes; while still
   starting, everything closes at once.

The results go to `runs/editor-<date>-<time>-<id>/` (the same files as a terminal run, plus the tree).

### 3. Check it worked

A good run ends with `"outcome": "completed"`, all lanes in `lanes_completed` and a lane-centre
error (`lane_rms_error_m`) under about 0.15 m. Three 6-lane runs on the tested laptop (one from a
fresh clone set up with this guide): 37–117 mm RMS, worst 0.36–0.52 m, 371–393 s. In `path.png` the
green line is the planned path, blue where the drone really flew, dashed orange where it believed it
was.

---

## Stopping and cleaning up

The normal ways to end a Gazebo run close everything (Gazebo, ArduCopter, OpenVINS, the helpers):

- **■ Stop** in the editor, or stopping the editor (Ctrl+C in its terminal);
- **Ctrl+C** in the terminal running `run_ardupilot_sim.sh`;
- the mission ending by itself.

**Closing the Gazebo window does not end the run**: ArduCopter, OpenVINS and the mission keep running
and wait (without Gazebo the simulation clock stands still). Use Stop or Ctrl+C instead, or clean up
by hand.

See what is running (no output: nothing is):

```bash
ps -eo pid,args | grep -E "[g]z sim|[a]rducopter|[v]io_bridge|[r]owpilot\.(imu_synth|depth_view)|[r]owpilot ardupilot|[r]un_ardupilot_sim"
```

Stop all of it by hand:

```bash
pkill -f "gz sim"; pkill -x arducopter; pkill -x vio_bridge; pkill -f rowpilot.imu_synth; pkill -f rowpilot.depth_view; pkill -f "rowpilot ardupilot"; pkill -f run_ardupilot_sim.sh
```

ArduCopter sometimes ignores that; then force it, and check again:

```bash
pkill -9 -x arducopter; pkill -9 -f "gz sim"
```

A new run also stops any leftover simulation before it starts.

---

## Troubleshooting

| What you see | Why, and what to do |
|---|---|
| Gazebo is extremely slow (real-time factor below 0.1 in the bottom-right corner) | It renders on the CPU. Check `nvidia-smi` works; the run script then uses the NVIDIA GPU automatically (PRIME offload). |
| "Gazebo did not start (attempt 1), retrying ..." | Gazebo sometimes crashes while starting; the script retries up to 3 times. Only a failure after 3 attempts is a problem: see `runs/<run>/gazebo.log`. |
| The Gazebo window opens but stays empty, nothing happens for more than a minute | Gazebo's window and server sometimes miss each other at start. The script retries after 3 minutes; quicker: Stop (or Ctrl+C) and start again. |
| Stuck at "waiting for EKF3 to use the visual odometry" for minutes | OpenVINS is not tracking: see `runs/<run>/openvins.log`. Usually the simulation runs too slowly (see the first row). |
| "Arm: Accels inconsistent" for more than a minute | The drone did not settle after spawning; stop and start again. |
| `ModuleNotFoundError: gz` or Gazebo Python errors | Run through the scripts (they use `.venv-sim` and set `PYTHONNOUSERSITE=1`). Python packages installed with `pip install --user` into `~/.local` break Gazebo's bindings; the scripts hide them. |
| Gazebo hangs at start on a laptop with Wi-Fi and Docker | Handled: the scripts keep Gazebo's messaging on the local machine (`GZ_IP=127.0.0.1`). If you run `gz` commands by hand, set `export GZ_IP=127.0.0.1` too. |
| "ArduPilot disarmed in flight" | ArduCopter's crash check fired. Look at `runs/<run>/logs/*.BIN` with [UAV Log Viewer](https://plot.ardupilot.org) (attitude, vibration). |
| The build of ArduPilot fails | Delete `~/ardupilot` and run the setup script again (it fetches the tested commit). |
| Processes left after closing the Gazebo window | See [Stopping and cleaning up](#stopping-and-cleaning-up). |
| Editor: "web page not built" in the terminal, or an empty page | `cd web && npm ci && npm run build`, restart the editor, reload the page (Ctrl+Shift+R). |
| Editor: "cannot reach the server" in the page | The editor is not running (or on another port): `python -m rowpilot editor`. |
| Editor: the Gazebo choice is greyed out | Level 3 is not set up, or not where the editor looks: the Run settings and the editor's start message say what is missing. Other folders: `--ardupilot`, `--ardupilot-gazebo`. |
| Editor: "address already in use" | Another editor is running (`python -m rowpilot editor --port 8001`, or stop the other one). |
| Editor: the page shows old behaviour after an update | Rebuild the page (`npm run build`), restart the editor, reload with Ctrl+Shift+R. |
| Editor: a node has a red dashed border, Run is disabled | The tree has a problem; the node's tooltip and the Node tab say what (e.g. a lane the greenhouse does not have, a missing child, a name used twice). |

## Where things are

| | |
|---|---|
| [README.md](../README.md) | overview, results |
| [reference.md](reference.md) | every command, script, option and environment variable |
| [editor.md](editor.md) | the behaviour tree editor |
| [how-it-works.md](how-it-works.md) | the whole system: processes, the behaviour tree node by node, failsafes, perception, control, localization |
| [bt/](bt/) | behaviour tree diagrams (regenerate with `python scripts/bt_diagrams.py`) |
| [deployment.md](deployment.md) | notes on running on the real drone |
| `rowpilot/` | the Python package: tree, perception, control, simulator, ArduPilot vehicle |
| `rowpilot/editor/` | the editor's server: node types, runs, API |
| `web/` | the editor's page (React + React Flow) |
| `ardupilot/` | Gazebo models, world template, window layout, ArduCopter parameters |
| `vio_bridge/` | OpenVINS bridge (C++) |
| `scripts/` | setup and run scripts, camera views, diagrams |
| `tests/` | the tests (`pytest -q`; the page's: `cd web && npm test`) |
