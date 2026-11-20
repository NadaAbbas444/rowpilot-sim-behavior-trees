# Commands and options

Every command, script and setting in one place. How to set things up and what to run first:
[setup.md](setup.md).

All `python -m rowpilot ...` commands run from the repository folder. Use the right Python:

| Environment | Made by | Used for |
|---|---|---|
| `.venv/` | you (`python3 -m venv .venv`, [setup.md](setup.md) level 1) | the Python simulator, the editor, tests |
| `.venv-sim/` | `scripts/setup_ardupilot_sim.sh` | everything that talks to Gazebo (it sees Gazebo's Python bindings) |

Keep `PYTHONNOUSERSITE=1` set when you use `.venv-sim` by hand: packages installed with
`pip install --user` (in `~/.local`) break Gazebo's bindings. The scripts set it themselves.

## `python -m rowpilot run`: a mission in the Python simulator

```bash
python -m rowpilot run --out runs/nominal
```

| Option | Default | |
|---|---|---|
| `--lanes N` | 6 | lanes in the greenhouse |
| `--length M` | 13.33 | crop row length (m) |
| `--path-width M` | 1.6 | free lane width between the crop faces (m) |
| `--open-ends` | off | lanes open at both ends (serpentine); default: dead ends, as in the real greenhouse |
| `--seed N` | 0 | random seed: gusts, odometry drift, depth noise |
| `--battery PCT` | 100 | start battery (45 shows the low-battery failsafe) |
| `--wind M_S` | 0.3 | gust strength (m/s) |
| `--vo-dropout START:SECONDS` | – | visual odometry lost at START for SECONDS; repeat for more |
| `--altitude M` | 1.5 | flight height |
| `--speed M_S` | 0.8 | sweep speed |
| `--out DIR` | – | save `events.jsonl`, `trace.csv`, `summary.json`, `path.png` there |
| `--quiet` | off | print only the summary |

Exit code 1 if the drone crashed, else 0.

## `python -m rowpilot tree`: print the behaviour tree

```bash
python -m rowpilot tree             # as text
python -m rowpilot tree --json      # nodes and edges
```

Takes the greenhouse options (`--lanes`, `--length`, `--path-width`, `--open-ends`).

## `python -m rowpilot editor`: the behaviour tree editor

```bash
python -m rowpilot editor           # http://127.0.0.1:8000
```

| Option | Default | |
|---|---|---|
| `--host` | 127.0.0.1 | `0.0.0.0` to reach it from another computer on the network |
| `--port` | 8000 | |
| `--trees DIR` | `trees` | where **Save** puts trees |
| `--ardupilot DIR` | `~/ardupilot` | for Gazebo runs |
| `--ardupilot-gazebo DIR` | `~/ardupilot_gazebo` | for Gazebo runs |

Needs the editor's Python packages (`pip install -e ".[editor]"`, included in `.[dev]`) and the built
web page (`cd web && npm ci && npm run build`). At start it prints whether Gazebo runs are ready, or
what is missing. Stop it with Ctrl+C (this also stops a run it started). See [editor.md](editor.md).

## `scripts/setup_ardupilot_sim.sh`: install the full simulation (once)

```bash
./scripts/setup_ardupilot_sim.sh [install_dir]      # default: your home folder
```

Installs Gazebo Harmonic and build packages (asks for `sudo` once, skipped if installed), creates
`.venv-sim/`, builds ArduPilot SITL into `install_dir/ardupilot`, the ardupilot_gazebo plugin into
`install_dir/ardupilot_gazebo`, OpenVINS into `install_dir/open_vins`, and rowpilot's VIO bridge into
`build/vio/`. Uses the exact commits listed at the top of the script. Safe to run again: it skips
what is there. Ubuntu 22.04.

## `scripts/run_ardupilot_sim.sh`: fly the mission in Gazebo

```bash
./scripts/run_ardupilot_sim.sh [ardupilot_dir] [ardupilot_gazebo_dir]   # default: ~/ardupilot ~/ardupilot_gazebo
```

Starts Gazebo (with its window), ArduCopter SITL, OpenVINS (`vio_bridge` + `imu_synth`), the depth
display, then `rowpilot ardupilot`. Ctrl+C lands and closes everything. Results in
`runs/ardupilot-<date>-<time>/`.

Environment variables (put them before the command, e.g. `HEADLESS=1 ./scripts/run_ardupilot_sim.sh`):

| Variable | Default | |
|---|---|---|
| `HEADLESS=1` | 0 | no Gazebo window (faster; servers) |
| `VEHICLE=iris` | firefly | fly the ArduPilot Iris instead of the FireFly 2 |
| `ISAAC_LOOK=gazebo` | isaac | with `SIM=isaac`: the Gazebo world's plain look instead of Isaac Sim's materials, sky and detailed plants (faster to render) |
| `ISAAC_PICAM=1` | 0 | with `SIM=isaac`: render the Pi Camera too (costs about a quarter of the render time; nothing in the flight uses it) |
| `ISAAC_WINDOW=1` | 0 | with `SIM=isaac`: Isaac Sim's own window instead of the `isaac_view` window (needs a display Vulkan can draw on) |
| `SIM=isaac` | gazebo | simulate in NVIDIA Isaac Sim instead of Gazebo (FireFly only; `scripts/setup_isaac_sim.sh` once; `ISAAC_PYTHON` to use another Isaac Sim Python) |
| `GPU=0` | 1 | do not use the NVIDIA GPU (only if PRIME offload causes problems) |
| `GREENHOUSE_ARGS="--lanes 4 --open-ends"` | – | another greenhouse (the world and the mission): `--lanes`, `--length`, `--path-width`, `--open-ends` |
| `TREE=<file.json>` | – | fly this tree (a graph saved by the editor) instead of the mission |
| `OUT_DIR=<folder>` | `runs/ardupilot-<date>-<time>` | where the results go |
| `STREAM=1` | – | print the run as JSON lines (what the editor reads) |
| `GZ_IP` | 127.0.0.1 | Gazebo's network interface (loopback avoids hangs with Wi-Fi + Docker) |

## `python -m rowpilot ardupilot`: the tree against a running ArduPilot SITL

Started by `run_ardupilot_sim.sh`; run it by hand only with Gazebo and ArduCopter already running.
Use `.venv-sim/bin/python`.

| Option | Default | |
|---|---|---|
| greenhouse options | | as for `run` (must match the Gazebo world) |
| `--url` | `tcp:127.0.0.1:5760` | MAVLink connection to ArduCopter |
| `--vehicle` | firefly | `firefly` or `iris` (sensor placement) |
| `--max-sim-time S` | 900 | give up after this much simulated time |
| `--out DIR` | – | save the results there |
| `--tree FILE` | – | fly an editor tree instead of the mission |
| `--stream` | off | JSON lines for the editor; `stop` on stdin lands and ends the run |

## `python -m rowpilot world`: write the Gazebo world

```bash
python -m rowpilot world --template ardupilot/world_template.sdf --out build/gz/worlds/greenhouse.sdf \
    --assets-dir build/gz/models/greenhouse_assets --vehicle model://gh_firefly
```

Greenhouse options as for `run`. `--assets-dir` writes the procedural tomato plants (omit for plain
boxes); `--vehicle` puts that model at home. `ardupilot/install.sh` runs this for every Gazebo run.

## `ardupilot/install.sh`: the Gazebo files

```bash
ardupilot/install.sh <out_dir> [python] [firefly|iris]
```

Writes the drone models (FireFly from its CAD, with the D435i and Pi Camera), the greenhouse world,
OpenVINS's configuration, ArduCopter's parameter files and the window layout into `out_dir`
(`build/gz/` when called by the run script). `GREENHOUSE_ARGS` as above.

## Other scripts

| Command | What it does |
|---|---|
| `.venv-sim/bin/python scripts/camera_views.py --out docs/views --prefix lane-` | saves the D435i infrared, depth and Pi Camera images from a running Gazebo simulation (`--model`, `--timeout`) |
| `python scripts/bt_diagrams.py` | redraws the behaviour tree diagrams in `docs/bt/` from the code (`--out`) |
| `./scripts/setup_px4_sim.sh`, `./scripts/run_px4_sim.sh` | the same mission on PX4 SITL instead of ArduPilot (older; the drone uses ArduPilot) |
| `python -m rowpilot px4` | the tree against PX4 SITL (`--url`, `--world`, `--model`, `--max-time`, `--out`) |

## Tests and checks

```bash
pytest -q                          # Python: tree, perception, control, editor server (no Gazebo needed)
ruff check . && ruff format --check .
cd web && npm test                 # the editor page (Vitest)
cd web && npm run build            # type check + build
```

CI (`.github/workflows/test.yml`) runs all of these, plus three example missions, on every push.

## Ports and processes of a Gazebo run

| Process | What | Port / topic |
|---|---|---|
| `gz sim server`, `gz sim gui` | Gazebo: physics, cameras; the window | gz-transport (loopback) |
| `arducopter` | ArduCopter SITL | MAVLink tcp:5760 (rowpilot), 5762 (spare, e.g. a ground station); JSON UDP 9002 (Gazebo plugin) |
| `vio_bridge` | OpenVINS on the D435i images | publishes `/rowpilot/vio/odometry` |
| `python -m rowpilot.imu_synth` | the D435i IMU with a physics-based accelerometer | `/gh_firefly/d435i_imu_kin` |
| `python -m rowpilot.depth_view` | colours the depth image for the window | `/gh_firefly/depth_view` |
| `python -m rowpilot ardupilot` | the behaviour tree | MAVLink client |
| `python -m rowpilot editor` | the editor (if used) | HTTP 8000 |

To see what is running, and to stop everything by hand: [setup.md, stopping and cleaning up](setup.md#stopping-and-cleaning-up).
