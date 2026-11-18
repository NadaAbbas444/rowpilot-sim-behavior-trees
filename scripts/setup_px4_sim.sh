#!/usr/bin/env bash
# One-time setup: PX4 SITL + Gazebo Harmonic + the rowpilot greenhouse drone (Ubuntu 22.04 / 24.04).
# Takes 20-40 minutes, mostly downloading and compiling PX4. Run from the repository folder:
#   ./scripts/setup_px4_sim.sh            (PX4 goes to ~/PX4-Autopilot)
#   ./scripts/setup_px4_sim.sh /some/dir  (PX4 goes to /some/dir/PX4-Autopilot)
set -euo pipefail
export PYTHONNOUSERSITE=1  # keep ~/.local packages (e.g. a newer protobuf) out of the simulator environment
REPO=$(cd "$(dirname "$0")/.." && pwd)
PX4=${1:-$HOME}/PX4-Autopilot

echo "== 1/5 PX4 source -> $PX4"
if [ ! -d "$PX4" ]; then
  git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/PX4/PX4-Autopilot.git "$PX4"
fi

echo "== 2/5 PX4 build tools and Gazebo Harmonic (asks for your sudo password)"
bash "$PX4/Tools/setup/ubuntu.sh" --no-nuttx

echo "== 3/5 Gazebo Python bindings (to read the simulated lidar and ground truth)"
sudo apt-get install -y python3-gz-transport13 python3-gz-msgs10

echo "== 4/5 Python environment .venv-sim (sees the Gazebo bindings) with rowpilot and MAVSDK"
python3 -m venv --system-site-packages "$REPO/.venv-sim"
"$REPO/.venv-sim/bin/pip" install -U pip
"$REPO/.venv-sim/bin/pip" install mavsdk -e "$REPO"

echo "== 5/5 greenhouse world + gh_x500 drone into PX4, then build"
"$REPO/px4/install.sh" "$PX4" "$REPO/.venv-sim/bin/python"
make -C "$PX4" px4_sitl

echo
echo "Done. Fly it with:  ./scripts/run_px4_sim.sh $PX4"
