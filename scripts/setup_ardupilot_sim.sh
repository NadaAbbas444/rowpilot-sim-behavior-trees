#!/usr/bin/env bash
# One-time setup: ArduPilot SITL (ArduCopter) + ardupilot_gazebo + OpenVINS + the rowpilot VIO bridge,
# on Ubuntu 22.04 / 24.04 with Gazebo Harmonic. Takes 20-40 minutes. Run from the rowpilot-bt folder:
#   ./scripts/setup_ardupilot_sim.sh [install_dir]     (default: ~, i.e. ~/ardupilot, ~/ardupilot_gazebo, ~/open_vins)
# Repeats the steps used to fly the mission in the cloud test machine (Ubuntu 24.04).
set -euo pipefail
export PYTHONNOUSERSITE=1
REPO=$(cd "$(dirname "$0")/.." && pwd)
DIR=${1:-$HOME}

echo "== 1/6 system packages (Gazebo Harmonic dev, OpenVINS dependencies)"
if ! command -v gz > /dev/null; then
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
fi
sudo apt-get update -q
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q --no-install-recommends \
  gz-harmonic libgz-sim8-dev libgz-transport13-dev libgz-msgs10-dev python3-gz-transport13 python3-gz-msgs10 \
  rapidjson-dev libopencv-dev libeigen3-dev libceres-dev libboost-system-dev libboost-filesystem-dev \
  libboost-thread-dev libboost-date-time-dev cmake g++ git python3-venv \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

echo "== 2/6 ArduPilot SITL (ArduCopter) -> $DIR/ardupilot"
[ -d "$DIR/ardupilot" ] || git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/ArduPilot/ardupilot.git "$DIR/ardupilot"
(cd "$DIR/ardupilot" && python3 -m pip install --user -q empy==3.3.4 pexpect future lxml && ./waf configure --board sitl && ./waf copter)

echo "== 3/6 ardupilot_gazebo plugin -> $DIR/ardupilot_gazebo"
[ -d "$DIR/ardupilot_gazebo" ] || git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$DIR/ardupilot_gazebo"
cmake -S "$DIR/ardupilot_gazebo" -B "$DIR/ardupilot_gazebo/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$DIR/ardupilot_gazebo/build" -j"$(nproc)"

echo "== 4/6 OpenVINS (without ROS) -> $DIR/open_vins"
[ -d "$DIR/open_vins" ] || git clone --depth 1 https://github.com/rpng/open_vins.git "$DIR/open_vins"
cmake -S "$DIR/open_vins/ov_msckf" -B "$DIR/open_vins/ov_msckf/build" -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$DIR/open_vins/ov_msckf/build" --target ov_msckf_lib -j"$(nproc)"

echo "== 5/6 rowpilot VIO bridge (D435i -> OpenVINS) -> build/vio/vio_bridge"
cmake -S "$REPO/vio_bridge" -B "$REPO/build/vio" -DOPENVINS_DIR="$DIR/open_vins"
cmake --build "$REPO/build/vio" -j"$(nproc)"

echo "== 6/6 Python environment .venv-sim (sees the Gazebo bindings)"
[ -d "$REPO/.venv-sim" ] || python3 -m venv --system-site-packages "$REPO/.venv-sim"
"$REPO/.venv-sim/bin/pip" install -q -U pip
"$REPO/.venv-sim/bin/pip" install -q pymavlink -e "$REPO"

echo
echo "Done. Fly it with:  ./scripts/run_ardupilot_sim.sh $DIR/ardupilot $DIR/ardupilot_gazebo"
