#!/usr/bin/env bash
# One-time setup: ArduPilot SITL (ArduCopter) + ardupilot_gazebo + OpenVINS + the rowpilot VIO bridge,
# on Ubuntu 22.04 with Gazebo Harmonic. Takes 20-40 minutes, ~4 GB. Run from the repository folder:
#   ./scripts/setup_ardupilot_sim.sh [install_dir]     (default: ~, i.e. ~/ardupilot, ~/ardupilot_gazebo, ~/open_vins)
# ArduPilot, ardupilot_gazebo and OpenVINS are checked out at the commits the mission was flown with
# (below); an existing checkout in install_dir is used as it is. See docs/setup.md.
set -euo pipefail
export PYTHONNOUSERSITE=1
REPO=$(cd "$(dirname "$0")/.." && pwd)
DIR=${1:-$HOME}
# Tested versions (2026-11-18): change these only together with a test flight.
ARDUPILOT_SHA=d536b382aa2cb2c804a4530b25c11f8388d1ee06       # ArduPilot master, 2026-10-16
ARDUPILOT_GAZEBO_SHA=f116b03552d05c8d0b0bed5ed67033a1183121d1 # ardupilot_gazebo main, 2026-10-29
OPENVINS_SHA=69488123ed9362dd44b6f28e7f4680abbff1442b         # OpenVINS master, 2025-11-30

# clone_at <url> <dir> <commit>: a shallow checkout of exactly that commit, unless <dir> exists.
clone_at() {
  if [ -d "$2" ]; then
    local have
    have=$(git -C "$2" rev-parse HEAD 2> /dev/null || echo "?")
    [ "$have" = "$3" ] || echo "note: using the existing $2 at ${have:0:7} (tested: ${3:0:7})"
    return 0
  fi
  git init -q "$2"
  git -C "$2" remote add origin "$1"
  git -C "$2" fetch -q --depth 1 origin "$3"
  git -C "$2" checkout -q FETCH_HEAD
}

echo "== 1/6 system packages (Gazebo Harmonic dev, OpenVINS dependencies)"
if ! command -v gz > /dev/null; then
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
fi
PKGS="gz-harmonic libgz-sim8-dev libgz-transport13-dev libgz-msgs10-dev python3-gz-transport13 python3-gz-msgs10 \
  rapidjson-dev libopencv-dev libeigen3-dev libceres-dev libboost-system-dev libboost-filesystem-dev \
  libboost-thread-dev libboost-date-time-dev cmake g++ git python3-venv \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"
# shellcheck disable=SC2086
if dpkg -s $PKGS > /dev/null 2>&1; then
  echo "already installed"
else
  sudo apt-get update -q
  # shellcheck disable=SC2086
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q --no-install-recommends $PKGS
fi

echo "== 2/6 Python environment .venv-sim (sees the Gazebo bindings; also ArduPilot's build tools)"
# Not pip --user: PYTHONNOUSERSITE=1 hides ~/.local, so waf would not find empy there.
[ -d "$REPO/.venv-sim" ] || python3 -m venv --system-site-packages "$REPO/.venv-sim"
"$REPO/.venv-sim/bin/pip" install -q -U pip
"$REPO/.venv-sim/bin/pip" install -q empy==3.3.4 pexpect future lxml pymavlink -e "$REPO[plot]"

echo "== 3/6 ArduPilot SITL (ArduCopter) -> $DIR/ardupilot"
clone_at https://github.com/ArduPilot/ardupilot.git "$DIR/ardupilot" "$ARDUPILOT_SHA"
# An existing checkout may lack submodules (waf, mavlink, littlefs ...): fetch only the missing ones,
# leave the ones already checked out (possibly with local changes) alone.
# A submodule whose checkout lost all its files is repaired too; any other local change is kept.
(cd "$DIR/ardupilot" && git submodule status | awk '/^-/ {print $2}' | xargs -r git submodule update --init --depth 1 &&
  git submodule foreach --quiet 'git ls-files | grep -q . || echo "$sm_path"' |
  xargs -r git submodule update --init --force --depth 1)
(cd "$DIR/ardupilot" &&
  PATH="$REPO/.venv-sim/bin:$PATH" ./waf configure --board sitl && PATH="$REPO/.venv-sim/bin:$PATH" ./waf copter)

echo "== 4/6 ardupilot_gazebo plugin -> $DIR/ardupilot_gazebo"
clone_at https://github.com/ArduPilot/ardupilot_gazebo.git "$DIR/ardupilot_gazebo" "$ARDUPILOT_GAZEBO_SHA"
cmake -S "$DIR/ardupilot_gazebo" -B "$DIR/ardupilot_gazebo/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$DIR/ardupilot_gazebo/build" -j"$(nproc)"

echo "== 5/6 OpenVINS (without ROS) -> $DIR/open_vins"
clone_at https://github.com/rpng/open_vins.git "$DIR/open_vins" "$OPENVINS_SHA"
cmake -S "$DIR/open_vins/ov_msckf" -B "$DIR/open_vins/ov_msckf/build" -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$DIR/open_vins/ov_msckf/build" --target ov_msckf_lib -j"$(nproc)"

echo "== 6/6 rowpilot VIO bridge (D435i -> OpenVINS) -> build/vio/vio_bridge"
cmake -S "$REPO/vio_bridge" -B "$REPO/build/vio" -DOPENVINS_DIR="$DIR/open_vins"
cmake --build "$REPO/build/vio" -j"$(nproc)"


echo
echo "Done. Fly it with:  ./scripts/run_ardupilot_sim.sh $DIR/ardupilot $DIR/ardupilot_gazebo"
