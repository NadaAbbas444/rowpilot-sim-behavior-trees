#!/usr/bin/env bash
# Fly the rowpilot mission with ArduPilot SITL (ArduCopter), as on the real drone:
#   Gazebo greenhouse -> D435i -> OpenVINS -> ArduPilot EKF3 (GPS-denied) -> behaviour tree.
# Usage: scripts/run_ardupilot_sim.sh [ardupilot_dir] [ardupilot_gazebo_dir] [openvins_dir]
#        HEADLESS=1 to run without the Gazebo window.
set -euo pipefail
export PYTHONNOUSERSITE=1
REPO=$(cd "$(dirname "$0")/.." && pwd)
AP=${1:-$HOME/ardupilot}
AG=${2:-$HOME/ardupilot_gazebo}
OV_BRIDGE=${VIO_BRIDGE:-$REPO/build/vio/vio_bridge}
PY=$REPO/.venv-sim/bin/python
GZ=$REPO/build/gz
OUT=$REPO/runs/ardupilot-$(date +%Y%m%d-%H%M%S)
mkdir -p "$OUT"

stop_all() {
  pkill -x vio_bridge || true; pkill -x arducopter || true; pkill -f "gz sim" || true
  for _ in $(seq 1 20); do pgrep -x arducopter > /dev/null || pgrep -f "gz sim" > /dev/null || return 0; sleep 0.5; done
  pkill -9 -x arducopter || true; pkill -9 -f "gz sim" || true
}
trap stop_all EXIT
stop_all

"$REPO/ardupilot/install.sh" "$GZ" "$PY" > /dev/null
export GZ_SIM_SYSTEM_PLUGIN_PATH=$AG/build
export GZ_SIM_RESOURCE_PATH=$GZ/models:$GZ/worlds:$AG/models:$AG/worlds
GZ_FLAGS="-r"; [ "${HEADLESS:-0}" = 1 ] && GZ_FLAGS="-s -r --headless-rendering"
echo "Starting Gazebo (log: $OUT/gazebo.log) ..."
# shellcheck disable=SC2086
gz sim $GZ_FLAGS "$GZ/worlds/greenhouse.sdf" > "$OUT/gazebo.log" 2>&1 &
sleep 15
echo "Starting ArduCopter SITL (log: $OUT/arducopter.log) ..."
(cd "$OUT" && "$AP/build/sitl/bin/arducopter" --model JSON --speedup 1 --slave 0 -I0 \
  --defaults "$AP/Tools/autotest/default_params/copter.parm,$AP/Tools/autotest/default_params/gazebo-iris.parm,$GZ/gh_iris.parm" \
  > "$OUT/arducopter.log" 2>&1 &)
echo "Starting OpenVINS on the D435i (log: $OUT/openvins.log) ..."
"$OV_BRIDGE" "$GZ/openvins/estimator_config.yaml" /gh_iris > "$OUT/openvins.log" 2>&1 &
sleep 5
echo "Flying the mission (Ctrl+C to stop) ..."
"$PY" -u -m rowpilot ardupilot --out "$OUT"
echo "Results in $OUT"
