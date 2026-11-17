#!/usr/bin/env bash
# Start PX4 SITL with the Gazebo window, then fly the rowpilot behaviour tree through the greenhouse.
#   ./scripts/run_px4_sim.sh [path/to/PX4-Autopilot]    add HEADLESS=1 in front to run without the window
set -euo pipefail
# Ignore packages installed with "pip install --user" (~/.local): a newer protobuf there breaks
# Gazebo's Python bindings, which need the protobuf version that came with them from apt.
export PYTHONNOUSERSITE=1
REPO=$(cd "$(dirname "$0")/.." && pwd)
PX4=${1:-$HOME/PX4-Autopilot}
OUT=$REPO/runs/px4-$(date +%Y%m%d-%H%M%S)
# Repair the environment if a package is missing, e.g. when it was set up while ~/.local was
# visible and pip skipped packages it found there.
if ! "$REPO/.venv-sim/bin/python" -c "import numpy, py_trees, mavsdk, rowpilot" 2> /dev/null; then
  echo "Installing missing Python packages into .venv-sim ..."
  "$REPO/.venv-sim/bin/pip" install -q mavsdk -e "$REPO"
fi
POSE=$("$REPO/.venv-sim/bin/python" -c "from rowpilot.gazebo_world import spawn_pose; from rowpilot.layout import GreenhouseLayout; print(spawn_pose(GreenhouseLayout()))")
mkdir -p "$OUT"

# PX4 runs as a process named "px4"; Gazebo as "gz sim ...".
running() { pgrep -x px4 > /dev/null || pgrep -f "gz sim" > /dev/null; }
cleanup() {  # stop PX4 and Gazebo, and wait until they are really gone
  pkill -x px4 || true
  pkill -f "gz sim" || true
  for _ in $(seq 1 20); do running || return 0; sleep 0.5; done
  pkill -9 -x px4 || true
  pkill -9 -f "gz sim" || true
}
trap cleanup EXIT
cleanup

echo "Starting PX4 + Gazebo (log: $OUT/px4.log) ..."
(cd "$PX4/build/px4_sitl_default" &&
  PX4_SYS_AUTOSTART=4099 PX4_GZ_WORLD=greenhouse PX4_GZ_MODEL_POSE="$POSE" ./bin/px4 -d > "$OUT/px4.log" 2>&1 &)
for i in $(seq 1 120); do
  grep -q "Ready for takeoff" "$OUT/px4.log" 2>/dev/null && break
  if grep -q -E "Startup script returned with return value|server already running" "$OUT/px4.log" 2>/dev/null ||
     [ "$i" = 120 ]; then
    echo "PX4 did not start, see $OUT/px4.log:" >&2; tail -5 "$OUT/px4.log" >&2; exit 1
  fi
  sleep 1
done

echo "PX4 ready. Flying the mission (Ctrl+C to stop) ..."
"$REPO/.venv-sim/bin/python" -u -m rowpilot px4 --out "$OUT"
echo "Results in $OUT"
