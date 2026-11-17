#!/usr/bin/env bash
# Start PX4 SITL with the Gazebo window, then fly the rowpilot behaviour tree through the greenhouse.
#   ./scripts/run_px4_sim.sh [path/to/PX4-Autopilot]    add HEADLESS=1 in front to run without the window
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
PX4=${1:-$HOME/PX4-Autopilot}
OUT=$REPO/runs/px4-$(date +%Y%m%d-%H%M%S)
POSE=$("$REPO/.venv-sim/bin/python" -c "from rowpilot.gazebo_world import spawn_pose; from rowpilot.layout import GreenhouseLayout; print(spawn_pose(GreenhouseLayout()))")
mkdir -p "$OUT"

cleanup() { pkill -f "px4_sitl_default/bin/px4" || true; pkill -f "gz sim" || true; }
trap cleanup EXIT
cleanup; sleep 1

echo "Starting PX4 + Gazebo (log: $OUT/px4.log) ..."
(cd "$PX4/build/px4_sitl_default" &&
  PX4_SYS_AUTOSTART=4099 PX4_GZ_WORLD=greenhouse PX4_GZ_MODEL_POSE="$POSE" ./bin/px4 -d > "$OUT/px4.log" 2>&1 &)
until grep -q "Ready for takeoff" "$OUT/px4.log" 2>/dev/null; do sleep 1; done

echo "PX4 ready. Flying the mission (Ctrl+C to stop) ..."
"$REPO/.venv-sim/bin/python" -u -m rowpilot px4 --out "$OUT"
echo "Results in $OUT"
