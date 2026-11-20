#!/usr/bin/env bash
# Fly the rowpilot mission with ArduPilot SITL (ArduCopter), as on the real drone:
#   Gazebo greenhouse -> D435i -> OpenVINS -> ArduPilot EKF3 (GPS-denied) -> behaviour tree.
# Usage: scripts/run_ardupilot_sim.sh [ardupilot_dir] [ardupilot_gazebo_dir]
#        HEADLESS=1 to run without the Gazebo window.
#        VEHICLE=iris to fly the ArduPilot Iris instead of the FireFly 2 (the real drone).
#        VEHICLE=trolley for the pipe-rail trolley: no ArduPilot (plain motor control), see trolley_gazebo.py.
#        Used by the behaviour tree editor: TREE=<graph.json> flies that tree, STREAM=1 prints the run
#        as JSON lines, OUT_DIR=<folder> for the results, GREENHOUSE_ARGS="--lanes 2 ..." for the
#        greenhouse (the world and the mission).
set -euo pipefail
export PYTHONNOUSERSITE=1
REPO=$(cd "$(dirname "$0")/.." && pwd)
AP=${1:-$HOME/ardupilot}
AG=${2:-$HOME/ardupilot_gazebo}
OV_BRIDGE=${VIO_BRIDGE:-$REPO/build/vio/vio_bridge}
PY=$REPO/.venv-sim/bin/python
GZ=$REPO/build/gz
OUT=${OUT_DIR:-$REPO/runs/ardupilot-$(date +%Y%m%d-%H%M%S)}
VEHICLE=${VEHICLE:-firefly}
MODEL=gh_$VEHICLE
case $VEHICLE in
  firefly) FRAME_PARM=$GZ/firefly.parm ;;
  iris) FRAME_PARM=$AP/Tools/autotest/default_params/gazebo-iris.parm ;;
  trolley) FRAME_PARM= ;;
  *) echo "VEHICLE must be firefly, iris or trolley" >&2; exit 2 ;;
esac
mkdir -p "$OUT"
# One simulation at a time: a second run would stop this one's Gazebo and drive the same model. The lock
# is held while this script (and what it started) runs.
mkdir -p "$REPO/runs"
exec 9> "$REPO/runs/.simulation.lock"
if ! flock -n 9; then
  echo "Another simulation run is going (scripts/run_ardupilot_sim.sh, maybe from the editor): stop it first." >&2
  exit 1
fi

stop_all() {
  pkill -x vio_bridge || true; pkill -f "rowpilot.imu_synth" || true; pkill -f "rowpilot.depth_view" || true; pkill -x arducopter || true
  pkill -f "gz sim" || true
  for _ in $(seq 1 20); do pgrep -x arducopter > /dev/null || pgrep -f "gz sim" > /dev/null || return 0; sleep 0.5; done
  pkill -9 -x arducopter || true; pkill -9 -f "gz sim" || true
}
trap stop_all EXIT
stop_all

GREENHOUSE_ARGS=${GREENHOUSE_ARGS:-} "$REPO/ardupilot/install.sh" "$GZ" "$PY" "$VEHICLE" > /dev/null
# Keep gz-transport on loopback: with several network interfaces (Wi-Fi, docker0 ...) the GUI's
# "starting world" message can miss the server, which then waits for it forever.
export GZ_IP=${GZ_IP:-127.0.0.1}
# Render the cameras on the NVIDIA GPU if there is one: on hybrid (PRIME) laptops OpenGL otherwise
# falls back to llvmpipe on the CPU, and the D435i + Pi camera rendering slows Gazebo ~100x.
# GPU=0 to keep the default renderer.
if [ "${GPU:-1}" = 1 ] && nvidia-smi -L > /dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
  if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  fi
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH=$AG/build
export GZ_SIM_RESOURCE_PATH=$GZ/models:$GZ/worlds:$AG/models:$AG/worlds
# The window shows the 3D view following the drone and live screens of its cameras (gui.config).
GZ_FLAGS="-r --gui-config $GZ/gui.config"; [ "${HEADLESS:-0}" = 1 ] && GZ_FLAGS="-s -r --headless-rendering"
echo "Starting Gazebo (log: $OUT/gazebo.log) ..."
# The world is loaded once the ArduPilot plugin listens for SITL (UDP 9002); the trolley's world, once
# its simulation clock ticks. Gazebo sometimes segfaults while starting (a getenv race in gz-transport's
# discovery thread): start it again.
gz_loaded() {
  for _ in $(seq 1 180); do
    if [ "$VEHICLE" = trolley ]; then
      timeout 2 gz topic -e -n 1 -t /world/greenhouse/clock > /dev/null 2>&1 && return 0
    else
      ss -Huln 'sport = :9002' | grep -q . && return 0
    fi
    kill -0 "$GZ_PID" 2> /dev/null || return 1
    sleep 1
  done
  return 1
}
for attempt in 1 2 3; do
  # shellcheck disable=SC2086
  gz sim $GZ_FLAGS "$GZ/worlds/greenhouse.sdf" >> "$OUT/gazebo.log" 2>&1 &
  GZ_PID=$!
  gz_loaded && break
  [ "$attempt" = 3 ] && { echo "Gazebo did not load the world, see $OUT/gazebo.log" >&2; exit 1; }
  echo "Gazebo did not start (attempt $attempt), retrying ..."
  stop_all
done
if [ "$VEHICLE" != trolley ]; then
  echo "Starting ArduCopter SITL, $VEHICLE (log: $OUT/arducopter.log) ..."
  (cd "$OUT" && "$AP/build/sitl/bin/arducopter" --model JSON --speedup 1 --slave 0 -I0 \
    --defaults "$AP/Tools/autotest/default_params/copter.parm,$GZ/gps_denied.parm,$FRAME_PARM" \
    > "$OUT/arducopter.log" 2>&1 &)
fi
echo "Starting OpenVINS on the D435i (log: $OUT/openvins.log) ..."
# Gazebo's IMU accelerometer is wrong while the drone rotates: OpenVINS gets one computed from the
# physics (rowpilot.imu_synth), with Gazebo's gyro.
"$PY" -u -m rowpilot.imu_synth --model "$MODEL" > "$OUT/imu_synth.log" 2>&1 &
"$OV_BRIDGE" "$GZ/openvins/estimator_config.yaml" "/$MODEL" /rowpilot/vio/odometry "/$MODEL/d435i_imu_kin" \
  > "$OUT/openvins.log" 2>&1 &
if [ "${HEADLESS:-0}" != 1 ]; then
  # Window: depth colour-coded for its live screen, and the 3D view following the drone.
  "$PY" -u -m rowpilot.depth_view --model "$MODEL" > "$OUT/depth_view.log" 2>&1 &
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 \
    --req "data: \"$MODEL\"" > /dev/null 2>&1 || true
  gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 5000 \
    --req "x: -1.2, y: -0.6, z: 2.8" > /dev/null 2>&1 || true
fi
sleep 5
if [ "$VEHICLE" = trolley ]; then
  echo "Driving the mission (Ctrl+C to stop) ..."
  # shellcheck disable=SC2086
  "$PY" -u -m rowpilot trolley-gazebo --out "$OUT" ${GREENHOUSE_ARGS:-} ${TREE:+--tree "$TREE"} ${STREAM:+--stream}
else
  echo "Flying the mission (Ctrl+C to stop) ..."
  # shellcheck disable=SC2086
  "$PY" -u -m rowpilot ardupilot --vehicle "$VEHICLE" --out "$OUT" ${GREENHOUSE_ARGS:-} \
    ${TREE:+--tree "$TREE"} ${STREAM:+--stream}
fi
echo "Results in $OUT"
