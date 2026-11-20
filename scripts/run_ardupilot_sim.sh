#!/usr/bin/env bash
# Fly the rowpilot mission with ArduPilot SITL (ArduCopter), as on the real drone:
#   Gazebo greenhouse -> D435i -> OpenVINS -> ArduPilot EKF3 (GPS-denied) -> behaviour tree.
# Usage: scripts/run_ardupilot_sim.sh [ardupilot_dir] [ardupilot_gazebo_dir]
#        HEADLESS=1 to run without the simulator window.
#        VEHICLE=iris to fly the ArduPilot Iris instead of the FireFly 2 (the real drone).
#        Used by the behaviour tree editor: TREE=<graph.json> flies that tree, STREAM=1 prints the run
#        as JSON lines, OUT_DIR=<folder> for the results, GREENHOUSE_ARGS="--lanes 2 ..." for the
#        greenhouse (the world and the mission).
#        SIM=isaac to simulate in NVIDIA Isaac Sim instead of Gazebo (FireFly only; set it up once with
#        scripts/setup_isaac_sim.sh). Same world, ArduPilot, OpenVINS and tree; see isaac/greenhouse_isaac.py.
#        Isaac Sim renders headless and rowpilot.isaac_view shows the drone and its cameras in a window;
#        ISAAC_WINDOW=1 opens Isaac Sim's own window instead (needs a display Vulkan can draw on).
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
SIM=${SIM:-gazebo}
ISAAC_PY=${ISAAC_PYTHON:-$REPO/.venv-isaac/bin/python}
MODEL=gh_$VEHICLE
case $SIM in
  gazebo) ;;
  isaac)
    [ "$VEHICLE" = firefly ] || { echo "SIM=isaac flies the FireFly only (VEHICLE=firefly)" >&2; exit 2; }
    [ -x "$ISAAC_PY" ] || { echo "No Isaac Sim Python at $ISAAC_PY: run scripts/setup_isaac_sim.sh (or set ISAAC_PYTHON)" >&2; exit 2; } ;;
  *) echo "SIM must be gazebo or isaac" >&2; exit 2 ;;
esac
case $VEHICLE in
  firefly) FRAME_PARM=$GZ/firefly.parm ;;
  iris) FRAME_PARM=$AP/Tools/autotest/default_params/gazebo-iris.parm ;;
  *) echo "VEHICLE must be firefly or iris" >&2; exit 2 ;;
esac
mkdir -p "$OUT"

stop_all() {
  pkill -x vio_bridge || true; pkill -f "rowpilot.imu_synth" || true; pkill -f "rowpilot.depth_view" || true; pkill -f "rowpilot.isaac_view" || true; pkill -x arducopter || true
  pkill -f "gz sim" || true; pkill -f "isaac/greenhouse_isaac.py" || true; pkill -f "rowpilot.isaac_relay" || true
  for _ in $(seq 1 20); do
    pgrep -x arducopter > /dev/null || pgrep -f "gz sim" > /dev/null || pgrep -f "isaac/greenhouse_isaac.py" > /dev/null || return 0
    sleep 0.5
  done
  pkill -9 -x arducopter || true; pkill -9 -f "gz sim" || true; pkill -9 -f "isaac/greenhouse_isaac.py" || true
}
trap stop_all EXIT
stop_all

# ArduCopter SITL's MAVLink port (TCP 5760) and the simulator's SITL port (UDP 9002) must be free. Another
# ArduPilot simulation (e.g. one in a docker container on the host network, which pkill above cannot
# stop) would otherwise answer instead: our ArduCopter fails to bind and the mission flies the other one.
busy=$( (ss -Htln 'sport = :5760'; ss -Huln 'sport = :9002') | awk '{print $4}')
if [ -n "$busy" ]; then
  echo "Ports in use by another ArduPilot simulation: $(echo "$busy" | tr '\n' ' ')" >&2
  echo "Stop it first (a docker container: docker ps; docker stop <name>), then run this again." >&2
  exit 1
fi

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
# The world is loaded once the simulator listens for SITL (UDP 9002).
sim_loaded() {  # <pid> <seconds>
  for _ in $(seq 1 "$2"); do
    ss -Huln 'sport = :9002' | grep -q . && return 0
    kill -0 "$1" 2> /dev/null || return 1
    sleep 1
  done
  return 1
}
if [ "$SIM" = isaac ]; then
  # Isaac Sim streams the sensors to rowpilot.isaac_relay, which republishes them on the Gazebo topics.
  echo "Starting the Isaac Sim relay (log: $OUT/isaac_relay.log) ..."
  "$PY" -u -m rowpilot.isaac_relay > "$OUT/isaac_relay.log" 2>&1 &
  echo "Starting Isaac Sim (log: $OUT/isaac.log; the first start compiles shaders, several minutes) ..."
  ISAAC_FLAGS="--headless"; [ "${ISAAC_WINDOW:-0}" = 1 ] && [ "${HEADLESS:-0}" != 1 ] && ISAAC_FLAGS=""
  # Each camera costs render time: the follow view only when a window shows it, the Pi Camera on request.
  [ "${HEADLESS:-0}" = 1 ] && ISAAC_FLAGS="$ISAAC_FLAGS --chase 0"
  [ "${ISAAC_PICAM:-0}" = 1 ] && ISAAC_FLAGS="$ISAAC_FLAGS --picam"
  [ -n "${ISAAC_LOOK:-}" ] && ISAAC_FLAGS="$ISAAC_FLAGS --look $ISAAC_LOOK"
  # shellcheck disable=SC2086
  "$ISAAC_PY" -u "$REPO/isaac/greenhouse_isaac.py" --gz-dir "$GZ" --model "$MODEL" $ISAAC_FLAGS > "$OUT/isaac.log" 2>&1 &
  sim_loaded $! 900 || { echo "Isaac Sim did not load the greenhouse, see $OUT/isaac.log" >&2; exit 1; }
  if [ "${HEADLESS:-0}" != 1 ] && [ "${ISAAC_WINDOW:-0}" != 1 ]; then
    echo "Opening the Isaac Sim view (log: $OUT/isaac_view.log) ..."
    "$PY" -u -m rowpilot.isaac_view --model "$MODEL" > "$OUT/isaac_view.log" 2>&1 &
  fi
else
  echo "Starting Gazebo (log: $OUT/gazebo.log) ..."
  # Gazebo sometimes segfaults while starting (a getenv race in gz-transport's discovery thread): start it again.
  for attempt in 1 2 3; do
    # shellcheck disable=SC2086
    gz sim $GZ_FLAGS "$GZ/worlds/greenhouse.sdf" >> "$OUT/gazebo.log" 2>&1 &
    GZ_PID=$!
    sim_loaded "$GZ_PID" 180 && break
    [ "$attempt" = 3 ] && { echo "Gazebo did not load the world, see $OUT/gazebo.log" >&2; exit 1; }
    echo "Gazebo did not start (attempt $attempt), retrying ..."
    stop_all
  done
fi
echo "Starting ArduCopter SITL, $VEHICLE (log: $OUT/arducopter.log) ..."
(cd "$OUT" && "$AP/build/sitl/bin/arducopter" --model JSON --speedup 1 --slave 0 -I0 \
  --defaults "$AP/Tools/autotest/default_params/copter.parm,$GZ/gps_denied.parm,$FRAME_PARM" \
  > "$OUT/arducopter.log" 2>&1 &)
echo "Starting OpenVINS on the D435i (log: $OUT/openvins.log) ..."
# Gazebo's IMU accelerometer is wrong while the drone rotates: OpenVINS gets one computed from the
# physics (rowpilot.imu_synth), with Gazebo's gyro. Isaac Sim computes that IMU itself.
if [ "$SIM" = gazebo ]; then
  "$PY" -u -m rowpilot.imu_synth --model "$MODEL" > "$OUT/imu_synth.log" 2>&1 &
fi
"$OV_BRIDGE" "$GZ/openvins/estimator_config.yaml" "/$MODEL" /rowpilot/vio/odometry "/$MODEL/d435i_imu_kin" \
  > "$OUT/openvins.log" 2>&1 &
if [ "${HEADLESS:-0}" != 1 ] && [ "$SIM" = gazebo ]; then
  # Window: depth colour-coded for its live screen, and the 3D view following the drone.
  "$PY" -u -m rowpilot.depth_view --model "$MODEL" > "$OUT/depth_view.log" 2>&1 &
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 \
    --req "data: \"$MODEL\"" > /dev/null 2>&1 || true
  gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 5000 \
    --req "x: -1.2, y: -0.6, z: 2.8" > /dev/null 2>&1 || true
fi
sleep 5
echo "Flying the mission (Ctrl+C to stop) ..."
# shellcheck disable=SC2086
"$PY" -u -m rowpilot ardupilot --vehicle "$VEHICLE" --out "$OUT" ${GREENHOUSE_ARGS:-} \
  ${TREE:+--tree "$TREE"} ${STREAM:+--stream}
echo "Results in $OUT"
