#!/usr/bin/env bash
# Behaves like run_ardupilot_sim.sh with STREAM=1: progress lines, OpenVINS's log, then the run as JSON.
set -e
echo "Starting Gazebo (log: $OUT_DIR/gazebo.log) ..."
if [ "${FAKE_SLOW_START:-0}" = 1 ]; then sleep 600; fi
echo "greenhouse: $GREENHOUSE_ARGS window: $HEADLESS vehicle: $VEHICLE"
printf '\033[33m[init]: failed static init: no accel jerk detected\n[init]: failed static init: no accel jerk detected\n[init]: successful initialization in 0.0002 seconds\n[ZUPT]: accepted\n' > "$OUT_DIR/openvins.log"
sleep 0.8
ID=$(python3 -c "import json,sys; g=json.load(open(sys.argv[1])); print(next(n['id'] for n in g['nodes'] if n['name']=='Takeoff'))" "$TREE")
echo '@rowpilot {"kind": "event", "type": "system", "source": "ekf", "node": "ekf", "msg": "EKF3 has a position from the visual odometry"}'
echo '@rowpilot {"kind": "event", "type": "system", "source": "mavlink", "node": "mavlink", "msg": "GUIDED mode, motors armed"}'
echo "@rowpilot {\"kind\": \"event\", \"t\": 18.0, \"type\": \"status\", \"node\": \"Takeoff\", \"id\": \"$ID\", \"status\": \"RUNNING\"}"
for i in 1 2 3; do
  echo "@rowpilot {\"kind\": \"tick\", \"t\": 18.$i, \"x\": -2.5, \"y\": 0.0, \"z\": 0.$i, \"yaw\": 0.0, \"est_x\": -2.5, \"est_y\": 0.0, \"battery\": 100, \"vo_ok\": true, \"armed\": true, \"node\": \"Takeoff\"}"
done
if [ "${FAKE_WAIT_FOR_STOP:-0}" = 1 ]; then
  read -r line
  echo '@rowpilot {"kind": "event", "type": "system", "source": "editor", "node": "editor", "msg": "stop requested: landing"}'
  OUTCOME="stopped by user (ArduPilot landing in place)"
else
  OUTCOME=completed
fi
echo "@rowpilot {\"kind\": \"summary\", \"outcome\": \"$OUTCOME\", \"sim_duration_s\": 20.5, \"battery_end_pct\": 99, \"lanes_completed\": [], \"lane_rms_error_m\": {}, \"max_lane_error_m\": 0}"
echo "Results in $OUT_DIR"
