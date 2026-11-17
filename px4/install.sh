#!/usr/bin/env bash
# Add the greenhouse world, the gh_x500 model and its airframe to a PX4-Autopilot checkout.
# Usage: px4/install.sh /path/to/PX4-Autopilot [python]   then: make px4_sitl (rebuild picks up the airframe)
set -euo pipefail
PX4=${1:?path to PX4-Autopilot}
PY=${2:-python3}
HERE=$(cd "$(dirname "$0")" && pwd)
GZ=$PX4/Tools/simulation/gz
AIRFRAMES=$PX4/ROMFS/px4fmu_common/init.d-posix/airframes

cp -r "$HERE/models/gh_x500" "$GZ/models/"
cp "$HERE/airframes/4099_gz_gh_x500" "$AIRFRAMES/"
grep -q 4099_gz_gh_x500 "$AIRFRAMES/CMakeLists.txt" ||
  sed -i 's/^\t4013_gz_x500_lidar_2d$/\t4013_gz_x500_lidar_2d\n\t4099_gz_gh_x500/' "$AIRFRAMES/CMakeLists.txt"
grep -q 4099_gz_gh_x500 "$AIRFRAMES/CMakeLists.txt" || { echo "could not register airframe" >&2; exit 1; }

PYTHONPATH="$HERE/..${PYTHONPATH:+:$PYTHONPATH}" "$PY" -m ghdrone world \
  --template "$GZ/worlds/default.sdf" --out "$GZ/worlds/greenhouse.sdf"
echo "Installed. Rebuild with: make -C $PX4 px4_sitl"
