#!/usr/bin/env bash
# Generate the rowpilot Gazebo resources for ArduPilot SITL into one folder:
#   <out>/models/firefly            FireFly 2 airframe (from its CAD, ArduPilotPlugin)
#   <out>/models/gh_firefly         FireFly 2 + RealSense D435i + Pi Camera Module 3 (the real drone)
#   <out>/models/gh_iris            ArduPilot Iris + the same sensors
#   <out>/models/greenhouse_assets  procedural tomato plants and floor texture
#   <out>/worlds/greenhouse.sdf     the greenhouse with the chosen drone at home
#   <out>/openvins/                 OpenVINS config for the D435i
#   <out>/*.parm                    ArduCopter parameters
# Usage: ardupilot/install.sh <out_dir> [python] [firefly|iris]
set -euo pipefail
OUT=${1:?output folder}
PY=${2:-python3}
VEHICLE=${3:-firefly}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT/models/firefly/meshes" "$OUT/models/gh_firefly" "$OUT/models/gh_iris" "$OUT/worlds"
cp "$HERE"/firefly/meshes/*.stl "$OUT/models/firefly/meshes/"
export PYTHONPATH="$HERE/..${PYTHONPATH:+:$PYTHONPATH}"
"$PY" - "$OUT" <<'PY'
import sys
from pathlib import Path
from rowpilot.airframe import firefly_sdf, gh_firefly_sdf
from rowpilot.openvins_config import write_openvins_config
from rowpilot.sensors import gh_iris_sdf
out = Path(sys.argv[1])


def model(name: str, sdf: str, description: str) -> None:
    (out / f"models/{name}/model.sdf").write_text(sdf)
    (out / f"models/{name}/model.config").write_text(
        f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
        f"  <sdf version=\"1.9\">model.sdf</sdf>\n  <description>{description}</description>\n</model>\n")


model("firefly", firefly_sdf(), "FireFly 2 airframe (Drone V3.2 D435) for ArduPilot SITL.")
model("gh_firefly", gh_firefly_sdf(), "FireFly 2 with the real drone sensors: RealSense D435i (front, "
      "tilted down) and Pi Camera Module 3 (left).")
model("gh_iris", gh_iris_sdf(), "ArduPilot Iris with the real drone sensors: RealSense D435i (front, "
      "tilted down) and Pi Camera Module 3 (left).")
write_openvins_config(out / "openvins")
PY
"$PY" -m rowpilot world --template "$HERE/world_template.sdf" --out "$OUT/worlds/greenhouse.sdf" \
  --assets-dir "$OUT/models/greenhouse_assets" --vehicle "model://gh_$VEHICLE"
cp "$HERE/gps_denied.parm" "$HERE/firefly.parm" "$OUT/"
echo "Installed into $OUT ($VEHICLE)"
