#!/usr/bin/env bash
# Generate the rowpilot Gazebo resources for ArduPilot SITL into one folder:
#   <out>/models/gh_iris            Iris (ArduPilotPlugin) + RealSense D435i + Pi Camera Module 3
#   <out>/models/greenhouse_assets  procedural tomato plants and floor texture
#   <out>/worlds/greenhouse.sdf     the greenhouse with the drone at home
#   <out>/openvins/                 OpenVINS config for the D435i
# Usage: ardupilot/install.sh <out_dir> [python]
set -euo pipefail
OUT=${1:?output folder}
PY=${2:-python3}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT/models/gh_iris" "$OUT/worlds"
export PYTHONPATH="$HERE/..${PYTHONPATH:+:$PYTHONPATH}"
"$PY" - "$OUT" <<'PY'
import sys
from pathlib import Path
from rowpilot.openvins_config import write_openvins_config
from rowpilot.sensors import gh_iris_sdf
out = Path(sys.argv[1])
(out / "models/gh_iris/model.sdf").write_text(gh_iris_sdf())
(out / "models/gh_iris/model.config").write_text(
    '<?xml version="1.0"?>\n<model>\n  <name>gh_iris</name>\n  <version>1.0</version>\n'
    '  <sdf version="1.9">model.sdf</sdf>\n  <description>ArduPilot Iris with the real drone sensors: '
    "RealSense D435i (front, tilted down) and Pi Camera Module 3 (left).</description>\n</model>\n")
write_openvins_config(out / "openvins")
PY
"$PY" -m rowpilot world --template "$HERE/world_template.sdf" --out "$OUT/worlds/greenhouse.sdf" \
  --assets-dir "$OUT/models/greenhouse_assets" --vehicle model://gh_iris
cp "$HERE/gh_iris.parm" "$OUT/gh_iris.parm"
echo "Installed into $OUT"
