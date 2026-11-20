#!/usr/bin/env bash
# One-time setup for SIM=isaac: NVIDIA Isaac Sim 4.5 (pip package, Python 3.10) in .venv-isaac.
# Run scripts/setup_ardupilot_sim.sh first: ArduPilot SITL, OpenVINS, the VIO bridge and .venv-sim are
# shared with the Gazebo simulation. Then fly it with:
#   SIM=isaac ./scripts/run_ardupilot_sim.sh ~/ardupilot ~/ardupilot_gazebo
#
# Needs: Ubuntu 22.04+, an NVIDIA RTX GPU with driver >= 535, ~25 GB of disk for the packages and their
# caches (more for the shader cache), python3.10 with venv. NVIDIA recommends 16 GB+ of GPU memory; on
# 8 GB, run with HEADLESS=1 and close other GPU programs.
# The first start asks you to accept NVIDIA's Omniverse licence (EULA) in this terminal.
set -euo pipefail
export PYTHONNOUSERSITE=1
REPO=$(cd "$(dirname "$0")/.." && pwd)
VENV=$REPO/.venv-isaac
NEED_GB=${ISAAC_DISK_GB:-30}

echo "== 1/3 checks"
if ! nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader; then
  echo "No NVIDIA GPU / driver found: Isaac Sim needs an RTX GPU." >&2; exit 1
fi
drv=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1 | cut -d. -f1)
[ "$drv" -ge 535 ] || { echo "NVIDIA driver $drv is too old for Isaac Sim 4.5 (needs >= 535)." >&2; exit 1; }
free_gb=$(df -BG --output=avail "$REPO" | tail -1 | tr -dc 0-9)
if [ "$free_gb" -lt "$NEED_GB" ]; then
  echo "Only ${free_gb} GB free on $(df --output=target "$REPO" | tail -1); Isaac Sim needs ~${NEED_GB} GB." >&2
  echo "Free some space, or set ISAAC_DISK_GB to a lower figure to try anyway." >&2
  exit 1
fi
command -v python3.10 > /dev/null || { echo "python3.10 not found (Isaac Sim 4.5 needs exactly 3.10)." >&2; exit 1; }
[ -x "$REPO/.venv-sim/bin/python" ] || echo "note: .venv-sim is missing; run scripts/setup_ardupilot_sim.sh too."

echo "== 2/3 Isaac Sim 4.5 -> $VENV (a large download)"
[ -d "$VENV" ] || python3.10 -m venv "$VENV"
"$VENV/bin/pip" install -q -U pip
"$VENV/bin/pip" install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com

echo "== 3/3 first start (licence prompt, then a headless start-up check)"
# With -c, not a heredoc: the licence prompt reads its answer from stdin, which must stay the terminal.
"$VENV/bin/python" -c '
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
print("Isaac Sim started")
app.close()
'

echo
echo "Done. Fly it with:  SIM=isaac ./scripts/run_ardupilot_sim.sh ~/ardupilot ~/ardupilot_gazebo"
