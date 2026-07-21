#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo with home set to a planned mission's AO.
#
# Run this ON THE UBUNTU VM (see ../RUNBOOK.md). It is untested from macOS —
# macOS cannot run PX4 SITL + Gazebo. The home-injection logic it relies on
# (`droneflight sitl-env`) IS unit-tested; this wrapper is just glue.
#
# Usage:
#   ./run_sitl.sh /path/to/mission.plan [/path/to/PX4-Autopilot]
#
# Then, in a second shell (inside the VM):
#   droneflight fly --plan /path/to/mission.plan --url udp://:14540
set -euo pipefail

PLAN="${1:?usage: run_sitl.sh <mission.plan> [PX4-Autopilot dir]}"
PX4_DIR="${2:-${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}}"

if ! command -v droneflight >/dev/null 2>&1; then
  echo "error: droneflight not on PATH — activate the venv / pip install -e ." >&2
  exit 1
fi
if [[ ! -d "$PX4_DIR" ]]; then
  echo "error: PX4-Autopilot not found at $PX4_DIR (pass it as arg 2)" >&2
  exit 1
fi

# Place the SITL vehicle at the plan's home, not PX4's Zurich default.
eval "$(droneflight sitl-env --plan "$PLAN")"
echo "PX4 SITL home: $PX4_HOME_LAT, $PX4_HOME_LON @ ${PX4_HOME_ALT} m"

cd "$PX4_DIR"
exec make px4_sitl gz_x500
