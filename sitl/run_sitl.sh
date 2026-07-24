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
#
# Defaults to HEADLESS=1: Gazebo's 3D acceleration is broken in Apple-Silicon VMs
# (UTM renders a black world, Parallels renders garbage), and software rendering
# is slow. We validate mission execution from telemetry, not pixels. To force the
# GUI anyway: HEADLESS=0 ./run_sitl.sh ...
#
# To fly over the PLANNED TERRAIN instead of PX4's flat default world, generate
# it with `droneplanner gz-world`, copy the .sdf into
# $PX4_DIR/Tools/simulation/gz/worlds/, then:
#   PX4_GZ_WORLD=planner_terrain ./run_sitl.sh <plan> $PX4_DIR
set -euo pipefail

export HEADLESS="${HEADLESS:-1}"

PLAN="${1:?usage: run_sitl.sh <mission.plan> [PX4-Autopilot dir] [gz world name]}"
PX4_DIR="${2:-${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}}"
# World as a positional arg so terrain selection can't be silently forgotten
# (PX4 falls back to its FLAT default.sdf, which invalidates terrain testing).
WORLD="${3:-${PX4_GZ_WORLD:-}}"
if [[ -n "$WORLD" ]]; then
  export PX4_GZ_WORLD="$WORLD"
  if [[ ! -f "$PX4_DIR/Tools/simulation/gz/worlds/$WORLD.sdf" ]]; then
    echo "error: world '$WORLD' not found at $PX4_DIR/Tools/simulation/gz/worlds/$WORLD.sdf" >&2
    exit 1
  fi
fi

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
echo "PX4 SITL home: $PX4_HOME_LAT, $PX4_HOME_LON @ ${PX4_HOME_ALT} m (HEADLESS=$HEADLESS)"
# Tether plugin (tether_plugin/): if it has been built, put it on Gazebo's
# system-plugin path so the <plugin> block in the x500 model SDF resolves. Not
# built = not on the path = the block loads nothing, which Gazebo reports but
# does not treat as an error — hence the explicit banner either way.
TETHER_BUILD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/tether_plugin/build"
if [[ -d "$TETHER_BUILD" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}$TETHER_BUILD"
  echo "tether: plugin path $TETHER_BUILD (DRONEFLIGHT_TETHER_ENABLED=${DRONEFLIGHT_TETHER_ENABLED:-1}${DRONEFLIGHT_TETHER_CSV:+, csv=$DRONEFLIGHT_TETHER_CSV})"
  echo "        confirm it actually loaded: look for a '[tether] ENABLED ...' line below."
else
  echo "tether: plugin NOT built (no $TETHER_BUILD) — flying without payout disturbance."
fi

if [[ -n "${PX4_GZ_WORLD:-}" ]]; then
  echo "world: $PX4_GZ_WORLD"
else
  echo "world: *** PX4 default (default.sdf) — FLAT. Terrain-following is NOT being tested. ***"
  echo "       pass a world name as arg 3, e.g.: run_sitl.sh <plan> <px4-dir> planner_terrain"
fi

cd "$PX4_DIR"
exec make px4_sitl gz_x500
