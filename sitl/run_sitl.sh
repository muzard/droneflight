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
# The plugin reads DRONEFLIGHT_TETHER_* in the Gazebo SERVER process. Export
# them so they reach it however this script was invoked — a bare `VAR=val
# ./run_sitl.sh` inherits, but `export VAR; ./run_sitl.sh` would not propagate
# past here without this. (An unexported ENABLED=0 is half of why the first
# "OFF" arm recorded enabled=1.)
export DRONEFLIGHT_TETHER_ENABLED DRONEFLIGHT_TETHER_CSV DRONEFLIGHT_TETHER_SPAN

# ⚠️ Kill a stale Gazebo server before launching. PX4 ATTACHES to an already-
# running gz server, and the plugin reads its env ONCE, in Configure — so a
# leftover server from the previous arm silently reuses THAT arm's settings, and
# the A/B becomes two copies of the first run. This is the other half of the
# bad "OFF" arm: it inherited the ON server's enabled=1. Idempotent; harmless if
# nothing is running.
# It also silently ignores PX4_GZ_WORLD: a leftover server keeps serving the
# PREVIOUS world, so an arm launched as `forest_sparse` flies `forest_dense` and
# the only clue is a console line most people scroll past. So: kill, WAIT, and
# HARD-FAIL if anything survives. Launching anyway produces data that looks fine
# and is wrong, which is worse than not launching.
_reap() {  # pattern, label, pgrep-flag
  local pattern="$1" label="$2" flag="${3:--f}"
  pgrep "$flag" "$pattern" >/dev/null 2>&1 || return 0
  echo "sim: killing stale $label"
  pkill "$flag" "$pattern" 2>/dev/null || true
  for _ in $(seq 1 20); do
    pgrep "$flag" "$pattern" >/dev/null 2>&1 || return 0
    sleep 0.5
  done
  pkill -9 "$flag" "$pattern" 2>/dev/null || true
  sleep 1
  pgrep "$flag" "$pattern" >/dev/null 2>&1 || return 0
  echo "error: stale $label survived SIGKILL — refusing to launch, because PX4 would" >&2
  echo "       attach to it and silently fly the PREVIOUS world. Investigate:" >&2
  pgrep "$flag" -a "$pattern" >&2 || true
  return 1
}
# -x for px4: matching its full command line would also match this script, whose
# arguments contain the PX4-Autopilot path.
_reap px4 "PX4 instance" -x || exit 1
_reap "gz sim" "Gazebo server" || exit 1
_reap "gz-sim" "Gazebo server" || exit 1

TETHER_BUILD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/tether_plugin/build"
if [[ -d "$TETHER_BUILD" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}$TETHER_BUILD"
  echo "tether: plugin path $TETHER_BUILD (DRONEFLIGHT_TETHER_ENABLED=${DRONEFLIGHT_TETHER_ENABLED:-1}${DRONEFLIGHT_TETHER_SPAN:+, span=$DRONEFLIGHT_TETHER_SPAN}${DRONEFLIGHT_TETHER_CSV:+, csv=$DRONEFLIGHT_TETHER_CSV})"
  echo "        confirm it loaded: the CSV must exist with 'enabled=' in its header (banner is now a gzwarn)."
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
