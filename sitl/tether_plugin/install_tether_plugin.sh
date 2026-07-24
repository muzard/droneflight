#!/usr/bin/env bash
# Attach the tether plugin to PX4's x500 model SDF. Idempotent.
#
# Run this ON THE UBUNTU VM, after building the plugin (see README.md).
#
# Usage:
#   ./install_tether_plugin.sh [PX4-Autopilot dir] [model sdf path]
#   ./install_tether_plugin.sh --remove [PX4-Autopilot dir] [model sdf path]
#
# Why the MODEL sdf and not the world sdf: `make px4_sitl gz_x500` spawns the
# vehicle into the selected world at runtime, so no world file contains an x500
# to hang the plugin off. A <plugin> block added to the world loads nothing and
# says nothing about it — a silent no-op that looks exactly like a working run.
set -euo pipefail

REMOVE=0
if [[ "${1:-}" == "--remove" ]]; then
  REMOVE=1
  shift
fi

PX4_DIR="${1:-${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}}"
SDF="${2:-}"

if [[ -z "$SDF" ]]; then
  # PX4 has moved the gz models around between releases; try the known homes.
  for candidate in \
    "$PX4_DIR/Tools/simulation/gz/models/x500/model.sdf" \
    "$PX4_DIR/Tools/simulation/gz/models/x500_base/model.sdf"; do
    if [[ -f "$candidate" ]]; then
      SDF="$candidate"
      break
    fi
  done
fi

if [[ -z "$SDF" || ! -f "$SDF" ]]; then
  echo "error: x500 model.sdf not found under $PX4_DIR" >&2
  echo "       pass it explicitly: $0 [px4-dir] /path/to/x500/model.sdf" >&2
  echo "       (find it with: find \"$PX4_DIR/Tools/simulation/gz\" -name model.sdf -path '*x500*')" >&2
  exit 1
fi

MARKER='name="droneflight::TetherPlugin"'
SNIPPET="$(dirname "$(readlink -f "$0")")/x500_tether_snippet.sdf"
[[ -f "$SNIPPET" ]] || { echo "error: $SNIPPET missing" >&2; exit 1; }

if [[ "$REMOVE" == 1 ]]; then
  if ! grep -qF "$MARKER" "$SDF"; then
    echo "not installed in $SDF — nothing to do"
    exit 0
  fi
  # Drop everything from the plugin's leading comment through its </plugin>.
  awk '
    /Trailing-fiber tether disturbance \(droneflight v0\)/ { skip = 1 }
    /name="droneflight::TetherPlugin"/ { skip = 1 }
    skip && /<\/plugin>/ { skip = 0; next }
    !skip { print }
  ' "$SDF" > "$SDF.tmp"
  mv "$SDF.tmp" "$SDF"
  echo "removed the tether plugin from $SDF"
  exit 0
fi

if grep -qF "$MARKER" "$SDF"; then
  echo "already installed in $SDF (edit the <plugin> block there to retune)"
  echo "  A/B without editing:  DRONEFLIGHT_TETHER_ENABLED=0|1"
  exit 0
fi

cp -n "$SDF" "$SDF.orig" 2>/dev/null || true  # keep the pristine PX4 file, once

# Insert before the LAST </model>: the x500 SDF merges an included base model,
# and anchoring on the first close tag would land the block in the wrong scope
# on any layout that nests. The snippet is read by awk rather than passed with
# -v, which cannot carry a multi-line value.
awk -v snippet="$SNIPPET" '
  { lines[NR] = $0; if ($0 ~ /<\/model>/) last = NR }
  END {
    if (last == 0) { print "no </model> found" > "/dev/stderr"; exit 1 }
    for (i = 1; i <= NR; i++) {
      if (i == last) {
        while ((getline line < snippet) > 0) print "    " line
        close(snippet)
      }
      print lines[i]
    }
  }
' "$SDF" > "$SDF.tmp"

mv "$SDF.tmp" "$SDF"
echo "installed the tether plugin into $SDF (pristine copy at $SDF.orig)"
echo
echo "next: make sure the built library is on the plugin path, e.g."
echo "  export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:$(dirname "$(readlink -f "$0")")/build"
echo "then launch SITL as usual; the console must show a '[tether] ENABLED ...' line."
echo "If that line is absent the plugin did NOT load — do not score the run."
