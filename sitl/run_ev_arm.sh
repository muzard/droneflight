#!/usr/bin/env bash
# Run one arm of the GPS-denied acceptance experiment (RUNBOOK §3c).
#
# The three arms differ ONLY in the basemap given to the VTRN corrector. Typing
# those flags by hand three times is how an A/B quietly becomes three runs of the
# same thing, so they live here instead.
#
# Run this ON THE UBUNTU VM, with PX4 SITL already up and the second MAVLink
# endpoint started (see RUNBOOK §3b.1 — `mavlink start` does NOT survive a reboot):
#
#   ./run_ev_arm.sh <arm> <mission.plan> [record-dir]
#
#     arm = vtrn      40% forest / 10% stale  — the realistic basemap
#           novtrn    no absolute fixes       — dead-reckoning control
#           conflict  70% forest / 30% stale  — conflict-zone basemap
#
# Then, in another pane, once this prints `est-pub` lines:
#   droneflight fly --plan <mission.plan> --url udpin://0.0.0.0:14540
set -euo pipefail

ARM="${1:?usage: run_ev_arm.sh <vtrn|novtrn|conflict> <mission.plan> [record-dir]}"
PLAN="${2:?usage: run_ev_arm.sh <vtrn|novtrn|conflict> <mission.plan> [record-dir]}"
OUTDIR="${3:-$HOME/logs}"

case "$ARM" in
  vtrn)     ARGS=(--forest-frac 0.4 --stale-frac 0.1) ;;
  novtrn)   ARGS=(--no-vtrn) ;;
  conflict) ARGS=(--forest-frac 0.7 --stale-frac 0.3) ;;
  *) echo "error: unknown arm '$ARM' (want vtrn, novtrn or conflict)" >&2; exit 2 ;;
esac

if ! command -v droneflight >/dev/null 2>&1; then
  echo "error: droneflight not on PATH — activate the venv" >&2
  exit 1
fi
[[ -f "$PLAN" ]] || { echo "error: plan not found: $PLAN" >&2; exit 1; }

mkdir -p "$OUTDIR"
RECORD="$OUTDIR/ev-$ARM.jsonl"
LOG="$OUTDIR/ev-$ARM.log"

# Every arm shares one drift seed and one fidelity seed. Changing them between
# arms would confound the comparison: a different seed is a different weather
# system, and the whole point is that only the basemap differs.
echo "arm '$ARM': ${ARGS[*]}  (seed 0, fidelity-seed 1)"
echo "record: $RECORD"
echo "log:    $LOG"
# No `exec` here: it would only replace the pipeline's subshell, and the bridge
# needs to receive Ctrl-C itself so it can stop cleanly and print its summary.
droneflight ev-bridge \
  --plan "$PLAN" \
  --seed 0 --fidelity-seed 1 \
  "${ARGS[@]}" \
  --record "$RECORD" 2>&1 | tee "$LOG"
