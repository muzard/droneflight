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
#   Basemap arms (flown 2026-07-22 — differ only in the VTRN basemap):
#     arm = vtrn      40% forest / 10% stale  — the realistic basemap
#           novtrn    no absolute fixes       — dead-reckoning control
#           conflict  70% forest / 30% stale  — conflict-zone basemap
#
#   Error-axis arms (NOT yet flown — each adds ONE axis on top of the `vtrn`
#   basemap, so the already-flown vtrn arm at 11.8 m is their control. Do not
#   combine them: they fail through different mechanisms and varying both at
#   once repeats the forest/stale mistake):
#     arm = dropout      + EV lock loss every ~500 m for 3 s  (UNDER EKF2's 5 s
#                          no-aiding timeout — tests riding through the gap)
#           dropout-long + EV lock loss every ~500 m for 6 s  (OVER it — tests
#                          the position-loss FAILSAFE, which is the untested path)
#           yaw          + 1.0 deg/100 m heading drift (~7.7 deg by the end)
#
# Then, in another pane, once this prints `est-pub` lines:
#   droneflight fly --plan <mission.plan> --url udpin://0.0.0.0:14540
set -euo pipefail

USAGE="usage: run_ev_arm.sh <vtrn|novtrn|conflict|dropout|dropout-long|yaw> <mission.plan> [record-dir]"
ARM="${1:?$USAGE}"
PLAN="${2:?$USAGE}"
OUTDIR="${3:-$HOME/logs}"

# Every error-axis arm keeps the vtrn basemap, so the only difference from the
# flown vtrn arm is the axis being added.
BASEMAP=(--forest-frac 0.4 --stale-frac 0.1)

case "$ARM" in
  vtrn)     ARGS=("${BASEMAP[@]}") ;;
  novtrn)   ARGS=(--no-vtrn) ;;
  conflict) ARGS=(--forest-frac 0.7 --stale-frac 0.3) ;;
  dropout)
    ARGS=("${BASEMAP[@]}" --dropout-interval 500 --dropout-duration 3) ;;
  dropout-long)
    ARGS=("${BASEMAP[@]}" --dropout-interval 500 --dropout-duration 6) ;;
  yaw)
    ARGS=("${BASEMAP[@]}" --yaw-drift 1.0) ;;
  *) echo "error: unknown arm '$ARM'" >&2; echo "$USAGE" >&2; exit 2 ;;
esac

if ! command -v droneflight >/dev/null 2>&1; then
  echo "error: droneflight not on PATH — activate the venv" >&2
  exit 1
fi
[[ -f "$PLAN" ]] || { echo "error: plan not found: $PLAN" >&2; exit 1; }

mkdir -p "$OUTDIR"
RECORD="$OUTDIR/ev-$ARM.jsonl"
LOG="$OUTDIR/ev-$ARM.log"

# Refuse to silently clobber an existing record. Two runs into the same OUTDIR
# with the same arm name is exactly how the F/G session overwrote the arm-A
# acceptance record (ev-vtrn.jsonl) and left a 15 s stub in place of a full
# flight. Pass a fresh OUTDIR per arm, or set EV_ARM_FORCE=1 to overwrite.
if [[ -s "$RECORD" && "${EV_ARM_FORCE:-0}" != "1" ]]; then
  echo "error: $RECORD already exists ($(wc -c <"$RECORD") bytes)." >&2
  echo "       Use a different record-dir, or EV_ARM_FORCE=1 to overwrite." >&2
  exit 3
fi

# Every arm shares one drift seed and one fidelity seed. Changing them between
# arms would confound the comparison: a different seed is a different weather
# system, and the whole point is that only the arm's own variable differs.
#
# This banner goes THROUGH the tee, not around it. Writing it straight to the
# terminal (the obvious `echo` before the pipeline) is why the first three arm
# logs recorded no seeds and no arm name at all — the arm had to be reconstructed
# afterwards from the drift vectors in the ULog.
{
  echo "arm '$ARM': ${ARGS[*]}  (seed 0, fidelity-seed 1)"
  echo "record: $RECORD"
  echo "log:    $LOG"
  # No `exec` here: it would only replace the pipeline's subshell, and the bridge
  # needs to receive Ctrl-C itself so it can stop cleanly and print its summary.
  droneflight ev-bridge \
    --plan "$PLAN" \
    --seed 0 --fidelity-seed 1 \
    "${ARGS[@]}" \
    --record "$RECORD" 2>&1
} | tee "$LOG"
