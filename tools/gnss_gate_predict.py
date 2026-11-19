"""Predict a ``ev-bridge --gnss`` SITL arm before flying it (RUNBOOK §3g).

    python tools/gnss_gate_predict.py                       # honest, 5.13 km at 6 m/s
    python tools/gnss_gate_predict.py --pull 0.5            # fast carry-off
    python tools/gnss_gate_predict.py --pull 0.03 --seeds 20

Runs the bridge's own error model and GNSS gate (same classes, same default
seeds as the CLI) over a straight route at cruise speed. Where the gate closes
does not depend on the closed loop: the monitor sees GNSS minus VIO, and both
are truth plus an error that depends on distance flown only. So the alarm
distance printed here is the one the flight should show for the same seeds.

The estimate error is the gate's own — GNSS while open, then VIO anchored at the
alarm point (no rewind, as EKF2 cannot be wound back). EKF2's estimate should
sit near it; how far it does not is what the flight measures.
"""

from __future__ import annotations

import argparse
import math

from droneflight.gnss_gate import GnssGate, GnssGateConfig
from droneflight.gnss_spoof import MonitorConfig, SpoofAttack
from droneflight.nav import DriftConfig, ErrorInjector


def run(
    args: argparse.Namespace, drift_seed: int, gnss_seed: int
) -> tuple[float, float, float, float]:
    attack = (
        SpoofAttack(onset_m=args.onset, pull_rate=args.pull, bearing_deg=args.bearing)
        if args.pull > 0.0
        else None
    )
    gate = GnssGate(
        GnssGateConfig(
            home_lat=60.0,
            home_lon=25.0,
            home_alt_m=0.0,
            attack=attack,
            monitor=MonitorConfig(vio_rate=args.bound, gnss_sigma_m=args.sigma),
            seed=gnss_seed,
        )
    )
    inj = ErrorInjector(DriftConfig(rate=args.drift, seed=drift_seed), None)
    step = args.speed / args.hz
    n = int(args.route / step)
    worst = 0.0
    err = 0.0
    for i in range(1, n + 1):
        t = i / args.hz
        inj.advance(step, 0.0, 0.0)
        e, nn = inj.error
        truth = (step * i, 0.0, 10.0)
        fix = gate.step(t, truth, (truth[0] + e, nn), inj.distance_m, (0.0, args.speed, 0.0))
        if fix is None:
            continue
        est = gate.monitor.estimate((fix.east_m, fix.north_m), (truth[0] + e, nn))
        err = math.hypot(est[0] - truth[0], est[1])
        worst = max(worst, err)
    return gate.alarm_after_onset_m, worst, err, math.hypot(*inj.error)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--route", type=float, default=5130.0, help="Route length (m).")
    ap.add_argument("--speed", type=float, default=6.0, help="Cruise (m/s).")
    ap.add_argument("--hz", type=float, default=25.0, help="Truth rate (Hz).")
    ap.add_argument("--drift", type=float, default=0.02, help="True VIO drift rate.")
    ap.add_argument("--bound", type=float, default=0.03, help="Monitor's VIO bound.")
    ap.add_argument("--sigma", type=float, default=1.5)
    ap.add_argument("--pull", type=float, default=0.0)
    ap.add_argument("--onset", type=float, default=1000.0)
    ap.add_argument("--bearing", type=float, default=90.0)
    ap.add_argument("--seed", type=int, default=0, help="Drift seed (ev-bridge --seed).")
    ap.add_argument("--gnss-seed", type=int, default=4)
    ap.add_argument("--seeds", type=int, default=1, help="Sweep this many seed pairs.")
    args = ap.parse_args()
    print(
        f"{args.route:.0f} m at {args.speed:g} m/s, "
        f"drift {args.drift:.1%} (bound {args.bound:.1%}), "
        + (f"pull {args.pull:g} m/m from {args.onset:.0f} m" if args.pull else "honest GNSS")
    )
    print(f"{'seeds':>9} {'gate closes':>16} {'worst err':>10} {'end err':>8} {'VIO only':>9}")
    for k in range(args.seeds):
        ds, gs = args.seed + k, args.gnss_seed + k
        after, worst, end, vio = run(args, ds, gs)
        if math.isinf(after):
            closes = "never"
        elif after < 0:
            closes = "FALSE ALARM"
        else:
            closes = f"{after:.0f} m after onset"
        print(f"{ds:>4}/{gs:<4} {closes:>16} {worst:>9.1f}m {end:>7.1f}m {vio:>8.1f}m")


if __name__ == "__main__":
    main()
