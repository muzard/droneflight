"""How far can a GNSS spoofer drag the estimate past a VIO-consistency monitor?

    python tools/gnss_spoof_sweep.py              # 5 km route, onset at 1 km, 20 seeds
    python tools/gnss_spoof_sweep.py --vio-rate 0.03 --seeds 50

See ``droneflight.gnss_spoof`` for the model and its limits.
"""

from __future__ import annotations

import argparse
import math

from droneflight.gnss_spoof import MonitorConfig, simulate_spoof, sweep_pull_rates

RATES = (0.0, 0.005, 0.01, 0.02, 0.03, 0.04, 0.06, 0.1, 0.2, 0.5, 1.0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--route", type=float, default=5000.0)
    ap.add_argument("--onset", type=float, default=1000.0)
    ap.add_argument("--seeds", type=int, default=20)
    args = ap.parse_args()
    mon = MonitorConfig()
    honest = [simulate_spoof(args.route, None, mon, seed=s) for s in range(args.seeds)]
    print(
        f"monitor: windows {mon.windows_m} m, VIO bound {mon.vio_rate:.0%}/m, "
        f"noise allowance {mon.k_noise:g} sigma of {mon.gnss_sigma_m} m"
    )
    print(
        f"honest GNSS: {sum(r.false_alarm for r in honest)}/{len(honest)} false alarms, "
        f"end error {sorted(r.error_at_end_m for r in honest)[len(honest) // 2]:.1f} m "
        f"(VIO alone {sorted(r.vio_only_error_at_end_m for r in honest)[len(honest) // 2]:.1f} m)"
    )
    print(
        f"{'pull m/m':>9} {'caught':>7} {'after':>8} "
        f"{'worst err':>10} {'end err':>8} {'VIO only':>9}"
    )
    for r in sweep_pull_rates(
        tuple(x for x in RATES if x > 0), route_m=args.route, onset_m=args.onset, seeds=args.seeds
    ):
        after = "-" if math.isinf(r.median_detect_after_m) else f"{r.median_detect_after_m:.0f} m"
        print(
            f"{r.pull_rate:>9g} {r.detected_fraction:>7.0%} {after:>8} "
            f"{r.worst_max_error_m:>9.1f}m {r.median_error_at_end_m:>7.1f}m "
            f"{r.median_vio_only_end_m:>8.1f}m"
        )


if __name__ == "__main__":
    main()
