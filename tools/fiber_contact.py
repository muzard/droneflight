"""Would the laid fiber's airborne span touch trunks on a flown forest track?

    python tools/fiber_contact.py --record ../sim-logs/mvp_local/denied3.jsonl \
        --trunks ../droneplanner/worlds/forest_mvp_trunks.csv

Reads the TRUE track from a ``fly-local --record`` JSONL and counts the trunks the
straight span from touchdown point to aircraft would cross, for a range of
touchdown distances (`tether.airborne_span_contacts`). The touchdown distance is
the unknown — it depends on payout hardware and downwash — so the sweep IS the
result: it says how close behind the aircraft the fiber has to reach the ground.

Measured 2026-09-24 on the three SITL forest flights (200/ha twice, 400/ha):
0 contacts at <=5 m behind, 3-6 per forest km at 10 m, 9-39 at 20 m, 36-108 at
40 m (the tension model's default span).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from droneflight.tether import TetherConfig, airborne_span_contacts, free_fall_touchdown_lag_m
from droneflight.trunk_manifest import load_trunk_manifest

LAGS_M = (3.0, 5.0, 10.0, 20.0, 40.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--record", type=Path, required=True, action="append")
    ap.add_argument("--trunks", type=Path, required=True)
    args = ap.parse_args()
    trunks = load_trunk_manifest(args.trunks).trunks
    cfg = TetherConfig()
    print(
        "free-fall touchdown estimate at 9 m AGL: "
        + ", ".join(
            f"{v} m/s -> {free_fall_touchdown_lag_m(9.0, v, cfg):.0f} m"
            for v in (1.5, 2.0, 3.0, 6.0)
        )
    )
    for record in args.record:
        track = [tuple(json.loads(line)["true"][:2]) for line in record.read_text().splitlines()]
        print(f"\n{record.name}: {len(track)} samples")
        for lag in LAGS_M:
            c = airborne_span_contacts(track[::5], trunks, lag)  # type: ignore[arg-type]
            print(
                f"  touchdown {lag:4.0f} m behind: {len(c.trunks):4d} trunks crossed, "
                f"{c.per_km:6.1f} per km of forest ({c.forest_length_m:.0f} m)"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
