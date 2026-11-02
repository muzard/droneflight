"""TEST-PLAN B4: degrade the oracle depth sensor until the local planner breaks.

    python tools/sensor_ladder.py        # from droneflight/, ~10 min on 10 cores

Every local-planner result so far used `perception.sense` — perfect ranges. This
wraps it with range noise that grows with range (stereo-like), dropped returns,
and a shorter maximum range, and flies the MVP forest (200 stems/ha, 3.0 m/s,
GPS-denied with bled-in VTRN fixes) for the covered first 1.8 km.

Measured 2026-09-24: 0 strikes at every rung; 0.6 m noise at 10 m stalled one of
two seeds (the safe failure). What it cannot model: phantom returns (false
positives) and systematically missed trunks.
"""

from __future__ import annotations

import random
from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import droneflight.reactive as reactive
from droneflight.avoidance import AvoidanceConfig
from droneflight.local_planner import LocalPlannerConfig
from droneflight.nav import DriftConfig, VtrnConfig
from droneflight.perception import DepthScan, ObstacleField
from droneflight.perception import sense as oracle
from droneflight.plan_loader import load_qgc_plan
from droneflight.trunk_manifest import load_trunk_manifest

PLANNER = Path(__file__).resolve().parents[2] / "droneplanner"
PLAN = PLANNER / "out/mvp/mission.plan"
TRUNKS = PLANNER / "worlds/forest_mvp_trunks.csv"

LADDER: tuple[tuple[str, float, float, float], ...] = (
    # (label, noise sigma at 10 m, dropout fraction, max range m)
    ("oracle", 0.0, 0.0, 30.0),
    ("noise 0.10 m @ 10 m", 0.10, 0.0, 30.0),
    ("noise 0.30 m @ 10 m", 0.30, 0.0, 30.0),
    ("noise 0.60 m @ 10 m", 0.60, 0.0, 30.0),
    ("dropout 30 %", 0.0, 0.3, 30.0),
    ("dropout 60 %", 0.0, 0.6, 30.0),
    ("range 15 m", 0.0, 0.0, 15.0),
    ("range 8 m", 0.0, 0.0, 8.0),
    ("stereo-like: 0.3 m, 30 %, 15 m", 0.3, 0.3, 15.0),
)
SEEDS = (1, 2)


def degraded(noise: float, drop: float, max_range_m: float, seed: int) -> Callable[..., DepthScan]:
    rng = random.Random(seed)

    def sense(
        field: ObstacleField,
        x: float,
        y: float,
        heading: float,
        fov_rad: float,
        n_rays: int,
        max_range: float,
    ) -> DepthScan:
        scan = oracle(field, x, y, heading, fov_rad, n_rays, max_range)
        ranges = []
        for r in scan.ranges:
            if r >= scan.max_range or r > max_range_m or rng.random() < drop:
                ranges.append(scan.max_range)  # no return
            else:
                # Stereo depth error grows with range; quoted at 10 m.
                ranges.append(max(0.05, r + rng.gauss(0.0, noise * r / 10.0)))
        return DepthScan(scan.bearings_rel, tuple(ranges), scan.max_range, scan.fov_rad)

    return sense


def run(case: tuple[str, float, float, float, int]) -> str:
    label, noise, drop, max_range_m, seed = case
    reactive.sense = degraded(noise, drop, max_range_m, seed)  # type: ignore[assignment]
    mission = load_qgc_plan(PLAN)
    field = load_trunk_manifest(TRUNKS).obstacle_field()
    config = reactive.ForestFlightConfig(
        fov_deg=90.0,
        n_rays=91,
        sensor_range_m=30.0,
        avoidance=AvoidanceConfig(cruise_speed_ms=6.0),
    )
    r = reactive.fly_with_local_planner(
        mission,
        field,
        LocalPlannerConfig(cruise_speed_ms=6.0),
        config,
        drift=DriftConfig(rate=0.02, seed=seed),
        vtrn=VtrnConfig(fix_interval_m=50.0, residual_error_m=2.0),
        fix_ramp_m=10.0,
        stop_m=1800.0,
        max_time_s=2000.0,
    )
    return (
        f"{label:32s} seed{seed}: reached={r.reached_goal} strikes={r.strikes} "
        f"min gap {r.min_clearance_m:+.2f} m, {r.flight_time_s:.0f} s, blocked {r.blocked_s:.0f} s"
    )


def main() -> int:
    cases = [(label, n, d, m, s) for label, n, d, m in LADDER for s in SEEDS]
    with ProcessPoolExecutor() as ex:
        for line in ex.map(run, cases):
            print(line, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
