"""Measure the onboard local planner's speed envelope: stems/ha x speed -> clean?

    python tools/envelope_sweep.py --density 500 600 800 --speed 1.0 1.5 2.0 \
        --seeds 0 1 2 3 --length 1200

Flies `reactive.fly_with_local_planner` (PX4 accel/jerk limits, oracle depth
sensor, 90 deg FOV, 91 rays, 30 m range, truth navigation) along the first
``--length`` metres of the below-canopy MVP route through a uniform Poisson
forest, trunk radii 0.08-0.25 m, in a +-40 m band. One JSON line per run.

This is where ``droneplanner/fields/stem_density.ONBOARD_PLANNING_ENVELOPE`` comes
from. The rule used to pick an anchor: the fastest speed at which EVERY seed
reaches the goal with 0 strikes and a minimum trunk-surface gap that is not
marginal (> ~0.15 m) — 400/ha at 3.0 m/s was clean by 5 cm and was rejected.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import random
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from droneflight.avoidance import AvoidanceConfig
from droneflight.local_planner import LocalPlannerConfig
from droneflight.model import FlightMission
from droneflight.perception import ObstacleField, Trunk
from droneflight.plan_loader import load_qgc_plan
from droneflight.reactive import (
    ForestFlightConfig,
    _slice_polyline,
    _to_local,
    fly_with_local_planner,
)

PLAN = Path(__file__).resolve().parents[2] / "droneplanner/out/mvp_belowcanopy/mission.plan"
BAND_M = 40.0


def _seg_dist(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    abx, aby = b[0] - a[0], b[1] - a[1]
    d = abx * abx + aby * aby
    t = 0.0 if d == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / d))
    return math.hypot(p[0] - a[0] - abx * t, p[1] - a[1] - aby * t)


def poisson_forest(
    mission: FlightMission, per_ha: float, stop_m: float, seed: int
) -> ObstacleField:
    """Uniform random stems in a band round the first ``stop_m`` of the route."""
    pts = [_to_local(w.lat, w.lon, mission.home_lat, mission.home_lon) for w in mission.waypoints]
    route = _slice_polyline(pts, 0.0, stop_m)
    xs, ys = [p[0] for p in route], [p[1] for p in route]
    x0, x1, y0, y1 = min(xs) - BAND_M, max(xs) + BAND_M, min(ys) - BAND_M, max(ys) + BAND_M
    rng = random.Random(seed)
    trunks = []
    for _ in range(int(per_ha * (x1 - x0) * (y1 - y0) / 1e4)):
        p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        r = rng.uniform(0.08, 0.25)
        if min(_seg_dist(p, a, b) for a, b in itertools.pairwise(route)) <= BAND_M:
            trunks.append(Trunk(p[0], p[1], r))
    # Do not spawn the aircraft inside a trunk, or put one on the goal.
    s, g = route[0], route[-1]
    keep = tuple(t for t in trunks if math.dist((t.x, t.y), s) > 3 and math.dist((t.x, t.y), g) > 3)
    return ObstacleField(trunks=keep)


def run(case: tuple[float, float, int, float]) -> dict[str, object]:
    per_ha, speed, seed, length = case
    mission = load_qgc_plan(PLAN)
    field = poisson_forest(mission, per_ha, length, seed)
    config = ForestFlightConfig(
        fov_deg=90.0,
        n_rays=91,
        sensor_range_m=30.0,
        avoidance=AvoidanceConfig(cruise_speed_ms=speed),
    )
    r = fly_with_local_planner(
        mission,
        field,
        LocalPlannerConfig(cruise_speed_ms=speed),
        config,
        stop_m=length,
        max_time_s=max(600.0, 3.0 * length / speed),
    )
    return {
        "per_ha": per_ha,
        "speed": speed,
        "seed": seed,
        "clean": r.clean,
        "reached": r.reached_goal,
        "strikes": r.strikes,
        "min_gap_m": round(r.min_clearance_m, 3),
        "time_s": round(r.flight_time_s),
        "blocked_s": round(r.blocked_s, 1),
        "plan_failures": r.plan_failures,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--density", type=float, nargs="+", required=True)
    ap.add_argument("--speed", type=float, nargs="+", required=True)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3])
    ap.add_argument("--length", type=float, default=1200.0)
    ap.add_argument("--workers", type=int, default=None)
    args = ap.parse_args()
    cases = [
        (d, v, s, args.length)
        for d, v, s in itertools.product(args.density, args.speed, args.seeds)
    ]
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for result in ex.map(run, cases):
            print(json.dumps(result), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
