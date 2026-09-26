"""Replay every dead end in a fly-local flight record against the planner's own memory.

    python tools/replay_blocked.py --record h1g.jsonl --plan mvp400_forest.plan

A record written since 2026-09-26 dumps the planner's whole obstacle memory and the
corridor on the first step of each blockage (``mem_dump``). This re-runs
``plan_local`` on exactly that memory from the believed position toward the carrot,
then tries the variants a fix would change — no corridor, wider search windows,
merged memory — so the cause of a dead end is measured, not guessed.

Why it exists: SITL h1g (1200 stems/ha) blocked for 20 s and escaped over the canopy,
yet replayed against the TRUE trunks — with symmetric smear, with a two-offset memory,
with wider windows — the planner always found a path. Only the real memory can say
what closed the gaps.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
from pathlib import Path

from droneflight.local_planner import LocalPlannerConfig, plan_local
from droneflight.offboard import CarrotGuidance, route_to_local
from droneflight.plan_loader import load_qgc_plan

Point = tuple[float, float]


def merged(points: list[Point], radius_m: float) -> list[Point]:
    """Greedy merge: points within ``radius_m`` of a kept one collapse into it."""
    kept: list[Point] = []
    for p in points:
        if all(math.dist(p, q) > radius_m for q in kept):
            kept.append(p)
    return kept


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--record", type=Path, required=True)
    ap.add_argument("--plan", type=Path, required=True)
    args = ap.parse_args()

    route = route_to_local(load_qgc_plan(args.plan))
    cfg = LocalPlannerConfig()
    events = 0
    for line in args.record.read_text().splitlines():
        row = json.loads(line)
        if "mem_dump" not in row:
            continue
        events += 1
        est: Point = (row["est"][0], row["est"][1])
        guidance = CarrotGuidance(route=route, lookahead_m=cfg.horizon_m)
        guidance.index = max(int(row["idx"]) - 1, 0)
        carrot = guidance.advance(*est)
        goal: Point = (carrot.east_m, carrot.north_m)
        mem: list[Point] = [(p[0], p[1]) for p in row["mem_dump"]]
        corridor = [((c[0], c[1]), c[2]) for c in row.get("corridor", [])] or None
        near = sum(1 for p in mem if math.dist(p, est) <= 10.0)
        print(
            f"t={row['t']:.1f} route {row['idx']}: memory {len(mem)} points "
            f"({near} within 10 m), goal {math.dist(est, goal):.1f} m ahead"
        )
        variants: list[tuple[str, list[Point], LocalPlannerConfig, object]] = [
            ("as flown", mem, cfg, corridor),
            ("no corridor", mem, cfg, None),
            ("window 15 m", mem, dataclasses.replace(cfg, window_margin_m=15.0), corridor),
        ]
        for r in (0.3, 0.6, 1.0):
            variants.append((f"memory merged at {r} m", merged(mem, r), cfg, corridor))
        for label, pts, c, corr in variants:
            path = plan_local(pts, est, goal, c, corr)  # type: ignore[arg-type]
            verdict = "NO PATH" if path is None else f"path {len(path)} pts"
            print(f"    {label:24s} {verdict}")
    if not events:
        print("no mem_dump rows: the record predates the dump, or the flight never blocked")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
