"""``droneflight`` CLI: validate and fly a planner ``.plan`` on PX4.

droneflight validate --plan out/mission.plan          # offline: policies + sensor mode
droneflight fly --plan out/mission.plan --url udp://:14540   # live: execute on PX4/SITL
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from pathlib import Path

from droneflight import __version__
from droneflight.config import FlightConfig
from droneflight.executor import fly_mission, prepare_items
from droneflight.nav import DriftConfig, VtrnConfig, simulate_navigation
from droneflight.plan_loader import PlanLoadError, load_qgc_plan
from droneflight.sensors import SensorSelectionError, select_sensor_mode


def _validate(args: argparse.Namespace) -> int:
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    config = FlightConfig(night_operation=args.night)
    items = prepare_items(mission, config)
    slowest = min(items, key=lambda it: it.speed_ms)

    print(f"mission: {mission.name}  ({len(mission.waypoints)} waypoints)")
    print(f"home: {mission.home_lat:.6f}, {mission.home_lon:.6f} @ {mission.home_alt_m:.1f} m")
    print(
        f"cruise speed: {mission.cruise_speed_ms:.1f} m/s  (slowest leg {slowest.speed_ms:.1f} m/s)"
    )
    try:
        sensor = select_sensor_mode(
            config.night_operation, config.sensors_suite, config.sensors_policy
        )
    except SensorSelectionError as exc:
        print(f"sensor: NO VALID MODE — {exc}", file=sys.stderr)
        return 3
    print(f"sensor mode: {sensor.mode.value} (covert={sensor.covert})")
    for warning in sensor.warnings:
        print(f"  warning: {warning}")
    return 0


def _nav(args: argparse.Namespace) -> int:
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    drift = DriftConfig(rate=args.drift_rate)
    vio_only = simulate_navigation(mission, drift, None)
    with_vtrn = simulate_navigation(mission, drift, VtrnConfig(fix_interval_m=args.vtrn_interval))
    print(f"route distance: {vio_only.distance_m:.0f} m")
    print(
        f"VIO only ({args.drift_rate * 100:.1f}% drift): "
        f"max error {vio_only.max_error_m:.0f} m, endpoint {vio_only.endpoint_error_m:.0f} m"
    )
    print(
        f"VIO + VTRN (fix every {args.vtrn_interval:.0f} m): "
        f"max error {with_vtrn.max_error_m:.0f} m over {with_vtrn.fixes} fixes "
        f"— {'within' if with_vtrn.within_corridor else 'EXCEEDS'} corridor"
    )
    return 0


def _simfly(args: argparse.Namespace) -> int:
    from droneflight.reactive import ForestFlightConfig, fly_through_forest, scatter_forest

    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    forest = scatter_forest(mission, trees_per_100m=args.density, seed=args.seed)
    cfg = ForestFlightConfig()
    drift = DriftConfig(rate=args.drift_rate, seed=args.seed) if args.drift_rate > 0 else None
    vtrn = VtrnConfig(fix_interval_m=args.vtrn_interval) if args.vtrn_interval > 0 else None

    on = fly_through_forest(mission, forest, cfg, drift=drift, vtrn=vtrn, reactive=True)
    off = fly_through_forest(mission, forest, cfg, drift=drift, vtrn=vtrn, reactive=False)
    print(
        f"forest: {len(forest.trunks)} trunks, {args.density:.0f}/100 m, "
        f"drift {args.drift_rate * 100:.1f}%"
    )
    n_wp = len(mission.waypoints)

    def line(tag: str, r) -> str:  # type: ignore[no-untyped-def]
        status = (
            "REACHED"
            if r.reached_goal and not r.collided
            else ("COLLIDED" if r.collided else "STUCK")
        )
        return (
            f"  avoidance {tag}: {status}  "
            f"waypoints {r.waypoints_reached}/{n_wp}, "
            f"min clearance {r.min_clearance_m:.2f} m"
        )

    print(line("OFF (blind)", off))
    print(line("ON  (react)", on))
    return 0 if (on.reached_goal and not on.collided) else 1


def _fly(args: argparse.Namespace) -> int:  # pragma: no cover - requires a live autopilot
    from droneflight.mavsdk_connector import MavsdkConnector

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    config = FlightConfig(connection_url=args.url, night_operation=args.night)
    result = asyncio.run(fly_mission(MavsdkConnector(), mission, config))
    print(
        f"{'COMPLETED' if result.completed else 'ABORTED'}: "
        f"{result.items_flown}/{result.items_total} items"
        + (f" — {result.aborted_reason}" if result.aborted_reason else "")
    )
    return 0 if result.completed else 1


def app(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="droneflight", description=__doc__)
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)

    v = sub.add_parser("validate", help="Validate a .plan offline (policies + sensor mode).")
    v.add_argument("--plan", type=Path, required=True)
    v.add_argument("--night", action="store_true", help="Plan for night operation.")
    v.set_defaults(func=_validate)

    n = sub.add_parser("nav", help="Emulate GPS-denied navigation drift over a .plan.")
    n.add_argument("--plan", type=Path, required=True)
    n.add_argument(
        "--drift-rate", type=float, default=0.02, help="VIO drift as fraction of distance."
    )
    n.add_argument("--vtrn-interval", type=float, default=500.0, help="Meters between VTRN fixes.")
    n.set_defaults(func=_nav)

    s = sub.add_parser("simfly", help="Closed-loop forest-flight sim (avoidance ON vs OFF).")
    s.add_argument("--plan", type=Path, required=True)
    s.add_argument("--density", type=float, default=20.0, help="Trees per 100 m of route.")
    s.add_argument("--drift-rate", type=float, default=0.0, help="VIO drift fraction (0 = off).")
    s.add_argument("--vtrn-interval", type=float, default=0.0, help="Meters between VTRN fixes.")
    s.add_argument("--seed", type=int, default=1)
    s.set_defaults(func=_simfly)

    f = sub.add_parser("fly", help="Execute a .plan on PX4 (SITL or hardware).")
    f.add_argument("--plan", type=Path, required=True)
    f.add_argument("--url", default="udp://:14540", help="MAVLink connection URL.")
    f.add_argument("--night", action="store_true")
    f.set_defaults(func=_fly)

    args = parser.parse_args(argv)
    result: int = args.func(args)
    return result


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(app())
