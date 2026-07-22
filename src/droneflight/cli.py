"""``droneflight`` CLI: validate and fly a planner ``.plan`` on PX4.

droneflight validate --plan out/mission.plan          # offline: policies + sensor mode
droneflight fly --plan out/mission.plan --url udp://:14540   # live: execute on PX4/SITL
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import math
import signal
import sys
from collections.abc import Callable
from pathlib import Path

from droneflight import __version__
from droneflight.config import FlightConfig
from droneflight.correction import CorrectionConfig
from droneflight.ev_bridge import (
    BridgeSample,
    EvBridgeError,
    EvBridgeStats,
    GazeboTopic,
    analyse_track,
    load_track,
    plan_track_enu,
)
from droneflight.executor import fly_mission, prepare_items
from droneflight.model import FlightMission
from droneflight.nav import (
    DriftConfig,
    ErrorInjector,
    VtrnConfig,
    route_reference,
    simulate_corrected_navigation,
    simulate_navigation,
)
from droneflight.plan_loader import (
    PlanLoadError,
    flatten_plan_altitudes,
    load_plan_document,
    load_qgc_plan,
)
from droneflight.sensors import SensorSelectionError, select_sensor_mode
from droneflight.sitl_launch import format_env_exports, px4_home_env


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

    alts = [w.alt_agl_m for w in mission.waypoints]
    print(f"altitude (relative to home): {min(alts):.1f} .. {max(alts):.1f} m")
    below = sum(1 for a in alts if a < 0.0)
    if below:
        # Relative-to-home altitudes go negative wherever the terrain drops away
        # from the launch point — correct over real terrain, underground in PX4's
        # flat default.sdf. See `flatten-plan`.
        print(
            f"  warning: {below}/{len(alts)} waypoints are BELOW the home point "
            f"(min {min(alts):.1f} m). Correct over real terrain; in a FLAT sim "
            f"world they are underground — use `droneflight flatten-plan` first."
        )
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
        f"VIO + VTRN (idealized, fix every {args.vtrn_interval:.0f} m): "
        f"max error {with_vtrn.max_error_m:.0f} m over {with_vtrn.fixes} fixes "
        f"— {'within' if with_vtrn.within_corridor else 'EXCEEDS'} corridor"
    )

    # Realistic, fidelity-limited optical correction over a spatially varying
    # reference: fixes are only as good (and as available) as the basemap.
    ref = route_reference(
        mission, forest_frac=args.forest_frac, stale_frac=args.stale_frac, seed=args.fidelity_seed
    )
    opt = simulate_corrected_navigation(mission, drift, ref)
    print(
        f"VIO + VTRN (optical, {args.forest_frac * 100:.0f}% forest / "
        f"{args.stale_frac * 100:.0f}% stale basemap):"
    )
    print(
        f"    max error {opt.max_error_m:.0f} m, endpoint {opt.endpoint_error_m:.0f} m "
        f"— {'within' if opt.within_corridor else 'EXCEEDS'} corridor"
    )
    print(
        f"    fixes {opt.accepted}/{opt.attempts} accepted "
        f"({opt.fix_availability * 100:.0f}% avail); "
        f"declined: {opt.no_features} no-texture, {opt.low_confidence} low-conf, "
        f"{opt.gated_out} gated-out; {opt.corrupt_accepted} stale blunders slipped through"
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


def _sitl_env(args: argparse.Namespace) -> int:
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(format_env_exports(px4_home_env(mission)))
    return 0


def _flatten_plan(args: argparse.Namespace) -> int:
    """Rewrite a plan to a constant altitude — flat-world SITL testing only."""
    try:
        doc = load_plan_document(args.plan)
        doc, changed = flatten_plan_altitudes(doc, args.alt)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    args.out.write_text(json.dumps(doc, indent=4), encoding="utf-8")
    print(f"flattened {changed} waypoints to {args.alt:.1f} m above home -> {args.out}")
    print(
        "  ⚠️  terrain-following is GONE from this plan. It is for PX4's flat "
        "default.sdf only — never fly it over real terrain."
    )
    return 0


def _build_injector(args: argparse.Namespace, mission: FlightMission) -> ErrorInjector:
    """Assemble the GPS-denied error model the bridge will inject.

    ``--no-vtrn`` drops the reference map entirely: pure dead reckoning, no
    absolute fixes. That is the control arm of the acceptance A/B.
    """
    reference = (
        None
        if not args.vtrn
        else route_reference(
            mission,
            forest_frac=args.forest_frac,
            stale_frac=args.stale_frac,
            seed=args.fidelity_seed,
        )
    )
    return ErrorInjector(
        DriftConfig(rate=args.drift_rate, seed=args.seed),
        reference,
        CorrectionConfig(attempt_interval_m=args.attempt_interval),
    )


def _print_corridor(
    label: str, track: list[tuple[float, float]], mission: FlightMission, tolerance_m: float
) -> None:
    report = analyse_track(track, plan_track_enu(mission), tolerance_m=tolerance_m)
    print(
        f"{label}: max {report.max_deviation_m:.1f} m, mean {report.mean_deviation_m:.1f} m, "
        f"endpoint {report.endpoint_deviation_m:.1f} m "
        f"(tolerance {report.tolerance_m:.0f} m) — "
        f"{'WITHIN corridor' if report.within_corridor else 'LEFT the corridor'}"
    )


def _ev_report(args: argparse.Namespace) -> int:
    """Corridor analysis of a recorded EV-bridge run (offline, runs anywhere)."""
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    samples = load_track(args.record)
    if not samples:
        print(f"error: no samples in {args.record}", file=sys.stderr)
        return 2

    truth = [(s.truth_enu[0], s.truth_enu[1]) for s in samples]
    fixes = [s.fix for s in samples if s.fix]
    accepted = sum(1 for f in fixes if f.startswith("accepted"))
    print(f"samples: {len(samples)}  ({samples[-1].t_s - samples[0].t_s:.0f} s of sim time)")
    print(
        f"injected error: final {math.hypot(*samples[-1].error_en):.1f} m, "
        f"max {max(math.hypot(*s.error_en) for s in samples):.1f} m"
    )
    if fixes:
        print(f"optical fixes: {accepted}/{len(fixes)} accepted")
    try:
        _print_corridor("TRUE track vs plan", truth, mission, args.corridor)
    except EvBridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


def _format_bridge_stats(stats: EvBridgeStats) -> list[str]:
    """Render the bridge's run summary (pure, so it is testable without a sim)."""
    lines = [
        f"samples {stats.samples}, published {stats.published}, flown {stats.distance_m:.0f} m",
        f"injected error: max {stats.max_error_m:.1f} m, final {stats.final_error_m:.1f} m",
    ]
    if stats.attempts:
        lines.append(
            f"optical fixes: {stats.accepted}/{stats.attempts} accepted "
            f"({stats.fix_availability * 100:.0f}% avail); declined: "
            f"{stats.no_features} no-texture, {stats.low_confidence} low-conf, "
            f"{stats.gated_out} gated-out; {stats.corrupt_accepted} blunders slipped through"
        )
    return lines


def _live_monitor(interval_s: float = 5.0) -> Callable[[BridgeSample], None]:
    """Periodic one-liner: is EKF2 actually fusing what we publish?

    ``est-pub`` is PX4's own estimate minus the pose we just sent it. Near zero
    means EKF2 has locked onto the external vision; large or growing means the
    stream is being rejected (usually the message timestamp — see RUNBOOK §3b).
    It is a **readback**, never an input to the error model.
    """
    last = [-math.inf]

    def monitor(sample: BridgeSample) -> None:
        if sample.t_s - last[0] < interval_s:
            return
        last[0] = sample.t_s
        line = (
            f"t={sample.t_s:7.1f}  truth E,N=({sample.truth_enu[0]:8.1f},"
            f"{sample.truth_enu[1]:8.1f})  injected err={math.hypot(*sample.error_en):6.1f} m"
        )
        if sample.estimate_ned is not None:
            n, e, _d = sample.estimate_ned
            delta = math.hypot(n - sample.published.north_m, e - sample.published.east_m)
            line += f"  est-pub={delta:5.2f} m"
        else:
            line += "  est-pub=  n/a"
        print(line, flush=True)

    return monitor


async def _run_ev_bridge(  # pragma: no cover - requires sim + autopilot
    args: argparse.Namespace, mission: FlightMission, injector: ErrorInjector, topic: GazeboTopic
) -> int:
    from droneflight.ev_bridge import (
        EvBridge,
        EvBridgeConfig,
        GazeboTruthSource,
        RecordingPublisher,
        TrackRecorder,
        VisionPublisher,
    )

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError):
            loop.add_signal_handler(sig, stop.set)

    estimate: list[tuple[float, float, float] | None] = [None]
    watcher: asyncio.Task[None] | None = None
    publisher: VisionPublisher

    if args.dry_run:
        publisher = RecordingPublisher()
        print("dry run: reading truth and computing poses, NOT publishing to PX4")
    else:
        from mavsdk import System

        from droneflight.ev_bridge import MavsdkMocapPublisher

        system = System()
        await system.connect(system_address=args.url)
        async for state in system.core.connection_state():
            if state.is_connected:
                break
        print(f"connected to PX4 at {args.url}")
        publisher = MavsdkMocapPublisher(system)

        if args.set_origin:
            from droneflight.ev_bridge import set_global_origin

            try:
                await set_global_origin(
                    system, mission.home_lat, mission.home_lon, mission.home_alt_m
                )
            except EvBridgeError as exc:
                print(f"error: {exc}", file=sys.stderr)
                return 4
            print(
                f"EKF2 global origin set to {mission.home_lat:.6f}, {mission.home_lon:.6f} "
                f"@ {mission.home_alt_m:.1f} m — the plan's home"
            )

        async def watch_estimate() -> None:
            async for pv in system.telemetry.position_velocity_ned():
                estimate[0] = (pv.position.north_m, pv.position.east_m, pv.position.down_m)

        watcher = asyncio.create_task(watch_estimate())

    config = EvBridgeConfig(
        rate_hz=args.rate,
        corridor_tolerance_m=args.corridor,
        timestamp_mode=args.timestamp,
        max_duration_s=args.duration,
    )
    try:
        with contextlib.ExitStack() as stack:
            recorder = stack.enter_context(TrackRecorder(args.record)) if args.record else None
            bridge = EvBridge(
                injector,
                config,
                recorder=recorder,
                estimate_probe=None if args.dry_run else (lambda: estimate[0]),
                on_sample=_live_monitor(),
            )
            stats = await bridge.run(
                GazeboTruthSource(topic, gz_binary=args.gz), publisher, stop=stop
            )
    except EvBridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 4
    finally:
        if watcher is not None:
            watcher.cancel()

    for line in _format_bridge_stats(stats):
        print(line)
    if stats.track_enu:
        _print_corridor("TRUE track vs plan", stats.track_enu, mission, args.corridor)
    if args.record:
        print(f"track written to {args.record}")
    return 0


def _ev_bridge(args: argparse.Namespace) -> int:  # pragma: no cover - requires sim + autopilot
    """Stream Gazebo truth + injected error into PX4 as external vision."""
    from droneflight.ev_bridge import resolve_truth_topic

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    try:
        topic = resolve_truth_topic(args.world, args.model, gz_binary=args.gz)
    except EvBridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 4

    print(f"ground truth: {topic.topic} ({topic.kind}) — Gazebo, not PX4's estimate")
    print(
        f"error model: {args.drift_rate * 100:.1f}% VIO drift, "
        + (
            f"optical fixes every {args.attempt_interval:.0f} m over a "
            f"{args.forest_frac * 100:.0f}% forest / {args.stale_frac * 100:.0f}% stale basemap"
            if args.vtrn
            else "NO absolute fixes (dead reckoning only — the control arm)"
        )
    )
    return asyncio.run(_run_ev_bridge(args, mission, _build_injector(args, mission), topic))


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
    n.add_argument(
        "--forest-frac",
        type=float,
        default=0.4,
        help="Fraction of route over low-texture forest canopy (few optical fixes).",
    )
    n.add_argument(
        "--stale-frac",
        type=float,
        default=0.1,
        help="Fraction of route over a stale/changed basemap (blunder-prone fixes).",
    )
    n.add_argument("--fidelity-seed", type=int, default=1, help="Seed for the fidelity patches.")
    n.set_defaults(func=_nav)

    s = sub.add_parser("simfly", help="Closed-loop forest-flight sim (avoidance ON vs OFF).")
    s.add_argument("--plan", type=Path, required=True)
    s.add_argument("--density", type=float, default=20.0, help="Trees per 100 m of route.")
    s.add_argument("--drift-rate", type=float, default=0.0, help="VIO drift fraction (0 = off).")
    s.add_argument("--vtrn-interval", type=float, default=0.0, help="Meters between VTRN fixes.")
    s.add_argument("--seed", type=int, default=1)
    s.set_defaults(func=_simfly)

    se = sub.add_parser(
        "sitl-env", help="Print PX4 SITL home env for a .plan (eval before make px4_sitl)."
    )
    se.add_argument("--plan", type=Path, required=True)
    se.set_defaults(func=_sitl_env)

    fp = sub.add_parser(
        "flatten-plan",
        help="Rewrite a plan to a constant altitude for PX4's FLAT world (destroys "
        "terrain-following).",
    )
    fp.add_argument("--plan", type=Path, required=True)
    fp.add_argument("--out", type=Path, required=True)
    fp.add_argument("--alt", type=float, default=30.0, help="Altitude above home (m).")
    fp.set_defaults(func=_flatten_plan)

    eb = sub.add_parser(
        "ev-bridge",
        help="Stream Gazebo ground truth + injected GPS-denied error into PX4 as external vision.",
    )
    eb.add_argument("--plan", type=Path, required=True)
    eb.add_argument(
        "--url",
        default="udp://:14541",
        help="MAVLink URL (use a SECOND endpoint: `droneflight fly` holds 14540).",
    )
    eb.add_argument("--world", default="default", help="Gazebo world name.")
    eb.add_argument("--model", default="x500_0", help="Gazebo model name to take truth from.")
    eb.add_argument("--gz", default="gz", help="Gazebo CLI binary.")
    eb.add_argument("--rate", type=float, default=30.0, help="Vision publish rate (Hz, sim time).")
    eb.add_argument("--drift-rate", type=float, default=0.02, help="VIO drift, fraction of dist.")
    eb.add_argument("--seed", type=int, default=0, help="Drift seed.")
    eb.add_argument(
        "--vtrn",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Apply optical VTRN fixes. --no-vtrn is the dead-reckoning control arm.",
    )
    eb.add_argument("--attempt-interval", type=float, default=50.0, help="Meters between fixes.")
    eb.add_argument("--forest-frac", type=float, default=0.4)
    eb.add_argument("--stale-frac", type=float, default=0.1)
    eb.add_argument("--fidelity-seed", type=int, default=1)
    eb.add_argument("--corridor", type=float, default=15.0, help="Corridor tolerance (m).")
    eb.add_argument("--duration", type=float, default=0.0, help="Stop after N s of sim time.")
    eb.add_argument("--record", type=Path, help="Write the run as JSONL for `ev-report`.")
    eb.add_argument(
        "--timestamp",
        choices=("none", "epoch", "sim"),
        default="none",
        help="Vision message timestamp; switch if EKF2 rejects the stream (RUNBOOK §3b).",
    )
    eb.add_argument("--dry-run", action="store_true", help="Read truth but do not publish to PX4.")
    eb.add_argument(
        "--set-origin",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Anchor EKF2's local frame to the plan's home so a lat/lon mission can fly.",
    )
    eb.set_defaults(func=_ev_bridge)

    er = sub.add_parser("ev-report", help="Corridor analysis of a recorded ev-bridge run.")
    er.add_argument("--plan", type=Path, required=True)
    er.add_argument("--record", type=Path, required=True, help="JSONL from `ev-bridge --record`.")
    er.add_argument("--corridor", type=float, default=15.0)
    er.set_defaults(func=_ev_report)

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
