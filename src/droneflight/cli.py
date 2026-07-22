"""``droneflight`` CLI: validate and fly a planner ``.plan`` on PX4.

droneflight validate --plan out/mission.plan          # offline: policies + sensor mode
droneflight fly --plan out/mission.plan --url udpin://0.0.0.0:14540  # live: execute on PX4/SITL
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
from dataclasses import dataclass
from pathlib import Path

from droneflight import __version__
from droneflight.config import FlightConfig
from droneflight.correction import CorrectionConfig
from droneflight.ev_bridge import (
    BridgeSample,
    CorridorReport,
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
    DropoutConfig,
    DropoutModel,
    ErrorInjector,
    VtrnConfig,
    YawDriftConfig,
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
from droneflight.sitl_launch import (
    ParamFileError,
    format_env_exports,
    format_pxh_param_sets,
    parse_qgc_params,
    px4_home_env,
)


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


def _px4_params(args: argparse.Namespace) -> int:
    """Emit `param set` lines for pxh from QGC .params files (pxh can't read them)."""
    pairs: list[tuple[str, str]] = []
    for path in args.file:
        try:
            pairs.extend(parse_qgc_params(Path(path).read_text(encoding="utf-8")))
        except (OSError, ParamFileError) as exc:
            print(f"error: {path}: {exc}", file=sys.stderr)
            return 2
    print(format_pxh_param_sets(pairs))
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
            # The bridge samples at Gazebo's home-relative ENU truth position.
            home_relative=True,
        )
    )
    return ErrorInjector(
        DriftConfig(rate=args.drift_rate, seed=args.seed),
        reference,
        CorrectionConfig(attempt_interval_m=args.attempt_interval),
        correction_ramp_m=getattr(args, "fix_ramp", 0.0),
        yaw=YawDriftConfig(
            bias_deg=getattr(args, "yaw_bias", 0.0),
            wander_deg_per_100m=getattr(args, "yaw_drift", 0.0),
            seed=getattr(args, "yaw_seed", 3),
        ),
    )


def _print_corridor(
    label: str,
    track: list[tuple[float, float]] | None,
    mission: FlightMission | None,
    tolerance_m: float,
    report: CorridorReport | None = None,
) -> None:
    if report is None:
        assert track is not None and mission is not None
        report = analyse_track(track, plan_track_enu(mission), tolerance_m=tolerance_m)
    # Endpoint deviation only means something if the route was flown to the end;
    # after an abort it just measures how far from the goal the run stopped.
    endpoint = (
        f"endpoint {report.endpoint_deviation_m:.1f} m"
        if report.endpoint_deviation_m <= 10.0 * tolerance_m
        else f"endpoint {report.endpoint_deviation_m:.0f} m (route not completed — ignore)"
    )
    print(
        f"{label}: max {report.max_deviation_m:.1f} m, mean {report.mean_deviation_m:.1f} m, "
        f"{endpoint} (tolerance {report.tolerance_m:.0f} m) — "
        f"{'WITHIN corridor' if report.within_corridor else 'LEFT the corridor'}"
    )


def _flying_mission(sample: BridgeSample) -> bool:
    return bool(sample.mode and "mission" in sample.mode.lower())


@dataclass(frozen=True)
class _RecordSummary:
    """One flown arm, reduced to the numbers the A/B/C compares."""

    name: str
    samples: int
    duration_s: float
    max_error_m: float
    final_error_m: float
    accepted: int
    attempts: int
    blunders: int
    report: CorridorReport
    scored: str  # which samples the corridor number covers
    excluded: str
    ev_down: int = 0
    max_yaw_error_deg: float = 0.0


def _summarise_record(path: Path, mission: FlightMission, corridor_m: float) -> _RecordSummary:
    samples = load_track(path)
    if not samples:
        raise EvBridgeError(f"no samples in {path}")
    fixes = [s.fix for s in samples if s.fix]
    # Score the MISSION phase only. After an abort, RTL flies a straight line
    # home across a tortuous planned route — scored naively that is a ~120 m
    # "navigation error" which buries the ~10 m the experiment actually measured.
    in_mission = [s for s in samples if _flying_mission(s)]
    other = [s for s in samples if not _flying_mission(s)]
    scored_samples = in_mission or samples
    scored = f"MISSION, {len(in_mission)}" if in_mission else "WHOLE RUN (no mode field)"
    excluded = ", ".join(sorted({s.mode or "?" for s in other})) if in_mission and other else ""
    return _RecordSummary(
        name=path.stem,
        samples=len(samples),
        duration_s=samples[-1].t_s - samples[0].t_s,
        max_error_m=max(math.hypot(*s.error_en) for s in samples),
        final_error_m=math.hypot(*samples[-1].error_en),
        accepted=sum(1 for f in fixes if f.startswith("accepted")),
        attempts=len(fixes),
        blunders=sum(1 for f in fixes if f == "accepted_corrupt"),
        report=analyse_track(
            [(s.truth_enu[0], s.truth_enu[1]) for s in scored_samples],
            plan_track_enu(mission),
            tolerance_m=corridor_m,
        ),
        scored=scored,
        excluded=excluded,
        ev_down=sum(1 for s in samples if s.ev_down),
        max_yaw_error_deg=math.degrees(max(abs(s.yaw_error_rad) for s in samples)),
    )


def _ev_report(args: argparse.Namespace) -> int:
    """Corridor analysis of recorded EV-bridge runs (offline, runs anywhere).

    Pass ``--record`` more than once to compare the acceptance arms side by side.
    """
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    try:
        rows = [_summarise_record(p, mission, args.corridor) for p in args.record]
    except (EvBridgeError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if len(rows) == 1:
        r = rows[0]
        print(f"samples: {r.samples}  ({r.duration_s:.0f} s of sim time)")
        print(f"injected error: final {r.final_error_m:.1f} m, max {r.max_error_m:.1f} m")
        if r.attempts:
            print(
                f"optical fixes: {r.accepted}/{r.attempts} accepted "
                f"({r.accepted / r.attempts * 100:.0f}%), {r.blunders} stale blunders accepted"
            )
        if r.ev_down:
            print(
                f"ev dropout: {r.ev_down}/{r.samples} samples with no vision published "
                f"({(1 - r.ev_down / r.samples) * 100:.1f}% uptime)"
            )
        if r.max_yaw_error_deg:
            print(f"yaw error: max {r.max_yaw_error_deg:.2f}°")
        _print_corridor(f"TRUE track vs plan [{r.scored}]", None, None, args.corridor, r.report)
        if r.excluded:
            print(f"  excluded samples in: {r.excluded}")
        elif "WHOLE RUN" in r.scored:
            print(
                "  warning: no flight-mode field in this record, so a post-abort "
                "RTL/loiter\n  is being scored as navigation error. Re-record to "
                "segment the phases."
            )
        return 0

    print(f"corridor tolerance {args.corridor:.0f} m — scoring the MISSION phase of each arm\n")
    head = (
        f"{'arm':<18}{'max dev':>9}{'mean':>8}{'max err':>9}{'fixes':>10}"
        f"{'blund':>7}{'EV up':>8}{'yaw':>8}  verdict"
    )
    print(head)
    print("-" * len(head))
    for r in rows:
        fixes = f"{r.accepted}/{r.attempts}" if r.attempts else "—"
        # Uptime and yaw belong in the comparison, not just the single-record
        # view: without them an arm whose axis silently never engaged is
        # indistinguishable from one that engaged and changed nothing.
        uptime = f"{(1 - r.ev_down / r.samples) * 100:.1f}%" if r.samples else "—"
        yaw = f"{r.max_yaw_error_deg:.1f}°" if r.max_yaw_error_deg else "—"
        print(
            f"{r.name[:17]:<18}{r.report.max_deviation_m:8.1f}m{r.report.mean_deviation_m:7.1f}m"
            f"{r.max_error_m:8.1f}m{fixes:>10}{r.blunders:7d}{uptime:>8}{yaw:>8}  "
            f"{'WITHIN' if r.report.within_corridor else 'LEFT'}"
        )
    held = [r for r in rows if r.report.within_corridor]
    left = [r for r in rows if not r.report.within_corridor]
    print(
        f"\n{len(held)} arm(s) held the corridor, {len(left)} left it. The experiment only "
        "means\nsomething if the arms DISAGREE — all-pass or all-fail says the route, not "
        "the\nnavigation, decided the outcome."
    )
    # Corridor deviation is a *position* metric. Measured 2026-07-22: a 6.1 deg
    # injected heading error and a 12.4 deg dropout-induced one both left it
    # unmoved, because a multirotor on position setpoints does not steer with its
    # nose. An arm that only perturbs heading will read as a null result here.
    if any(r.max_yaw_error_deg for r in rows) or any(r.ev_down for r in rows):
        print(
            "\nnote: an arm perturbing HEADING or EV availability may show no corridor\n"
            "change and still be significant — cross-track is near-blind to yaw.\n"
            "Score those arms on EKF2 heading error from the ULog, not on this table."
        )
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
    if stats.dropout_events or stats.dropout_suppressed:
        lines.append(
            f"ev dropout: {stats.dropout_events} lock losses, "
            f"{stats.dropout_suppressed} publishes withheld "
            f"({stats.ev_uptime * 100:.1f}% uptime)"
        )
    if stats.max_yaw_error_rad:
        lines.append(f"yaw error: max {math.degrees(stats.max_yaw_error_rad):.2f}°")
    return lines


# Any est-pub delta beyond this is a frame artefact, not a navigation number: the
# whole AO is ~5 km across, so even total EV rejection cannot produce it.
_EST_PROBE_SANE_M = 100_000.0


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
        if sample.ev_down:
            line += "  [EV DOWN]"
        if sample.estimate_ned is not None:
            n, e, _d = sample.estimate_ned
            delta = math.hypot(n - sample.published.north_m, e - sample.published.east_m)
            # Before EKF2 has an origin, position_velocity_ned reads out in a
            # different frame and the delta comes back in the millions — measured
            # at exactly 5421242.25 m on the first sample of all four SITL runs.
            # Printing that verbatim sends operators debugging a non-problem, and
            # the RUNBOOK tells them to watch this number, so name it instead.
            line += (
                "  est-pub=  (no EKF2 origin yet)"
                if delta > _EST_PROBE_SANE_M
                else f"  est-pub={delta:5.2f} m"
            )
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

    def connect_help() -> str:
        return (
            f"error: no MAVLink system on {args.url} within {args.connect_timeout:.0f}s.\n"
            "  PX4 must have an endpoint streaming to that port, and `mavlink start` does\n"
            "  NOT survive a reboot — re-run it in pxh after every restart:\n"
            "      mavlink start -x -u 14581 -r 4000000 -f -m onboard -o 14541\n"
            "  Confirm with `mavlink status` that two onboard instances are up (RUNBOOK §3b.1)."
        )

    estimate: list[tuple[float, float, float] | None] = [None]
    mode: list[str | None] = [None]
    watcher: asyncio.Task[None] | None = None
    mode_watcher: asyncio.Task[None] | None = None
    publisher: VisionPublisher

    if args.dry_run:
        publisher = RecordingPublisher()
        print("dry run: reading truth and computing poses, NOT publishing to PX4")
    else:
        from mavsdk import System

        from droneflight.ev_bridge import MavsdkMocapPublisher

        # MAVSDK-Python spawns its own `mavsdk_server` on a FIXED gRPC port
        # (default 50051) with no conflict check. `droneflight fly` runs
        # concurrently and takes that default, so the bridge must move off it —
        # otherwise the second server fails to bind and the two processes end up
        # sharing one MAVLink connection, silently ignoring one of the --url values.
        system = System(port=args.server_port)
        try:
            # Bounded: MAVSDK's own discovery wait never returns, so without this
            # a missing endpoint looks like the bridge silently doing nothing.
            async with asyncio.timeout(args.connect_timeout):
                await system.connect(system_address=args.url)
                async for state in system.core.connection_state():
                    if state.is_connected:
                        break
        except TimeoutError:
            print(connect_help(), file=sys.stderr)
            return 4
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

        async def watch_mode() -> None:
            async for flight_mode in system.telemetry.flight_mode():
                mode[0] = str(flight_mode)

        watcher = asyncio.create_task(watch_estimate())
        mode_watcher = asyncio.create_task(watch_mode())

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
                mode_probe=None if args.dry_run else (lambda: mode[0]),
                on_sample=_live_monitor(),
                dropout=DropoutModel(
                    DropoutConfig(
                        mean_interval_m=args.dropout_interval,
                        duration_s=args.dropout_duration,
                        seed=args.dropout_seed,
                    )
                )
                if args.dropout_interval > 0.0
                else None,
            )
            stats = await bridge.run(
                GazeboTruthSource(topic, gz_binary=args.gz), publisher, stop=stop
            )
    except EvBridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 4
    finally:
        for task in (watcher, mode_watcher):
            if task is not None:
                task.cancel()

    for line in _format_bridge_stats(stats):
        print(line)
    if stats.track_enu:
        _print_corridor("TRUE track vs plan", stats.track_enu, mission, args.corridor)
    if args.record:
        print(f"track written to {args.record}")
    return 0


def _ev_bridge(args: argparse.Namespace) -> int:  # pragma: no cover - requires sim + autopilot
    """Stream Gazebo truth + injected error into PX4 as external vision."""
    from droneflight.ev_bridge import wait_for_truth_topic

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    try:
        topic = wait_for_truth_topic(
            args.world, args.model, timeout_s=args.wait_topic, gz_binary=args.gz
        )
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
    # The run script's own banner is echoed to the terminal, not into the tee'd
    # log, so this line is the only in-log record of how an arm was configured.
    # Seeds included deliberately: a lost seed makes a run unreproducible.
    print(
        f"seeds: drift={args.seed}, fidelity={args.fidelity_seed}, "
        f"dropout={args.dropout_seed}, yaw={args.yaw_seed}"
    )
    print(
        "ev dropout: "
        + (
            f"every ~{args.dropout_interval:.0f} m for {args.dropout_duration:.1f} s"
            if args.dropout_interval > 0.0
            else "OFF (stream never interrupted)"
        )
        + " | yaw error: "
        + (
            f"{args.yaw_bias:+.1f}° bias, {args.yaw_drift:.2f}°/100 m drift"
            if args.yaw_bias or args.yaw_drift
            else "OFF (heading is ground truth)"
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
    config = FlightConfig(
        connection_url=args.url,
        night_operation=args.night,
        progress_timeout_s=args.progress_timeout,
    )
    result = asyncio.run(fly_mission(MavsdkConnector(), mission, config))
    print(
        f"{'COMPLETED' if result.completed else 'ABORTED'}: "
        f"{result.items_flown}/{result.items_total} items"
        + (f" — {result.aborted_reason}" if result.aborted_reason else "")
    )
    return 0 if result.completed else 1


def build_parser() -> argparse.ArgumentParser:
    """The full CLI grammar.

    Split out from :func:`app` so defaults can be asserted without running a
    command — several of them (the acceptance-arm flags in particular) are load-
    bearing for reproducing past flights.
    """
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

    pp = sub.add_parser(
        "px4-params",
        help="Emit `param set` lines for the pxh console from QGC .params files.",
    )
    pp.add_argument("--file", action="append", required=True, help="A .params file (repeatable).")
    pp.set_defaults(func=_px4_params)

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
        default="udpin://0.0.0.0:14541",
        help="MAVLink URL. Use a SECOND endpoint — `droneflight fly` holds 14540.",
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
    eb.add_argument(
        "--fix-ramp",
        type=float,
        default=10.0,
        help="Metres over which an accepted fix is bled into the published pose "
        "(0 = instant, which EKF2 rejects as an outlier).",
    )
    eb.add_argument("--forest-frac", type=float, default=0.4)
    eb.add_argument("--stale-frac", type=float, default=0.1)
    eb.add_argument("--fidelity-seed", type=int, default=1)
    # Two error axes the acceptance flights never exercised: the vision stream was
    # never interrupted, and heading was always ground truth. Both default to off,
    # so an arm that does not ask for them reproduces the flown runs exactly.
    eb.add_argument(
        "--dropout-interval",
        type=float,
        default=0.0,
        help="Mean METRES between vision lock losses (0 = never drop out).",
    )
    eb.add_argument(
        "--dropout-duration", type=float, default=2.0, help="Seconds each lock loss lasts."
    )
    eb.add_argument("--dropout-seed", type=int, default=2, help="Lock-loss seed.")
    eb.add_argument("--yaw-bias", type=float, default=0.0, help="Constant heading error (degrees).")
    eb.add_argument(
        "--yaw-drift",
        type=float,
        default=0.0,
        help="Heading random-walk growth (degrees per 100 m).",
    )
    eb.add_argument("--yaw-seed", type=int, default=3, help="Yaw-drift seed.")
    eb.add_argument("--corridor", type=float, default=15.0, help="Corridor tolerance (m).")
    eb.add_argument("--duration", type=float, default=0.0, help="Stop after N s of sim time.")
    eb.add_argument("--record", type=Path, help="Write the run as JSONL for `ev-report`.")
    eb.add_argument(
        "--timestamp",
        choices=("none", "epoch", "sim"),
        default="none",
        help="Vision message timestamp; switch if EKF2 rejects the stream (RUNBOOK §3b).",
    )
    eb.add_argument(
        "--server-port",
        type=int,
        default=50052,
        help="gRPC port for this process's mavsdk_server. MUST differ from `fly`'s (50051).",
    )
    eb.add_argument(
        "--connect-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for a MAVLink system on --url before giving up.",
    )
    eb.add_argument(
        "--wait-topic",
        type=float,
        default=30.0,
        help="Seconds to wait for Gazebo to advertise the truth topic before giving up.",
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
    er.add_argument(
        "--record",
        type=Path,
        required=True,
        action="append",
        help="JSONL from `ev-bridge --record`. Repeat to compare acceptance arms.",
    )
    er.add_argument("--corridor", type=float, default=15.0)
    er.set_defaults(func=_ev_report)

    f = sub.add_parser("fly", help="Execute a .plan on PX4 (SITL or hardware).")
    f.add_argument("--plan", type=Path, required=True)
    f.add_argument("--url", default="udpin://0.0.0.0:14540", help="MAVLink connection URL.")
    f.add_argument("--night", action="store_true")
    f.add_argument(
        "--progress-timeout",
        type=float,
        default=None,
        help="Seconds without waypoint progress before abort+RTL "
        "(default: derived from the plan's longest leg).",
    )
    f.set_defaults(func=_fly)

    return parser


def app(argv: list[str] | None = None) -> int:
    # Piping to `tee` makes stdout block-buffered while library logging goes to
    # unbuffered stderr — so progress lines vanish and the run looks stalled.
    with contextlib.suppress(AttributeError, ValueError):
        sys.stdout.reconfigure(line_buffering=True)  # type: ignore[union-attr]

    args = build_parser().parse_args(argv)
    result: int = args.func(args)
    return result


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(app())
