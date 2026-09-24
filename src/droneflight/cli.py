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
from droneflight.arrival import STEEP_STEP_M, assess_arrivals
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

    # Vertical acceptance: the per-leg altitude STEP is what separated the route
    # that stalled (18-40 m steps) from the one that flew clean (8.56 m). Counting
    # descending arrivals was measured worthless — see arrival.py.
    arrivals = assess_arrivals(mission)
    steep = arrivals.steep
    print(
        f"vertical acceptance: largest altitude step {arrivals.max_step_m:.1f} m "
        f"({len(steep)} leg(s) over {STEEP_STEP_M:.0f} m)"
    )
    for a in steep[:3]:
        print(
            f"  steep leg into wp {a.index}: {a.descent_m:.1f} m down over "
            f"{a.leg_length_m:.0f} m (gradient {a.required_gradient:.3f})"
        )
    if steep:
        print(
            "  large steps on long legs are what stalled three missions; consider "
            "a smaller waypoint_spacing_m"
        )

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


def _tether_check(args: argparse.Namespace) -> int:
    """Validate the tether-safe speed policy against the v0 payout-tension model.

    Runs the mission's payout tension twice — at the policy speeds and at flat
    cruise — against the break tension.

    Both arms are scored through the **measured PX4 corner prior** (see
    ``px4_profile``): the autopilot decelerates into corners whatever it was
    commanded, so a literal "flat cruise" arm is a profile no vehicle flies.
    ``--no-px4-prior`` restores the old uncalibrated scoring, which over-predicted
    the flown corner load 7-18x at sharp bends. Expect the two arms to agree
    closely with the prior on — acceptance §3 measured that null in flight.
    """
    from droneflight.speed_policy import SpeedPolicy
    from droneflight.tether import TetherConfig, analyse_tether, speed_ceiling_ms

    try:
        mission = load_qgc_plan(args.plan)
    except PlanLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    config = TetherConfig(break_tension_n=args.break_tension, back_tension_n=args.back_tension)
    policy = SpeedPolicy(
        cruise_speed_ms=mission.cruise_speed_ms,
        min_speed_ms=args.min_speed,
        slow_above_deg=args.slow_above,
        reversal_deg=args.reversal_deg,
    )
    prior = not args.no_px4_prior
    with_policy = analyse_tether(mission, config, policy=policy, px4_corner_prior=prior)
    no_policy = analyse_tether(mission, config, policy=None, px4_corner_prior=prior)

    print(
        f"tether v0: break {config.break_tension_n:.1f} N, back-tension "
        f"{config.back_tension_n:.1f} N, cruise {mission.cruise_speed_ms:.1f} m/s"
        f"{'' if prior else '  [PX4 corner prior OFF — uncalibrated]'}"
    )
    for label, rep in (("no policy (plan speeds)", no_policy), ("speed policy", with_policy)):
        peak = rep.peak
        assert peak is not None  # a loaded mission always has >=1 waypoint
        verdict = (
            "WITHIN break limit" if rep.within_limit else f"{len(rep.breaks)} waypoint(s) SNAP"
        )
        print(
            f"  {label:<24}: max {rep.max_tension_n:5.2f} N "
            f"({peak.load_fraction * 100:3.0f}% of break) at wp{peak.index} "
            f"(bend {peak.deflection_deg:.0f}°, {peak.speed_ms:.1f} m/s) — {verdict}"
        )
    if no_policy.breaks and with_policy.within_limit:
        print(
            f"  → the policy prevents {len(no_policy.breaks)} fiber break(s) the flat-cruise "
            "profile would cause."
        )
    elif prior and no_policy.within_limit:
        # The §3 result, made visible at the point of use rather than buried in docs.
        print(
            "  → both arms survive: PX4 decelerates into corners on its own, so the "
            "policy is not what keeps the fiber intact here (acceptance §3, sim-logs/4.7)."
        )
    fastest = max(mission.leg_speed_ms(i) for i in range(len(mission.waypoints)))
    warn = speed_ceiling_ms(args.agl, config, config.warn_fraction)
    snap = speed_ceiling_ms(args.agl, config)
    print(
        f"  straight-leg ceiling at {args.agl:.0f} m AGL if the span free-falls "
        f"(drag ~v^3): warn {warn:.1f} m/s, break {snap:.1f} m/s — plan's fastest leg "
        f"{fastest:.1f} m/s{'  ⚠️ ABOVE WARN' if fastest > warn else ''}"
    )
    return 0 if with_policy.within_limit else 1


def _tether_plugin_check(args: argparse.Namespace) -> int:
    """Score a Gazebo tether-plugin CSV against the offline model (TETHER_PLUGIN.md §1).

    The acceptance gate for the in-loop plugin: every tension it applied is
    re-derived here from ``tether.py`` and diffed. Anything but an exact match
    means the sim was flying a different tether model than the one the offline
    studies were measured against.
    """
    from droneflight.tether import TetherLogError, check_plugin_parity, parse_plugin_log

    try:
        log = parse_plugin_log(args.log)
    except (OSError, TetherLogError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    parity = check_plugin_parity(log)
    budget = args.break_tension
    duration_s = log.rows[-1].t_s - log.rows[0].t_s

    print(
        f"tether plugin log: {parity.samples} samples over {duration_s:.0f} s on {log.link} "
        f"({parity.moving_samples} moving)"
    )
    if not log.enabled:
        print(
            "  ⚠️  plugin was DISABLED for this run — no force was applied. "
            "This is the control arm, not a tether run."
        )
    print(
        f"  config: back {log.config.back_tension_n:.3f} N, span "
        f"{log.config.airborne_span_m:.1f} m, Cd {log.config.drag_coefficient:.2f}, "
        f"d {log.config.fiber_diameter_m * 1000:.3f} mm, floor {log.speed_floor_ms:.2f} m/s"
    )
    print(
        f"  applied: peak {parity.peak_tension_n:.4f} N "
        f"({parity.load_fraction(budget) * 100:.0f}% of a {budget:.1f} N budget), "
        f"mean {parity.mean_tension_n:.4f} N, at up to {parity.peak_speed_ms:.2f} m/s"
    )
    print(f"  turn transient (logged, not injected): peak {parity.peak_turn_n:.4f} N")
    print(
        f"  parity vs tether.py: drag {parity.max_drag_error_n:.3g} N, "
        f"tension {parity.max_tension_error_n:.3g} N, turn {parity.max_turn_error_n:.3g} N, "
        f"|F| {parity.max_force_error_n:.3g} N"
    )

    if parity.exact:
        print("  → EXACT: the in-loop force is the offline model, not a copy of it.")
    else:
        print(
            "  → DIVERGED: the plugin is applying a different model. Re-check "
            "tether_math.hpp against tether.py before scoring any flight with it."
        )
        if parity.bad_static_samples:
            print(
                f"     ({parity.bad_static_samples} sub-floor samples were neither 0 N nor "
                "back-tension)"
            )
    if parity.peak_tension_n > budget:
        print(f"  ⚠️  peak exceeded the {budget:.1f} N break budget — the fiber would snap.")
    return 0 if parity.exact else 1


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
        publish_velocity=getattr(args, "publish_velocity", True),
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
        speed_policy_enabled=not args.no_speed_policy,
    )
    result = asyncio.run(fly_mission(MavsdkConnector(), mission, config))
    print(
        f"{'COMPLETED' if result.completed else 'ABORTED'}: "
        f"{result.items_flown}/{result.items_total} items"
        + (f" — {result.aborted_reason}" if result.aborted_reason else "")
    )
    return 0 if result.completed else 1


def read_groundtruth_track(
    ulog_path: Path,
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float]]:
    """Gazebo truth out of a ULog as ENU metres, ready for `strikes.score_track`.

    ``vehicle_local_position_groundtruth`` is Gazebo's own pose, so its origin is
    the world origin = the vehicle's spawn = plan home — the same frame the trunk
    manifest uses. **Do not substitute ``vehicle_local_position``**: that is
    relative to the EKF local origin, whose datum differs and whose ``ref_alt`` is
    NaN in these logs. That substitution has already produced one confidently
    wrong conclusion in this project.

    NED -> ENU: east = y, north = x, up = -z.
    """
    from pyulog import ULog  # optional analysis dependency

    log = ULog(str(ulog_path), ["vehicle_local_position_groundtruth"])
    if not log.data_list:
        raise ValueError(
            f"{ulog_path} has no vehicle_local_position_groundtruth — it was not a SITL "
            "run, or ground truth logging was off. There is nothing to score."
        )
    d = log.data_list[0].data
    return (
        [t / 1e6 for t in d["timestamp"]],
        list(d["y"]),
        list(d["x"]),
        [-z for z in d["z"]],
        list(d["vy"]),
        list(d["vx"]),
    )


def _b2_score(args: argparse.Namespace) -> int:
    """TEST-PLAN B2: was the flight clean, or did it arrive wrecked?"""
    from droneflight.strikes import StrikeConfig, score_track
    from droneflight.trunk_manifest import TrunkManifestError, load_trunk_manifest

    try:
        manifest = load_trunk_manifest(args.trunks)
    except TrunkManifestError as exc:
        print(f"ERROR: {exc}")
        return 2
    try:
        t, east, north, up, ve, vn = read_groundtruth_track(args.ulog)
    except (OSError, ValueError, KeyError) as exc:
        print(f"ERROR: cannot read {args.ulog}: {exc}")
        return 2

    report = score_track(
        t, east, north, up, ve, vn, manifest, StrikeConfig(accel_threshold_g=args.threshold_g)
    )

    print(f"ulog     {args.ulog.name}")
    print(f"manifest {args.trunks.name} ({len(manifest.trunks)} trunks)")
    print(
        f"samples  {report.samples} ({report.airborne_samples} airborne), "
        f"{report.samples_in_trunk_span} level with a trunk"
    )
    clearance = (
        "n/a" if not math.isfinite(report.min_clearance_m) else f"{report.min_clearance_m:+.3f} m"
    )
    print(f"min trunk-surface clearance  {clearance}")
    if report.min_agl_m is not None:
        print(f"min AGL (from trunk bases)   {report.min_agl_m:+.2f} m")
    print(f"samples inside a trunk       {report.samples_inside_trunk}")
    print()
    print(f"TREE STRIKES   {report.tree_strikes}")
    print(f"GROUND IMPACTS {report.ground_impacts}")
    other = len(report.episodes) - report.tree_strikes - report.ground_impacts
    if other:
        print(f"unattributed   {other}  (usually the ejection tail of a counted impact)")
    if args.verbose and report.episodes:
        print()
        print("    t(s)     kind    peak_g   in->out m/s    clearance      pos")
        for e in report.episodes:
            print(
                f"  {e.t_s:7.1f}  {e.kind:>8s}  {e.peak_g:6.1f}  "
                f"{e.speed_in_ms:5.2f}->{e.speed_out_ms:5.2f}  "
                f"{e.clearance_m:+8.3f} m  E{e.east_m:7.0f} N{e.north_m:7.0f}"
            )
    print()
    print("VERDICT: CLEAN" if report.clean else "VERDICT: NOT CLEAN — arriving is not succeeding")
    return 0 if report.clean else 1


def _fly_corridor(args: argparse.Namespace) -> int:  # pragma: no cover - needs a live autopilot
    """TEST-PLAN B2b: fly the route as a corridor, avoidance in the loop."""
    from droneflight.mavsdk_offboard import MavsdkOffboardConnector
    from droneflight.offboard import OffboardConfig, OffboardResult, fly_corridor
    from droneflight.trunk_manifest import TrunkManifestError, load_trunk_manifest

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        mission = load_qgc_plan(args.plan)
        manifest = load_trunk_manifest(args.trunks)
    except (PlanLoadError, TrunkManifestError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    from droneflight.avoidance import AvoidanceConfig

    print(f"forest: {len(manifest.trunks)} trunks from {args.trunks}")
    config = OffboardConfig(
        lookahead_m=args.lookahead,
        cruise_speed_ms=args.cruise if args.cruise is not None else mission.cruise_speed_ms,
        control_hz=args.control_hz,
        sensor_range_m=args.sensor_range,
        n_rays=args.n_rays,
        stall_timeout_s=args.stall_timeout,
        avoidance=AvoidanceConfig(
            gap_policy=args.gap_policy,
            transient_margin_m=args.transient_margin,
        ),
    )
    print(
        f"avoidance: gap={args.gap_policy}, transient margin={args.transient_margin:.2f} m, "
        f"{args.n_rays} rays"
    )
    obstacles = manifest.obstacle_field()
    if args.no_avoidance:
        # The control arm: same executor, same guidance, blind. Isolates the
        # avoidance layer from the corridor-following that carries it.
        from droneflight.perception import ObstacleField

        obstacles = ObstacleField()
        print("avoidance: DISABLED (control arm) — the trees are still in the world")

    connector = MavsdkOffboardConnector(control_hz=args.control_hz)

    async def run() -> OffboardResult:
        await connector.connect(args.url, ready_timeout_s=60.0)
        await connector.arm()
        await connector.takeoff(args.takeoff_alt)
        try:
            return await fly_corridor(connector, mission, obstacles, config)
        finally:
            await connector.stop_offboard()
            await connector.disconnect()

    result = asyncio.run(run())
    print(
        f"{'REACHED GOAL' if result.reached_goal else 'DID NOT REACH GOAL'}: "
        f"route point {result.route_index}/{result.route_points - 1}, "
        f"{result.steps} control steps, closest obstacle {result.min_clearance_m:.1f} m, "
        f"{result.blocked_steps} step(s) with no free heading"
        + (f" — {result.aborted_reason}" if result.aborted_reason else "")
    )
    return 0 if result.reached_goal else 1


def _fly_local(args: argparse.Namespace) -> int:  # pragma: no cover - needs a live autopilot
    """Fly a .plan offboard with the onboard local planner (2026-09-23 MVP)."""
    from droneflight.ev_bridge import GazeboTruthSource, wait_for_truth_topic
    from droneflight.local_offboard import LocalOffboardConfig, Pose, fly_local
    from droneflight.local_planner import LocalPlannerConfig
    from droneflight.mavsdk_offboard import MavsdkOffboardConnector
    from droneflight.trunk_manifest import TrunkManifestError, load_trunk_manifest

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        mission = load_qgc_plan(args.plan)
        manifest = load_trunk_manifest(args.trunks)
    except (PlanLoadError, TrunkManifestError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    speeds = sorted({round(mission.leg_speed_ms(i), 2) for i in range(len(mission.waypoints))})
    print(
        f"plan: {len(mission.waypoints)} waypoints, leg speeds {speeds} m/s; "
        f"forest: {len(manifest.trunks)} trunks from {args.trunks}"
    )
    config = LocalOffboardConfig(
        planner=LocalPlannerConfig(cruise_speed_ms=max(speeds)),
        fov_deg=args.fov,
        n_rays=int(args.fov) + 1,
        sensor_range_m=args.sensor_range,
        stall_timeout_s=args.stall_timeout,
        max_speed_override_ms=args.max_speed,
    )
    obstacles = manifest.obstacle_field()
    connector = MavsdkOffboardConnector(control_hz=args.control_hz)
    latest: dict[str, Pose] = {}

    async def pump_truth() -> None:
        topic = await asyncio.to_thread(
            wait_for_truth_topic, world=args.world, model=args.model, timeout_s=30.0
        )
        print(f"truth: {topic.topic} ({topic.kind})")
        async for sample in GazeboTruthSource(topic).stream():
            # t_s = SIMULATOR time: the planner must run on the vehicle's clock
            # (Gazebo fell to 0.34x real time with 3125 trunks; see local_offboard).
            latest["pose"] = Pose(
                sample.east_m, sample.north_m, sample.up_m, sample.euler_enu()[2], t_s=sample.t_s
            )

    async def run() -> object:
        truth_task = None
        if args.truth == "gazebo":
            truth_task = asyncio.create_task(pump_truth())
            for _ in range(300):
                if "pose" in latest or truth_task.done():
                    break
                await asyncio.sleep(0.1)
            if "pose" not in latest:
                if truth_task.done() and truth_task.exception() is not None:
                    raise RuntimeError(f"truth source failed: {truth_task.exception()}")
                raise RuntimeError("no ground truth from Gazebo after 30 s")
        await connector.connect(args.url, ready_timeout_s=120.0)
        await connector.arm()
        await connector.takeoff(args.takeoff_alt)
        try:
            return await fly_local(
                connector,
                mission,
                obstacles,
                config,
                truth=(lambda: latest.get("pose")) if truth_task is not None else None,
                record=args.record,
            )
        finally:
            await connector.stop_offboard()
            if truth_task is not None:
                # Await the cancellation so the gz subprocess is closed while the
                # loop is still alive (else: "Event loop is closed" at exit).
                truth_task.cancel()
                with contextlib.suppress(BaseException):
                    await truth_task
            await connector.disconnect()

    result = asyncio.run(run())
    print(result)
    ok = getattr(result, "reached_goal", False) and getattr(result, "contacts", 1) == 0
    print("VERDICT (oracle — confirm with b2-score):", "CLEAN" if ok else "NOT CLEAN")
    return 0 if ok else 1


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

    t = sub.add_parser(
        "tether-check", help="Validate the speed policy against the v0 payout-tension model."
    )
    t.add_argument("--plan", type=Path, required=True)
    t.add_argument("--break-tension", type=float, default=2.0, help="Fiber break tension (N).")
    t.add_argument("--back-tension", type=float, default=0.5, help="Spool back-tension (N).")
    t.add_argument("--min-speed", type=float, default=1.0, help="Speed-policy floor (m/s).")
    t.add_argument("--slow-above", type=float, default=30.0, help="Bend (deg) to start slowing.")
    t.add_argument(
        "--reversal-deg", type=float, default=150.0, help="Bend (deg) treated as reversal."
    )
    t.add_argument(
        "--no-px4-prior",
        action="store_true",
        help=(
            "Score at the commanded speed instead of PX4's measured corner speed. "
            "Reproduces pre-2026-07-26 numbers; over-predicts the flown corner load 7-18x."
        ),
    )
    t.add_argument(
        "--agl",
        type=float,
        default=12.0,
        help="Height the fiber falls from on open legs, for the speed ceiling (m).",
    )
    t.set_defaults(func=_tether_check)

    tp = sub.add_parser(
        "tether-plugin-check",
        help="Score a Gazebo tether-plugin CSV against the offline model (parity gate).",
    )
    tp.add_argument("--log", type=Path, required=True, help="CSV written by the plugin's log_csv.")
    tp.add_argument("--break-tension", type=float, default=2.0, help="Fiber break tension (N).")
    tp.set_defaults(func=_tether_plugin_check)

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
        default=30.0,
        help="Metres over which an accepted fix is bled into the published pose "
        "(0 = instant, which EKF2 rejects as an outlier). 30 since 2026-09-24: at "
        "10 m a 4-5 m fix moved the estimate at ~1.4 m/s, EKF2's velocity took up "
        "part of it, and GPS-denied local-planner flights grazed trunks (s3b); "
        "the same seeds at 30 m flew clean with the same nav error (s3c).",
    )
    eb.add_argument(
        "--publish-velocity",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Also send the VIO velocity (VISION_SPEED_ESTIMATE); set EKF2_EV_CTRL 15 "
        "to fuse it. Without it a VTRN fix drags EKF2's velocity estimate.",
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
    f.add_argument(
        "--no-speed-policy",
        action="store_true",
        help="Fly every waypoint at cruise (disable the tether-safe slowdown). "
        "The control arm for the in-loop speed-policy test (RUNBOOK §4.7).",
    )
    f.set_defaults(func=_fly)

    fl = sub.add_parser(
        "fly-local",
        help="Fly a .plan offboard with the onboard LOCAL PLANNER (remembers trunks, plans "
        "through gaps) — the 2026-09-23 replacement for fly-corridor's reactive layer.",
    )
    fl.add_argument("--plan", type=Path, required=True)
    fl.add_argument(
        "--trunks",
        type=Path,
        required=True,
        help="The trunk manifest the Gazebo world was built from (oracle depth sensor).",
    )
    fl.add_argument("--url", default="udpin://0.0.0.0:14540")
    fl.add_argument("--control-hz", type=float, default=10.0)
    fl.add_argument("--sensor-range", type=float, default=30.0)
    fl.add_argument("--fov", type=float, default=90.0)
    fl.add_argument("--takeoff-alt", type=float, default=10.0)
    fl.add_argument("--stall-timeout", type=float, default=90.0)
    fl.add_argument(
        "--max-speed", type=float, default=None, help="Cap every leg (m/s), below the plan."
    )
    fl.add_argument(
        "--truth",
        choices=("gazebo", "none"),
        default="gazebo",
        help="Where the oracle sensor's pose comes from. 'gazebo' (default) = simulator "
        "truth, required GPS-denied: PX4's estimate is deliberately wrong there, and a "
        "camera sees the real trees from the real aircraft. 'none' = PX4's estimate "
        "(GPS-aided runs only). Must run on the sim host (needs `gz`).",
    )
    fl.add_argument("--world", default="forest_mvp")
    fl.add_argument("--model", default="x500_0")
    fl.add_argument(
        "--record", type=Path, default=None, help="Per-step JSONL (truth, estimate, commands)."
    )
    fl.set_defaults(func=_fly_local)

    fc = sub.add_parser(
        "fly-corridor",
        help="TEST-PLAN B2b: fly a .plan as a corridor with reactive avoidance (offboard).",
    )
    fc.add_argument("--plan", type=Path, required=True)
    fc.add_argument(
        "--trunks",
        type=Path,
        required=True,
        help="Trunk manifest from `droneplanner gz-world --forest-dsm` — the SAME file the "
        "Gazebo world was built from, so perception senses what physics will hit.",
    )
    fc.add_argument("--url", default="udpin://0.0.0.0:14540")
    fc.add_argument("--lookahead", type=float, default=25.0, help="Carrot distance (m).")
    fc.add_argument("--control-hz", type=float, default=10.0)
    fc.add_argument("--sensor-range", type=float, default=30.0)
    fc.add_argument("--takeoff-alt", type=float, default=15.0)
    fc.add_argument(
        "--cruise",
        type=float,
        default=None,
        help="Override the plan's cruise speed (m/s). The reaction distance is derived "
        "from it, so this is the knob the B2 speed-ceiling result says matters: at "
        "107 stems/ha the geometry supports about 3.5 m/s, not the plan's 6.0.",
    )
    fc.add_argument(
        "--n-rays",
        type=int,
        default=91,
        help="Rays across the FOV. 91 over 90 deg = 1.00 deg, which resolves a 0.15 m "
        "trunk to ~17 m. Use 31 to reproduce the B2 arms, which could not see one "
        "past ~6 m.",
    )
    fc.add_argument(
        "--gap-policy",
        choices=("widest", "nearest_goal"),
        default="nearest_goal",
        help="Which free gap to steer into. 'nearest_goal' (default) is the minimum "
        "deviation that clears — the policy N2 flew into 34-41 strikes, kept as the "
        "default only because 'widest' measured 3.3x WORSE offline (243 vs 73). Fly "
        "'widest' to test that offline result in the loop.",
    )
    fc.add_argument(
        "--transient-margin",
        type=float,
        default=0.0,
        help="Metres of lateral room kept between the commanded heading and the edge "
        "of the gap, so the turn transient has somewhere to overshoot. Measured a "
        "wash offline (35 -> 38 strikes); 0.6 is the value worth flying.",
    )
    fc.add_argument(
        "--stall-timeout",
        type=float,
        default=90.0,
        help="Abort after this many seconds without route progress. The dense arm is "
        "expected to hover when avoidance finds no free bearing.",
    )
    fc.add_argument(
        "--no-avoidance",
        action="store_true",
        help="Control arm: fly the identical guidance with an EMPTY obstacle field. "
        "The trees stay in the world, so this measures what avoidance is worth.",
    )
    fc.set_defaults(func=_fly_corridor)

    bs = sub.add_parser(
        "b2-score",
        help="TEST-PLAN B2: score a flown ULog for tree strikes against its trunk manifest.",
    )
    bs.add_argument("--ulog", type=Path, required=True, help="PX4 .ulg from the arm.")
    bs.add_argument(
        "--trunks",
        type=Path,
        required=True,
        help="The trunk manifest the world was built from. Scoring an arm against "
        "the OTHER arm's manifest is the check that catches a stale Gazebo server "
        "serving the wrong world — impacts align with the world that was loaded.",
    )
    bs.add_argument(
        "--threshold-g",
        type=float,
        default=2.0,
        help="Deceleration counted as contact. The default is 4x the ceiling the "
        "tree-free B1 control ever reached; re-derive it if the airframe changes.",
    )
    bs.add_argument("--verbose", action="store_true", help="List every episode.")
    bs.set_defaults(func=_b2_score)

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
