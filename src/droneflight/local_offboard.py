"""Fly the route offboard with the onboard local planner in the loop (2026-09-23).

This is the live counterpart of `reactive.fly_with_local_planner`: the same
`local_planner.LocalPlanner`, fed the same kind of scan, driving PX4 in offboard
velocity mode. It replaces `offboard.fly_corridor`'s reactive `avoid()` step,
which flew into 34-41 trunks (TEST-PLAN N2), with a planner that remembers what
it has seen and plans through the gaps.

TWO POSES, AND WHY BOTH ARE NEEDED
GPS-denied, PX4's estimate is wrong by design — the EV bridge injects drift into
it. The oracle depth sensor must see the *real* trees from the *real* aircraft, so
it is swept from the simulator's **truth** pose; its returns are then placed in
the map, and the plan followed, in the **believed** (PX4) frame. That is exactly
what a real aircraft does — its camera sees true geometry, its software knows only
the estimate — and it is the property that makes local planning immune to slowly
varying navigation error (see `local_planner`). Without a truth source (a
GPS-aided run) the believed pose is used for both, which is what B2 did.

ALTITUDE
Commanded by us, from the plan's profile: the **highest** planned altitude
between the aircraft's projected position and the local goal. Climbing for the
highest point in view and descending only once it is behind is the offboard
equivalent of `droneplanner`'s CFIT rule — PX4 mission mode front-loaded descents
into a hillside once (SHORTFALLS item 0), and taking the altitude *at* a carrot
25 m ahead would reproduce that on every falling slope.

SPEED
The planner's per-leg speed from the ``.plan`` (``DO_CHANGE_SPEED``, the
stem-density cap) is a ceiling; the local planner flies slower when its own
limits (curvature, sight, gap, feasibility) say so.

⚠️ The live MAVSDK loop is excluded from coverage like every other adapter here;
the per-step decision (:func:`local_step`) and the altitude rule are pure and
unit-tested.
"""

from __future__ import annotations

import itertools
import json
import logging
import math
import time
from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

from droneflight.local_planner import Corridor, LocalPlanner, LocalPlannerConfig
from droneflight.model import FlightMission
from droneflight.offboard import (
    CarrotGuidance,
    RoutePoint,
    VelocityCommand,
    enu_bearing_to_ned_yaw,
    ned_yaw_to_enu,
    route_to_local,
)
from droneflight.perception import OVERFLY_MARGIN_M, ObstacleField, sense

_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class LocalOffboardConfig:
    planner: LocalPlannerConfig = field(default_factory=LocalPlannerConfig)
    fov_deg: float = 90.0
    n_rays: int = 91
    sensor_range_m: float = 30.0
    climb_gain: float = 0.5
    max_climb_ms: float = 2.0
    max_descent_ms: float = 1.5
    """PX4's ``MPC_Z_VEL_MAX_DN`` default — the 2026-09-14 MVP flight measured the
    vehicle clamping descent there, not at the airframe's 2.0."""

    arrival_radius_m: float = 5.0
    escape_after_s: float | None = 20.0
    """ESCAPE CLIMB (2026-09-25, user: "rise over the canopy if it looks unpassable
    below"): after this long without route progress, while trunks rise above the
    aircraft, climb straight up over them instead of waiting for the stall abort.
    None = off. The preflight report states this policy and how high it goes
    (droneplanner finding ``escape_climb_policy``), so it is never a surprise."""

    escape_margin_m: float = 3.0
    """Height kept above the tallest trunk top within ``escape_radius_m``."""

    escape_radius_m: float = 15.0
    escape_hold_m: float = 50.0
    """Route distance flown at the escape height before descending again — past the
    stand that blocked it. The descent senses at its target height (o4e)."""

    stall_timeout_s: float = 90.0
    progress_log_s: float = 5.0
    use_corridor: bool = True
    """Bound lateral deviation by the plan's corridor widths when it carries them."""

    max_speed_override_ms: float | None = None
    """Cap every leg at this speed regardless of the plan (for a first cautious run)."""


@dataclass(frozen=True)
class Pose:
    """Home-relative ENU position and math heading (rad, CCW from east)."""

    east_m: float
    north_m: float
    up_m: float
    heading_rad: float
    t_s: float | None = None
    """Simulator time of the sample, when it comes from the simulator."""


@dataclass(frozen=True)
class StepResult:
    command: VelocityCommand
    speed_ms: float
    blocked: bool
    gap_ahead_m: float
    route_index: int
    target_up_m: float
    true_gap_m: float
    limit: str = ""


@dataclass
class Escape:
    """An in-flight climb over a stand the local planner cannot get through."""

    up_m: float | None = None
    """Believed-frame altitude held while escaping; None = not escaping."""

    release_at_m: float = 0.0
    """Route arc length at which the escape ends."""

    count: int = 0
    at_height: bool = False
    """Reached the escape height (and dropped the memory mapped below it)."""

    best_along_m: float = 0.0
    last_progress_t: float = 0.0

    def update(
        self,
        *,
        t_s: float,
        along_m: float,
        index: int,
        arc_m: list[float],
        believed: Pose,
        truth: Pose | None,
        obstacles: ObstacleField,
        planner: LocalPlanner,
        config: LocalOffboardConfig,
    ) -> tuple[list[str], bool]:
        """Advance the escape state machine one step; shared by ``fly_local`` and the
        offline harness so the two fly the same logic. Returns (log lines, whether an
        escape just started — the caller's stall timer restarts then)."""
        logs: list[str] = []
        # 🔴 Progress is DISTANCE along the route, not the vertex index: at 0.5 m/s a
        # 25 m leg takes 50 s, and SITL x4e escaped 20 s after take-off at route 0.
        if along_m > self.best_along_m + ESCAPE_PROGRESS_M:
            self.best_along_m, self.last_progress_t = along_m, t_s
        if self.up_m is not None and not self.at_height and believed.up_m >= self.up_m - 1.0:
            # 🔴 SITL x5e: escaped, then stalled at height 90 s (1130 failed plans) —
            # memory held every trunk mapped from below. At the escape height those
            # are beneath the aircraft; anything that really reaches it is sensed afresh.
            planner.memory.clear()
            planner.path = None
            self.at_height = True
            logs.append(f"escape at height {believed.up_m:.1f} m: memory from below dropped")
        if self.up_m is not None and arc_m[index] >= self.release_at_m:
            self.up_m = None
            logs.append(f"escape climb released at route {index} — descending")
        started = False
        if (
            config.escape_after_s is not None
            and self.up_m is None
            and truth is not None
            and t_s - self.last_progress_t > config.escape_after_s
        ):
            alt = escape_altitude(
                obstacles,
                truth,
                believed,
                radius_m=config.escape_radius_m,
                margin_m=config.escape_margin_m,
            )
            if alt is not None:
                self.up_m, self.at_height = alt, False
                self.release_at_m = arc_m[index] + config.escape_hold_m
                self.count += 1
                self.last_progress_t = t_s
                started = True
                logs.append(
                    f"ESCAPE CLIMB #{self.count} at route {index}: no progress for "
                    f"{config.escape_after_s:.0f} s among trunks — climbing from "
                    f"{believed.up_m:.1f} to {alt:.1f} m, holding for "
                    f"{config.escape_hold_m:.0f} m of route"
                )
        return logs, started


ESCAPE_PROGRESS_M = 1.0
"""Along-route gain that counts as progress for the escape trigger."""


def route_arc_m(route: list[RoutePoint]) -> list[float]:
    """Cumulative horizontal distance along the route at each vertex."""
    out = [0.0]
    for a, b in itertools.pairwise(route):
        out.append(out[-1] + math.hypot(b.east_m - a.east_m, b.north_m - a.north_m))
    return out


def escape_altitude(
    obstacles: ObstacleField, truth: Pose, believed: Pose, *, radius_m: float, margin_m: float
) -> float | None:
    """Believed-frame altitude that clears every trunk top within ``radius_m``, or
    None if nothing nearby rises above the aircraft (then climbing cannot help).

    The simulator knows the trunk tops (the manifest); a real aircraft would take
    the canopy top from its own sensing or the plan's over-canopy altitude."""
    tops = [
        t.top_z
        for t in obstacles.trunks
        if math.isfinite(t.top_z)
        and math.hypot(t.x - truth.east_m, t.y - truth.north_m) <= radius_m
    ]
    if not tops or max(tops) + OVERFLY_MARGIN_M <= truth.up_m:
        return None
    return believed.up_m + (max(tops) + margin_m - truth.up_m)


CLIMB_AT_VERTEX_M = 4.0
"""A climb step bigger than this between consecutive route points starts AT the
lower point (where the planner put the canopy opening), not up to a look-ahead
early, and is flown in place before moving on. Smaller steps — ordinary terrain
following — keep the early climb."""

CLIMB_IN_PLACE_M = 3.0
"""Climb error above which horizontal motion waits for the climb."""


def window_altitude(
    route: list[RoutePoint],
    guidance: CarrotGuidance,
    under_canopy: Callable[[int], bool] | None = None,
) -> float:
    """Highest planned altitude from the current route leg to the carrot.

    Starts from the vertex *behind* the aircraft, so a descent begins only once
    the higher vertex is behind it (descend late), and walks forward vertex by
    vertex through the lookahead (climb early). Reads the route only — it must
    not touch ``guidance``'s state, whose ``finished`` flag ends the flight.
    """
    i = guidance.index
    best = route[i].up_m
    remaining = guidance.lookahead_m
    while i < len(route) - 1 and remaining > 0.0:
        a, b = route[i], route[i + 1]
        if (
            b.up_m - a.up_m > CLIMB_AT_VERTEX_M
            and i > guidance.index
            and (under_canopy is None or under_canopy(i))
        ):
            # A big step is a climb OUT of the canopy (or over a stand); the planner
            # put it at a canopy opening at vertex ``i``. Do not start it early —
            # climbing before the opening means climbing into crowns (2026-09-25).
            break
        best = max(best, b.up_m)
        remaining -= math.hypot(b.east_m - a.east_m, b.north_m - a.north_m)
        i += 1
    return best


def mission_corridor(mission: FlightMission, route: list[RoutePoint]) -> Corridor | None:
    """The plan's per-vertex lateral freedom, or None if the plan states none."""
    widths = [w.corridor_half_width_m for w in mission.waypoints]
    if len(widths) != len(route) or any(w is None for w in widths):
        return None
    return [((r.east_m, r.north_m), float(w or 0.0)) for r, w in zip(route, widths, strict=True)]


def altitude_datum_error(
    believed: Pose, truth: Pose | None, route: list[RoutePoint], guidance: CarrotGuidance
) -> str | None:
    """Why the altitude frame looks wrong at offboard start, or None if it looks fine.

    🔴 The first GPS-denied run fed EKF2's local z, which with external vision sat
    124.6 m below truth (home AMSL). The controller then climbed at full rate for a
    target 118 m "above" it. Just after takeoff the aircraft must be within a few
    tens of metres of the plan's first altitude, and of the simulator's truth.
    """
    planned = route[min(guidance.index, len(route) - 1)].up_m
    if abs(believed.up_m - planned) > DATUM_TOLERANCE_M:
        return (
            f"believed altitude {believed.up_m:.1f} m vs planned {planned:.1f} m at start "
            "— altitude datum mismatch (is 'up' relative to home?)"
        )
    if truth is not None and abs(believed.up_m - truth.up_m) > DATUM_TOLERANCE_M:
        return (
            f"believed altitude {believed.up_m:.1f} m vs simulator truth {truth.up_m:.1f} m "
            "— altitude datum mismatch"
        )
    return None


DATUM_TOLERANCE_M = 25.0


def local_step(
    *,
    believed: Pose,
    velocity_en: tuple[float, float],
    truth: Pose | None,
    obstacles: ObstacleField,
    planner: LocalPlanner,
    guidance: CarrotGuidance,
    route: list[RoutePoint],
    mission: FlightMission,
    config: LocalOffboardConfig,
    t_s: float,
    drone_radius_m: float = 0.45,
    corridor: Corridor | None = None,
    escape: Escape | None = None,
) -> StepResult:
    """One control step. Pure apart from the planner's own state (and ``escape``,
    which the caller arms and releases)."""
    sensor = truth or believed
    escaping = escape is not None and escape.up_m is not None
    # Below-canopy legs are the plan's slow ones (stem-density caps); a big climb at
    # the end of one is a climb out of the canopy. Over the canopy the skim profile
    # has many > 4 m steps at cruise — those keep the early, moving climb (the first
    # SITL try of the rule stopped at each and crawled at 0.35 m/s, o6e).
    fastest = max(mission.leg_speed_ms(k) for k in range(len(mission.waypoints)))

    def under_canopy(i: int) -> bool:
        return i >= 1 and mission.leg_speed_ms(i - 1) < fastest - 1e-6

    heading_to = window_altitude(route, guidance, under_canopy)
    if escape is not None and escape.up_m is not None:
        heading_to = max(heading_to, escape.up_m)
    # 🔴 Sense at the height the aircraft is GOING to, when that is lower: SITL o4e
    # (2026-09-25) passed over 9 m trunks at 14 m — rightly ignoring them — then
    # descended back under the canopy straight onto one it had never mapped. A
    # level-looking sensor cannot see trunk tops below it; rejoining the trunk
    # layer needs a look at it first (a real camera pitches down on a descent).
    sense_z: float | None = None
    if truth is not None:
        sense_z = min(truth.up_m, heading_to + (truth.up_m - believed.up_m))
    scan = sense(
        obstacles,
        x=sensor.east_m,
        y=sensor.north_m,
        heading=sensor.heading_rad,
        fov_rad=math.radians(config.fov_deg),
        n_rays=config.n_rays,
        max_range=config.sensor_range_m,
        # Only simulator truth shares the manifest's world z frame.
        z=sense_z,
    )
    # The scan is body-relative: place it from the BELIEVED pose.
    added = planner.observe(
        scan,
        believed.east_m,
        believed.north_m,
        believed.heading_rad,
        velocity=velocity_en,
        t_s=t_s,
    )
    carrot = guidance.advance(believed.east_m, believed.north_m)
    leg_cap = mission.leg_speed_ms(min(guidance.index, len(mission.waypoints) - 1))
    if config.max_speed_override_ms is not None:
        leg_cap = min(leg_cap, config.max_speed_override_ms)
    cmd = planner.command(
        believed.east_m,
        believed.north_m,
        (carrot.east_m, carrot.north_m),
        t_s,
        added,
        velocity=velocity_en,
        max_speed_ms=leg_cap,
        corridor=corridor,
    )

    target_up = window_altitude(route, guidance, under_canopy)
    if escape is not None and escape.up_m is not None:
        target_up = max(target_up, escape.up_m)
    err = target_up - believed.up_m
    climb = max(-config.max_descent_ms, min(config.max_climb_ms, err * config.climb_gain))

    # An escape climbs straight up before it moves on: the aircraft is stuck among
    # trunks, and the only free direction it has shown is up.
    climbing_out = (escaping and believed.up_m < target_up - 1.0) or (
        target_up - believed.up_m > CLIMB_IN_PLACE_M
        and mission.leg_speed_ms(guidance.index) < fastest - 1e-6
    )
    if cmd.blocked or climbing_out:
        ve = vn = 0.0
        look = math.atan2(carrot.north_m - believed.north_m, carrot.east_m - believed.east_m)
    else:
        ve, vn = cmd.vx, cmd.vy
        look = math.atan2(vn, ve)
    command = VelocityCommand(
        north_ms=vn,
        east_ms=ve,
        down_ms=-climb,
        yaw_deg=enu_bearing_to_ned_yaw(look),
        blocked=cmd.blocked,
        clearance_ahead_m=cmd.min_gap_ahead_m,
    )
    true_gap = (
        obstacles.clearance(
            sensor.east_m, sensor.north_m, truth.up_m if truth is not None else None
        )
        - drone_radius_m
    )
    return StepResult(
        command=command,
        speed_ms=cmd.speed_ms,
        blocked=cmd.blocked,
        gap_ahead_m=cmd.min_gap_ahead_m,
        route_index=guidance.index,
        target_up_m=target_up,
        true_gap_m=true_gap,
        limit=cmd.limit,
    )


class LocalOffboardConnector(Protocol):
    async def start_offboard(self) -> None: ...
    async def set_velocity_ned(self, command: VelocityCommand) -> None: ...
    def telemetry_full(self) -> AsyncIterator[tuple[float, float, float, float, float, float]]:
        """``(east_m, north_m, up_m, yaw_deg, v_east_ms, v_north_ms)``, believed frame."""
        ...

    async def stop_offboard(self) -> None: ...


@dataclass(frozen=True)
class LocalOffboardResult:
    reached_goal: bool
    steps: int
    route_index: int
    route_points: int
    contacts: int
    """Steps with the TRUE airframe inside a trunk's surface (oracle; confirm with
    ``b2-score`` on the ULog, which is the authority)."""

    min_true_gap_m: float
    replans: int
    plan_failures: int
    reverts: int
    infeasible: int
    memory_resets: int
    blocked_steps: int
    elapsed_s: float
    aborted_reason: str | None = None
    escapes: int = 0
    """Escape climbs flown (``LocalOffboardConfig.escape_after_s``)."""


async def fly_local(
    connector: LocalOffboardConnector,
    mission: FlightMission,
    obstacles: ObstacleField,
    config: LocalOffboardConfig,
    *,
    truth: Callable[[], Pose | None] | None = None,
    record: Path | None = None,
    max_steps: int = 1_000_000,
) -> LocalOffboardResult:  # pragma: no cover - needs a live autopilot
    """Fly ``mission`` with the local planner. ``truth`` returns the latest sim pose."""
    route = route_to_local(mission)
    guidance = CarrotGuidance(route=route, lookahead_m=config.planner.horizon_m)
    planner = LocalPlanner(config=config.planner)
    escape = Escape()
    arc = route_arc_m(route)
    corridor = mission_corridor(mission, route) if config.use_corridor else None
    _log.info(
        "corridor: %s",
        f"from the plan (floor {config.planner.min_corridor_half_width_m:.1f} m)"
        if corridor
        else "NONE — the plan carries no widths, lateral freedom is unbounded",
    )
    started = time.monotonic()
    last_log = started
    last_progress = 0.0
    best_index = -1
    steps = contacts = blocked = 0
    min_gap = math.inf
    in_contact = False
    sim_t0: float | None = None
    last_t = 0.0
    reached = False
    aborted: str | None = None
    out = record.open("w", encoding="utf-8") if record is not None else None
    _log.info(
        "local planner: %d route points, %d trunks, horizon %.0f m, FOV %.0f deg / %d rays",
        len(route),
        len(obstacles.trunks),
        config.planner.horizon_m,
        config.fov_deg,
        config.n_rays,
    )
    if truth is None:
        _log.warning("local planner: NO TRUTH SOURCE — sensing from PX4's estimate")
    await connector.start_offboard()
    try:
        async for east, north, up, yaw_deg, v_e, v_n in connector.telemetry_full():
            steps += 1
            now = time.monotonic()
            true_pose = truth() if truth is not None else None
            # 🔴 The planner's clock must be the VEHICLE's clock. PX4 reports
            # velocity in sim time; with 3125 trunk models Gazebo ran at ~0.38x real
            # time, so wall-clock steps made every step look like a 0.36 m
            # navigation "correction", the map was shifted each step, trunks smeared
            # and the 400/ha flight stalled with 718 failed plans. Sim time when the
            # simulator provides it; wall time on a real aircraft.
            if true_pose is not None and true_pose.t_s is not None:
                if sim_t0 is None:
                    sim_t0 = true_pose.t_s
                t = true_pose.t_s - sim_t0
            else:
                t = now - started
            believed = Pose(east, north, up, ned_yaw_to_enu(yaw_deg))
            if steps == 1:
                datum_error = altitude_datum_error(believed, true_pose, route, guidance)
                if datum_error is not None:
                    aborted = datum_error
                    _log.error("local planner: REFUSING TO FLY — %s", aborted)
                    break
            step = local_step(
                believed=believed,
                velocity_en=(v_e, v_n),
                truth=true_pose,
                obstacles=obstacles,
                planner=planner,
                guidance=guidance,
                route=route,
                mission=mission,
                config=config,
                t_s=t,
                corridor=corridor,
                escape=escape,
            )
            await connector.set_velocity_ned(step.command)
            min_gap = min(min_gap, step.true_gap_m)
            if step.true_gap_m < 0.0 and not in_contact:
                contacts += 1
                _log.error(
                    "CONTACT #%d at route %d (true gap %.2f m)",
                    contacts,
                    step.route_index,
                    step.true_gap_m,
                )
            in_contact = step.true_gap_m < 0.0
            blocked += int(step.blocked)
            if out is not None:
                out.write(
                    json.dumps(
                        {
                            "t": round(t, 3),
                            "est": [round(east, 3), round(north, 3), round(up, 3)],
                            "true": (
                                [
                                    round(true_pose.east_m, 3),
                                    round(true_pose.north_m, 3),
                                    round(true_pose.up_m, 3),
                                ]
                                if true_pose
                                else None
                            ),
                            "v": [round(v_e, 3), round(v_n, 3)],
                            "cmd": [
                                round(step.command.east_ms, 3),
                                round(step.command.north_ms, 3),
                                round(step.command.down_ms, 3),
                            ],
                            "speed": round(step.speed_ms, 3),
                            "gap_ahead": round(step.gap_ahead_m, 3)
                            if math.isfinite(step.gap_ahead_m)
                            else None,
                            "true_gap": round(step.true_gap_m, 3)
                            if math.isfinite(step.true_gap_m)
                            else None,
                            "idx": step.route_index,
                            "tgt_up": round(step.target_up_m, 2),
                            "escape": escape.up_m is not None,
                            "blocked": step.blocked,
                            "limit": step.limit,
                            "yaw": round(math.degrees(believed.heading_rad), 1),
                            "true_yaw": round(math.degrees(true_pose.heading_rad), 1)
                            if true_pose
                            else None,
                            "mem": len(planner.memory),
                            # What the planner believed, so a contact replays
                            # exactly (replaying inputs diverged within seconds).
                            "path": [
                                [round(px, 2), round(py, 2)]
                                for px, py in (planner.path or [])[:30:2]
                            ],
                            "cands": [
                                [round(cv, 2), round(cg, 3)] for cv, cg in planner.last_candidates
                            ],
                            "mem_near": [
                                [round(mx, 2), round(my, 2)]
                                for mx, my in planner.memory.points()
                                if abs(mx - east) < 5.0 and abs(my - north) < 5.0
                            ][:40],
                        }
                    )
                    + "\n"
                )
            if guidance.index > best_index:
                best_index, last_progress = guidance.index, t
            if now - last_log >= config.progress_log_s:
                rtf = (t - last_t) / max(now - last_log, 1e-6)
                last_log, last_t = now, t
                err = (
                    math.hypot(true_pose.east_m - east, true_pose.north_m - north)
                    if true_pose
                    else float("nan")
                )
                _log.info(
                    "route %d/%d  est %.0f,%.0f  alt %.1f->%.1f  v %.1f (cap %.1f)  "
                    "gap %.2f  mem %d  replans %d  nav err %.1f m  contacts %d  rtf %.2f",
                    guidance.index,
                    len(route) - 1,
                    east,
                    north,
                    up,
                    step.target_up_m,
                    math.hypot(v_e, v_n),
                    step.speed_ms,
                    step.true_gap_m,
                    len(planner.memory),
                    planner.replans,
                    err,
                    contacts,
                    rtf,
                )
            logs, started = escape.update(
                t_s=t,
                along_m=guidance.along_m(arc),
                index=guidance.index,
                arc_m=arc,
                believed=believed,
                truth=true_pose,
                obstacles=obstacles,
                planner=planner,
                config=config,
            )
            for line in logs:
                _log.warning(line)
            if started:
                last_progress = t
            if t - last_progress > config.stall_timeout_s:
                aborted = f"no route progress for {config.stall_timeout_s:.0f}s at {guidance.index}"
                _log.error("local planner: %s", aborted)
                break
            last = route[-1]
            if (
                guidance.finished
                and math.hypot(last.east_m - east, last.north_m - north) <= config.arrival_radius_m
            ):
                reached = True
                break
            if steps >= max_steps:
                break
    finally:
        if out is not None:
            out.close()
        await connector.stop_offboard()
    return LocalOffboardResult(
        reached_goal=reached,
        steps=steps,
        route_index=guidance.index,
        route_points=len(route),
        contacts=contacts,
        min_true_gap_m=min_gap,
        replans=planner.replans,
        plan_failures=planner.failures,
        reverts=planner.reverts,
        infeasible=planner.infeasible,
        memory_resets=planner.memory_resets,
        blocked_steps=blocked,
        elapsed_s=time.monotonic() - started,
        aborted_reason=aborted,
        escapes=escape.count,
    )
