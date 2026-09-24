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
from droneflight.perception import ObstacleField, sense

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


def window_altitude(route: list[RoutePoint], guidance: CarrotGuidance) -> float:
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
) -> StepResult:
    """One control step. Pure apart from the planner's own state."""
    sensor = truth or believed
    scan = sense(
        obstacles,
        x=sensor.east_m,
        y=sensor.north_m,
        heading=sensor.heading_rad,
        fov_rad=math.radians(config.fov_deg),
        n_rays=config.n_rays,
        max_range=config.sensor_range_m,
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

    target_up = window_altitude(route, guidance)
    err = target_up - believed.up_m
    climb = max(-config.max_descent_ms, min(config.max_climb_ms, err * config.climb_gain))

    if cmd.blocked:
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
    true_gap = obstacles.clearance(sensor.east_m, sensor.north_m) - drone_radius_m
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
    )
