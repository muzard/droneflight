"""Fly a route as a *corridor*, with reactive avoidance in the loop (TEST-PLAN B2b/B3).

WHY THIS EXISTS
PX4 mission mode tracks the planned polyline and interpolates altitude between
waypoints. That is the wrong executor for this program, and two measurements say
so:

* **A2** — the 282-waypoint route reduces to **10 gates** with concealment intact,
  but the altitude step per leg goes 8.6 m → 57.7 m, which is the CFIT shape that
  already crashed an aircraft. So gates are viable only for an executor that
  supplies its own altitude.
* **B1** — mission mode held cross-track to 0.42 m mean. It uses *none* of the
  lateral freedom the corridor grants, so it cannot dodge anything.

This module is that other executor: it follows the route as a corridor, steers
around what it senses, and commands altitude itself.

HOW IT FLIES
A carrot-chasing guidance law with the reactive layer in between:

1. take the vehicle's position from telemetry (NED, relative to the plan's home);
2. find the **carrot** — a point ``lookahead_m`` ahead along the planned
   polyline — which is what makes this corridor-following rather than
   waypoint-chasing: the aircraft is never required to *reach* a waypoint, only
   to make progress past it, so no vertical-acceptance test can stall it;
3. sense the trunks around it (``perception.sense``) and ask ``avoidance.avoid``
   for a heading that makes progress toward the carrot without hitting anything;
4. command a velocity in that direction, with the **altitude taken from the
   planned profile at the carrot** — the terrain-following the planner already
   solved, rather than an interpolation between distant gates.

FRAMES — the recurring trap in this codebase
Telemetry is **NED** (x=north, y=east, z=down, yaw clockwise from north).
``perception``/``avoidance`` work in the **ENU/math** convention the trunk
manifest uses (x=east, y=north, bearing CCW from east). :func:`ned_yaw_to_enu`
is the one place that converts, and it is unit-tested, because getting it wrong
produces a drone that confidently steers into trees at 90° to the danger.

⚠️ **The live MAVSDK path is not unit-testable** and is excluded from coverage,
exactly like ``mavsdk_connector.py``. Everything that can be tested off the
autopilot — guidance, frames, the altitude profile, arrival — is pure and lives
above the connector boundary.
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import AsyncIterator
from dataclasses import dataclass, field, replace
from typing import Protocol, runtime_checkable

from droneflight.avoidance import AvoidanceConfig, avoid
from droneflight.maneuver import (
    max_speed_for_react_distance_ms,
    required_rays,
    required_react_distance_m,
)
from droneflight.model import FlightMission
from droneflight.perception import ObstacleField, sense

_log = logging.getLogger(__name__)

_EARTH_R = 6_371_000.0


def ned_yaw_to_enu(yaw_deg: float) -> float:
    """PX4 yaw (deg, clockwise from north) -> math bearing (rad, CCW from east)."""
    return math.radians(90.0 - yaw_deg)


def enu_bearing_to_ned_yaw(bearing_rad: float) -> float:
    """The inverse, for commanding a yaw setpoint."""
    return (90.0 - math.degrees(bearing_rad)) % 360.0


@dataclass(frozen=True)
class RoutePoint:
    """A planned point in the local ENU frame, with its commanded altitude."""

    east_m: float
    north_m: float
    up_m: float
    """Altitude relative to home — the plan's own frame."""


def route_to_local(mission: FlightMission) -> list[RoutePoint]:
    """Project the mission's WGS84 waypoints into home-relative ENU metres."""
    lat0, lon0 = mission.home_lat, mission.home_lon
    cos_lat = math.cos(math.radians(lat0))
    return [
        RoutePoint(
            east_m=math.radians(w.lon - lon0) * cos_lat * _EARTH_R,
            north_m=math.radians(w.lat - lat0) * _EARTH_R,
            up_m=w.alt_agl_m,
        )
        for w in mission.waypoints
    ]


@dataclass(frozen=True)
class OffboardConfig:
    """How the corridor is flown."""

    lookahead_m: float = 25.0
    """How far ahead on the route the carrot sits. This is the corridor-following
    knob: larger cuts corners and grants the avoidance layer more room, smaller
    tracks the planned line more tightly."""

    cruise_speed_ms: float = 6.0
    arrival_radius_m: float = 15.0
    """Distance at which the final waypoint counts as reached. Unlike PX4's
    acceptance radius this applies only to the *last* point — intermediate ones
    are passed, never awaited, which is what removes the stall mode."""

    climb_gain: float = 0.5
    """Vertical velocity per metre of altitude error, capped by ``max_climb_ms``."""

    max_climb_ms: float = 2.0
    control_hz: float = 10.0
    progress_log_s: float = 5.0
    """How often to report progress. A silent loop is indistinguishable from a
    hang — the first B2 run was killed by hand after 11 minutes of flying
    perfectly well, because nothing was printed."""

    stall_timeout_s: float = 90.0
    """Abort if the route index has not advanced in this long. The dense-forest
    arm is *expected* to hover when avoidance finds no free bearing, and without
    this it hovers until someone notices."""

    fov_deg: float = 90.0
    n_rays: int = 91
    """Rays across the FOV. 91 over 90 deg = 1.00 deg, which resolves a 0.15 m
    trunk to ~17 m and so can actually feed the derived reaction distance.

    🔴 Was **31** (3.00 deg) for every B2 flight, which resolves a trunk only to
    ~6 m — detection falls to **60.9 %** by 15 m, measured against the B2 manifest
    along the flown track. That is why the 6.0 m reaction distance looked
    self-consistent: past it, trunks fell *between* rays. This is a limit of the
    ray model, not of optics (1280 px over 90 deg is 0.070 deg/px), so raising it
    costs CPU and nothing else. See `maneuver.required_rays`.
    """

    sensor_range_m: float = 30.0
    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)

    @property
    def avoidance_config(self) -> AvoidanceConfig:
        """Avoidance tuning with **this executor's** cruise speed forced into it.

        ``AvoidanceConfig`` carries its own ``cruise_speed_ms`` (5.0, tuned for the
        kinematic sim). Left alone it silently overrides the planned speed, so the
        aircraft flies slower than the plan says and every time-based number is
        wrong. One speed, set here.

        That matters more than it used to: ``react_distance_m`` is now **derived
        from this speed** (`maneuver.required_react_distance_m`), so the cruise
        speed forced in here sets how far ahead the aircraft reacts. Fly faster,
        react earlier — which is exactly what B2 showed a constant cannot do.
        """
        return replace(self.avoidance, cruise_speed_ms=self.cruise_speed_ms)

    def sensing_shortfall(self) -> str | None:
        """Why this configuration cannot avoid an obstacle, or None if it can.

        Checked before takeoff. B2 flew a configuration that was *geometrically*
        unable to miss a head-on trunk and reported ``REACHED GOAL`` through 37
        strikes; refusing to launch it is cheaper than scoring the ULog afterwards.
        """
        cfg = self.avoidance_config
        if not cfg.is_survivable:
            need = required_react_distance_m(
                self.cruise_speed_ms,
                cfg.lateral_clearance_m,
                replace(cfg.limits, safety_factor=1.0),
            )
            survivable_ms = max_speed_for_react_distance_ms(
                cfg.react_distance_m, cfg.lateral_clearance_m, cfg.limits
            )
            return (
                f"react distance {cfg.react_distance_m:.1f} m is below the {need:.1f} m "
                f"needed to dodge a {cfg.obstacle_radius_m:.2f} m obstacle at "
                f"{self.cruise_speed_ms:.1f} m/s — the airframe cannot miss it. "
                f"Slow to {survivable_ms:.1f} m/s or raise the reaction distance."
            )
        need_rays = required_rays(
            math.radians(self.fov_deg), cfg.react_distance_m, cfg.obstacle_radius_m
        )
        if self.n_rays < need_rays:
            return (
                f"{self.n_rays} rays over {self.fov_deg:.0f} deg cannot resolve a "
                f"{cfg.obstacle_radius_m:.2f} m obstacle at the {cfg.react_distance_m:.1f} m "
                f"reaction distance — {need_rays} needed, so obstacles fall between rays."
            )
        return None


@dataclass(frozen=True)
class VelocityCommand:
    """A velocity setpoint in NED, plus the yaw to point at."""

    north_ms: float
    east_ms: float
    down_ms: float
    yaw_deg: float
    blocked: bool
    clearance_ahead_m: float


def _project_onto_segment(
    east_m: float, north_m: float, a: RoutePoint, b: RoutePoint
) -> tuple[float, float]:
    """Return ``(t, distance)`` — where on segment a→b the point projects, and how far off."""
    abx, aby = b.east_m - a.east_m, b.north_m - a.north_m
    denom = abx * abx + aby * aby
    if denom <= 0.0:
        return 0.0, math.hypot(east_m - a.east_m, north_m - a.north_m)
    t = ((east_m - a.east_m) * abx + (north_m - a.north_m) * aby) / denom
    t = max(0.0, min(1.0, t))
    return t, math.hypot(east_m - (a.east_m + abx * t), north_m - (a.north_m + aby * t))


@dataclass
class CarrotGuidance:
    """Tracks progress along the route and produces the carrot to chase.

    Progress is by **projection onto the polyline**, not by proximity to the next
    vertex. That distinction is load-bearing: a reactive detour can leave the
    aircraft further from its next waypoint than the lookahead, and a
    proximity rule would then sit chasing a point it has already flown past —
    a stall with no error message. Projection also makes progress naturally
    monotonic (the search only ever runs forward), so a detour that pushes the
    aircraft backwards cannot make it re-fly a section.
    """

    route: list[RoutePoint]
    lookahead_m: float
    index: int = 0
    carrot_is_last: bool = False
    segment_t: float = 0.0
    """Projected position along the current leg (0 at ``route[index]``, 1 at the next)."""

    def advance(self, east_m: float, north_m: float) -> RoutePoint:
        """Update progress from the current position and return the carrot."""
        best_i, best_t, best_d = self.index, 0.0, math.inf
        for i in range(self.index, len(self.route) - 1):
            t, distance = _project_onto_segment(east_m, north_m, self.route[i], self.route[i + 1])
            if distance < best_d:
                best_i, best_t, best_d = i, t, distance
        self.index, self.segment_t = best_i, best_t
        return self._point_ahead(best_i, best_t)

    def along_m(self, arc_m: list[float]) -> float:
        """Distance along the route of the projected position (``arc_m`` = cumulative)."""
        if self.index >= len(arc_m) - 1:
            return arc_m[-1]
        return arc_m[self.index] + self.segment_t * (arc_m[self.index + 1] - arc_m[self.index])

    def _point_ahead(self, index: int, t: float) -> RoutePoint:
        """Walk ``lookahead_m`` forward along the path from the projected position."""
        remaining = self.lookahead_m
        i, frac = index, t
        while i < len(self.route) - 1:
            a, b = self.route[i], self.route[i + 1]
            leg = math.hypot(b.east_m - a.east_m, b.north_m - a.north_m)
            left = leg * (1.0 - frac)
            if left >= remaining:
                f = frac + (remaining / leg if leg > 0.0 else 0.0)
                self.carrot_is_last = False
                return RoutePoint(
                    east_m=a.east_m + (b.east_m - a.east_m) * f,
                    north_m=a.north_m + (b.north_m - a.north_m) * f,
                    up_m=a.up_m + (b.up_m - a.up_m) * f,
                )
            remaining -= left
            i += 1
            frac = 0.0
        self.carrot_is_last = True
        return self.route[-1]

    @property
    def finished(self) -> bool:
        """True once the carrot has run out of route — the goal is the target."""
        return self.carrot_is_last


def compute_command(
    guidance: CarrotGuidance,
    obstacles: ObstacleField,
    *,
    east_m: float,
    north_m: float,
    up_m: float,
    yaw_deg: float,
    config: OffboardConfig,
) -> VelocityCommand:
    """One control step: sense, steer, and command a velocity. Pure — no I/O."""
    carrot = guidance.advance(east_m, north_m)
    heading = ned_yaw_to_enu(yaw_deg)

    goal_bearing = math.atan2(carrot.north_m - north_m, carrot.east_m - east_m)
    goal_rel = math.atan2(math.sin(goal_bearing - heading), math.cos(goal_bearing - heading))

    scan = sense(
        obstacles,
        x=east_m,
        y=north_m,
        heading=heading,
        fov_rad=math.radians(config.fov_deg),
        n_rays=config.n_rays,
        max_range=config.sensor_range_m,
    )
    command = avoid(scan, goal_rel, config.avoidance_config)

    steer = heading + command.desired_heading_rel
    speed = min(command.speed_ms, config.cruise_speed_ms)

    # Altitude is ours to command, not PX4's to interpolate — that is the whole
    # point of flying offboard (TEST-PLAN A2).
    climb = max(
        -config.max_climb_ms,
        min(config.max_climb_ms, (carrot.up_m - up_m) * config.climb_gain),
    )
    return VelocityCommand(
        north_ms=speed * math.sin(steer),
        east_ms=speed * math.cos(steer),
        down_ms=-climb,
        yaw_deg=enu_bearing_to_ned_yaw(steer),
        blocked=command.blocked,
        clearance_ahead_m=command.clearance_ahead_m,
    )


@runtime_checkable
class OffboardConnector(Protocol):
    """The autopilot operations the corridor executor needs."""

    async def connect(self, url: str, ready_timeout_s: float) -> None: ...
    async def arm(self) -> None: ...
    async def takeoff(self, altitude_m: float) -> None: ...
    async def start_offboard(self) -> None: ...
    async def set_velocity_ned(self, command: VelocityCommand) -> None: ...
    def telemetry(self) -> AsyncIterator[tuple[float, float, float, float]]:
        """Yields ``(east_m, north_m, up_m, yaw_deg)``, home-relative."""
        ...

    async def stop_offboard(self) -> None: ...
    async def return_to_launch(self) -> None: ...
    async def disconnect(self) -> None: ...


@dataclass(frozen=True)
class OffboardResult:
    """What the run did — and, critically, how close it came to the trees."""

    reached_goal: bool
    steps: int
    route_index: int
    route_points: int
    min_clearance_m: float
    """Closest any trunk came, over the whole flight. The safety margin, measured.

    ⚠️ This is *forward perception*, not contact: it is what the sensor reported
    ahead, so it says nothing about a trunk struck off-axis. Use `strikes`."""

    blocked_steps: int
    """Control steps where avoidance found no free heading and hovered."""

    strikes: int = 0
    """Control steps where the airframe was inside an obstacle's surface.

    🔴 B2 (2026-07-28) reported ``REACHED GOAL`` on two flights that hit 19 and 37
    trees, because nothing in this result could express "arrived, wrecked". The
    executor's own log printed ``closest obstacle 0.0 m`` beside the success line.
    Reaching the goal is not the success criterion — reaching it *intact* is."""

    aborted_reason: str | None = None
    """Set when the stall watchdog fired instead of the route finishing."""

    @property
    def succeeded(self) -> bool:
        """Arrived **and** never hit anything. The criterion B2 was missing."""
        return self.reached_goal and self.strikes == 0


async def fly_corridor(
    connector: OffboardConnector,
    mission: FlightMission,
    obstacles: ObstacleField,
    config: OffboardConfig,
    *,
    max_steps: int = 100_000,
) -> OffboardResult:
    """Fly the route as a corridor, steering around what the sensor sees."""
    route = route_to_local(mission)
    if len(route) < 2:
        raise ValueError("route needs at least two points")
    guidance = CarrotGuidance(route=route, lookahead_m=config.lookahead_m)

    steps = 0
    blocked_steps = 0
    strikes = 0
    min_clearance = math.inf
    reached = False
    aborted: str | None = None

    started = time.monotonic()
    last_log = started
    last_progress_at = started
    best_index = -1

    _log.info(
        "corridor: %d route points, lookahead %.0f m, %d trunks, cruise %.1f m/s",
        len(route),
        config.lookahead_m,
        len(obstacles.trunks),
        config.cruise_speed_ms,
    )
    _log.info(
        "corridor: react %.1f m (%d rays / %.2f deg), needs %.2f m lateral clearance",
        config.avoidance_config.react_distance_m,
        config.n_rays,
        config.fov_deg / max(1, config.n_rays - 1),
        config.avoidance_config.lateral_clearance_m,
    )
    shortfall = config.sensing_shortfall()
    if shortfall is not None:
        _log.error("corridor: CANNOT AVOID — %s", shortfall)
    await connector.start_offboard()
    async for east_m, north_m, up_m, yaw_deg in connector.telemetry():
        steps += 1
        command = compute_command(
            guidance,
            obstacles,
            east_m=east_m,
            north_m=north_m,
            up_m=up_m,
            yaw_deg=yaw_deg,
            config=config,
        )
        min_clearance = min(min_clearance, command.clearance_ahead_m)
        blocked_steps += int(command.blocked)
        # Contact is measured against the obstacle field directly, not against the
        # forward scan: a trunk clipped off-axis never enters the FOV, and B2's
        # strikes were invisible to `clearance_ahead_m` for exactly that reason.
        if obstacles.clearance(east_m, north_m) < config.avoidance_config.drone_radius_m:
            strikes += 1
            if strikes == 1:
                _log.error(
                    "corridor: STRIKE at %.0f,%.0f m (route %d/%d) — the airframe is "
                    "inside an obstacle; this run is not a success however it ends",
                    east_m,
                    north_m,
                    guidance.index,
                    len(route) - 1,
                )
        await connector.set_velocity_ned(command)

        now = time.monotonic()
        if guidance.index > best_index:
            best_index, last_progress_at = guidance.index, now
        if now - last_log >= config.progress_log_s:
            last_log = now
            _log.info(
                "route %d/%d (%.0f%%)  pos %.0f,%.0f m  alt %.1f m  clearance %.1f m  "
                "blocked %d/%d",
                guidance.index,
                len(route) - 1,
                100.0 * guidance.index / max(1, len(route) - 1),
                east_m,
                north_m,
                up_m,
                command.clearance_ahead_m,
                blocked_steps,
                steps,
            )
        if now - last_progress_at > config.stall_timeout_s:
            aborted = (
                f"no route progress for {config.stall_timeout_s:.0f}s at point "
                f"{guidance.index}/{len(route) - 1} "
                f"({blocked_steps}/{steps} steps with no free heading)"
            )
            _log.error("corridor: %s", aborted)
            break

        last = route[-1]
        if guidance.finished and (
            math.hypot(last.east_m - east_m, last.north_m - north_m) <= config.arrival_radius_m
        ):
            reached = True
            break
        if steps >= max_steps:
            break

    await connector.stop_offboard()
    return OffboardResult(
        reached_goal=reached,
        aborted_reason=aborted,
        steps=steps,
        route_index=guidance.index,
        route_points=len(route),
        min_clearance_m=min_clearance,
        blocked_steps=blocked_steps,
        strikes=strikes,
    )
