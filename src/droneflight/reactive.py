"""Closed-loop forest-flight simulation (kinematic).

Flies a planned mission through an obstacle field using the reactive avoidance
layer, at the airframe's turn-rate limit, and checks it reaches the goal without
hitting a trunk. This is the "can this software fly the route through a forest?"
test — a kinematic loop (no physics engine), so it runs on any machine and is
deterministic. Optional GPS-denied drift shows the interaction: reactive
avoidance senses *real* trees in the body frame, so it keeps dodging them even as
global drift offsets the route (which is what VTRN corrects).
"""

from __future__ import annotations

import itertools
import math
import random
from dataclasses import dataclass, field

from droneflight.avoidance import AvoidanceCommand, AvoidanceConfig, avoid
from droneflight.model import FlightMission
from droneflight.nav import DriftConfig, VtrnConfig
from droneflight.perception import ObstacleField, sense

_EARTH_R = 6_371_000.0


@dataclass(frozen=True)
class ForestFlightConfig:
    """Sensor + kinematic parameters for the closed-loop sim."""

    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)
    fov_deg: float = 120.0  # sensor FOV (wide stereo / lidar slice; passive mono is narrower)
    n_rays: int = 41
    sensor_range_m: float = 15.0
    dt_s: float = 0.2
    max_turn_rate_dps: float = 60.0
    capture_radius_m: float = 2.0
    max_steps: int = 20_000
    max_stall_steps: int = 200  # consecutive hovering steps before "stuck"


@dataclass(frozen=True)
class ForestFlightResult:
    reached_goal: bool
    collided: bool
    stuck: bool
    min_clearance_m: float  # closest body-to-trunk gap along the flight
    path_length_m: float
    steps: int
    waypoints_reached: int
    endpoint_offset_m: float  # true endpoint vs planned endpoint (drift effect)
    path: tuple[tuple[float, float], ...]


def _to_local(lat: float, lon: float, lat0: float) -> tuple[float, float]:
    x = math.radians(lon) * math.cos(math.radians(lat0)) * _EARTH_R
    y = math.radians(lat) * _EARTH_R
    return x, y


def _point_tangent(
    pts: list[tuple[float, float]], cum: list[float], s: float
) -> tuple[float, float, float, float]:
    """Point (x, y) and unit tangent (tx, ty) at arc-length ``s`` along a polyline."""
    for i in range(1, len(cum)):
        if s <= cum[i] or i == len(cum) - 1:
            a, b = pts[i - 1], pts[i]
            seg = cum[i] - cum[i - 1]
            t = 0.0 if seg == 0.0 else (s - cum[i - 1]) / seg
            tx, ty = b[0] - a[0], b[1] - a[1]
            length = math.hypot(tx, ty) or 1.0
            return a[0] + t * tx, a[1] + t * ty, tx / length, ty / length
    return pts[-1][0], pts[-1][1], 1.0, 0.0


def scatter_forest(
    mission: FlightMission,
    trees_per_100m: float = 40.0,
    band_m: float = 20.0,
    route_clearance_m: float = 3.5,
    intrusion_prob: float = 0.06,
    seed: int = 0,
) -> ObstacleField:
    """Scatter tree trunks in a corridor along the mission's route.

    Trees flank the planned polyline out to ``band_m``, keeping a navigable tube
    of half-width ``route_clearance_m`` clear (the planner routed through it). A
    small fraction (``intrusion_prob``) intrude *into* the tube — obstacles the
    map missed — which is what the reactive layer must dodge. VTRN-bounded drift
    stays within the tube; unbounded drift wanders out into the dense flank.
    """
    from droneflight.perception import Trunk

    lat0 = mission.home_lat
    pts = [_to_local(w.lat, w.lon, lat0) for w in mission.waypoints]
    cum = [0.0]
    for a, b in itertools.pairwise(pts):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    length = cum[-1]
    n = int(trees_per_100m * length / 100.0)

    rng = random.Random(seed)
    trunks: list[Trunk] = []
    for _ in range(n):
        x, y, tx, ty = _point_tangent(pts, cum, rng.uniform(0.0, length))
        nx, ny = -ty, tx  # unit normal to the route
        side = 1.0 if rng.random() < 0.5 else -1.0
        if rng.random() < intrusion_prob:
            d = rng.uniform(0.0, 0.7)  # intrusion in the flight path (map missed it)
        else:
            d = rng.uniform(route_clearance_m, band_m)
        jitter = rng.uniform(-3.0, 3.0)
        px = x + side * d * nx + jitter * tx
        py = y + side * d * ny + jitter * ty
        trunks.append(Trunk(x=px, y=py, radius=rng.uniform(0.1, 0.3)))
    return ObstacleField(trunks=tuple(trunks))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def fly_through_forest(
    mission: FlightMission,
    obstacles: ObstacleField,
    config: ForestFlightConfig | None = None,
    drift: DriftConfig | None = None,
    vtrn: VtrnConfig | None = None,
    reactive: bool = True,
) -> ForestFlightResult:
    """Simulate flying a mission through ``obstacles`` with reactive avoidance.

    ``drift`` injects GPS-denied VIO error into the position estimate; ``vtrn``
    periodically shrinks it toward a bounded residual (map-matching fixes),
    keeping the true path in the planned corridor. ``reactive=False`` flies the
    waypoints blindly (no avoidance) — the baseline that shows what the layer buys.
    """
    config = config or ForestFlightConfig()
    lat0 = mission.home_lat
    wps = [_to_local(w.lat, w.lon, lat0) for w in mission.waypoints]

    pos = wps[0]
    target_idx = 1
    heading = math.atan2(wps[1][1] - pos[1], wps[1][0] - pos[0]) if len(wps) > 1 else 0.0
    fov = math.radians(config.fov_deg)
    max_turn = math.radians(config.max_turn_rate_dps) * config.dt_s
    rd = config.avoidance.drone_radius_m

    rng = random.Random(drift.seed) if drift else None
    drift_heading = rng.uniform(0.0, 2.0 * math.pi) if rng else 0.0
    err_x = err_y = 0.0
    dist_since_fix = 0.0

    path = [pos]
    min_gap = math.inf
    stall = 0

    for step in range(1, config.max_steps + 1):
        # Perceive real obstacles in the body frame (true position).
        scan = sense(obstacles, pos[0], pos[1], heading, fov, config.n_rays, config.sensor_range_m)

        # Navigate toward the target using the (possibly drifted) position estimate.
        est = (pos[0] + err_x, pos[1] + err_y)
        target = wps[target_idx]
        goal_bearing = math.atan2(target[1] - est[1], target[0] - est[0])
        goal_rel = _wrap(goal_bearing - heading)
        if reactive:
            cmd = avoid(scan, goal_rel, config.avoidance)
        else:
            cmd = AvoidanceCommand(
                desired_heading_rel=goal_rel,
                speed_ms=config.avoidance.cruise_speed_ms,
                blocked=False,
                clearance_ahead_m=scan.min_range(),
            )

        # Turn-rate-limited heading update, then move.
        heading = _wrap(heading + max(-max_turn, min(max_turn, cmd.desired_heading_rel)))
        moved = cmd.speed_ms * config.dt_s
        pos = (pos[0] + moved * math.cos(heading), pos[1] + moved * math.sin(heading))
        path.append(pos)

        # Accumulate GPS-denied drift with distance flown; VTRN fixes reset it.
        if drift and rng and moved > 0.0:
            drift_heading += rng.gauss(0.0, math.radians(drift.wander_deg_per_100m) * moved / 100.0)
            err_x += drift.rate * moved * math.cos(drift_heading)
            err_y += drift.rate * moved * math.sin(drift_heading)
            dist_since_fix += moved
            if vtrn and dist_since_fix >= vtrn.fix_interval_m:
                # A map-matching fix shrinks the accumulated error toward zero
                # (bounded by the fix accuracy), preserving direction — not a jump.
                mag = math.hypot(err_x, err_y)
                if mag > vtrn.residual_error_m:
                    scale = vtrn.residual_error_m / mag
                    err_x *= scale
                    err_y *= scale
                dist_since_fix = 0.0

        gap = obstacles.clearance(pos[0], pos[1]) - rd
        min_gap = min(min_gap, gap)
        if gap < 0.0:
            return _result(False, True, False, min_gap, path, step, target_idx, est, wps)

        stall = stall + 1 if cmd.blocked else 0
        if stall >= config.max_stall_steps:
            return _result(False, False, True, min_gap, path, step, target_idx, est, wps)

        if math.hypot(est[0] - target[0], est[1] - target[1]) <= config.capture_radius_m:
            target_idx += 1
            if target_idx >= len(wps):
                return _result(True, False, False, min_gap, path, step, len(wps), est, wps)

    return _result(False, False, True, min_gap, path, config.max_steps, target_idx, est, wps)


def _result(
    reached: bool,
    collided: bool,
    stuck: bool,
    min_gap: float,
    path: list[tuple[float, float]],
    steps: int,
    wp_reached: int,
    est: tuple[float, float],
    wps: list[tuple[float, float]],
) -> ForestFlightResult:
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in itertools.pairwise(path))
    endpoint_offset = math.hypot(path[-1][0] - wps[-1][0], path[-1][1] - wps[-1][1])
    return ForestFlightResult(
        reached_goal=reached,
        collided=collided,
        stuck=stuck,
        min_clearance_m=0.0 if min_gap is math.inf else min_gap,
        path_length_m=length,
        steps=steps,
        waypoints_reached=wp_reached,
        endpoint_offset_m=endpoint_offset,
        path=tuple(path),
    )
