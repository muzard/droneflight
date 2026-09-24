"""Closed-loop forest-flight simulation (accel- and jerk-limited).

Flies a planned mission through an obstacle field using the reactive avoidance
layer and checks it reaches the goal without hitting a trunk. No physics engine,
so it runs on any machine and is deterministic. Optional GPS-denied drift shows
the interaction: reactive avoidance senses *real* trees in the body frame, so it
keeps dodging them even as global drift offsets the route (which is what VTRN
corrects).

🔴 WHY THIS IS NO LONGER PURELY KINEMATIC (TEST-PLAN B2, 2026-07-28)
This harness used to move the vehicle at exactly the commanded velocity, changing
it instantly between steps. That one simplification made it predict **the wrong
failure mode**, not merely the wrong magnitude:

============================  =========================  ========================
harness said (100 stems/ha)   flight did                 error
============================  =========================  ========================
fails, **79 % steps blocked**  reached goal, 0.17 %       ~500x
0 collisions modelled          **37 tree strikes**        total
============================  =========================  ========================

A drone with instant velocity response *stops dead* when avoidance finds no free
bearing, so the harness forecast paralysis. A real one carries momentum, never
enters the blocked state, and **makes progress by colliding**. The plan's caveat
— "real dynamics can only be worse" — was useless precisely because the failure
was different in kind rather than worse in degree.

So velocity is now a state, slewed toward the command under PX4's own
``MPC_ACC_HOR``/``MPC_JERK_AUTO`` (see `maneuver`), and a contact no longer aborts
the run: it is counted, the vehicle is stopped against the trunk, and the flight
continues — because that is what the aircraft did, and a harness that reports one
collision cannot be compared against a flight that had 37.
"""

from __future__ import annotations

import collections
import itertools
import math
import random
from dataclasses import dataclass, field

from droneflight.avoidance import AvoidanceCommand, AvoidanceConfig, avoid
from droneflight.local_planner import LocalPlanner, LocalPlannerConfig, PlannerCommand
from droneflight.maneuver import ManeuverLimits
from droneflight.model import FlightMission
from droneflight.nav import DriftConfig, VtrnConfig
from droneflight.perception import ObstacleField, sense
from droneflight.px4_profile import OFFBOARD_DELAY_S, OFFBOARD_TAU_S

_EARTH_R = 6_371_000.0


@dataclass(frozen=True)
class ForestFlightConfig:
    """Sensor + kinematic parameters for the closed-loop sim."""

    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)
    fov_deg: float = 120.0  # sensor FOV (wide stereo / lidar slice; passive mono is narrower)
    n_rays: int = 41
    sensor_range_m: float = 15.0
    dt_s: float = 0.05
    """Integration step. Was 0.2 s, which is coarser than the 0.75 s jerk ramp it
    now has to resolve — at 6 m/s it also teleports the vehicle 1.2 m per step,
    wide enough to step straight over a 0.3 m trunk without ever registering
    contact."""

    max_turn_rate_dps: float = 60.0
    capture_radius_m: float = 2.0
    max_steps: int = 80_000
    max_stall_steps: int = 800  # consecutive hovering steps before "stuck"
    contact_release_s: float = 0.5
    """How long the vehicle must be clear before a new contact counts as a new
    strike. Without it a vehicle grinding along a trunk registers a fresh strike
    every step — 39 892 of them in one blind-flight test — because the contact
    resolution pushes it just outside the surface each time. Matches the episode
    grouping `strikes.score_track` applies to a flown ULog, so harness and flight
    counts mean the same thing."""
    limits: ManeuverLimits = field(default_factory=ManeuverLimits)
    """The acceleration envelope. Setting `max_accel_ms2`/`max_jerk_ms3` very high
    recovers the old instant-response behaviour — see
    `test_instant_response_hides_the_collisions`, which is what this harness got
    wrong before B2."""


@dataclass(frozen=True)
class ForestFlightResult:
    reached_goal: bool
    strikes: int
    """Distinct contacts. **Not** a bool: B2's flights reached the goal having hit
    19 and 37 trees, and a harness that stops at the first one cannot be compared
    against that."""

    stuck: bool
    min_clearance_m: float  # closest body-to-trunk gap along the flight
    path_length_m: float
    steps: int
    waypoints_reached: int
    endpoint_offset_m: float  # true endpoint vs planned endpoint (drift effect)
    path: tuple[tuple[float, float], ...]

    @property
    def collided(self) -> bool:
        return self.strikes > 0

    @property
    def clean(self) -> bool:
        """Arrived *and* intact — the criterion B2 showed `reached_goal` is not."""
        return self.reached_goal and self.strikes == 0


def _to_local(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """WGS84 -> **home-relative** ENU metres (east, north).

    🔴 This used to return *absolute* projected metres (~1.1e6, 5.2e6) while every
    other consumer of a position in this program — the trunk manifest, the Gazebo
    world, `offboard.route_to_local` — is home-relative (~0..3500). Feeding a real
    manifest to `fly_through_forest` therefore placed the forest a million metres
    away and the flight sailed through an empty world reporting zero strikes and a
    clearance of exactly 0.000 m. It is the same frame mismatch that silently
    degenerated the VTRN basemap (`ev_bridge.route_reference`), so it is fixed
    here the same way: one frame, the one everything else uses.
    """
    east = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * _EARTH_R
    north = math.radians(lat - lat0) * _EARTH_R
    return east, north


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

    lat0, lon0 = mission.home_lat, mission.home_lon
    pts = [_to_local(w.lat, w.lon, lat0, lon0) for w in mission.waypoints]
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
    lat0, lon0 = mission.home_lat, mission.home_lon
    wps = [_to_local(w.lat, w.lon, lat0, lon0) for w in mission.waypoints]

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
    vel = (0.0, 0.0)
    acc = (0.0, 0.0)
    strikes = 0
    release_steps = max(1, int(config.contact_release_s / config.dt_s))
    steps_clear = release_steps  # start clear, so a contact on step 1 still counts

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

        # Turn-rate-limited heading update. The *sensor* points here immediately;
        # the airframe's velocity does not, which is the whole point.
        heading = _wrap(heading + max(-max_turn, min(max_turn, cmd.desired_heading_rel)))

        # Slew the velocity vector toward the command under accel + jerk limits,
        # then integrate. A multirotor cannot step its velocity: it must tilt,
        # and tilting is rate-limited.
        dt = config.dt_s
        want = (cmd.speed_ms * math.cos(heading), cmd.speed_ms * math.sin(heading))
        a_des = _clamp_norm(
            ((want[0] - vel[0]) / dt, (want[1] - vel[1]) / dt), config.limits.max_accel_ms2
        )
        d_acc = _clamp_norm((a_des[0] - acc[0], a_des[1] - acc[1]), config.limits.max_jerk_ms3 * dt)
        acc = (acc[0] + d_acc[0], acc[1] + d_acc[1])
        vel = (vel[0] + acc[0] * dt, vel[1] + acc[1] * dt)
        prev = pos
        pos = (pos[0] + vel[0] * dt, pos[1] + vel[1] * dt)
        moved = math.hypot(pos[0] - prev[0], pos[1] - prev[1])
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
            # Contact. Counted once per entry, then resolved as a *sliding* stop:
            # the velocity component into the surface is removed and the vehicle
            # is pushed back to the surface, leaving any tangential motion intact.
            # It then keeps flying, because that is what the aircraft did — and a
            # harness that aborts on first contact cannot be compared against a
            # flight that had 37. No bounce energy is invented.
            #
            # Removing the *whole* velocity instead pins the vehicle: it is reset
            # into contact every step, never builds speed, and hovers against the
            # trunk forever, which reads as a stall rather than a strike.
            if steps_clear >= release_steps:
                strikes += 1
            steps_clear = 0
            nx, ny = _surface_normal(obstacles, pos)
            pos = (pos[0] + nx * (-gap + _CONTACT_EPS_M), pos[1] + ny * (-gap + _CONTACT_EPS_M))
            path[-1] = pos
            into = vel[0] * nx + vel[1] * ny
            if into < 0.0:  # moving into the surface
                vel = (vel[0] - into * nx, vel[1] - into * ny)
            into_a = acc[0] * nx + acc[1] * ny
            if into_a < 0.0:
                acc = (acc[0] - into_a * nx, acc[1] - into_a * ny)
        else:
            steps_clear += 1

        stall = stall + 1 if cmd.blocked else 0
        if stall >= config.max_stall_steps:
            return _result(False, strikes, True, min_gap, path, step, target_idx, est, wps)

        if math.hypot(est[0] - target[0], est[1] - target[1]) <= config.capture_radius_m:
            target_idx += 1
            if target_idx >= len(wps):
                return _result(True, strikes, False, min_gap, path, step, len(wps), est, wps)

    return _result(False, strikes, True, min_gap, path, config.max_steps, target_idx, est, wps)


_CONTACT_EPS_M = 1e-3
"""How far outside the surface a contact is resolved to. Without it the vehicle
is left exactly on the boundary and re-triggers contact on the next step."""


def _surface_normal(obstacles: ObstacleField, pos: tuple[float, float]) -> tuple[float, float]:
    """Outward normal at a contact, from the gradient of the clearance field.

    Taken numerically so no nearest-trunk lookup is needed and the result stays
    correct where several trunks overlap. Clearance grows away from an obstacle,
    so its gradient points out of one.
    """
    eps = 0.01
    gx = obstacles.clearance(pos[0] + eps, pos[1]) - obstacles.clearance(pos[0] - eps, pos[1])
    gy = obstacles.clearance(pos[0], pos[1] + eps) - obstacles.clearance(pos[0], pos[1] - eps)
    mag = math.hypot(gx, gy)
    if mag == 0.0:  # dead centre of a trunk — any direction is outward
        return 1.0, 0.0
    return gx / mag, gy / mag


def _clamp_norm(v: tuple[float, float], limit: float) -> tuple[float, float]:
    """Scale a 2D vector down to ``limit`` without changing its direction."""
    mag = math.hypot(v[0], v[1])
    if mag <= limit or mag == 0.0:
        return v
    scale = limit / mag
    return (v[0] * scale, v[1] * scale)


def _result(
    reached: bool,
    strikes: int,
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
        strikes=strikes,
        stuck=stuck,
        min_clearance_m=0.0 if min_gap is math.inf else min_gap,
        path_length_m=length,
        steps=steps,
        waypoints_reached=wp_reached,
        endpoint_offset_m=endpoint_offset,
        path=tuple(path),
    )


@dataclass(frozen=True)
class StrikeEvent:
    """One contact, with what the planner knew about that trunk at the time."""

    t_s: float
    east_m: float
    north_m: float
    speed_ms: float
    trunk_in_memory: bool
    """Had the sensor ever returned a point on the struck trunk? False means it
    was hit *unseen* (outside the FOV or occluded) — a sensing failure; True
    means it was known and the path or the tracking failed to clear it."""

    commanded_speed_ms: float
    gap_ahead_m: float


@dataclass(frozen=True)
class LocalFlightResult:
    """What a local-planner run did, scored against the TRUE trunks and route."""

    reached_goal: bool
    strikes: int
    min_clearance_m: float
    """Closest true airframe-to-trunk-surface gap over the flight."""

    flight_time_s: float
    path_length_m: float
    max_cross_track_m: float
    """Worst true distance from the planned route — corridor keeping."""

    replans: int
    plan_failures: int
    blocked_s: float
    memory_resets: int
    max_nav_error_m: float
    path: tuple[tuple[float, float], ...]
    strike_log: tuple[StrikeEvent, ...] = ()

    @property
    def clean(self) -> bool:
        return self.reached_goal and self.strikes == 0

    @property
    def mean_speed_ms(self) -> float:
        return self.path_length_m / self.flight_time_s if self.flight_time_s > 0 else 0.0


def _distance_to_route(p: tuple[float, float], route: list[tuple[float, float]]) -> float:
    best = math.inf
    for a, b in itertools.pairwise(route):
        abx, aby = b[0] - a[0], b[1] - a[1]
        denom = abx * abx + aby * aby
        t = 0.0 if denom == 0.0 else ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / denom
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(p[0] - a[0] - abx * t, p[1] - a[1] - aby * t))
    return best


def fly_with_local_planner(
    mission: FlightMission,
    obstacles: ObstacleField,
    planner_config: LocalPlannerConfig | None = None,
    config: ForestFlightConfig | None = None,
    *,
    drift: DriftConfig | None = None,
    vtrn: VtrnConfig | None = None,
    start_m: float = 0.0,
    stop_m: float | None = None,
    max_time_s: float = 7200.0,
    fix_ramp_m: float = 0.0,
    use_corridor: bool = True,
    plant_seed: int | None = None,
) -> LocalFlightResult:
    """Fly ``mission`` through ``obstacles`` with the onboard local planner.

    Same dynamics and contact model as :func:`fly_through_forest` (velocity slewed
    under PX4's accel/jerk limits; a contact is counted and slid along, not
    aborted), so strike counts compare directly with the reactive numbers on
    record. Differences, all deliberate:

    * the sensor is swept from the **true** pose but its returns are placed in the
      map from the **believed** pose — exactly what a real aircraft does, and the
      property that should make this immune to slow navigation drift;
    * progress is by projection onto the route (`offboard.CarrotGuidance`), the
      same guidance the live offboard executor uses, not waypoint capture;
    * ``start_m``/``stop_m`` fly only a slice of the route, because only the
      concealed opening is in the forest;
    * ``fix_ramp_m`` bleeds each VTRN fix in over that many metres of travel, as
      the live EV bridge does (``ev-bridge --fix-ramp``, default 10 m). 0 = an
      instantaneous jump — which is what hid the "clear observed space on every
      correction" defect offline until SITL found it.
    """
    from droneflight.offboard import CarrotGuidance, RoutePoint

    config = config or ForestFlightConfig()
    planner = LocalPlanner(config=planner_config or LocalPlannerConfig())
    pcfg = planner.config
    lat0, lon0 = mission.home_lat, mission.home_lon
    full = [_to_local(w.lat, w.lon, lat0, lon0) for w in mission.waypoints]
    route = _slice_polyline(full, start_m, stop_m)
    guidance = CarrotGuidance(
        route=[RoutePoint(east_m=x, north_m=y, up_m=0.0) for x, y in route],
        lookahead_m=pcfg.horizon_m,
    )
    # The plan's per-leg speed (DO_CHANGE_SPEED), mapped onto the sliced route by
    # arc length so start_m/stop_m slices keep the right speeds.
    leg_speeds = _sliced_leg_speeds(full, route, mission)
    corridor = _sliced_corridor(full, route, mission) if use_corridor else None

    pos = route[0]
    heading = _heading_of(route[0], route[1])
    fov = math.radians(config.fov_deg)
    max_turn = math.radians(config.max_turn_rate_dps) * config.dt_s
    rd = config.avoidance.drone_radius_m
    dt = config.dt_s

    rng = random.Random(drift.seed) if drift else None
    drift_heading = rng.uniform(0.0, 2.0 * math.pi) if rng else 0.0
    err_x = err_y = 0.0
    dist_since_fix = 0.0
    max_err = 0.0
    pend_x = pend_y = 0.0
    bleed_per_m = 0.0

    path = [pos]
    vel = (0.0, 0.0)
    acc = (0.0, 0.0)
    # The airframe's response. ``plant_seed`` draws it from inside the fit's
    # uncertainty (px4_profile.OFFBOARD_*) so the planner's own prediction — which
    # assumes the nominal values — is never exactly right, as in the loop. With a
    # perfect match the harness could not show a prediction-error strike at all.
    delay_s, tau_s = OFFBOARD_DELAY_S, OFFBOARD_TAU_S
    if plant_seed is not None:
        prng = random.Random(plant_seed)
        delay_s, tau_s = prng.uniform(0.2, 0.4), prng.uniform(0.3, 0.55)
    # Setpoints sent but not yet acted on; the aircraft starts at rest.
    in_flight: collections.deque[tuple[float, float]] = collections.deque(
        [(0.0, 0.0)] * max(0, round(delay_s / dt))
    )
    strikes = 0
    min_gap = math.inf
    max_xt = 0.0
    blocked_s = 0.0
    release_steps = max(1, int(config.contact_release_s / dt))
    steps_clear = release_steps
    reached = False
    t = 0.0
    goal = route[-1]
    events: list[StrikeEvent] = []

    while t < max_time_s:
        t += dt
        est = (pos[0] + err_x, pos[1] + err_y)
        scan = sense(obstacles, pos[0], pos[1], heading, fov, config.n_rays, config.sensor_range_m)
        added = planner.observe(scan, est[0], est[1], heading, velocity=vel, t_s=t)
        carrot = guidance.advance(*est)
        cmd = planner.command(
            est[0],
            est[1],
            (carrot.east_m, carrot.north_m),
            t,
            added,
            velocity=vel,
            max_speed_ms=leg_speeds[min(guidance.index, len(leg_speeds) - 1)],
            corridor=corridor,
        )
        if cmd.blocked:
            blocked_s += dt
            want = (0.0, 0.0)
            # Hovering: turn toward the goal to look for a way through.
            desired = math.atan2(carrot.north_m - est[1], carrot.east_m - est[0])
        else:
            want = (cmd.vx, cmd.vy)
            desired = math.atan2(cmd.vy, cmd.vx)
        # The sensor looks where the aircraft is going (yaw follows the command).
        heading = _wrap(heading + max(-max_turn, min(max_turn, _wrap(desired - heading))))

        # The measured OFFBOARD response (px4_profile): the setpoint takes effect
        # after a dead time, then through a first-order lag. Not the AUTO-mode
        # jerk-limited generator `fly_through_forest` uses — offboard velocity
        # setpoints bypass it, and that model mispredicted SITL 1 s ahead by 0.4 m.
        in_flight.append(want)
        applied = in_flight.popleft()
        acc = _clamp_norm(
            ((applied[0] - vel[0]) / tau_s, (applied[1] - vel[1]) / tau_s),
            config.limits.max_accel_ms2,
        )
        vel = (vel[0] + acc[0] * dt, vel[1] + acc[1] * dt)
        prev = pos
        pos = (pos[0] + vel[0] * dt, pos[1] + vel[1] * dt)
        moved = math.hypot(pos[0] - prev[0], pos[1] - prev[1])
        path.append(pos)

        if drift and rng and moved > 0.0:
            drift_heading += rng.gauss(0.0, math.radians(drift.wander_deg_per_100m) * moved / 100.0)
            err_x += drift.rate * moved * math.cos(drift_heading)
            err_y += drift.rate * moved * math.sin(drift_heading)
            dist_since_fix += moved
            if vtrn and dist_since_fix >= vtrn.fix_interval_m:
                mag = math.hypot(err_x, err_y)
                if mag > vtrn.residual_error_m:
                    scale = vtrn.residual_error_m / mag
                    if fix_ramp_m > 0.0:
                        pend_x, pend_y = err_x * (scale - 1.0), err_y * (scale - 1.0)
                        bleed_per_m = math.hypot(pend_x, pend_y) / fix_ramp_m
                    else:
                        err_x *= scale
                        err_y *= scale
                dist_since_fix = 0.0
            pend = math.hypot(pend_x, pend_y)
            if pend > 0.0:
                step = min(pend, bleed_per_m * moved)
                err_x += pend_x / pend * step
                err_y += pend_y / pend * step
                pend_x -= pend_x / pend * step
                pend_y -= pend_y / pend * step
            max_err = max(max_err, math.hypot(err_x, err_y))

        gap = obstacles.clearance(pos[0], pos[1]) - rd
        min_gap = min(min_gap, gap)
        if gap < 0.0:
            if steps_clear >= release_steps:
                strikes += 1
                events.append(
                    _strike_event(
                        t, pos, vel, obstacles, planner, (err_x, err_y), cmd.speed_ms, cmd
                    )
                )
            steps_clear = 0
            nx, ny = _surface_normal(obstacles, pos)
            pos = (pos[0] + nx * (-gap + _CONTACT_EPS_M), pos[1] + ny * (-gap + _CONTACT_EPS_M))
            path[-1] = pos
            into = vel[0] * nx + vel[1] * ny
            if into < 0.0:
                vel = (vel[0] - into * nx, vel[1] - into * ny)
            into_a = acc[0] * nx + acc[1] * ny
            if into_a < 0.0:
                acc = (acc[0] - into_a * nx, acc[1] - into_a * ny)
        else:
            steps_clear += 1

        if int(t / dt) % 20 == 0:
            max_xt = max(max_xt, _distance_to_route(pos, route))
        if guidance.finished and math.dist(est, goal) <= config.capture_radius_m:
            reached = True
            break

    length = sum(math.dist(a, b) for a, b in itertools.pairwise(path))
    return LocalFlightResult(
        reached_goal=reached,
        strikes=strikes,
        min_clearance_m=min_gap,
        flight_time_s=t,
        path_length_m=length,
        max_cross_track_m=max_xt,
        replans=planner.replans,
        plan_failures=planner.failures,
        blocked_s=blocked_s,
        memory_resets=planner.memory_resets,
        max_nav_error_m=max_err,
        path=tuple(path),
        strike_log=tuple(events),
    )


def _strike_event(
    t: float,
    pos: tuple[float, float],
    vel: tuple[float, float],
    obstacles: ObstacleField,
    planner: LocalPlanner,
    err: tuple[float, float],
    commanded: float,
    cmd: PlannerCommand,
) -> StrikeEvent:
    near = min(
        obstacles._nearby(pos[0], pos[1], 2.0) or obstacles.trunks,
        key=lambda tr: math.hypot(tr.x - pos[0], tr.y - pos[1]),
    )
    # Memory is in the believed frame: shift the true trunk by the nav error.
    cx, cy = near.x + err[0], near.y + err[1]
    seen = any(
        math.hypot(px - cx, py - cy) < near.radius + 0.35 for px, py in planner.memory.points()
    )
    return StrikeEvent(
        t_s=t,
        east_m=pos[0],
        north_m=pos[1],
        speed_ms=math.hypot(*vel),
        trunk_in_memory=seen,
        commanded_speed_ms=commanded,
        gap_ahead_m=cmd.min_gap_ahead_m,
    )


def _sliced_leg_speeds(
    full: list[tuple[float, float]], route: list[tuple[float, float]], mission: FlightMission
) -> list[float]:
    """Planned speed for each leg of ``route`` (a slice of ``full``).

    A slice keeps the original vertices between its cut points, so each sliced
    vertex (the cut points included) takes the leg speed of the nearest original
    vertex — exact for interior vertices, and the containing leg's at the cuts
    whenever the cut is nearer its start than its end.
    """
    out = []
    for p in route[:-1]:
        k = min(range(len(full)), key=lambda i: math.dist(full[i], p))
        out.append(mission.leg_speed_ms(k))
    return out or [mission.cruise_speed_ms]


def _sliced_corridor(
    full: list[tuple[float, float]], route: list[tuple[float, float]], mission: FlightMission
) -> list[tuple[tuple[float, float], float]] | None:
    """The plan's corridor widths on the sliced route (nearest original vertex)."""
    widths = [w.corridor_half_width_m for w in mission.waypoints]
    if any(w is None for w in widths):
        return None
    out = []
    for p in route:
        k = min(range(len(full)), key=lambda i: math.dist(full[i], p))
        out.append((p, float(widths[k] or 0.0)))
    return out


def _heading_of(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _slice_polyline(
    pts: list[tuple[float, float]], start_m: float, stop_m: float | None
) -> list[tuple[float, float]]:
    """The part of a polyline between two arc lengths (interpolated at the cuts)."""
    cum = [0.0]
    for a, b in itertools.pairwise(pts):
        cum.append(cum[-1] + math.dist(a, b))
    stop = cum[-1] if stop_m is None else min(stop_m, cum[-1])
    start = max(0.0, min(start_m, stop))

    def at(s: float) -> tuple[float, float]:
        x, y, _, _ = _point_tangent(pts, cum, s)
        return x, y

    inner = [p for p, c in zip(pts, cum, strict=True) if start < c < stop]
    out = [at(start), *inner, at(stop)]
    return out if len(out) >= 2 else [pts[0], pts[-1]]
