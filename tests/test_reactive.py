"""Tests for the closed-loop forest-flight simulation."""

from __future__ import annotations

import itertools

from droneflight.avoidance import AvoidanceConfig
from droneflight.maneuver import ManeuverLimits
from droneflight.model import FlightMission, Waypoint
from droneflight.nav import DriftConfig
from droneflight.perception import ObstacleField, Trunk
from droneflight.reactive import (
    ForestFlightConfig,
    _to_local,
    fly_through_forest,
    scatter_forest,
)


def _straight_mission(n: int = 4, spacing_deg: float = 0.0009) -> FlightMission:
    # ~100 m spacing north (0.0009 deg lat ~ 100 m).
    wps = tuple(Waypoint(lat=46.90 + spacing_deg * i, lon=15.0, alt_agl_m=6.0) for i in range(n))
    return FlightMission(
        name="line", home_lat=46.90, home_lon=15.0, cruise_speed_ms=5.0, waypoints=wps
    )


def test_reaches_goal_in_empty_forest() -> None:
    m = _straight_mission()
    res = fly_through_forest(m, ObstacleField(), ForestFlightConfig())
    assert res.reached_goal
    assert not res.collided


def test_avoidance_still_beats_blind_flight_but_no_longer_dodges_cleanly() -> None:
    """🔴 This test asserted a CLEAN dodge until B2 (2026-07-28) put accel and
    jerk limits in the harness. It now grazes.

    A trunk sitting exactly on a waypoint is the worst case *and* a knife-edge:
    the vehicle must approach head-on and the dodge direction is decided by which
    ray happens to land on the trunk, so the outcome flips non-monotonically with
    ray count (41 grazes, 61 clean, 81 grazes, 121 clean). It is kept as a
    *relative* check — avoidance beats blind flight — and deliberately not as an
    absolute one, because at this geometry the absolute answer is discretisation
    noise, not a property of the policy.
    """
    m = _straight_mission()
    wp1 = _to_local(m.waypoints[1].lat, m.waypoints[1].lon, m.home_lat, m.home_lon)
    field = ObstacleField(trunks=(Trunk(x=wp1[0], y=wp1[1], radius=0.3),))

    blind = fly_through_forest(m, field, ForestFlightConfig(), reactive=False)
    reactive = fly_through_forest(m, field, ForestFlightConfig(), reactive=True)

    assert blind.collided, "flying the waypoints blindly hits the trunk"
    assert reactive.reached_goal
    # Both now *arrive* — blind flight grinds past the trunk rather than stopping,
    # which is the B2 phenomenon in miniature. Only the clearance separates them.
    assert reactive.min_clearance_m > blind.min_clearance_m


def test_trunks_are_placed_in_the_same_frame_the_manifest_uses() -> None:
    """🔴 The harness projected to *absolute* metres (~5.2e6) while every trunk
    manifest is home-relative (~0..3500), so a real forest sat a million metres
    away and every flight came back clean. Same class as the degenerate VTRN
    basemap. Home must be the origin.
    """
    m = _straight_mission()
    east, north = _to_local(m.home_lat, m.home_lon, m.home_lat, m.home_lon)
    assert (east, north) == (0.0, 0.0)
    far = _to_local(m.waypoints[-1].lat, m.waypoints[-1].lon, m.home_lat, m.home_lon)
    assert abs(far[0]) < 1.0 and 0.0 < far[1] < 10_000.0


def test_scatter_forest_keeps_corridor_clear() -> None:
    m = _straight_mission(n=6)
    field = scatter_forest(m, trees_per_100m=30, route_clearance_m=3.5, intrusion_prob=0.0, seed=1)
    assert len(field.trunks) > 0
    # With no intrusions, every trunk is at least the clearance from the route.
    pts = [_to_local(w.lat, w.lon, m.home_lat, m.home_lon) for w in m.waypoints]
    for t in field.trunks:
        d = min(_seg_dist(t.x, t.y, a[0], a[1], b[0], b[1]) for a, b in itertools.pairwise(pts))
        assert d >= 3.5 - 1e-6


def _instant() -> ForestFlightConfig:
    """The old harness: velocity changes with no accel or jerk limit at all."""
    return ForestFlightConfig(limits=ManeuverLimits(max_accel_ms2=1e6, max_jerk_ms3=1e9))


def test_instant_response_hides_a_collision_that_real_dynamics_produces() -> None:
    """Half one of the B2 root cause, pinned.

    Same trunk, same avoidance, only the acceleration envelope differs: a vehicle
    that can turn its velocity instantly slips past (+0.064 m), one that must
    accelerate laterally grazes (-0.001 m). The old harness's clean dodges were an
    artifact of that freedom.
    """
    m = _straight_mission()
    wp1 = _to_local(m.waypoints[1].lat, m.waypoints[1].lon, m.home_lat, m.home_lon)
    field = ObstacleField(trunks=(Trunk(x=wp1[0], y=wp1[1], radius=0.3),))

    instant = fly_through_forest(m, field, _instant(), reactive=True)
    real = fly_through_forest(m, field, ForestFlightConfig(), reactive=True)

    assert instant.strikes == 0 and instant.min_clearance_m > 0.0
    assert real.strikes > 0, "with real dynamics the same dodge does not clear"


def test_instant_response_predicts_paralysis_where_the_vehicle_makes_progress() -> None:
    """Half two, and the one that mattered: the failure mode is wrong, not just
    the magnitude.

    Against a line of trunks across the route, an instantly-responsive drone stops
    dead and is reported **stuck** — the "79 % steps blocked" that B2 falsified. A
    drone with momentum carries through and reaches the goal.

    ⚠️ Both arms pin ``gap_policy="nearest_goal"``. This documents a *dynamics*
    finding, so the steering policy has to be held at the one that was in force
    when it was made — the widest-gap policy that replaced it in N2 escapes this
    wall outright, which is the point of the test below.
    """
    m = _straight_mission()
    wp1 = _to_local(m.waypoints[1].lat, m.waypoints[1].lon, m.home_lat, m.home_lon)
    wall = ObstacleField(
        trunks=tuple(Trunk(x=wp1[0] + dx, y=wp1[1] - 20.0, radius=0.3) for dx in range(-12, 13, 2))
    )
    historical = AvoidanceConfig(gap_policy="nearest_goal")

    instant = fly_through_forest(
        m,
        wall,
        ForestFlightConfig(
            avoidance=historical, limits=ManeuverLimits(max_accel_ms2=1e6, max_jerk_ms3=1e9)
        ),
        reactive=True,
    )
    real = fly_through_forest(m, wall, ForestFlightConfig(avoidance=historical), reactive=True)

    assert instant.stuck and not instant.reached_goal, "the old harness's prediction"
    assert real.reached_goal and not real.stuck, "the aircraft's actual behaviour"


def test_widest_gap_escapes_a_wall_that_traps_the_nearest_goal_policy() -> None:
    """TEST-PLAN N2: why the steering policy changed.

    ``nearest_goal`` takes the minimum deviation that clears, so against a line of
    trunks it noses into the first slot it finds and jams. ``widest`` picks the
    largest opening and keeps a transient margin off both edges, so it goes round.
    """
    m = _straight_mission()
    wp1 = _to_local(m.waypoints[1].lat, m.waypoints[1].lon, m.home_lat, m.home_lon)
    wall = ObstacleField(
        trunks=tuple(Trunk(x=wp1[0] + dx, y=wp1[1] - 20.0, radius=0.3) for dx in range(-12, 13, 2))
    )
    limits = ManeuverLimits(max_accel_ms2=1e6, max_jerk_ms3=1e9)

    old = fly_through_forest(
        m,
        wall,
        ForestFlightConfig(avoidance=AvoidanceConfig(gap_policy="nearest_goal"), limits=limits),
        reactive=True,
    )
    new = fly_through_forest(
        m,
        wall,
        ForestFlightConfig(avoidance=AvoidanceConfig(gap_policy="widest"), limits=limits),
        reactive=True,
    )

    assert old.stuck and not old.reached_goal
    assert new.reached_goal and not new.stuck
    assert new.strikes <= old.strikes


def test_a_sustained_grind_along_a_trunk_counts_as_one_strike() -> None:
    """Without release hysteresis a blind flight scraping a trunk registered
    39 892 strikes, because each step's contact resolution pushed it just clear.
    """
    m = _straight_mission()
    wp1 = _to_local(m.waypoints[1].lat, m.waypoints[1].lon, m.home_lat, m.home_lon)
    field = ObstacleField(trunks=(Trunk(x=wp1[0], y=wp1[1], radius=0.3),))
    res = fly_through_forest(m, field, ForestFlightConfig(), reactive=False)
    assert res.collided
    assert res.strikes < 10, f"one contact should not read as {res.strikes} strikes"


def test_drift_offsets_the_endpoint() -> None:
    m = _straight_mission(n=8)
    field = ObstacleField()
    clean = fly_through_forest(m, field, ForestFlightConfig())
    drifted = fly_through_forest(
        m, field, ForestFlightConfig(), drift=DriftConfig(rate=0.05, seed=2)
    )
    assert drifted.endpoint_offset_m > clean.endpoint_offset_m


def _seg_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    import math

    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    if ll == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / ll))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
