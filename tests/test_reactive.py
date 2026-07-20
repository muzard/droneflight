"""Tests for the closed-loop forest-flight simulation."""

from __future__ import annotations

import itertools

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


def test_avoidance_dodges_in_path_trunk_that_blind_flight_hits() -> None:
    m = _straight_mission()
    # Put a trunk exactly on the route, at the second waypoint's position.
    wp1 = _to_local(m.waypoints[1].lat, m.waypoints[1].lon, m.home_lat)
    field = ObstacleField(trunks=(Trunk(x=wp1[0], y=wp1[1], radius=0.3),))

    blind = fly_through_forest(m, field, ForestFlightConfig(), reactive=False)
    assert blind.collided  # flying the waypoints blindly hits the trunk

    reactive = fly_through_forest(m, field, ForestFlightConfig(), reactive=True)
    assert reactive.reached_goal
    assert not reactive.collided
    assert reactive.min_clearance_m > 0.0


def test_scatter_forest_keeps_corridor_clear() -> None:
    m = _straight_mission(n=6)
    field = scatter_forest(m, trees_per_100m=30, route_clearance_m=3.5, intrusion_prob=0.0, seed=1)
    assert len(field.trunks) > 0
    # With no intrusions, every trunk is at least the clearance from the route.
    pts = [_to_local(w.lat, w.lon, m.home_lat) for w in m.waypoints]
    for t in field.trunks:
        d = min(_seg_dist(t.x, t.y, a[0], a[1], b[0], b[1]) for a, b in itertools.pairwise(pts))
        assert d >= 3.5 - 1e-6


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
