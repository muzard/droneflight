"""Tests for the tether-safe speed policy."""

from __future__ import annotations

from droneflight.model import FlightMission, Waypoint
from droneflight.speed_policy import SpeedPolicy, apply_speed_policy


def _mission(coords: list[tuple[float, float]]) -> FlightMission:
    return FlightMission(
        name="t",
        home_lat=coords[0][0],
        home_lon=coords[0][1],
        cruise_speed_ms=6.0,
        waypoints=tuple(Waypoint(lat=la, lon=lo, alt_agl_m=30.0) for la, lo in coords),
    )


def test_straight_line_stays_at_cruise() -> None:
    m = _mission([(46.90, 15.00), (46.91, 15.00), (46.92, 15.00)])
    items = apply_speed_policy(m, SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0))
    assert all(abs(it.speed_ms - 6.0) < 1e-6 for it in items)


def test_sharp_turn_slows_down() -> None:
    # 90-degree corner at the middle waypoint.
    m = _mission([(46.90, 15.00), (46.91, 15.00), (46.91, 15.01)])
    items = apply_speed_policy(m, SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0))
    assert items[1].speed_ms < 6.0
    assert items[0].speed_ms == 6.0  # endpoints keep cruise
    assert items[-1].speed_ms == 6.0


def test_reversal_holds_min_speed() -> None:
    # Near-180-degree reversal.
    m = _mission([(46.90, 15.00), (46.91, 15.00), (46.9001, 15.00)])
    items = apply_speed_policy(
        m, SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0, reversal_deg=150.0)
    )
    assert abs(items[1].speed_ms - 1.0) < 1e-6


def test_policy_rejects_bad_speeds() -> None:
    import pytest

    with pytest.raises(ValueError, match="min_speed_ms"):
        SpeedPolicy(cruise_speed_ms=2.0, min_speed_ms=5.0)


def test_duplicate_cluster_turn_still_slows() -> None:
    """The known planner defect emits turns as clusters of near-duplicate points.

    Measuring the bend on raw consecutive waypoints smears a real corner across
    sub-meter segments, so no waypoint shows a slow-worthy angle and the policy
    never fires (measured 0/77 on a real plan). Consolidation restores it.
    """
    # 90° corner with the vertex split into a ~0.8 m micro-cluster.
    m = _mission(
        [
            (46.9000, 15.0000),
            (46.9010, 15.0000),
            (46.90100, 15.00001),  # the cluster point
            (46.9010, 15.0010),
        ]
    )
    items = apply_speed_policy(m, SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0))
    # Some waypoint in/around the corner is slowed below cruise.
    assert any(it.speed_ms < 6.0 - 1e-6 for it in items)


def test_consolidation_does_not_disturb_distinct_waypoints() -> None:
    # Well-separated waypoints (the existing tests' regime) must be unaffected.
    m = _mission([(46.90, 15.00), (46.91, 15.00), (46.91, 15.01)])
    a = apply_speed_policy(m, SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0))
    assert a[1].speed_ms < 6.0  # the 90° corner still slows
    assert a[0].speed_ms == 6.0 and a[-1].speed_ms == 6.0
