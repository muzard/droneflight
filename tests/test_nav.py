"""Tests for the GPS-denied navigation-error emulation."""

from __future__ import annotations

from droneflight.model import FlightMission, Waypoint
from droneflight.nav import DriftConfig, VtrnConfig, simulate_navigation


def _straight_mission(length_deg: float = 0.05) -> FlightMission:
    # A straight north run; 0.05 deg latitude ~ 5.5 km.
    return FlightMission(
        name="t",
        home_lat=46.90,
        home_lon=15.0,
        cruise_speed_ms=6.0,
        waypoints=(
            Waypoint(lat=46.90, lon=15.0, alt_agl_m=30.0),
            Waypoint(lat=46.90 + length_deg, lon=15.0, alt_agl_m=30.0),
        ),
    )


def test_is_deterministic() -> None:
    m = _straight_mission()
    a = simulate_navigation(m, DriftConfig(seed=1), None)
    b = simulate_navigation(m, DriftConfig(seed=1), None)
    assert a == b


def test_vio_only_drift_grows_with_rate() -> None:
    m = _straight_mission()
    low = simulate_navigation(m, DriftConfig(rate=0.01, seed=2), None)
    high = simulate_navigation(m, DriftConfig(rate=0.04, seed=2), None)
    assert high.endpoint_error_m > low.endpoint_error_m
    assert low.fixes == 0


def test_vio_only_drift_is_substantial_over_km() -> None:
    m = _straight_mission()  # ~5.5 km
    res = simulate_navigation(m, DriftConfig(rate=0.02, seed=3), None)
    # 2% of ~5.5 km is on the order of tens to >100 m (Tech-Selection §2.1).
    assert res.max_error_m > 20.0


def test_vtrn_bounds_the_error() -> None:
    m = _straight_mission()
    vio = simulate_navigation(m, DriftConfig(rate=0.02, seed=4), None)
    corrected = simulate_navigation(
        m, DriftConfig(rate=0.02, seed=4), VtrnConfig(fix_interval_m=500.0, residual_error_m=2.0)
    )
    assert corrected.fixes > 0
    assert corrected.max_error_m < vio.max_error_m
    # Bounded ~ residual + drift over one fix interval (2 m + 2% of 500 m = 12 m),
    # with some wander headroom.
    assert corrected.max_error_m < 30.0


def test_within_corridor_flag() -> None:
    m = _straight_mission()
    corrected = simulate_navigation(
        m,
        DriftConfig(rate=0.02, seed=5),
        VtrnConfig(fix_interval_m=300.0, residual_error_m=2.0),
        corridor_tolerance_m=20.0,
    )
    assert corrected.within_corridor
