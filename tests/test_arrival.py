"""Unit tests for vertical-acceptance exposure (TEST-PLAN B1).

The defect these describe is a silent one: PX4 will not sequence past a waypoint
it misses vertically, and the vehicle keeps flying anyway. It froze missions at
11/79 and 51/78. What predicts it, across 12 flights, is not any single
waypoint's geometry but *how many* waypoints the vehicle reaches while still
descending — each one is a draw from a heavy-tailed error distribution.
"""

from __future__ import annotations

import math

import pytest

from droneflight.arrival import assess_arrivals
from droneflight.model import FlightMission, Waypoint

_M_PER_DEG_LAT = math.radians(1.0) * 6_371_000.0


def _mission(alts: list[float], leg_m: float = 100.0) -> FlightMission:
    """A straight north-running mission with the given relative altitudes."""
    step = leg_m / _M_PER_DEG_LAT
    return FlightMission(
        name="t",
        home_lat=0.0,
        home_lon=0.0,
        home_alt_m=100.0,
        cruise_speed_ms=6.0,
        waypoints=tuple(Waypoint(lat=i * step, lon=0.0, alt_agl_m=a) for i, a in enumerate(alts)),
    )


def test_level_flight_has_no_exposure() -> None:
    report = assess_arrivals(_mission([30.0, 30.0, 30.0, 30.0]))
    assert report.exposure_count == 0
    assert report.stall_free_probability(1 / 6) == 1.0


def test_climbing_arrivals_are_not_exposed() -> None:
    """Only descents put the vehicle below its commanded altitude on arrival."""
    report = assess_arrivals(_mission([10.0, 20.0, 30.0]))
    assert report.exposure_count == 0


def test_each_descending_waypoint_is_one_dice_roll() -> None:
    report = assess_arrivals(_mission([40.0, 30.0, 20.0, 20.0, 10.0]))
    assert report.exposure_count == 3
    assert [a.index for a in report.descending] == [1, 2, 4]


def test_descending_arrivals_are_ranked_steepest_first() -> None:
    report = assess_arrivals(_mission([40.0, 39.0, 30.0]))  # 1 m then 9 m over 100 m
    steepest = report.descending[0]
    assert steepest.index == 2
    assert steepest.required_gradient == pytest.approx(9.0 / 100.0, rel=1e-3)


def test_sub_decimetre_steps_are_bookkeeping_not_descents() -> None:
    report = assess_arrivals(_mission([30.0, 29.95, 29.9]))
    assert report.exposure_count == 0


def test_exposure_scales_the_bet_with_route_density() -> None:
    """The point of the whole module: density is the risk, not any one waypoint.

    NAV_MC_ALT_RAD = 4.0 was established on a route with two such waypoints. The
    same per-waypoint risk over a denser route is a different bet entirely.
    """
    sparse = assess_arrivals(_mission([40.0, 35.0, 30.0]))
    dense = assess_arrivals(_mission([40.0 - i for i in range(60)]))
    assert sparse.exposure_count == 2
    assert dense.exposure_count == 59
    assert dense.stall_free_probability(1 / 6) < sparse.stall_free_probability(1 / 6)
    assert dense.stall_free_probability(1 / 6) < 0.01


def test_probability_argument_is_validated() -> None:
    report = assess_arrivals(_mission([30.0, 20.0]))
    assert report.stall_free_probability(0.0) == 1.0
    with pytest.raises(ValueError, match="per_waypoint_risk"):
        report.stall_free_probability(1.5)


def test_zero_length_leg_has_no_gradient() -> None:
    mission = FlightMission(
        name="t",
        home_lat=0.0,
        home_lon=0.0,
        home_alt_m=0.0,
        cruise_speed_ms=6.0,
        waypoints=(
            Waypoint(lat=0.0, lon=0.0, alt_agl_m=30.0),
            Waypoint(lat=0.0, lon=0.0, alt_agl_m=20.0),
        ),
    )
    arrival = assess_arrivals(mission).arrivals[0]
    assert arrival.leg_length_m == pytest.approx(0.0)
    assert arrival.required_gradient == 0.0
    assert arrival.arrives_descending  # a 10 m drop in place is still a descent
