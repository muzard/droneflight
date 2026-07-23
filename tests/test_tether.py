"""Tests for the v0 trailing-fiber tether model.

Pure and deterministic — no Gazebo. Covers the tension terms, the payout-tension
profile, and the two findings the model surfaced: it separates the speed policy
from flat cruise on a reversal, and it is robust to the duplicate-waypoint
clusters that otherwise smear a turn to ~0°.
"""

from __future__ import annotations

import pytest

from droneflight.model import FlightMission, Waypoint
from droneflight.speed_policy import SpeedPolicy
from droneflight.tether import (
    TetherConfig,
    analyse_tether,
    drag_tension_n,
    tension_n,
    turn_tension_n,
)


def _mission(coords: list[tuple[float, float]], cruise: float = 6.0) -> FlightMission:
    return FlightMission(
        name="t",
        home_lat=coords[0][0],
        home_lon=coords[0][1],
        cruise_speed_ms=cruise,
        waypoints=tuple(Waypoint(lat=la, lon=lo, alt_agl_m=30.0) for la, lo in coords),
    )


# --------------------------------------------------------------------------- #
# Tension terms
# --------------------------------------------------------------------------- #


def test_config_rejects_nonsense() -> None:
    with pytest.raises(ValueError, match="break_tension_n"):
        TetherConfig(break_tension_n=0.0)
    with pytest.raises(ValueError, match="coupling_length_m"):
        TetherConfig(coupling_length_m=0.0)
    with pytest.raises(ValueError, match="warn_fraction"):
        TetherConfig(warn_fraction=1.5)


def test_drag_scales_with_speed_squared_and_span() -> None:
    c = TetherConfig()
    assert drag_tension_n(c, 0.0) == 0.0
    d1 = drag_tension_n(c, 3.0)
    d2 = drag_tension_n(c, 6.0)
    assert d2 == pytest.approx(4.0 * d1)  # doubling speed quadruples drag
    wide = TetherConfig(airborne_span_m=80.0)
    assert drag_tension_n(wide, 6.0) == pytest.approx(2.0 * d2)  # linear in span


def test_turn_tension_zero_straight_max_at_reversal() -> None:
    c = TetherConfig()
    assert turn_tension_n(c, 6.0, 0.0) == 0.0  # straight leg: no turn load
    quarter = turn_tension_n(c, 6.0, 90.0)
    reversal = turn_tension_n(c, 6.0, 180.0)
    assert reversal > quarter > 0.0
    # (1 - cos): 90° -> 1, 180° -> 2, so a reversal is exactly twice a right angle.
    assert reversal == pytest.approx(2.0 * quarter)


def test_turn_tension_scales_with_speed_squared() -> None:
    c = TetherConfig()
    slow = turn_tension_n(c, 2.0, 180.0)
    fast = turn_tension_n(c, 4.0, 180.0)
    assert fast == pytest.approx(4.0 * slow)  # halving speed quarters the turn load


def test_total_tension_is_the_sum() -> None:
    c = TetherConfig()
    total = tension_n(c, speed_ms=5.0, deflection_deg=120.0)
    assert total == pytest.approx(
        c.back_tension_n + drag_tension_n(c, 5.0) + turn_tension_n(c, 5.0, 120.0)
    )


# --------------------------------------------------------------------------- #
# The speed-policy validation — the whole point
# --------------------------------------------------------------------------- #


def test_speed_policy_prevents_a_reversal_break() -> None:
    """A reversal at cruise snaps the fiber; the policy's slowdown saves it.

    This is the model validating the speed policy against payout physics — the
    reason the module exists.
    """
    m = _mission([(46.90, 15.0), (46.9018, 15.0), (46.9002, 15.0)])  # out then reverse
    cfg = TetherConfig()
    no_policy = analyse_tether(m, cfg, policy=None)
    with_policy = analyse_tether(m, cfg, policy=SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0))

    assert no_policy.breaks  # flat cruise through the reversal snaps
    assert no_policy.peak is not None and no_policy.peak.deflection_deg > 150.0
    assert with_policy.within_limit  # the policy slows enough to survive
    assert with_policy.max_tension_n < no_policy.max_tension_n


def test_straight_route_never_breaks_and_policy_is_inactive() -> None:
    m = _mission([(46.90, 15.0), (46.91, 15.0), (46.92, 15.0)])
    rep = analyse_tether(m, TetherConfig(), policy=SpeedPolicy(cruise_speed_ms=6.0))
    assert rep.within_limit
    # Straight line: every vertex holds cruise, tension is just back + drag.
    assert all(load.speed_ms == pytest.approx(6.0) for load in rep.loads)


def test_report_flags_breaks_and_warnings() -> None:
    m = _mission([(46.90, 15.0), (46.9018, 15.0), (46.9002, 15.0)])
    # A tiny break budget so the reversal is over it and a mid leg only warns.
    rep = analyse_tether(m, TetherConfig(break_tension_n=1.0, warn_fraction=0.5), policy=None)
    assert rep.breaks
    assert rep.peak is not None and rep.peak.exceeds_break
    assert rep.peak.load_fraction > 1.0


# --------------------------------------------------------------------------- #
# Robustness to the duplicate-waypoint cluster defect
# --------------------------------------------------------------------------- #


def test_duplicate_cluster_does_not_hide_a_turn() -> None:
    """A sharp turn smeared into sub-meter waypoints is still seen at full angle.

    The planner emits duplicate clusters (0.2-1.0 m apart); measuring bends on
    the raw waypoints would spread a 90° corner across several ~0° segments and
    the tether load — and the speed policy — would miss it entirely.
    """
    # A 90° corner where the corner vertex is split into a 0.5 m micro-cluster.
    m = _mission(
        [
            (46.9000, 15.0000),
            (46.9010, 15.0000),  # heading north
            (46.90100, 15.00001),  # ~0.8 m micro-step — the cluster
            (46.9010, 15.0010),  # now heading east: a 90° turn hidden in the cluster
        ]
    )
    rep = analyse_tether(m, TetherConfig(), policy=None)
    assert rep.peak is not None
    # After consolidation the corner shows as a real turn, not ~0°.
    assert max(load.deflection_deg for load in rep.loads) > 60.0
