"""The maneuver budget that TEST-PLAN B2 measured the hard way."""

from __future__ import annotations

import math

import pytest

from droneflight.avoidance import AvoidanceConfig
from droneflight.maneuver import (
    ManeuverLimits,
    lateral_offset_m,
    max_speed_for_react_distance_ms,
    required_rays,
    required_react_distance_m,
    time_to_offset_s,
)


def test_lateral_offset_respects_the_jerk_ramp_not_half_a_t_squared():
    """Before the accel cap is reached the profile is jerk-limited, so a naive
    0.5*a*t^2 overestimates badly — which is how 6 m looked sufficient."""
    limits = ManeuverLimits()
    t = 0.5  # inside the 0.75 s ramp
    naive = 0.5 * limits.max_accel_ms2 * t**2
    assert lateral_offset_m(t, limits) == pytest.approx(limits.max_jerk_ms3 * t**3 / 6.0)
    assert lateral_offset_m(t, limits) < naive / 2.0


def test_the_flown_b2_configuration_could_not_have_missed():
    """The measured headline: 6 m at 6 m/s buys 0.66 m against 1.10 m needed."""
    cfg = AvoidanceConfig(cruise_speed_ms=6.0, react_distance_override_m=6.0)
    assert cfg.lateral_clearance_m == pytest.approx(1.10)
    achievable = lateral_offset_m(6.0 / 6.0, cfg.limits)
    assert achievable == pytest.approx(0.66, abs=0.01)
    assert achievable < cfg.lateral_clearance_m
    assert not cfg.is_survivable


def test_bare_miss_needs_7_2_m_at_cruise():
    """Zero-margin reaction distance, the number the 6.0 m constant was under."""
    limits = ManeuverLimits(safety_factor=1.0, latency_s=0.0)
    t = time_to_offset_s(1.10, limits)
    assert t == pytest.approx(1.20, abs=0.01)
    assert required_react_distance_m(6.0, 1.10, limits) == pytest.approx(7.2, abs=0.1)


def test_derived_react_distance_is_survivable_and_scales_with_speed():
    slow = AvoidanceConfig(cruise_speed_ms=3.0)
    fast = AvoidanceConfig(cruise_speed_ms=6.0)
    assert slow.is_survivable and fast.is_survivable
    # Linear in speed: twice as fast must look twice as far ahead.
    assert fast.react_distance_m == pytest.approx(2.0 * slow.react_distance_m)
    assert 11.0 < fast.react_distance_m < 13.0


def test_slowing_down_is_the_other_lever():
    """Hold 6 m and the geometry caps speed — the fallback when a real sensor
    cannot deliver the range.

    Two figures, and the difference matters: **4.99 m/s** is pure geometry, and
    **4.60 m/s** is what survives once the 0.1 s control latency is charged. The
    executor pays the latency, so 4.60 is the one to fly.
    """
    bare = ManeuverLimits(safety_factor=1.0, latency_s=0.0)
    assert max_speed_for_react_distance_ms(6.0, 1.10, bare) == pytest.approx(4.99, abs=0.05)

    with_latency = ManeuverLimits(safety_factor=1.0)
    v = max_speed_for_react_distance_ms(6.0, 1.10, with_latency)
    assert v == pytest.approx(4.60, abs=0.05)
    assert AvoidanceConfig(cruise_speed_ms=v * 0.98, react_distance_override_m=6.0).is_survivable
    for too_fast in (v * 1.02, 6.0):
        assert not AvoidanceConfig(
            cruise_speed_ms=too_fast, react_distance_override_m=6.0
        ).is_survivable


def test_round_trip_offset_and_time():
    limits = ManeuverLimits()
    for d in (0.1, 0.66, 1.10, 5.0):
        assert lateral_offset_m(time_to_offset_s(d, limits), limits) == pytest.approx(d, abs=1e-6)


def test_required_rays_explains_why_6_m_looked_self_consistent():
    """31 rays over 90 deg resolves a trunk to ~6 m and no further."""
    fov = math.radians(90.0)
    assert required_rays(fov, 6.0, 0.15) <= 33
    assert required_rays(fov, 15.0, 0.15) == 80
    # The flown scan was 31; the derived reaction distance needs far more.
    assert required_rays(fov, AvoidanceConfig(cruise_speed_ms=6.0).react_distance_m, 0.15) > 31


def test_zero_and_negative_inputs_are_rejected_or_neutral():
    assert lateral_offset_m(0.0) == 0.0
    assert lateral_offset_m(-1.0) == 0.0
    assert time_to_offset_s(0.0) == 0.0
    assert required_react_distance_m(0.0, 1.1) == 0.0
    assert max_speed_for_react_distance_ms(0.0, 1.1) == 0.0
    with pytest.raises(ValueError):
        ManeuverLimits(max_accel_ms2=0.0)
    with pytest.raises(ValueError):
        ManeuverLimits(safety_factor=0.0)
    with pytest.raises(ValueError):
        required_rays(math.radians(90.0), 0.0, 0.15)


def test_override_reproduces_the_historical_constant():
    """Old kinematic sweeps must stay reproducible — but must not be the default."""
    cfg = AvoidanceConfig(cruise_speed_ms=6.0, react_distance_override_m=6.0)
    assert cfg.react_distance_m == 6.0
    assert AvoidanceConfig(cruise_speed_ms=6.0).react_distance_m != 6.0
