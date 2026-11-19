"""GNSS as an untrusted input: carry-off spoofing vs a VIO-consistency monitor."""

from __future__ import annotations

import math

import pytest

from droneflight.gnss_spoof import (
    MonitorConfig,
    SpoofAttack,
    simulate_spoof,
    sweep_pull_rates,
)


def test_honest_gnss_never_alarms_even_at_the_vio_bound() -> None:
    for seed in range(30):
        for rate in (0.02, 0.03):
            run = simulate_spoof(5000.0, None, seed=seed, vio_rate=rate)
            assert not run.false_alarm
            assert not run.detected
            assert run.error_at_end_m < 8.0  # GNSS noise, not VIO drift
            assert run.vio_only_error_at_end_m > 50.0


def test_fast_carry_off_is_caught_quickly_and_bounded_by_vio() -> None:
    run = simulate_spoof(5000.0, SpoofAttack(onset_m=1000.0, pull_rate=0.5), seed=3)
    assert run.detected
    assert run.detect_after_m < 60.0
    # After the alarm the estimate is VIO from the window start: about VIO-only.
    assert run.error_at_end_m < 1.2 * run.vio_only_error_at_end_m


def test_slow_carry_off_is_undetectable_but_no_worse_than_its_own_rate() -> None:
    attack = SpoofAttack(onset_m=1000.0, pull_rate=0.01)
    run = simulate_spoof(5000.0, attack, seed=1)
    assert not run.detected
    assert run.error_at_end_m == pytest.approx(0.01 * 4000.0, abs=8.0)


def test_the_gap_between_vio_rate_and_threshold_is_where_it_hurts() -> None:
    """A pull just above true VIO drift but under the bound is the worst case."""
    res = {r.pull_rate: r for r in sweep_pull_rates((0.03, 0.04, 0.2), seeds=10)}
    assert res[0.2].detected_fraction == 1.0
    assert res[0.03].detected_fraction < 1.0
    worst_gap = max(res[0.03].worst_max_error_m, res[0.04].worst_max_error_m)
    assert worst_gap > res[0.2].worst_max_error_m
    assert all(r.false_alarms == 0 for r in res.values())


def test_a_tighter_vio_bound_catches_slower_spoofers() -> None:
    attack = SpoofAttack(onset_m=500.0, pull_rate=0.03)
    loose = [simulate_spoof(4000.0, attack, seed=s).detected for s in range(10)]
    tight = [
        simulate_spoof(
            4000.0, attack, MonitorConfig(vio_rate=0.015), seed=s, vio_rate=0.01
        ).detected
        for s in range(10)
    ]
    assert sum(tight) > sum(loose)


def test_an_alarm_before_onset_is_a_false_alarm() -> None:
    hair = MonitorConfig(vio_rate=0.0, k_noise=0.0)
    run = simulate_spoof(2000.0, SpoofAttack(onset_m=1500.0), hair, seed=0)
    assert run.false_alarm
    assert not run.detected
    assert math.isinf(run.detect_after_m)
