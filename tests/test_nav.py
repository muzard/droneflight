"""Tests for the GPS-denied navigation-error emulation."""

from __future__ import annotations

import math
from itertools import pairwise

import pytest

from droneflight.model import FlightMission, Waypoint
from droneflight.nav import (
    DriftConfig,
    DriftWalker,
    DropoutConfig,
    DropoutModel,
    ErrorInjector,
    VtrnConfig,
    YawDriftConfig,
    YawWalker,
    simulate_navigation,
)


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


# --------------------------------------------------------------------------- #
# Yaw-error injection
#
# Heading was ground truth in every flight so far (three acceptance arms,
# 94,753 samples), so this whole axis was unexercised. The first requirement is
# not that it be realistic — it is that switching it ON cannot disturb anything
# that was already measured.
# --------------------------------------------------------------------------- #


def _walk(
    injector: ErrorInjector, steps: int = 200, step_m: float = 5.0
) -> list[tuple[float, float]]:
    """Step an injector along a straight line and collect the position error."""
    out = []
    for i in range(steps):
        injector.advance(step_m, 0.0, i * step_m)
        out.append(injector.error)
    return out


def test_yaw_layer_absent_and_zeroed_are_the_same_thing() -> None:
    # A default YawDriftConfig must be indistinguishable from passing nothing,
    # or every existing caller silently changes behaviour on upgrade.
    absent = _walk(ErrorInjector(DriftConfig(rate=0.02, seed=7)))
    zeroed = _walk(ErrorInjector(DriftConfig(rate=0.02, seed=7), yaw=YawDriftConfig()))
    assert absent == zeroed


def test_enabling_yaw_does_not_perturb_position_drift() -> None:
    """The separate-RNG guarantee, asserted bit-for-bit.

    If the yaw walk drew from the drift RNG it would shift every subsequent
    position draw, and the offline/online parity the acceptance rests on would
    quietly break.
    """
    without = _walk(ErrorInjector(DriftConfig(rate=0.02, seed=8)))
    with_yaw = _walk(
        ErrorInjector(
            DriftConfig(rate=0.02, seed=8),
            yaw=YawDriftConfig(bias_deg=5.0, wander_deg_per_100m=3.0, seed=99),
        )
    )
    assert without == with_yaw


def test_yaw_bias_applies_before_any_distance_is_flown() -> None:
    inj = ErrorInjector(DriftConfig(seed=1), yaw=YawDriftConfig(bias_deg=10.0))
    assert inj.yaw_error_rad == pytest.approx(math.radians(10.0))


def test_yaw_error_is_zero_when_the_layer_is_off() -> None:
    assert ErrorInjector(DriftConfig(seed=1)).yaw_error_rad == 0.0


def test_yaw_drift_grows_with_distance_and_is_deterministic() -> None:
    def run() -> float:
        inj = ErrorInjector(
            DriftConfig(seed=2), yaw=YawDriftConfig(wander_deg_per_100m=5.0, seed=4)
        )
        near = None
        for i in range(400):
            inj.advance(5.0, 0.0, i * 5.0)
            if i == 19:  # 100 m in
                near = abs(inj.yaw_error_rad)
        assert near is not None
        return abs(inj.yaw_error_rad) - near

    growth = run()
    assert growth > 0.0  # 2 km of heading walk exceeds the first 100 m
    assert run() == growth  # ...and repeats exactly


def test_pure_yaw_bias_never_walks() -> None:
    inj = ErrorInjector(DriftConfig(seed=3), yaw=YawDriftConfig(bias_deg=-4.0))
    for i in range(100):
        inj.advance(5.0, 0.0, i * 5.0)
    assert inj.yaw_error_rad == pytest.approx(math.radians(-4.0))


# --------------------------------------------------------------------------- #
# EV dropout
# --------------------------------------------------------------------------- #


def test_dropout_disabled_never_fires() -> None:
    d = DropoutModel(DropoutConfig())
    assert not any(d.step(10.0, i * 0.1) for i in range(1000))
    assert d.events == 0


def test_dropout_fires_and_lasts_the_configured_time() -> None:
    d = DropoutModel(DropoutConfig(mean_interval_m=100.0, duration_s=2.0, seed=1))
    down_at = [i * 0.1 for i in range(2000) if d.step(1.0, i * 0.1)]
    assert d.events > 0

    # Split the down samples into contiguous runs; each must span duration_s
    # rather than blinking off after a single sample.
    runs: list[list[float]] = [[down_at[0]]]
    for prev, t in pairwise(down_at):
        if t - prev < 0.15:
            runs[-1].append(t)
        else:
            runs.append([t])
    for run in runs[:-1]:  # the last may be truncated by the end of the walk
        assert run[-1] - run[0] == pytest.approx(2.0, abs=0.15)


def test_dropout_onset_is_distance_driven_not_sample_driven() -> None:
    """Same distance flown, different sample rate -> same number of lock losses.

    Onset must not depend on how often the caller happens to step the model,
    or a rate change would silently rewrite the failure schedule.
    """
    coarse = DropoutModel(DropoutConfig(mean_interval_m=200.0, duration_s=1.0, seed=5))
    for i in range(500):  # 500 x 4 m = 2000 m
        coarse.step(4.0, i * 1.0)
    fine = DropoutModel(DropoutConfig(mean_interval_m=200.0, duration_s=1.0, seed=5))
    for i in range(2000):  # 2000 x 1 m = 2000 m, same route
        fine.step(1.0, i * 0.25)
    assert coarse.events == fine.events


def test_dropouts_never_overlap() -> None:
    # A single estimator cannot lose lock while already having lost it: the next
    # onset is only scheduled once the current dropout clears.
    d = DropoutModel(DropoutConfig(mean_interval_m=10.0, duration_s=5.0, seed=2))
    for i in range(500):
        d.step(20.0, i * 0.5)  # onsets due constantly, dropouts are long
    elapsed = 500 * 0.5
    assert d.events <= elapsed / 5.0 + 1


def test_dropout_is_deterministic() -> None:
    def run() -> list[bool]:
        d = DropoutModel(DropoutConfig(mean_interval_m=150.0, duration_s=1.5, seed=11))
        return [d.step(2.0, i * 0.05) for i in range(1000)]

    assert run() == run()


def test_dropout_seeds_differ() -> None:
    def run(seed: int) -> list[bool]:
        d = DropoutModel(DropoutConfig(mean_interval_m=150.0, duration_s=1.5, seed=seed))
        return [d.step(2.0, i * 0.05) for i in range(1000)]

    assert run(1) != run(2)


def test_yaw_walk_is_step_size_invariant() -> None:
    """The property DriftWalker's heading wander does NOT have.

    A random walk accumulates variance with distance, so stepping the same route
    more finely must not change how far the heading gets. Scaling the per-step
    deviation linearly (rather than by sqrt) fails this: at the bridge's ~0.24 m
    steps the walk flattens into a straight line, which is what the flown
    control arm actually shows.
    """
    import statistics

    def spread(step_m: float) -> float:
        finals = []
        for seed in range(200):
            w = YawWalker(YawDriftConfig(wander_deg_per_100m=10.0, seed=seed))
            for _ in range(int(2000 / step_m)):
                w.advance(step_m)
            finals.append(w.error_rad)
        return statistics.pstdev(finals)

    fine, coarse = spread(0.25), spread(25.0)
    # Same route, 100x different granularity -> same accumulated spread.
    assert fine == pytest.approx(coarse, rel=0.2)
    # ...and it is the theoretical value: 10 deg/100 m over 2000 m = 10*sqrt(20).
    assert math.degrees(fine) == pytest.approx(10.0 * math.sqrt(20.0), rel=0.2)


def test_drift_walker_heading_wander_is_step_size_dependent() -> None:
    """Pins the known defect in the *existing* model so it cannot be forgotten.

    Deliberately not fixed: the three acceptance arms and the 40-seed Monte
    Carlo were all measured with this behaviour, and silently correcting it
    would invalidate them. See YawWalker.advance for the correct scaling.
    """
    import statistics

    def spread(step_m: float) -> float:
        finals = []
        for seed in range(200):
            w = DriftWalker(DriftConfig(rate=0.02, wander_deg_per_100m=20.0, seed=seed))
            for _ in range(int(2000 / step_m)):
                w.advance(step_m)
            finals.append(math.hypot(*w.error))
        return statistics.mean(finals)

    fine, coarse = spread(0.25), spread(100.0)
    # At fine steps the heading never wanders, so error is just rate * distance.
    assert fine == pytest.approx(0.02 * 2000, rel=0.05)
    assert coarse < fine * 0.9  # coarse stepping *does* wander, and cancels
