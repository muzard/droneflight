"""Tests for the optical terrain-relative correction (VTRN) module."""

from __future__ import annotations

import math

from droneflight.correction import (
    CorrectionConfig,
    FixOutcome,
    MapSample,
    OpticalCorrector,
    PatchyReference,
    UniformReference,
)
from droneflight.model import FlightMission, Waypoint
from droneflight.nav import (
    DriftConfig,
    route_reference,
    simulate_corrected_navigation,
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


# --- ReferenceMap ---------------------------------------------------------


def test_uniform_reference_is_constant() -> None:
    ref = UniformReference(matchability=0.7, staleness=0.05, accuracy_m=2.5)
    a = ref.sample(0.0, 0.0)
    b = ref.sample(1234.0, -987.0)
    assert a == b == MapSample(0.7, 0.05, 2.5)


def test_patchy_reference_has_forest_and_open_buckets() -> None:
    pts = [(0.0, 0.0), (0.0, 1000.0)]
    ref = PatchyReference.along_route(pts, n_buckets=10, forest_frac=0.4, stale_frac=0.0, seed=1)
    samples = [ref.sample(0.0, s) for s in range(0, 1000, 25)]
    matchabilities = {round(s.matchability, 3) for s in samples}
    # Two distinct regimes: open (high) and forest (low).
    assert ref.forest_matchability in matchabilities
    assert ref.open_matchability in matchabilities
    # ~40% of the route is forest.
    forest = sum(1 for s in samples if s.matchability == ref.forest_matchability)
    assert 0.2 < forest / len(samples) < 0.6


def test_patchy_reference_projects_off_route_points() -> None:
    # A query 50 m to the side maps to the same bucket as the nearest on-route point.
    pts = [(0.0, 0.0), (0.0, 1000.0)]
    ref = PatchyReference.along_route(pts, n_buckets=20, seed=2)
    assert ref.sample(0.0, 500.0) == ref.sample(50.0, 500.0)


def test_patchy_reference_needs_two_points() -> None:
    try:
        PatchyReference.along_route([(0.0, 0.0)])
    except ValueError:
        pass
    else:  # pragma: no cover
        raise AssertionError("expected ValueError for a single point")


# --- OpticalCorrector (unit) ---------------------------------------------


def test_no_features_below_matchability_floor() -> None:
    cor = OpticalCorrector(CorrectionConfig(min_matchability=0.15), seed=0)
    sample = MapSample(matchability=0.05, staleness=0.0, accuracy_m=3.0)
    fix, err = cor.attempt((10.0, 0.0), sample)
    assert fix.outcome is FixOutcome.NO_FEATURES
    assert err == (10.0, 0.0)  # error unchanged — dead-reckoning continues


def test_low_confidence_between_floors() -> None:
    cfg = CorrectionConfig(min_matchability=0.1, confidence_floor=0.4)
    cor = OpticalCorrector(cfg, seed=0)
    # matchability high enough to attempt, below the confidence floor to accept.
    sample = MapSample(matchability=0.2, staleness=0.0, accuracy_m=3.0)
    # Try a few draws; availability is stochastic, but never accepted here.
    outcomes = set()
    for _ in range(50):
        fix, _ = cor.attempt((5.0, 0.0), sample)
        outcomes.add(fix.outcome)
    assert FixOutcome.ACCEPTED not in outcomes
    assert FixOutcome.LOW_CONFIDENCE in outcomes


def test_good_fix_shrinks_error_and_variance() -> None:
    cor = OpticalCorrector(seed=0)
    cor.propagate(300.0)  # realistic accumulated uncertainty
    var_before = cor.variance
    sample = MapSample(matchability=1.0, staleness=0.0, accuracy_m=3.0)
    fix, err = cor.attempt((8.0, 0.0), sample)
    assert fix.outcome is FixOutcome.ACCEPTED
    assert math.hypot(*err) < 8.0  # error pulled toward truth
    assert cor.variance < var_before  # a fix increases confidence


def test_blunder_is_gated_out_when_confident() -> None:
    # Small variance (confident estimate) + an always-stale map => the wrong
    # match disagrees with dead-reckoning and is rejected.
    cor = OpticalCorrector(CorrectionConfig(init_var_m2=1.0), seed=1)
    sample = MapSample(matchability=1.0, staleness=1.0, accuracy_m=3.0)
    fix, err = cor.attempt((1.0, 0.0), sample)
    assert fix.outcome is FixOutcome.GATED_OUT
    assert err == (1.0, 0.0)


def test_blunder_slips_through_when_drift_is_large() -> None:
    # The failure mode: after a long dead-reckoning stretch the variance is huge,
    # the gate opens wide, and a stale-map blunder is accepted as truth.
    cor = OpticalCorrector(seed=1)
    cor.propagate(20_000.0)  # very large accumulated uncertainty
    sample = MapSample(matchability=1.0, staleness=1.0, accuracy_m=3.0)
    fix, err = cor.attempt((30.0, 0.0), sample)
    assert fix.outcome is FixOutcome.ACCEPTED_CORRUPT
    assert fix.corrupt
    # It corrupts the estimate: error jumps far from truth.
    assert math.hypot(*err) > 30.0


# --- simulate_corrected_navigation (route-level) --------------------------


def test_corrected_navigation_is_deterministic() -> None:
    m = _straight_mission()
    ref = route_reference(m, seed=1)
    a = simulate_corrected_navigation(m, DriftConfig(seed=1), ref)
    b = simulate_corrected_navigation(m, DriftConfig(seed=1), ref)
    assert a == b


def test_good_map_bounds_error_far_below_vio_only() -> None:
    m = _straight_mission()
    drift = DriftConfig(rate=0.02, seed=3)
    vio_only = simulate_navigation(m, drift, None)
    corrected = simulate_corrected_navigation(
        m, drift, UniformReference(matchability=0.85, staleness=0.02)
    )
    assert corrected.max_error_m < 0.3 * vio_only.max_error_m
    assert corrected.within_corridor
    assert corrected.fix_availability > 0.5
    assert corrected.corrupt_accepted == 0


def test_forest_starves_fixes_and_error_grows() -> None:
    # Forest canopy — where the drone hides — has little matchable texture, so
    # fixes are sparse and the error is worse than over a good map.
    m = _straight_mission()
    drift = DriftConfig(rate=0.02, seed=3)
    good = simulate_corrected_navigation(m, drift, UniformReference())
    forest = simulate_corrected_navigation(
        m, drift, route_reference(m, forest_frac=0.6, stale_frac=0.0, seed=1)
    )
    assert forest.no_features > 0
    assert forest.fix_availability < good.fix_availability
    assert forest.max_error_m > good.max_error_m


def test_gate_prevents_stale_map_corruption() -> None:
    # A stale conflict-zone reference. With the innovation gate on, blunders are
    # rejected and the route stays bounded; with it effectively off, the blunders
    # are fused and the error blows up.
    m = _straight_mission()
    drift = DriftConfig(rate=0.02, seed=3)
    ref = route_reference(m, forest_frac=0.1, stale_frac=0.4, seed=1)
    gated = simulate_corrected_navigation(m, drift, ref)
    ungated = simulate_corrected_navigation(m, drift, ref, CorrectionConfig(gate_chi2=1e9))
    assert gated.gated_out > 0
    assert gated.corrupt_accepted == 0
    assert ungated.corrupt_accepted > 0
    assert ungated.max_error_m > gated.max_error_m
