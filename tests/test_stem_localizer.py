"""Unit tests for map-relative localisation from stems."""

from __future__ import annotations

import math

import pytest

from droneflight.stem_localizer import PoseFix, localize

Point = tuple[float, float]


def _grid(nx: int = 7, ny: int = 7, pitch: float = 6.0) -> list[Point]:
    """A regular stand. Regular is the HARD case — every offset looks alike."""
    return [(i * pitch, j * pitch) for i in range(nx) for j in range(ny)]


def _scatter(n: int = 40, span: float = 40.0, seed: int = 0) -> list[Point]:
    import random

    rng = random.Random(seed)
    return [(rng.uniform(0.0, span), rng.uniform(0.0, span)) for _ in range(n)]


def _displace(pts: list[Point], dx: float, dy: float, dyaw_deg: float) -> list[Point]:
    """Express ``pts`` in a frame that is wrong by (dx, dy, dyaw) — what the
    vehicle would compute from a drifted pose."""
    a = math.radians(-dyaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    return [(ca * (x - dx) - sa * (y - dy), sa * (x - dx) + ca * (y - dy)) for x, y in pts]


# --- It finds the pose -------------------------------------------------------------


def test_it_recovers_a_pure_translation() -> None:
    world = _scatter(seed=1)
    fix = localize(_displace(world, 5.0, -3.0, 0.0), world)
    assert fix.accepted
    assert fix.dx == pytest.approx(5.0, abs=0.05)
    assert fix.dy == pytest.approx(-3.0, abs=0.05)
    assert fix.dyaw_deg == pytest.approx(0.0, abs=0.5)


def test_it_recovers_translation_and_yaw_together() -> None:
    """Yaw matters: GPS-denied heading drifts, and a rotated map is a wrong map."""
    world = _scatter(seed=2)
    fix = localize(_displace(world, 4.0, 2.0, 9.0), world)
    assert fix.accepted
    assert fix.dx == pytest.approx(4.0, abs=0.1)
    assert fix.dy == pytest.approx(2.0, abs=0.1)
    assert fix.dyaw_deg == pytest.approx(9.0, abs=0.5)


def test_apply_maps_the_believed_frame_onto_the_map() -> None:
    world = _scatter(seed=3)
    observed = _displace(world, 6.0, -2.0, 5.0)
    fix = localize(observed, world)
    assert fix.accepted
    for seen, truth in zip(observed[:5], world[:5], strict=True):
        cx, cy = fix.apply(*seen)
        assert math.dist((cx, cy), truth) < 0.2


# --- It survives a map the battlespace has changed ---------------------------------


def test_stems_felled_since_the_survey_do_not_break_the_fix() -> None:
    """Mapped stems that are gone: the commonest form of staleness in a conflict."""
    world = _scatter(n=40, seed=4)
    standing = world[::2]  # half of them have been cut down
    fix = localize(_displace(standing, 5.0, -4.0, 6.0), world)
    assert fix.accepted
    assert fix.dx == pytest.approx(5.0, abs=0.2)
    assert fix.dy == pytest.approx(-4.0, abs=0.2)


def test_unmapped_stems_do_not_break_the_fix() -> None:
    """Regrowth, or stems the survey missed. They are outliers, not evidence."""
    world = _scatter(n=30, seed=5)
    extra = [(100.0 + i, 100.0 + 2 * i) for i in range(20)]
    observed = _displace(world + extra, 3.0, 3.0, 0.0)
    fix = localize(observed, world)
    assert fix.accepted
    assert fix.dx == pytest.approx(3.0, abs=0.2)
    # Two thirds of what it saw was unmapped and it still found the consensus.
    assert fix.inliers >= len(world) - 3


def test_a_majority_of_junk_still_yields_the_right_answer() -> None:
    """Consensus, not least squares: outliers must not drag the estimate."""
    world = _scatter(n=25, seed=6)
    junk = [(200.0 + 3 * i, 50.0 - 2 * i) for i in range(50)]
    fix = localize(_displace(world + junk, -6.0, 5.0, 0.0), world)
    assert fix.accepted
    assert fix.dx == pytest.approx(-6.0, abs=0.3)
    assert fix.dy == pytest.approx(5.0, abs=0.3)


# --- It refuses rather than lying ---------------------------------------------------


def test_too_few_stems_is_a_refusal_not_a_guess() -> None:
    fix = localize([(0.0, 0.0)], _scatter(seed=7))
    assert not fix.accepted
    assert "too few" in fix.reason


def test_nothing_in_common_is_refused() -> None:
    """Seeing a completely different stand must not produce a confident pose."""
    world = _scatter(n=30, seed=8)
    elsewhere = [(1000.0 + 3 * i, 2000.0 + 5 * i) for i in range(30)]
    fix = localize(elsewhere, world)
    assert not fix.accepted


def test_a_regular_plantation_is_reported_as_AMBIGUOUS() -> None:
    """🔑 The dangerous case. In a grid, an alignment one row over fits equally
    well, and 'one row wrong' is what flies into a tree. It must decline."""
    world = _grid(pitch=6.0)
    fix = localize(_displace(world, 1.0, 0.5, 0.0), world, search_radius_m=20.0)
    assert not fix.accepted
    assert "ambiguous" in fix.reason


def test_drift_beyond_the_search_radius_is_not_invented() -> None:
    """The bound is a safety property: an unbounded search finds a false match."""
    world = _scatter(seed=9)
    fix = localize(_displace(world, 60.0, 0.0, 0.0), world, search_radius_m=10.0)
    assert not fix.accepted


def test_a_refused_fix_still_reports_why() -> None:
    """A declined fix must be diagnosable, not silently absent."""
    fix = localize([(0.0, 0.0), (5.0, 0.0)], [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)])
    assert not fix.accepted
    assert fix.reason
    assert fix.observed == 2


# --- Bookkeeping --------------------------------------------------------------------


def test_inlier_fraction_and_shift_are_derived_consistently() -> None:
    fix = PoseFix(3.0, 4.0, 0.0, 5, 10, 0.1, True, "x")
    assert fix.shift_m == pytest.approx(5.0)
    assert fix.inlier_fraction == pytest.approx(0.5)


def test_empty_inputs_do_not_raise() -> None:
    assert not localize([], []).accepted
    assert not localize([(0.0, 0.0)], []).accepted


def test_an_exact_match_has_near_zero_residual() -> None:
    world = _scatter(seed=10)
    fix = localize(_displace(world, 2.0, 2.0, 0.0), world)
    assert fix.accepted
    assert fix.rmse_m < 0.05
