"""Strike scoring: contact is a dynamic event, not a proximity one."""

from __future__ import annotations

import math

import pytest

from droneflight.perception import Trunk
from droneflight.strikes import StrikeConfig, score_track
from droneflight.trunk_manifest import ForestManifest

DT = 0.02


def _manifest(*trunks: tuple[float, float, float, float]) -> ForestManifest:
    """Each entry is (east, north, base_z, height); radius fixed at the B2 0.15 m."""
    return ForestManifest(
        trunks=tuple(Trunk(x=e, y=n, radius=0.15) for e, n, _, _ in trunks),
        heights_m=tuple(h for _, _, _, h in trunks),
        base_z_m=tuple(b for _, _, b, _ in trunks),
    )


def _straight(n: int, speed: float, up: float = 10.0, east0: float = 0.0):
    """A constant-velocity eastward pass — the shape of undisturbed cruise."""
    t = [i * DT for i in range(n)]
    east = [east0 + speed * i * DT for i in range(n)]
    north = [0.0] * n
    ups = [up] * n
    return t, east, north, ups, [speed] * n, [0.0] * n


def test_flying_close_past_a_trunk_is_not_a_strike():
    """The sparse arm's closest *miss* was 0.075 m. Proximity is not contact."""
    manifest = _manifest((5.0, 0.20, 0.0, 20.0))
    report = score_track(*_straight(200, 6.0), manifest)
    assert report.tree_strikes == 0
    assert report.clean
    assert report.min_clearance_m < 0.1  # it really was that close


def test_an_abrupt_deceleration_at_a_trunk_is_a_strike():
    t, east, north, up, ve, vn = _straight(200, 6.0)
    # Collapse the velocity at the trunk and reverse it, as Gazebo's contact did.
    hit = 100
    for i in range(hit, len(ve)):
        ve[i] = -1.0
    for i in range(hit, len(east)):
        east[i] = east[hit]
    manifest = _manifest((east[hit] + 0.3, 0.0, 0.0, 20.0))
    report = score_track(t, east, north, up, ve, vn, manifest)
    assert report.tree_strikes == 1
    assert not report.clean
    episode = report.episodes[0]
    assert episode.kind == "tree"
    assert episode.peak_g > 2.0
    assert episode.speed_in_ms > episode.speed_out_ms


def test_one_contact_spanning_several_samples_counts_once():
    """Without episode grouping a single impact reads as a burst of strikes."""
    t, east, north, up, ve, vn = _straight(200, 6.0)
    for i in range(100, 105):  # five consecutive decelerating samples
        ve[i] = 6.0 - (i - 99) * 1.2
    for i in range(105, len(ve)):
        ve[i] = 0.0
    manifest = _manifest((east[100] + 0.3, 0.0, 0.0, 20.0))
    report = score_track(t, east, north, up, ve, vn, manifest)
    assert report.tree_strikes == 1


def test_vertical_span_is_respected_so_flying_over_a_stand_is_clean():
    """A 2D clearance would call this a strike; the vehicle was 20 m above it."""
    t, east, north, up, ve, vn = _straight(200, 6.0, up=30.0)
    hit = 100
    for i in range(hit, len(ve)):
        ve[i] = -1.0
    manifest = _manifest((east[hit] + 0.3, 0.0, 0.0, 12.0))  # crown at 12 m
    report = score_track(t, east, north, up, ve, vn, manifest)
    assert report.tree_strikes == 0
    assert not math.isfinite(report.min_clearance_m), "above every crown"


def test_a_deceleration_near_the_ground_is_attributed_to_terrain():
    t, east, north, up, ve, vn = _straight(200, 6.0, up=1.0)
    for i in range(100, len(ve)):
        ve[i] = 0.0
    # A trunk near enough to supply the terrain datum (base_z = 0), far enough
    # laterally that the impact cannot be attributed to the trunk itself.
    manifest = _manifest((east[100], 20.0, 0.0, 20.0))
    report = score_track(t, east, north, up, ve, vn, manifest)
    assert report.ground_impacts == 1
    assert report.tree_strikes == 0


def test_impacts_are_unattributed_where_the_terrain_proxy_has_no_data():
    """Trunk bases stand in for terrain, so away from trees there is no datum.

    The honest answer is 'unknown', not 'ground' — and the run is still not clean.
    """
    t, east, north, up, ve, vn = _straight(200, 6.0, up=1.0)
    for i in range(100, len(ve)):
        ve[i] = 0.0
    report = score_track(t, east, north, up, ve, vn, _manifest((500.0, 0.0, 0.0, 20.0)))
    assert report.min_agl_m is None
    assert (report.tree_strikes, report.ground_impacts) == (0, 0)
    assert not report.clean and len(report.episodes) == 1


def test_deceleration_is_measured_on_the_velocity_vector_so_a_bounce_counts():
    """A rebound barely changes *speed* while reversing direction entirely."""
    t, east, north, up, ve, vn = _straight(200, 6.0)
    for i in range(100, len(ve)):
        ve[i] = -6.0  # same speed, opposite direction
    manifest = _manifest((east[100] + 0.3, 0.0, 0.0, 20.0))
    report = score_track(t, east, north, up, ve, vn, manifest)
    assert report.tree_strikes == 1


def test_threshold_is_above_anything_ordinary_flight_produces():
    """The B1 negative control never exceeded 0.50 g; a hard but normal
    deceleration must not register."""
    t, east, north, up, ve, vn = _straight(400, 6.0)
    for i in range(100, len(ve)):  # brake at PX4's 3.0 m/s^2 limit == 0.31 g
        ve[i] = max(0.0, 6.0 - 3.0 * (i - 99) * DT)
    manifest = _manifest((east[100] + 0.3, 0.0, 0.0, 20.0))
    assert score_track(t, east, north, up, ve, vn, manifest).clean


def test_stationary_and_mismatched_input_are_handled():
    manifest = _manifest((5.0, 0.0, 0.0, 20.0))
    assert score_track([], [], [], [], [], [], manifest).clean
    with pytest.raises(ValueError):
        score_track([0.0, 1.0], [0.0], [0.0], [0.0], [0.0], [0.0], manifest)


def test_config_threshold_is_honoured():
    t, east, north, up, ve, vn = _straight(200, 6.0)
    for i in range(100, len(ve)):
        ve[i] = 3.0  # 3 m/s in 20 ms == 15 g
    manifest = _manifest((east[100] + 0.3, 0.0, 0.0, 20.0))
    assert score_track(t, east, north, up, ve, vn, manifest).tree_strikes == 1
    lenient = StrikeConfig(accel_threshold_g=50.0)
    assert score_track(t, east, north, up, ve, vn, manifest, lenient).clean
