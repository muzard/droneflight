"""Tests for the optical/depth perception model."""

from __future__ import annotations

import math

from droneflight.perception import ObstacleField, Trunk, sense


def test_clearance_to_nearest_trunk() -> None:
    field = ObstacleField(trunks=(Trunk(x=3.0, y=0.0, radius=0.2),))
    assert field.clearance(0.0, 0.0) == 3.0 - 0.2  # distance to the surface
    assert field.clearance(3.0, 0.0) == -0.2  # inside the trunk


def test_empty_field_is_clear() -> None:
    field = ObstacleField()
    assert field.clearance(0.0, 0.0) == math.inf
    assert field.ray_range(0.0, 0.0, 0.0, 15.0) == 15.0


def test_ray_hits_trunk_ahead() -> None:
    field = ObstacleField(trunks=(Trunk(x=10.0, y=0.0, radius=0.3),))
    # Ray east hits the near surface at ~9.7 m.
    assert abs(field.ray_range(0.0, 0.0, 0.0, 15.0) - 9.7) < 1e-6
    # Ray north misses it.
    assert field.ray_range(0.0, 0.0, math.pi / 2, 15.0) == 15.0


def test_ray_beyond_range_returns_max() -> None:
    field = ObstacleField(trunks=(Trunk(x=100.0, y=0.0, radius=0.2),))
    assert field.ray_range(0.0, 0.0, 0.0, 15.0) == 15.0


def test_scan_spans_fov() -> None:
    field = ObstacleField(trunks=(Trunk(x=8.0, y=0.0, radius=0.3),))
    scan = sense(field, 0.0, 0.0, 0.0, math.radians(120), 41, 15.0)
    assert len(scan.ranges) == 41
    assert abs(scan.bearings_rel[0] + math.radians(60)) < 1e-9
    assert abs(scan.bearings_rel[-1] - math.radians(60)) < 1e-9
    # The centre ray sees the trunk; the edges are clear.
    assert scan.min_range() < 8.0
    assert scan.ranges[0] == 15.0


def test_spatial_index_matches_bruteforce() -> None:
    import random

    rng = random.Random(0)
    trunks = tuple(
        Trunk(x=rng.uniform(-40, 40), y=rng.uniform(-40, 40), radius=0.2) for _ in range(200)
    )
    field = ObstacleField(trunks=trunks)
    for _ in range(20):
        x, y = rng.uniform(-40, 40), rng.uniform(-40, 40)
        brute = min(math.hypot(x - t.x, y - t.y) - t.radius for t in trunks)
        # Indexed clearance is correct whenever the nearest trunk is within a cell.
        if brute < field.cell_size_m:
            assert abs(field.clearance(x, y) - brute) < 1e-9


def test_a_sensor_above_a_trunk_top_does_not_see_or_touch_it() -> None:
    """Flying OVER a stand (droneplanner over_canopy_when_unpassable, 2026-09-25): the
    2D oracle used to treat every trunk as infinitely tall, so an aircraft passing
    over a thicket would still dodge and brake for trunks metres beneath it."""
    import math

    from droneflight.perception import OVERFLY_MARGIN_M, ObstacleField, Trunk, sense

    field = ObstacleField(trunks=(Trunk(5.0, 0.0, 0.15, top_z=10.0),))
    below = sense(field, 0.0, 0.0, 0.0, math.radians(90), 91, 30.0, z=9.0)
    above = sense(field, 0.0, 0.0, 0.0, math.radians(90), 91, 30.0, z=10.0 + OVERFLY_MARGIN_M)
    assert below.min_range() < 5.0
    assert above.min_range() == above.max_range
    assert field.clearance(5.0, 0.0, z=9.0) < 0.0  # inside it, level with it
    assert field.clearance(5.0, 0.0, z=11.0) == math.inf  # over it
    assert field.clearance(5.0, 0.0) < 0.0  # no z = the old, infinitely tall model
