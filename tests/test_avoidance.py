"""Tests for the reactive avoidance controller."""

from __future__ import annotations

import math

from droneflight.avoidance import AvoidanceConfig, avoid
from droneflight.perception import ObstacleField, Trunk, sense

_FOV = math.radians(120)


def _scan(field: ObstacleField, x: float, y: float, heading: float = 0.0):  # type: ignore[no-untyped-def]
    return sense(field, x, y, heading, _FOV, 41, 15.0)


def test_clear_path_heads_to_goal_at_cruise() -> None:
    cfg = AvoidanceConfig()
    cmd = avoid(_scan(ObstacleField(), 0.0, 0.0), 0.0, cfg)
    assert abs(cmd.desired_heading_rel) < 1e-6
    assert cmd.speed_ms == cfg.cruise_speed_ms
    assert not cmd.blocked


def test_steers_around_trunk_ahead() -> None:
    cfg = AvoidanceConfig()
    field = ObstacleField(trunks=(Trunk(x=5.0, y=0.0, radius=0.2),))
    cmd = avoid(_scan(field, 0.0, 0.0), 0.0, cfg)  # goal straight ahead, trunk at 5 m
    assert abs(cmd.desired_heading_rel) > math.radians(3)  # actively steers aside
    assert not cmd.blocked


def test_fully_blocked_hovers() -> None:
    cfg = AvoidanceConfig()
    # A wall of trunks across the whole FOV within the reaction distance.
    trunks = tuple(Trunk(x=2.0, y=y, radius=0.4) for y in [-2, -1, 0, 1, 2])
    cmd = avoid(_scan(ObstacleField(trunks=trunks), 0.0, 0.0), 0.0, cfg)
    assert cmd.blocked
    assert cmd.speed_ms == 0.0


def test_slows_when_clearance_tight() -> None:
    cfg = AvoidanceConfig()
    near = avoid(_scan(ObstacleField((Trunk(x=2.5, y=0.0, radius=0.2),)), 0.0, 0.0), 0.0, cfg)
    far = avoid(_scan(ObstacleField(), 0.0, 0.0), 0.0, cfg)
    assert near.speed_ms < far.speed_ms


def test_prefers_free_side_toward_goal() -> None:
    cfg = AvoidanceConfig()
    # Trunk ahead-left; goal straight ahead -> should veer right (negative).
    field = ObstacleField(trunks=(Trunk(x=4.0, y=1.0, radius=0.3),))
    cmd = avoid(_scan(field, 0.0, 0.0), 0.0, cfg)
    assert cmd.desired_heading_rel < 0.0
