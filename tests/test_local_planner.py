"""The onboard local planner: memory, planning through gaps, and the speed limits
that each exist because of a specific offline strike (see local_planner.py)."""

from __future__ import annotations

import math

import pytest

from droneflight.local_offboard import window_altitude
from droneflight.local_planner import (
    LocalPlanner,
    LocalPlannerConfig,
    ObstacleMemory,
    SeenSpace,
    plan_local,
)
from droneflight.model import FlightMission, Waypoint
from droneflight.offboard import CarrotGuidance, RoutePoint
from droneflight.perception import ObstacleField, Trunk, sense
from droneflight.reactive import ForestFlightConfig, fly_with_local_planner

CFG = LocalPlannerConfig()


def _min_dist(path: list[tuple[float, float]], pts: list[tuple[float, float]]) -> float:
    return min(math.dist(p, q) for p in path for q in pts)


# --- planning ------------------------------------------------------------------------


def test_a_clear_field_plans_a_straight_line() -> None:
    path = plan_local([], (0.0, 0.0), (20.0, 0.0), CFG)
    assert path is not None
    assert max(abs(y) for _, y in path) < 0.5


def test_it_plans_round_a_trunk_on_the_line_with_the_hard_margin() -> None:
    trunk = [(10.0, 0.0)]
    path = plan_local(trunk, (0.0, 0.0), (20.0, 0.0), CFG)
    assert path is not None
    assert _min_dist(path, trunk) >= CFG.hard_m - 1e-6


def test_it_threads_the_gap_rather_than_going_round_a_wall() -> None:
    # A wall across the route with one 4 m gap at y = 0.
    wall = [(10.0, y) for y in [-12 + 0.2 * k for k in range(51)] if abs(y) > 2.0]
    path = plan_local(wall, (0.0, 0.0), (20.0, 0.0), CFG)
    assert path is not None
    crossing = min(path, key=lambda p: abs(p[0] - 10.0))
    assert abs(crossing[1]) < 1.0


def test_no_path_is_none_never_a_straight_line() -> None:
    wall = [(10.0, -30 + 0.2 * k) for k in range(301)]
    assert plan_local(wall, (0.0, 0.0), (20.0, 0.0), CFG) is None


def test_the_shortcut_never_gives_back_clearance() -> None:
    """String-pulling that cut corners put the stem router back through trunks once."""
    pts = [(8.0, 1.0), (12.0, -1.0), (16.0, 1.5)]
    path = plan_local(pts, (0.0, 0.0), (24.0, 0.0), CFG)
    assert path is not None
    assert _min_dist(path, pts) >= CFG.hard_m - 1e-6


# --- memory ----------------------------------------------------------------------------


def test_a_scan_is_remembered_near_the_trunk_axis() -> None:
    field = ObstacleField(trunks=(Trunk(10.0, 0.0, 0.15),))
    mem = ObstacleMemory()
    scan = sense(field, 0.0, 0.0, 0.0, math.radians(90), 91, 30.0)
    mem.add_scan(scan, 0.0, 0.0, 0.0, push_m=0.15)
    assert mem.points()
    assert all(math.dist(p, (10.0, 0.0)) < 0.2 for p in mem.points())


def test_occlusion_is_not_mistaken_for_free_space() -> None:
    field = ObstacleField(trunks=(Trunk(6.0, 0.0, 0.3),))
    seen = SeenSpace(0.6)
    seen.add_scan(sense(field, 0.0, 0.0, 0.0, math.radians(90), 91, 30.0), 0.0, 0.0, 0.0)
    assert seen.seen(3.0, 3.0)  # an unobstructed ray went through here
    assert not seen.seen(12.0, 0.0)  # directly behind the trunk: never looked


def test_the_sight_limit_slows_for_space_it_has_not_seen() -> None:
    """🔴 Occlusion was the first strike mechanism offline: a trunk hidden behind
    another appeared too close to stop for."""
    field = ObstacleField(trunks=(Trunk(8.0, 0.4, 0.3),))
    p = LocalPlanner()
    p.observe(sense(field, 0.0, 0.0, 0.0, math.radians(90), 91, 30.0), 0.0, 0.0, 0.0)
    p.command(0.0, 0.0, (22.0, 0.0), 0.0)
    assert p.path is not None
    assert p._sight_speed(p.path) < CFG.cruise_speed_ms


def test_a_navigation_correction_moves_the_map_with_it() -> None:
    """A fix shifts the believed position without the aircraft moving; the
    remembered trunks must shift by the same amount, not stay put."""
    field = ObstacleField(trunks=(Trunk(10.0, 0.0, 0.15),))
    p = LocalPlanner()
    p.observe(
        sense(field, 0, 0, 0, math.radians(90), 91, 30), 0.0, 0.0, 0.0, velocity=(0, 0), t_s=0
    )
    before = p.memory.points()
    # Hovering, but the estimate jumps 3 m east: a correction of +3 m.
    p.observe(
        sense(field, 0, 0, 0, math.radians(90), 91, 30), 3.0, 0.0, 0.0, velocity=(0, 0), t_s=0.1
    )
    assert p.corrections == 1
    for bx, by in before:
        assert any(math.dist((bx + 3.0, by), q) < 0.25 for q in p.memory.points())


def test_motion_explained_by_velocity_is_not_a_correction() -> None:
    p = LocalPlanner()
    empty = sense(ObstacleField(), 0, 0, 0, math.radians(90), 91, 30)
    p.observe(empty, 0.0, 0.0, 0.0, velocity=(5.0, 0.0), t_s=0.0)
    p.observe(empty, 0.5, 0.0, 0.0, velocity=(5.0, 0.0), t_s=0.1)
    assert p.corrections == 0


# --- speed ------------------------------------------------------------------------------


def test_the_mission_speed_is_a_ceiling() -> None:
    p = LocalPlanner()
    p.observe(sense(ObstacleField(), 0, 0, 0, math.radians(90), 91, 30), 0.0, 0.0, 0.0)
    for k in range(40):  # let the rate-limited ramp settle
        cmd = p.command(0.0, 0.0, (22.0, 0.0), k * 0.1, max_speed_ms=3.0)
    assert cmd.speed_ms == pytest.approx(3.0)


def test_it_brakes_before_a_turn_not_in_it() -> None:
    """🔴 The first curvature rule let the aircraft reach a 45 deg bend at cruise.

    The rule is ``v^2 <= v_turn^2 + 2 a s``: the nearer the corner, the slower now.
    """
    p = LocalPlanner()

    def bend_at(s_m: float) -> list[tuple[float, float]]:
        n = int(s_m / 0.5)
        return [(x * 0.5, 0.0) for x in range(n + 1)] + [(s_m, y * 0.5) for y in range(1, 40)]

    straight = [(x * 0.5, 0.0) for x in range(50)]
    assert p._curvature_speed(straight) == pytest.approx(CFG.cruise_speed_ms)
    far, near = p._curvature_speed(bend_at(8.0)), p._curvature_speed(bend_at(1.0))
    assert near < far < CFG.cruise_speed_ms
    # 90 deg over a 2 m chord -> ~1.6 m/s cornering; 1 m to brake at 2 m/s^2 -> ~2.5.
    assert near < 2.6


# --- closed loop ------------------------------------------------------------------------


def _line_mission(length_m: float) -> FlightMission:
    lat0, lon0 = 60.0, 25.0
    dlon = length_m / (6_371_000.0 * math.cos(math.radians(lat0))) * 180.0 / math.pi
    return FlightMission(
        name="line",
        home_lat=lat0,
        home_lon=lon0,
        cruise_speed_ms=6.0,
        waypoints=(
            Waypoint(lat=lat0, lon=lon0, alt_agl_m=10.0),
            Waypoint(lat=lat0, lon=lon0 + dlon, alt_agl_m=10.0),
        ),
    )


def test_it_flies_round_trunks_on_the_route_without_a_strike() -> None:
    mission = _line_mission(150.0)
    field = ObstacleField(
        trunks=tuple(Trunk(x, y, 0.15) for x, y in [(30, 0.1), (60, -0.3), (62, 2.0), (95, 0.0)])
    )
    cfg = ForestFlightConfig(fov_deg=90.0, n_rays=91, sensor_range_m=30.0)
    r = fly_with_local_planner(mission, field, config=cfg, max_time_s=300)
    assert r.reached_goal
    assert r.strikes == 0


# --- live executor helpers ----------------------------------------------------------------


def test_window_altitude_climbs_early_and_leaves_guidance_alone() -> None:
    route = [
        RoutePoint(0, 0, 10),
        RoutePoint(10, 0, 10),
        RoutePoint(20, 0, 30),
        RoutePoint(40, 0, 5),
    ]
    g = CarrotGuidance(route=route, lookahead_m=15.0)
    g.advance(1.0, 0.0)
    finished = g.finished
    assert window_altitude(route, g) == 30.0  # the hill 19 m ahead is already in view
    assert g.finished == finished  # 🔴 an earlier version reset this flag


def test_an_altitude_datum_mismatch_refuses_to_fly() -> None:
    """🔴 EKF2's local z sat 124.6 m below truth under external vision; the executor
    climbed for a target it thought was 118 m above it."""
    from droneflight.local_offboard import Pose, altitude_datum_error

    route = [RoutePoint(0, 0, 12), RoutePoint(50, 0, 12)]
    g = CarrotGuidance(route=route, lookahead_m=20.0)
    ok = Pose(0.0, 0.0, 10.0, 0.0)
    assert altitude_datum_error(ok, ok, route, g) is None
    offset = Pose(0.0, 0.0, 10.0 - 124.6, 0.0)
    assert altitude_datum_error(offset, ok, route, g) is not None
    assert altitude_datum_error(ok, Pose(0, 0, 134.6, 0.0), route, g) is not None


def test_a_correction_moves_observed_space_instead_of_erasing_it() -> None:
    """🔴 Clearing it on every bled-in VTRN fix pinned the first GPS-denied SITL
    run at the 0.8 m/s floor in open ground."""
    p = LocalPlanner()
    empty = sense(ObstacleField(), 0, 0, 0, math.radians(90), 91, 30)
    p.observe(empty, 0.0, 0.0, 0.0, velocity=(0.0, 0.0), t_s=0.0)
    assert p._seen().seen(10.0, 0.0)
    p.observe(empty, 0.4, 0.0, 0.0, velocity=(0.0, 0.0), t_s=0.1)  # a 0.4 m correction
    assert p.corrections == 1
    assert p._seen().seen(10.4, 0.0)


# --- the plan's corridor --------------------------------------------------------------


def test_the_corridor_bounds_the_detour() -> None:
    """A wall with gaps both near and far from the route: unbounded it may take
    either; bounded to 5 m it must take the near one."""
    wall = [
        (10.0, -30 + 0.2 * k)
        for k in range(301)
        if not (2.0 < -30 + 0.2 * k < 5.0 or -20.0 < -30 + 0.2 * k < -14.0)
    ]
    corridor = [((0.0, 0.0), 5.0), ((40.0, 0.0), 5.0)]
    path = plan_local(wall, (0.0, 0.0), (20.0, 0.0), CFG, corridor)
    assert path is not None
    crossing = min(path, key=lambda p: abs(p[0] - 10.0))
    assert 2.0 < crossing[1] < 5.0


def test_outside_the_corridor_it_still_plans_its_way_back() -> None:
    corridor = [((0.0, 0.0), 2.0), ((60.0, 0.0), 2.0)]
    path = plan_local([], (0.0, 12.0), (20.0, 0.0), CFG, corridor)
    assert path is not None
    assert abs(path[-1][1]) < 1.0


def test_the_corridor_floor_keeps_a_zero_width_vertex_passable() -> None:
    """The planner grants 0 m where the route hugs its cover; a trunk there must
    not make the route impassable."""
    corridor = [((0.0, 0.0), 0.0), ((40.0, 0.0), 0.0)]
    path = plan_local([(10.0, 0.0)], (0.0, 0.0), (20.0, 0.0), CFG, corridor)
    assert path is not None
    assert _min_dist(path, [(10.0, 0.0)]) >= CFG.hard_m - 1e-6


def test_prediction_flies_the_setpoints_already_in_flight() -> None:
    """🔴 PX4 acts on a velocity setpoint ~0.3 s after it is sent (measured,
    px4_profile.OFFBOARD_*). A GPS-denied SITL flight carried 2 m/s east for half a
    second after the command turned north and hit a trunk the prediction — which
    assumed the new command took effect at once — had passed as clear."""
    path = [(0.0, 0.5 * k) for k in range(40)]  # the new plan: due north
    trunk = [(1.2, 0.9)]  # just east of it

    def gap(issued: tuple[float, float]) -> float:
        p = LocalPlanner()
        p._now = 1.0
        p._issued = [(0.7 + 0.1 * k, *issued) for k in range(4)]
        return p._predict_gap((0.0, 0.0), (2.0, 0.0), path, 1.0, trunk)

    still_east = gap((2.0, 0.0))  # the last 0.3 s of setpoints said east
    already_north = gap((0.0, 1.0))
    assert still_east < already_north - 0.1


def test_slow_means_tight_the_lookahead_scales_with_speed() -> None:
    """🔴 A fixed 2.5 m pursuit chord cuts inside a bend by the same amount at any
    speed, so near a trunk no candidate was predicted clear and the aircraft sat
    still for 700 s (800/ha). Creeping must track the path closely."""
    p = LocalPlanner()
    assert p._pursuit(0.4) == pytest.approx(CFG.min_pursuit_m)
    assert p._pursuit(1.5) == pytest.approx(1.8)
    assert p._pursuit(6.0) == pytest.approx(CFG.pursuit_m)
    # Creeping round a corner with a trunk just inside it: the old fixed chord cut
    # the corner into contact; the scaled one follows the path and clears.
    from dataclasses import replace

    path = [(0.5 * k, 0.0) for k in range(9)] + [(4.0, 0.5 * k) for k in range(1, 20)]
    trunk = [(3.3, 0.8)]
    fixed = LocalPlanner(config=replace(CFG, min_pursuit_m=CFG.pursuit_m))
    assert fixed._predict_gap((3.0, 0.0), (0.4, 0.0), path, 0.4, trunk) < 0.0
    assert p._predict_gap((3.0, 0.0), (0.4, 0.0), path, 0.4, trunk) > 0.0
