"""Unit tests for the corridor executor (TEST-PLAN B2b/B3).

Everything above the autopilot boundary is pure and tested here: frames,
guidance, altitude command, arrival. The MAVSDK driver below it is not testable
without a live vehicle and is excluded from coverage, as ``mavsdk_connector`` is.
"""

from __future__ import annotations

import math
from collections.abc import AsyncIterator
from dataclasses import replace

import pytest

from droneflight.avoidance import AvoidanceConfig
from droneflight.model import FlightMission, Waypoint
from droneflight.offboard import (
    CarrotGuidance,
    OffboardConfig,
    RoutePoint,
    VelocityCommand,
    compute_command,
    enu_bearing_to_ned_yaw,
    fly_corridor,
    ned_yaw_to_enu,
    route_to_local,
)
from droneflight.perception import ObstacleField, Trunk

_M_PER_DEG = math.radians(1.0) * 6_371_000.0


# --- Frames: the trap that would steer the aircraft into trees at 90 degrees ---


@pytest.mark.parametrize(
    ("yaw_deg", "expected_bearing_deg"),
    [(0.0, 90.0), (90.0, 0.0), (180.0, -90.0), (270.0, -180.0)],
)
def test_ned_yaw_to_enu_bearing(yaw_deg: float, expected_bearing_deg: float) -> None:
    """North-facing (yaw 0) is +90 deg in the math frame perception uses."""
    assert math.degrees(ned_yaw_to_enu(yaw_deg)) == pytest.approx(expected_bearing_deg)


@pytest.mark.parametrize("yaw_deg", [0.0, 45.0, 123.4, 359.9])
def test_yaw_conversion_round_trips(yaw_deg: float) -> None:
    assert enu_bearing_to_ned_yaw(ned_yaw_to_enu(yaw_deg)) == pytest.approx(yaw_deg, abs=1e-9)


def test_route_projects_into_home_relative_metres() -> None:
    mission = FlightMission(
        name="t",
        home_lat=0.0,
        home_lon=0.0,
        home_alt_m=100.0,
        cruise_speed_ms=6.0,
        waypoints=(
            Waypoint(lat=0.0, lon=0.0, alt_agl_m=10.0),
            Waypoint(lat=100.0 / _M_PER_DEG, lon=0.0, alt_agl_m=20.0),
        ),
    )
    route = route_to_local(mission)
    assert route[0] == RoutePoint(0.0, 0.0, 10.0)
    assert route[1].north_m == pytest.approx(100.0, rel=1e-3)
    assert route[1].east_m == pytest.approx(0.0, abs=1e-6)
    assert route[1].up_m == 20.0


# --- Guidance ---


def _line(n: int = 10, spacing: float = 20.0, up: float = 12.0) -> list[RoutePoint]:
    return [RoutePoint(east_m=0.0, north_m=i * spacing, up_m=up) for i in range(n)]


def test_carrot_sits_ahead_and_progress_only_moves_forward() -> None:
    guidance = CarrotGuidance(route=_line(), lookahead_m=25.0)
    carrot = guidance.advance(0.0, 0.0)
    assert carrot.north_m > 0.0

    guidance.advance(0.0, 100.0)
    ahead = guidance.index
    # A reactive detour that pushes the aircraft backwards must not rewind
    # progress, or it re-flies the section it already passed.
    guidance.advance(0.0, 0.0)
    assert guidance.index == ahead


def test_guidance_finishes_at_the_last_point() -> None:
    guidance = CarrotGuidance(route=_line(n=4, spacing=10.0), lookahead_m=5.0)
    for _ in range(10):
        guidance.advance(0.0, 1000.0)
    assert guidance.finished


# --- Control step ---


def _config(**kwargs: object) -> OffboardConfig:
    base = dict(lookahead_m=25.0, cruise_speed_ms=6.0, sensor_range_m=30.0)
    base.update(kwargs)
    return OffboardConfig(**base)  # type: ignore[arg-type]


def test_with_a_clear_path_it_flies_straight_at_the_carrot() -> None:
    guidance = CarrotGuidance(route=_line(), lookahead_m=25.0)
    command = compute_command(
        guidance,
        ObstacleField(),
        east_m=0.0,
        north_m=0.0,
        up_m=12.0,
        yaw_deg=0.0,  # facing north, which is where the route goes
        config=_config(),
    )
    assert command.north_ms == pytest.approx(6.0, rel=0.05)
    assert command.east_ms == pytest.approx(0.0, abs=0.5)
    assert command.down_ms == pytest.approx(0.0, abs=1e-6)
    assert not command.blocked


def _with_trunk_at(north_m: float) -> VelocityCommand:
    guidance = CarrotGuidance(route=_line(), lookahead_m=25.0)
    blocker = ObstacleField(trunks=(Trunk(x=0.0, y=north_m, radius=0.3),))
    return compute_command(
        guidance,
        blocker,
        east_m=0.0,
        north_m=0.0,
        up_m=12.0,
        yaw_deg=0.0,
        config=_config(),
    )


def test_a_trunk_inside_the_reaction_distance_steers_the_command_off_the_line() -> None:
    """The whole reason this executor exists."""
    command = _with_trunk_at(5.0)  # inside AvoidanceConfig.react_distance_m (6 m)
    assert abs(command.east_ms) > 0.5, "expected a lateral component to dodge the trunk"
    assert command.clearance_ahead_m < 6.0


def test_a_trunk_at_8_m_is_now_steered_around_not_merely_seen() -> None:
    """🔴 This test used to assert the opposite, and B2 flew that into 37 strikes.

    The old behaviour — trunk at 8 m *seen* (clearance drops) but not steered
    around — followed from the constant ``react_distance_m = 6.0``, which at 6 m/s
    is 1.0 s of warning against the 1.20 s the airframe needs to graze past. The
    reaction distance is now derived from speed and PX4's accel/jerk limits, so
    8 m is inside it and the command carries a lateral component.
    """
    command = _with_trunk_at(8.0)
    assert abs(command.east_ms) > 0.5, "expected a dodge, not just a slowdown"
    assert _config().avoidance_config.react_distance_m > 8.0


def test_the_flown_b2_configuration_is_refused_before_takeoff() -> None:
    """A configuration that cannot geometrically miss must say so, not fly."""
    doomed = replace(
        _config(),
        n_rays=31,
        avoidance=AvoidanceConfig(react_distance_override_m=6.0),
        cruise_speed_ms=6.0,
    )
    shortfall = doomed.sensing_shortfall()
    assert shortfall is not None and "cannot miss" in shortfall
    assert _config().sensing_shortfall() is None


def test_undersampled_scan_is_refused_even_when_the_maneuver_fits() -> None:
    """Reaction distance and angular sampling are separate failures; B2 had both."""
    undersampled = replace(_config(), n_rays=31)
    shortfall = undersampled.sensing_shortfall()
    assert shortfall is not None and "between rays" in shortfall


def test_altitude_is_commanded_from_the_plan_not_left_to_the_autopilot() -> None:
    """A2's finding: an offboard executor must supply its own altitude."""
    guidance = CarrotGuidance(route=_line(up=30.0), lookahead_m=25.0)
    low = compute_command(
        guidance,
        ObstacleField(),
        east_m=0.0,
        north_m=0.0,
        up_m=10.0,  # 20 m below the planned profile
        yaw_deg=0.0,
        config=_config(),
    )
    assert low.down_ms < 0.0  # NED down negative = climbing
    assert low.down_ms == pytest.approx(-2.0, rel=1e-6)  # capped at max_climb_ms


def test_climb_is_rate_limited_in_both_directions() -> None:
    guidance = CarrotGuidance(route=_line(up=0.0), lookahead_m=25.0)
    high = compute_command(
        guidance,
        ObstacleField(),
        east_m=0.0,
        north_m=0.0,
        up_m=100.0,
        yaw_deg=0.0,
        config=_config(),
    )
    assert high.down_ms == pytest.approx(2.0, rel=1e-6)


# --- The loop, against a fake autopilot ---


class _FakeConnector:
    """Flies the commanded velocity kinematically, so the loop can be tested."""

    def __init__(self, start: tuple[float, float, float], dt: float = 0.1, steps: int = 4000):
        self.east, self.north, self.up = start
        self.yaw_deg = 0.0
        self.dt = dt
        self.max_steps = steps
        self.commands: list[VelocityCommand] = []
        self.offboard_started = False
        self.offboard_stopped = False

    async def connect(self, url: str, ready_timeout_s: float) -> None: ...
    async def arm(self) -> None: ...
    async def takeoff(self, altitude_m: float) -> None: ...

    async def start_offboard(self) -> None:
        self.offboard_started = True

    async def set_velocity_ned(self, command: VelocityCommand) -> None:
        self.commands.append(command)
        self.north += command.north_ms * self.dt
        self.east += command.east_ms * self.dt
        self.up -= command.down_ms * self.dt
        self.yaw_deg = command.yaw_deg

    async def _stream(self) -> AsyncIterator[tuple[float, float, float, float]]:
        for _ in range(self.max_steps):
            yield self.east, self.north, self.up, self.yaw_deg

    def telemetry(self) -> AsyncIterator[tuple[float, float, float, float]]:
        return self._stream()

    async def stop_offboard(self) -> None:
        self.offboard_stopped = True

    async def return_to_launch(self) -> None: ...
    async def disconnect(self) -> None: ...


def _mission(n: int = 10, spacing_m: float = 20.0) -> FlightMission:
    return FlightMission(
        name="t",
        home_lat=0.0,
        home_lon=0.0,
        home_alt_m=100.0,
        cruise_speed_ms=6.0,
        waypoints=tuple(
            Waypoint(lat=i * spacing_m / _M_PER_DEG, lon=0.0, alt_agl_m=12.0) for i in range(n)
        ),
    )


@pytest.mark.asyncio
async def test_it_flies_an_empty_corridor_to_the_end() -> None:
    connector = _FakeConnector(start=(0.0, 0.0, 12.0))
    result = await fly_corridor(connector, _mission(), ObstacleField(), _config())
    assert result.reached_goal
    assert connector.offboard_started and connector.offboard_stopped
    assert result.blocked_steps == 0


@pytest.mark.asyncio
async def test_it_still_reaches_the_end_through_a_trunk_field() -> None:
    """Trees on the line, and the executor has to get past them."""
    trunks = tuple(Trunk(x=0.0, y=y, radius=0.3) for y in (40.0, 80.0, 120.0))
    connector = _FakeConnector(start=(0.0, 0.0, 12.0))
    result = await fly_corridor(connector, _mission(), ObstacleField(trunks=trunks), _config())
    assert result.reached_goal
    assert result.min_clearance_m < 30.0, "it should have seen the trunks"


@pytest.mark.asyncio
async def test_a_short_route_is_rejected_rather_than_flown_ambiguously() -> None:
    mission = _mission(n=1)
    with pytest.raises(ValueError, match="at least two points"):
        await fly_corridor(_FakeConnector((0.0, 0.0, 12.0)), mission, ObstacleField(), _config())


def test_guidance_reports_distance_along_the_route() -> None:
    """The escape climb watches along-route distance, not the vertex index: at 0.5 m/s
    a 25 m leg shows no index progress for 50 s (SITL x4e false escape at launch)."""
    from droneflight.local_offboard import route_arc_m

    route = [RoutePoint(0.0, 0.0, 10.0), RoutePoint(25.0, 0.0, 10.0), RoutePoint(50.0, 0.0, 10.0)]
    g = CarrotGuidance(route=route, lookahead_m=5.0)
    arc = route_arc_m(route)
    g.advance(10.0, 0.0)
    assert g.index == 0 and abs(g.along_m(arc) - 10.0) < 1e-9
    g.advance(30.0, 1.0)
    assert g.index == 1 and abs(g.along_m(arc) - 30.0) < 1e-9
