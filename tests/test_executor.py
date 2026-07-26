"""Tests for the mission executor orchestration + failsafe (fake connector)."""

from __future__ import annotations

import pytest

from droneflight.config import FlightConfig
from droneflight.connector import FakeConnector
from droneflight.executor import fly_mission, prepare_items
from droneflight.model import FlightMission, Waypoint
from droneflight.sensors import SensorMode, SensorPolicy, SensorSuite


def _mission(n: int = 3) -> FlightMission:
    wps = tuple(Waypoint(lat=46.90 + 0.001 * i, lon=15.0, alt_agl_m=30.0) for i in range(n))
    return FlightMission(
        name="t", home_lat=46.90, home_lon=15.0, cruise_speed_ms=6.0, waypoints=wps
    )


async def test_successful_mission() -> None:
    conn = FakeConnector()
    result = await fly_mission(conn, _mission(3), FlightConfig())
    assert result.completed
    assert result.items_flown == result.items_total == 3
    assert result.sensor.mode is SensorMode.PASSIVE_OPTICAL
    assert conn.calls == [
        "connect",
        "upload_mission",
        "arm",
        "start_mission",
        "mission_progress",
        "disconnect",
    ]


async def test_arm_failure_triggers_rtl() -> None:
    conn = FakeConnector(arm_raises=True)
    result = await fly_mission(conn, _mission(3), FlightConfig())
    assert not result.completed
    assert result.rtl_triggered
    assert result.aborted_reason is not None
    assert "return_to_launch" in conn.calls


async def test_failure_during_flight_aborts_partway() -> None:
    conn = FakeConnector(fail_at="progress:1")
    result = await fly_mission(conn, _mission(4), FlightConfig())
    assert not result.completed
    assert 0 < result.items_flown < result.items_total
    assert result.rtl_triggered


async def test_progress_stall_triggers_rtl() -> None:
    # A live PX4 that stops advancing must not hang the monitor forever: the
    # watchdog aborts and RTLs after progress_timeout_s.
    conn = FakeConnector(stall_after=1)
    result = await fly_mission(conn, _mission(4), FlightConfig(progress_timeout_s=0.05))
    assert not result.completed
    assert result.rtl_triggered
    assert result.aborted_reason is not None
    assert "no mission progress" in result.aborted_reason
    assert result.items_flown == 1  # made one item, then stalled


async def test_no_rtl_when_disabled() -> None:
    conn = FakeConnector(fail_at="upload_mission")
    result = await fly_mission(conn, _mission(3), FlightConfig(rtl_on_error=False))
    assert not result.completed
    assert not result.rtl_triggered
    assert "return_to_launch" not in conn.calls


async def test_night_non_covert_still_flies() -> None:
    config = FlightConfig(
        night_operation=True,
        sensors_suite=SensorSuite(optical=True, lidar=True),
        sensors_policy=SensorPolicy(allow_active_emission=True),
    )
    result = await fly_mission(FakeConnector(), _mission(2), config)
    assert result.completed
    assert result.sensor.mode is SensorMode.ACTIVE_LIDAR
    assert result.sensor.covert is False


def test_prepare_items_uses_mission_cruise_speed() -> None:
    items = prepare_items(_mission(3), FlightConfig())
    assert items[0].speed_ms == 6.0  # mission cruise speed honored


def _turning_mission() -> FlightMission:
    # A 90° corner at the middle waypoint so the speed policy has something to do.
    wps = (
        Waypoint(lat=46.90, lon=15.00, alt_agl_m=30.0),
        Waypoint(lat=46.91, lon=15.00, alt_agl_m=30.0),
        Waypoint(lat=46.91, lon=15.01, alt_agl_m=30.0),
    )
    return FlightMission(
        name="t", home_lat=46.90, home_lon=15.0, cruise_speed_ms=6.0, waypoints=wps
    )


def test_speed_policy_disabled_flies_every_waypoint_at_cruise() -> None:
    """The §3 control arm: --no-speed-policy => no slowdown through the corner."""
    m = _turning_mission()
    on = prepare_items(m, FlightConfig())
    off = prepare_items(m, FlightConfig(speed_policy_enabled=False))
    assert min(it.speed_ms for it in on) < 6.0  # policy slows the 90° corner
    assert all(it.speed_ms == 6.0 for it in off)  # disabled: everything at cruise


def test_not_ready_message_names_the_missing_flag() -> None:
    from droneflight.connector import not_ready_message

    msg = not_ready_message(30.0, False, False)
    assert "local_position_ok=False" in msg
    assert "ev-bridge" in msg  # points at the actual cause in a GPS-denied setup

    msg = not_ready_message(30.0, True, False)
    assert "home is not set" in msg
    assert "ev-bridge" not in msg

    msg = not_ready_message(30.0, None, None)
    assert "no telemetry at all" in msg


def test_auto_progress_timeout_outlasts_the_longest_leg() -> None:
    """A 60 s constant aborted a healthy SITL flight mid-leg at 41/77."""
    from droneflight.executor import auto_progress_timeout_s
    from droneflight.model import MissionItem

    # ~1.1 km leg at 6 m/s = ~188 s — the leg that actually tripped the watchdog.
    items = [
        MissionItem(lat=46.90, lon=15.0, alt_agl_m=30.0, speed_ms=6.0),
        MissionItem(lat=46.91017, lon=15.0, alt_agl_m=30.0, speed_ms=6.0),
    ]
    timeout = auto_progress_timeout_s(items)
    assert timeout > 188.0, "must outlast the leg it is monitoring"
    assert timeout == pytest.approx(188.5 * 3.0, rel=0.02)


def test_auto_progress_timeout_has_a_floor_for_short_routes() -> None:
    from droneflight.executor import auto_progress_timeout_s
    from droneflight.model import MissionItem

    close = [
        MissionItem(lat=46.90, lon=15.0, alt_agl_m=30.0, speed_ms=6.0),
        MissionItem(lat=46.9001, lon=15.0, alt_agl_m=30.0, speed_ms=6.0),
    ]
    assert auto_progress_timeout_s(close) == 60.0
    assert auto_progress_timeout_s(close[:1]) == 60.0


async def test_explicit_progress_timeout_overrides_the_derived_one() -> None:
    conn = FakeConnector(stall_after=1)
    result = await fly_mission(conn, _mission(4), FlightConfig(progress_timeout_s=0.05))
    assert "no mission progress within 0s" in (result.aborted_reason or "")


async def test_derived_timeout_is_used_when_config_leaves_it_unset() -> None:
    conn = FakeConnector()
    result = await fly_mission(conn, _mission(4), FlightConfig())
    assert result.completed  # no spurious abort with the default (auto) timeout
