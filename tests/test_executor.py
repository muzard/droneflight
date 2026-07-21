"""Tests for the mission executor orchestration + failsafe (fake connector)."""

from __future__ import annotations

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
