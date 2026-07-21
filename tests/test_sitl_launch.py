"""Tests for the PX4 SITL home-injection helper."""

from __future__ import annotations

from droneflight.model import FlightMission, Waypoint
from droneflight.sitl_launch import format_env_exports, px4_home_env


def _mission() -> FlightMission:
    return FlightMission(
        name="t",
        home_lat=46.9,
        home_lon=15.0,
        home_alt_m=382.5,
        cruise_speed_ms=6.0,
        waypoints=(Waypoint(lat=46.9, lon=15.0, alt_agl_m=30.0),),
    )


def test_home_env_carries_plan_home_not_default() -> None:
    env = px4_home_env(_mission())
    assert env["PX4_HOME_LAT"] == "46.90000000"
    assert env["PX4_HOME_LON"] == "15.00000000"
    assert env["PX4_HOME_ALT"] == "382.50"


def test_format_env_exports_is_evalable() -> None:
    out = format_env_exports(px4_home_env(_mission()))
    lines = out.splitlines()
    assert lines == [
        "export PX4_HOME_LAT=46.90000000",
        "export PX4_HOME_LON=15.00000000",
        "export PX4_HOME_ALT=382.50",
    ]
