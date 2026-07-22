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


def test_parse_qgc_params_skips_comments_and_types_values() -> None:
    from droneflight.sitl_launch import parse_qgc_params

    text = (
        "# a comment\n"
        "\n"
        "1\t1\tEKF2_EV_CTRL\t11\t6\n"
        "1\t1\tEKF2_EV_DELAY\t50\t9\n"
        "1 1 SIM_BAT_DRAIN 0.0 9   # trailing comment\n"
    )
    assert parse_qgc_params(text) == [
        ("EKF2_EV_CTRL", "11"),  # int32 stays an int
        ("EKF2_EV_DELAY", "50.0"),  # float gains a decimal point
        ("SIM_BAT_DRAIN", "0.0"),
    ]


def test_parse_qgc_params_rejects_malformed_input() -> None:
    import pytest

    from droneflight.sitl_launch import ParamFileError, parse_qgc_params

    with pytest.raises(ParamFileError, match="expected 5 fields"):
        parse_qgc_params("1 1 EKF2_EV_CTRL 11\n")
    with pytest.raises(ParamFileError, match="no parameters"):
        parse_qgc_params("# only comments\n")


def test_format_pxh_param_sets_ends_with_save() -> None:
    from droneflight.sitl_launch import format_pxh_param_sets

    out = format_pxh_param_sets([("A", "1"), ("B", "2.0")]).splitlines()
    assert out == ["param set A 1", "param set B 2.0", "param save"]


def test_px4_params_cli_concatenates_files(tmp_path, capsys) -> None:
    from droneflight.cli import app

    a, b = tmp_path / "a.params", tmp_path / "b.params"
    a.write_text("1\t1\tEKF2_GPS_CTRL\t0\t6\n")
    b.write_text("# hdr\n1\t1\tNAV_DLL_ACT\t0\t6\n")
    rc = app(["px4-params", "--file", str(a), "--file", str(b)])
    assert rc == 0
    assert capsys.readouterr().out.splitlines() == [
        "param set EKF2_GPS_CTRL 0",
        "param set NAV_DLL_ACT 0",
        "param save",
    ]


def test_px4_params_cli_reports_a_bad_file(tmp_path) -> None:
    from droneflight.cli import app

    bad = tmp_path / "bad.params"
    bad.write_text("nonsense\n")
    assert app(["px4-params", "--file", str(bad)]) == 2
