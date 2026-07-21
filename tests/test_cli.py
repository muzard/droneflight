"""Tests for the droneflight CLI (offline validate path)."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from droneflight.cli import app


def _write_plan(path: Path) -> None:
    doc = {
        "fileType": "Plan",
        "version": 1,
        "mission": {
            "cruiseSpeed": 6.0,
            "plannedHomePosition": [46.95, 15.0, 30.0],
            "items": [
                {"type": "SimpleItem", "command": 16, "params": [0, 0, 0, None, 46.95, 15.0, 30.0]},
                {"type": "SimpleItem", "command": 16, "params": [0, 0, 0, None, 46.96, 15.0, 32.0]},
            ],
        },
    }
    path.write_text(json.dumps(doc))


def test_validate_ok(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["validate", "--plan", str(plan)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "sensor mode: passive_optical" in out
    assert "2 waypoints" in out


def test_validate_night_without_thermal_errors(tmp_path: Path) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["validate", "--plan", str(plan), "--night"])
    assert rc == 3  # no covert night sensor in the default suite


def test_validate_bad_plan(tmp_path: Path) -> None:
    bad = tmp_path / "bad.plan"
    bad.write_text("{ not json")
    assert app(["validate", "--plan", str(bad)]) == 2


def test_simfly_reports_avoidance(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["simfly", "--plan", str(plan), "--density", "10", "--seed", "1"])
    out = capsys.readouterr().out
    assert rc in (0, 1)
    assert "avoidance OFF" in out
    assert "avoidance ON" in out
    assert "trunks" in out


def test_nav_reports_drift(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["nav", "--plan", str(plan), "--vtrn-interval", "500"])
    out = capsys.readouterr().out
    assert rc == 0
    assert "VIO only" in out
    assert "VIO + VTRN (idealized" in out
    assert "VIO + VTRN (optical" in out
    assert "avail" in out


def test_sitl_env_prints_plan_home(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["sitl-env", "--plan", str(plan)])
    out = capsys.readouterr().out
    assert rc == 0
    # Home comes from the plan (46.95), not PX4's Zurich default.
    assert "export PX4_HOME_LAT=46.95" in out
    assert "export PX4_HOME_LON=15.00" in out


def test_sitl_env_bad_plan(tmp_path: Path) -> None:
    bad = tmp_path / "bad.plan"
    bad.write_text("{ not json")
    assert app(["sitl-env", "--plan", str(bad)]) == 2


def test_nav_fidelity_flags(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["nav", "--plan", str(plan), "--forest-frac", "0.7", "--stale-frac", "0.3"])
    out = capsys.readouterr().out
    assert rc == 0
    assert "70% forest / 30% stale basemap" in out
