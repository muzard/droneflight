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
    assert "VIO + VTRN" in out
