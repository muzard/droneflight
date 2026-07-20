"""Tests for loading a QGC .plan into a FlightMission."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from droneflight.plan_loader import PlanLoadError, load_qgc_plan


def _plan(items: list[dict], home=None) -> dict:  # type: ignore[no-untyped-def]
    return {
        "fileType": "Plan",
        "version": 1,
        "mission": {
            "cruiseSpeed": 6.0,
            "plannedHomePosition": home or [46.95, 15.0, 30.0],
            "items": items,
        },
    }


def _wp(lat: float, lon: float, alt: float, command: int = 16) -> dict:  # type: ignore[no-untyped-def]
    return {"type": "SimpleItem", "command": command, "params": [0, 0, 0, None, lat, lon, alt]}


def test_load_valid_plan(tmp_path: Path) -> None:
    path = tmp_path / "m.plan"
    path.write_text(json.dumps(_plan([_wp(46.95, 15.0, 30.0), _wp(46.96, 15.01, 32.0)])))
    mission = load_qgc_plan(path)
    assert mission.name == "m"
    assert len(mission.waypoints) == 2
    assert mission.cruise_speed_ms == 6.0
    assert mission.waypoints[0].alt_agl_m == 30.0
    assert mission.home_lat == 46.95


def test_skips_non_waypoint_items(tmp_path: Path) -> None:
    path = tmp_path / "m.plan"
    path.write_text(json.dumps(_plan([_wp(46.95, 15.0, 30.0), _wp(46.96, 15.0, 30.0, command=22)])))
    mission = load_qgc_plan(path)
    assert len(mission.waypoints) == 1  # command 22 (takeoff) is not a waypoint


def test_rejects_non_plan(tmp_path: Path) -> None:
    path = tmp_path / "x.plan"
    path.write_text(json.dumps({"fileType": "Fence"}))
    with pytest.raises(PlanLoadError, match="not a QGC Plan"):
        load_qgc_plan(path)


def test_rejects_missing_home(tmp_path: Path) -> None:
    path = tmp_path / "x.plan"
    doc = _plan([_wp(46.95, 15.0, 30.0)])
    del doc["mission"]["plannedHomePosition"]
    path.write_text(json.dumps(doc))
    with pytest.raises(PlanLoadError, match="plannedHomePosition"):
        load_qgc_plan(path)


def test_rejects_no_waypoints(tmp_path: Path) -> None:
    path = tmp_path / "x.plan"
    path.write_text(json.dumps(_plan([])))
    with pytest.raises(PlanLoadError, match="no NAV_WAYPOINT"):
        load_qgc_plan(path)


def test_rejects_bad_json(tmp_path: Path) -> None:
    path = tmp_path / "x.plan"
    path.write_text("{ not json")
    with pytest.raises(PlanLoadError, match="invalid JSON"):
        load_qgc_plan(path)


def test_missing_file(tmp_path: Path) -> None:
    with pytest.raises(PlanLoadError, match="cannot read"):
        load_qgc_plan(tmp_path / "nope.plan")
