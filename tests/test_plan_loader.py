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


def test_do_change_speed_sets_the_leg_leaving_the_previous_waypoint(tmp_path: Path) -> None:
    """The planner's forest speed cap reaches the vehicle (2026-09-23): before, the
    .plan carried one cruise speed and the endurance verdict assumed another."""
    import json

    def wp(lat: float) -> dict[str, object]:
        return {"command": 16, "params": [0, 0, 0, None, lat, 15.0, 10.0]}

    def spd(v: float) -> dict[str, object]:
        return {"command": 178, "params": [1, v, -1, 0, 0, 0, 0]}

    doc = {
        "fileType": "Plan",
        "mission": {
            "cruiseSpeed": 6.0,
            "plannedHomePosition": [47.0, 15.0, 100.0],
            "items": [wp(47.0), spd(3.0), wp(47.001), wp(47.002), spd(6.0), wp(47.003)],
        },
    }
    path = tmp_path / "s.plan"
    path.write_text(json.dumps(doc))
    m = load_qgc_plan(path)
    assert [m.leg_speed_ms(i) for i in range(4)] == [3.0, 3.0, 6.0, 6.0]
    assert len(m.waypoints) == 4  # speed items are not waypoints


def test_corridor_widths_are_read_from_the_side_block(tmp_path: Path) -> None:
    import json

    wp = {"command": 16, "params": [0, 0, 0, None, 47.0, 15.0, 10.0]}
    doc = {
        "fileType": "Plan",
        "mission": {
            "cruiseSpeed": 6.0,
            "plannedHomePosition": [47.0, 15.0, 100.0],
            "items": [wp, wp],
        },
        "droneplanner": {"version": 1, "corridor_half_width_m": [0.0, 12.5]},
    }
    path = tmp_path / "c.plan"
    path.write_text(json.dumps(doc))
    m = load_qgc_plan(path)
    assert [w.corridor_half_width_m for w in m.waypoints] == [0.0, 12.5]
