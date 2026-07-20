"""Load a QGroundControl ``.plan`` into a ``FlightMission``.

This is the integration boundary with ``droneplanner``: it parses exactly the
``.plan`` the planner emits (fileType "Plan", PX4 firmware, NAV_WAYPOINT items
with ``[lat, lon, alt]`` in params[4:7], relative altitude).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from droneflight.model import FlightMission, Waypoint

_MAV_CMD_NAV_WAYPOINT = 16


class PlanLoadError(ValueError):
    """Raised when a .plan file cannot be parsed into a flight mission."""


def _waypoint_from_item(item: dict[str, Any]) -> Waypoint | None:
    if item.get("command") != _MAV_CMD_NAV_WAYPOINT:
        return None
    params = item.get("params")
    if not isinstance(params, list) or len(params) < 7:
        raise PlanLoadError(f"waypoint item has malformed params: {params!r}")
    lat, lon, alt = params[4], params[5], params[6]
    if lat is None or lon is None or alt is None:
        return None
    return Waypoint(lat=float(lat), lon=float(lon), alt_agl_m=float(alt))


def load_qgc_plan(path: str | Path, name: str | None = None) -> FlightMission:
    """Parse a QGC ``.plan`` file into a ``FlightMission``."""
    path = Path(path)
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise PlanLoadError(f"cannot read plan {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise PlanLoadError(f"invalid JSON in {path}: {exc}") from exc

    if doc.get("fileType") != "Plan":
        raise PlanLoadError(f"{path} is not a QGC Plan (fileType={doc.get('fileType')!r})")
    mission = doc.get("mission")
    if not isinstance(mission, dict):
        raise PlanLoadError(f"{path} has no mission block")

    home = mission.get("plannedHomePosition")
    if not isinstance(home, list) or len(home) < 3:
        raise PlanLoadError(f"{path} has no valid plannedHomePosition")

    waypoints = [
        wp
        for item in mission.get("items", [])
        if isinstance(item, dict) and (wp := _waypoint_from_item(item)) is not None
    ]
    if not waypoints:
        raise PlanLoadError(f"{path} contains no NAV_WAYPOINT items")

    try:
        return FlightMission(
            name=name or path.stem,
            home_lat=float(home[0]),
            home_lon=float(home[1]),
            home_alt_m=float(home[2]),
            cruise_speed_ms=float(mission.get("cruiseSpeed", 5.0)),
            waypoints=tuple(waypoints),
        )
    except Exception as exc:
        raise PlanLoadError(f"invalid mission in {path}: {exc}") from exc
