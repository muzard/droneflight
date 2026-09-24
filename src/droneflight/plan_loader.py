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
_MAV_CMD_DO_CHANGE_SPEED = 178


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


def flatten_plan_altitudes(doc: dict[str, Any], alt_m: float) -> tuple[dict[str, Any], int]:
    """Rewrite every waypoint altitude to a constant ``alt_m`` above home.

    **This destroys terrain-following** — it exists for one purpose: flying a
    terrain-shaped route in PX4's *flat* `default.sdf`. A planned route is
    relative-to-home, and over real terrain that goes *below* home wherever the
    ground drops away; in a flat world "below home" is underground, so the
    vehicle would fly into the ground for reasons that have nothing to do with
    what is being tested. Flattening keeps the horizontal route — the turns and
    corridor the concealment planner chose — and removes the vertical confound.

    Returns the rewritten document and the number of waypoints changed. Never
    use the output for anything but flat-world testing.
    """
    mission = doc.get("mission")
    if not isinstance(mission, dict):
        raise PlanLoadError("plan has no mission block")
    changed = 0
    for item in mission.get("items", []):
        if not isinstance(item, dict) or item.get("command") != _MAV_CMD_NAV_WAYPOINT:
            continue
        params = item.get("params")
        if isinstance(params, list) and len(params) >= 7 and params[6] is not None:
            params[6] = float(alt_m)
            changed += 1
    if not changed:
        raise PlanLoadError("plan contains no NAV_WAYPOINT items to flatten")
    return doc, changed


def load_plan_document(path: str | Path) -> dict[str, Any]:
    """Read a ``.plan`` as raw JSON (for transforms that must preserve the rest)."""
    path = Path(path)
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise PlanLoadError(f"cannot read plan {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise PlanLoadError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(doc, dict) or doc.get("fileType") != "Plan":
        raise PlanLoadError(f"{path} is not a QGC Plan")
    return doc


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

    # A DO_CHANGE_SPEED between two waypoints sets the speed of the leg leaving
    # the one before it, and persists until the next change (PX4 semantics).
    cruise = float(mission.get("cruiseSpeed", 5.0))
    current: float | None = None
    waypoints: list[Waypoint] = []
    for item in mission.get("items", []):
        if not isinstance(item, dict):
            continue
        if item.get("command") == _MAV_CMD_DO_CHANGE_SPEED:
            params = item.get("params")
            if isinstance(params, list) and len(params) >= 2 and params[1] is not None:
                speed = float(params[1])
                if speed > 0:
                    current = speed
                    if waypoints:
                        waypoints[-1] = waypoints[-1].model_copy(update={"speed_ms": speed})
            continue
        wp = _waypoint_from_item(item)
        if wp is not None:
            waypoints.append(wp.model_copy(update={"speed_ms": current}) if current else wp)
    if not waypoints:
        raise PlanLoadError(f"{path} contains no NAV_WAYPOINT items")
    # The planner's corridor widths ride in a side block PX4 never sees.
    extra = doc.get("droneplanner")
    widths = extra.get("corridor_half_width_m") if isinstance(extra, dict) else None
    if isinstance(widths, list) and len(widths) == len(waypoints):
        waypoints = [
            wp.model_copy(update={"corridor_half_width_m": float(w)}) if w is not None else wp
            for wp, w in zip(waypoints, widths, strict=True)
        ]

    try:
        return FlightMission(
            name=name or path.stem,
            home_lat=float(home[0]),
            home_lon=float(home[1]),
            home_alt_m=float(home[2]),
            cruise_speed_ms=cruise,
            waypoints=tuple(waypoints),
        )
    except Exception as exc:
        raise PlanLoadError(f"invalid mission in {path}: {exc}") from exc
