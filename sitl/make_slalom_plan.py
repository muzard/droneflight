#!/usr/bin/env python3
"""Emit a turn-rich QGC `.plan` — a flight-stack test course, not a planned route.

WHY THIS EXISTS
Acceptance §3 (tether speed policy + disturbance transient at corners, RUNBOOK §4)
needs a route with real turns. The concealment planner cannot supply one: its
any-angle Theta* minimises turns and its kinematic fillet then spreads whatever
remains across many shallow per-vertex angles (measured: a slalom forced through
an obstacle comb still peaked at 24° per vertex, below the policy's 30° trigger —
the same mechanism that lets the duplicate-waypoint clusters defeat the policy).
See droneplanner SHORTFALLS.

So this course is authored directly, the way `ev_test.plan` is a purpose-built
flat-world test. It is a **flight-stack instrument** — it deliberately does NOT
go through the planner — with a controlled spectrum of turn angles at wide
spacing (no fillet needed), constant altitude for PX4's flat world. It exercises
whether the speed policy slows through turns and the tether transient rises at
corners, which is exactly what §3 measures.

The home is `ev_test.plan`'s, so the same PX4 SITL home injection works.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

# ev_test.plan's home, so `droneflight sitl-env` / run_sitl.sh place it identically.
HOME_LAT = 46.95611629378311
HOME_LON = 15.00410709725129
HOME_ALT_M = 124.68102264404297
CRUISE_MS = 6.0
ALT_AGL_M = 30.0  # constant — flat world
LEG_M = 120.0  # long legs so each turn is a single clean vertex, not a fillet arc
_EARTH_R = 6_371_000.0

# The turn spectrum, in degrees of heading change at each vertex (sign = L/R).
# Spans the policy's whole response: below 30 (no slow), the taper band, and
# past 150 (reversal -> min speed). Repeated so there is a decent sample.
TURN_SEQUENCE_DEG = [
    20,
    -35,
    50,
    -70,
    90,
    -110,
    135,
    -160,
    160,
    -135,
    110,
    -90,
    70,
    -50,
    35,
    -20,
    45,
    -45,
    155,
    -155,
    60,
    -80,
    100,
    -120,
    30,
    -30,
    150,
    -150,
    75,
    -95,
]


def enu_to_latlon(east_m: float, north_m: float) -> tuple[float, float]:
    lat = HOME_LAT + math.degrees(north_m / _EARTH_R)
    lon = HOME_LON + math.degrees(east_m / (_EARTH_R * math.cos(math.radians(HOME_LAT))))
    return lat, lon


def build_track() -> list[tuple[float, float]]:
    """Walk the turn sequence into ENU (east, north) vertices."""
    east = north = 0.0
    heading = math.pi / 2.0  # start heading north
    pts = [(east, north)]
    for turn_deg in TURN_SEQUENCE_DEG:
        heading += math.radians(turn_deg)
        east += LEG_M * math.cos(heading)
        north += LEG_M * math.sin(heading)
        pts.append((east, north))
    # one final straight leg so the last turn is a real interior vertex
    east += LEG_M * math.cos(heading)
    north += LEG_M * math.sin(heading)
    pts.append((east, north))
    return pts


def _item(seq: int, lat: float, lon: float) -> dict[str, object]:
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": ALT_AGL_M,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 16,  # MAV_CMD_NAV_WAYPOINT
        "doJumpId": seq,
        "frame": 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        "params": [0.0, 0.0, 0.0, None, lat, lon, ALT_AGL_M],
        "type": "SimpleItem",
    }


def build_plan() -> dict[str, object]:
    track = build_track()
    items = []
    for i, (e, n) in enumerate(track, start=1):
        lat, lon = enu_to_latlon(e, n)
        items.append(_item(i, lat, lon))
    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": CRUISE_MS,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5.0,
            "items": items,
            "plannedHomePosition": [HOME_LAT, HOME_LON, HOME_ALT_M],
            "vehicleType": 2,
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


if __name__ == "__main__":
    out = Path(__file__).resolve().parents[1].parent / "plans" / "slalom_test.plan"
    out.write_text(json.dumps(build_plan(), indent=4), encoding="utf-8")
    print(f"wrote {out}  ({len(TURN_SEQUENCE_DEG) + 2} waypoints)")
