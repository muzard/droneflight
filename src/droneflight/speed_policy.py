"""Tether-safe speed policy.

The paying-out fiber snags and breaks if the drone whips through a sharp turn or
reverses at speed, so the flight speed is reduced where the trajectory bends hard
(and most at reversals). This is the flight-layer counterpart to the planner's
kinematic report, which flags reversals as acceptable *only* at low payout speed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from droneflight.model import FlightMission, MissionItem

_EARTH_R = 6_371_000.0


@dataclass(frozen=True)
class SpeedPolicy:
    """Speed reduction as a function of trajectory bend angle."""

    cruise_speed_ms: float = 5.0
    min_speed_ms: float = 1.0
    slow_above_deg: float = 30.0  # start slowing when the bend exceeds this
    reversal_deg: float = 150.0  # near-reversal: hold min speed

    def __post_init__(self) -> None:
        if not (0.0 < self.min_speed_ms <= self.cruise_speed_ms):
            raise ValueError("require 0 < min_speed_ms <= cruise_speed_ms")


def _to_local_m(lat: float, lon: float, lat0: float) -> tuple[float, float]:
    x = math.radians(lon) * math.cos(math.radians(lat0)) * _EARTH_R
    y = math.radians(lat) * _EARTH_R
    return x, y


def _deflection_deg(prev: MissionItem, cur: MissionItem, nxt: MissionItem, lat0: float) -> float:
    ax, ay = _to_local_m(prev.lat, prev.lon, lat0)
    bx, by = _to_local_m(cur.lat, cur.lon, lat0)
    cx, cy = _to_local_m(nxt.lat, nxt.lon, lat0)
    u1 = (bx - ax, by - ay)
    u2 = (cx - bx, cy - by)
    l1 = math.hypot(*u1)
    l2 = math.hypot(*u2)
    if l1 == 0.0 or l2 == 0.0:
        return 0.0
    dot = (u1[0] * u2[0] + u1[1] * u2[1]) / (l1 * l2)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _speed_for_bend(deg: float, policy: SpeedPolicy) -> float:
    if deg <= policy.slow_above_deg:
        return policy.cruise_speed_ms
    if deg >= policy.reversal_deg:
        return policy.min_speed_ms
    # Linear taper from cruise (at slow_above_deg) to min (at reversal_deg).
    frac = (deg - policy.slow_above_deg) / (policy.reversal_deg - policy.slow_above_deg)
    return policy.cruise_speed_ms + frac * (policy.min_speed_ms - policy.cruise_speed_ms)


def apply_speed_policy(mission: FlightMission, policy: SpeedPolicy) -> list[MissionItem]:
    """Build mission items with per-waypoint tether-safe speeds."""
    items = [
        MissionItem(lat=w.lat, lon=w.lon, alt_agl_m=w.alt_agl_m, speed_ms=policy.cruise_speed_ms)
        for w in mission.waypoints
    ]
    lat0 = mission.home_lat
    out: list[MissionItem] = []
    for i, item in enumerate(items):
        if i == 0 or i == len(items) - 1:
            speed = policy.cruise_speed_ms
        else:
            deg = _deflection_deg(items[i - 1], item, items[i + 1], lat0)
            speed = _speed_for_bend(deg, policy)
        out.append(item.model_copy(update={"speed_ms": speed}))
    return out
