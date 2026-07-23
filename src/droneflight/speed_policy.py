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
    # Waypoints closer than this to their predecessor are treated as one point
    # when measuring a bend. Without it the planner's duplicate-waypoint clusters
    # (0.2-1.0 m apart, a known defect) smear every turn across sub-meter segments,
    # so no single waypoint shows a slow-worthy angle and the policy never fires —
    # measured 0/77 slowed on a real plan. See [[tether-v0]] / SHORTFALLS.
    consolidate_below_m: float = 2.0

    def __post_init__(self) -> None:
        if not (0.0 < self.min_speed_ms <= self.cruise_speed_ms):
            raise ValueError("require 0 < min_speed_ms <= cruise_speed_ms")
        if self.consolidate_below_m < 0.0:
            raise ValueError("consolidate_below_m must be non-negative")


def _to_local_m(lat: float, lon: float, lat0: float) -> tuple[float, float]:
    x = math.radians(lon) * math.cos(math.radians(lat0)) * _EARTH_R
    y = math.radians(lat) * _EARTH_R
    return x, y


def _speed_for_bend(deg: float, policy: SpeedPolicy) -> float:
    if deg <= policy.slow_above_deg:
        return policy.cruise_speed_ms
    if deg >= policy.reversal_deg:
        return policy.min_speed_ms
    # Linear taper from cruise (at slow_above_deg) to min (at reversal_deg).
    frac = (deg - policy.slow_above_deg) / (policy.reversal_deg - policy.slow_above_deg)
    return policy.cruise_speed_ms + frac * (policy.min_speed_ms - policy.cruise_speed_ms)


def consolidate_indices(
    points: list[tuple[float, float]], min_segment_m: float
) -> tuple[list[int], list[int]]:
    """Group consecutive near-duplicate points into distinct route vertices.

    Returns ``(reps, group_of)`` where ``reps`` are the indices of the retained
    representative points (the first of each cluster) and ``group_of[i]`` is the
    representative index that original point ``i`` belongs to. This is how a bend
    is measured on the *real* turn geometry rather than on the sub-meter segments
    of a duplicate-waypoint cluster.
    """
    if not points:
        return [], []
    reps = [0]
    group_of = [0]
    anchor = points[0]
    for i in range(1, len(points)):
        if math.hypot(points[i][0] - anchor[0], points[i][1] - anchor[1]) >= min_segment_m:
            reps.append(i)
            anchor = points[i]
        group_of.append(len(reps) - 1)
    return reps, group_of


def apply_speed_policy(mission: FlightMission, policy: SpeedPolicy) -> list[MissionItem]:
    """Build mission items with per-waypoint tether-safe speeds.

    Bends are measured on the route consolidated to >= ``consolidate_below_m``
    segments, so a turn represented as a cluster of near-duplicate waypoints is
    seen at its true angle and the whole cluster inherits the reduced speed.
    """
    lat0 = mission.home_lat
    pts = [_to_local_m(w.lat, w.lon, lat0) for w in mission.waypoints]
    reps, group_of = consolidate_indices(pts, policy.consolidate_below_m)

    # Speed per representative vertex: cruise at the route ends, else by its bend.
    rep_speeds: list[float] = []
    for j, idx in enumerate(reps):
        if j == 0 or j == len(reps) - 1:
            rep_speeds.append(policy.cruise_speed_ms)
        else:
            a, b, c = pts[reps[j - 1]], pts[idx], pts[reps[j + 1]]
            u1 = (b[0] - a[0], b[1] - a[1])
            u2 = (c[0] - b[0], c[1] - b[1])
            l1, l2 = math.hypot(*u1), math.hypot(*u2)
            if l1 == 0.0 or l2 == 0.0:
                rep_speeds.append(policy.cruise_speed_ms)
                continue
            dot = (u1[0] * u2[0] + u1[1] * u2[1]) / (l1 * l2)
            deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
            rep_speeds.append(_speed_for_bend(deg, policy))

    out: list[MissionItem] = []
    for w, g in zip(mission.waypoints, group_of, strict=True):
        out.append(MissionItem(lat=w.lat, lon=w.lon, alt_agl_m=w.alt_agl_m, speed_ms=rep_speeds[g]))
    return out
