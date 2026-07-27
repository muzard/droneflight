"""How many times does a mission roll the vertical-acceptance dice?

WHY THIS EXISTS
PX4 sequences a mission waypoint only when the vehicle is inside both the
horizontal (``NAV_ACC_RAD``) and the vertical (``NAV_MC_ALT_RAD``) acceptance
window. Miss it vertically and the sequencer stops advancing — and this does not
look like a failure: the trajectory generator keeps flying, sliding past the
waypoint until something else stops it. It froze a real mission at 11/79 on
2026-07-27, and at 51/78 twice more.

Over 12 flights (`sim-logs/cfit/`, `cfit-rep/`, `cfit-rep2/`) one rule held with
no exceptions: **the mission stalls iff the vertical error at some waypoint
exceeds the window.** The error splits into two populations:

* waypoints the vehicle reaches **level** — 0.35 m mean, 0.64 m p95, worst 0.37 m
  on the flown route: never a problem;
* waypoints interior to a **descending** leg — usually 0.66-0.73 m, but spiking
  to **2.1-2.6 m in roughly 1 run in 6**. Those spikes are the stalls.

So the risk is not a property of a waypoint's geometry, it is a *count of
opportunities*: every waypoint reached while descending is another draw from that
heavy-tailed distribution. ``NAV_MC_ALT_RAD = 4.0`` covers the worst of 12 runs
(2.569 m) with 1.56x margin — but that margin was established over a route with
**two** such waypoints. A route with two hundred is a different bet entirely, and
that is what this module measures.

⚠️ **WHAT THIS DELIBERATELY DOES NOT DO: predict the error in metres.**
Two models of PX4's descent are on record here and they disagree:

* ``droneplanner.planner.terrain_profile`` front-loads at ``descent_rate /
  cruise_speed`` (0.333), justified by the 2026-07-24 crash leg, where 0.220 was
  measured;
* ``configs/px4/terrain_following.params``, written after 12 flights, records
  0.040 on an 18 m step and concludes PX4 descends throughout the leg, so no
  placement is "settled".

Those two numbers may not even be the same statistic — one reads like an initial
slope, the other like a leg average. Until a flight measures the altitude-vs-
along-track profile directly (that is part of B1's scoring), any metres-of-error
predictor would be fitting a model to an unresolved ambiguity, which is the
failure this project keeps paying for. **Counting exposure is robust to the
ambiguity; predicting magnitude is not.** So this counts.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from droneflight.model import FlightMission

_EARTH_R = 6_371_000.0

# Below this, a leg's altitude change is bookkeeping noise, not a descent the
# vehicle has to fly down. Well under the tightest observed level-arrival error.
LEVEL_TOLERANCE_M = 0.1


@dataclass(frozen=True)
class Arrival:
    """The vertical situation as the vehicle reaches one waypoint."""

    index: int
    leg_length_m: float
    descent_m: float
    """Altitude given up on the leg into this waypoint; 0 if level or climbing."""

    @property
    def required_gradient(self) -> float:
        """Metres down per metre forward if the descent were spread over the leg."""
        if self.leg_length_m <= 0.0:
            return 0.0
        return self.descent_m / self.leg_length_m

    @property
    def arrives_descending(self) -> bool:
        """True if the vehicle is still coming down when it gets here.

        Any descending leg counts. The 12-flight record found the vehicle
        descending at every interior point of one, so this is the conservative
        reading — and the honest one, given the model disagreement above.
        """
        return self.descent_m > LEVEL_TOLERANCE_M


@dataclass(frozen=True)
class ArrivalReport:
    """Vertical-acceptance exposure over a whole mission."""

    arrivals: tuple[Arrival, ...]

    @property
    def descending(self) -> tuple[Arrival, ...]:
        """Waypoints reached while descending, steepest first — the dice rolls."""
        return tuple(
            sorted(
                (a for a in self.arrivals if a.arrives_descending),
                key=lambda a: a.required_gradient,
                reverse=True,
            )
        )

    @property
    def exposure_count(self) -> int:
        return len(self.descending)

    def stall_free_probability(self, per_waypoint_risk: float) -> float:
        """Chance every descending waypoint clears, at a given per-waypoint failure rate.

        Independence is an assumption, and a shaky one — consecutive waypoints on
        the same leg share whatever transient causes a spike. Treat the result as
        an order of magnitude that shows how the bet scales with route density,
        not as a calibrated probability.
        """
        if not 0.0 <= per_waypoint_risk <= 1.0:
            raise ValueError("per_waypoint_risk must be in [0, 1]")
        return (1.0 - per_waypoint_risk) ** self.exposure_count


def _ground_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Equirectangular ground distance — exact enough over a few hundred metres."""
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    dx = math.radians(lon2 - lon1) * math.cos(mean_lat) * _EARTH_R
    dy = math.radians(lat2 - lat1) * _EARTH_R
    return math.hypot(dx, dy)


def assess_arrivals(mission: FlightMission) -> ArrivalReport:
    """Count how many waypoints the vehicle reaches while still descending."""
    arrivals: list[Arrival] = []
    wps = mission.waypoints
    for i in range(1, len(wps)):
        a, b = wps[i - 1], wps[i]
        arrivals.append(
            Arrival(
                index=i,
                leg_length_m=_ground_distance_m(a.lat, a.lon, b.lat, b.lon),
                descent_m=max(0.0, a.alt_agl_m - b.alt_agl_m),
            )
        )
    return ArrivalReport(arrivals=tuple(arrivals))
