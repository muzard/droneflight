"""Vertical-acceptance risk: what a mission asks PX4 to track, and how hard.

WHY THIS EXISTS
PX4 sequences a mission waypoint only when the vehicle is inside both the
horizontal (``NAV_ACC_RAD``) and the vertical (``NAV_MC_ALT_RAD``) acceptance
window. Miss it vertically and the sequencer stops advancing — and this does not
look like a failure: the trajectory generator keeps flying, sliding past the
waypoint until something else stops it. It froze a real mission at 11/79 on
2026-07-27, and at 51/78 twice more.

WHAT PREDICTS IT — and what does NOT (measured, TEST-PLAN B1, 2026-07-27)
The first version of this module counted **waypoints reached while descending**,
on the reasoning that a vehicle still coming down is a vehicle off its commanded
altitude. That predictor said the 282-waypoint below-canopy route (88 such
waypoints, against 2 on the route that tuned ``NAV_MC_ALT_RAD = 4.0``) would
almost certainly stall.

**It flew 282/282, and the predictor was measured worthless:**

| arrivals (274 scored, `sim-logs/canopy/21_38_21.ulg`) | mean | p95 | max |
|---|---|---|---|
| reached while **descending** | 0.165 m | 0.972 m | 1.817 m |
| reached **level or climbing** | 0.163 m | 0.950 m | 1.653 m |

Statistically identical. Correlation between a leg's altitude step and the error
at its endpoint: **-0.002**. So neither "is it descending" nor "how far does it
descend" predicts anything *within this route's range*.

What separates the two routes is the **size of the altitude step per leg**:
18-40 m on the CFIT route (which stalled), at most **8.56 m** here (which did
not). Densifying waypoints shrinks every step, and PX4 tracks small steps
trivially — mean error **0.164 m** here against 0.35 m over the sparse route.
**More waypoints made vertical tracking better, not worse.** That is the opposite
of what this module originally assumed, and it is why it now reports the *step
sizes a route demands* rather than counting arrivals.

⚠️ **Two flown routes is two data points.** The threshold below is drawn between
8.56 m (flew clean) and 18 m (stalled) — a wide gap with nothing in it. Treat
``STEEP_STEP_M`` as a screening line, not a calibrated limit.

⚠️ **This still does not predict the error in metres**, and B1 could not settle
why: the front-loaded-vs-linear descent disagreement between
``droneplanner.planner.terrain_profile`` (0.333) and
``configs/px4/terrain_following.params`` (0.040 on an 18 m step) needs a **long
leg with a big drop** to distinguish them, and this route has none (median leg
24.6 m). It matters less now — the planner no longer emits such legs — but it is
unresolved, not answered.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from droneflight.model import FlightMission

_EARTH_R = 6_371_000.0

# Below this, a leg's altitude change is bookkeeping noise, not a descent the
# vehicle has to fly down. Well under the tightest observed level-arrival error.
LEVEL_TOLERANCE_M = 0.1

# Altitude step above which a leg is worth flagging. Drawn between the two flown
# routes — 8.56 m max step completed 282/282 with 1.82 m worst error; 18-40 m
# steps stalled three missions. The gap between them is unsampled, so this is a
# screening line, not a calibrated limit.
STEEP_STEP_M = 12.0


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
        """Waypoints reached while descending, steepest first.

        ⚠️ Kept as a diagnostic, **not** as a risk metric: B1 measured these to be
        indistinguishable from level arrivals (0.165 m vs 0.163 m mean). See the
        module docstring.
        """
        return tuple(
            sorted(
                (a for a in self.arrivals if a.arrives_descending),
                key=lambda a: a.required_gradient,
                reverse=True,
            )
        )

    @property
    def steep(self) -> tuple[Arrival, ...]:
        """Legs whose altitude step exceeds :data:`STEEP_STEP_M`, largest first.

        This is the quantity that separated the route that stalled from the one
        that did not.
        """
        return tuple(
            sorted(
                (a for a in self.arrivals if a.descent_m > STEEP_STEP_M),
                key=lambda a: a.descent_m,
                reverse=True,
            )
        )

    @property
    def max_step_m(self) -> float:
        """Largest single-leg altitude step the route demands."""
        return max((a.descent_m for a in self.arrivals), default=0.0)


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
