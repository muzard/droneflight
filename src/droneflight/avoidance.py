"""Reactive obstacle avoidance (Technology-Selection §2.4, §3.3).

A local steering layer that sits on top of the global route: given a depth scan
and the direction to the next waypoint, it picks the freest heading that still
makes progress and slows (or hovers) when the way is tight. This is what keeps
the drone from hitting a tree the map missed or that drift nudged it toward — the
planner routes through navigable tubes, the avoidance layer handles what the plan
didn't see.

Method: a "follow-the-gap" / vector-field-histogram-lite policy — mask bearings
where an obstacle is within the reaction distance (angularly padded by the
airframe half-width), group what remains into gaps, then steer into one, with
speed proportional to the clearance ahead. Reactive layers have known local-minima
limitations; this is a safety layer, not a replacement for the global planner.

WHICH GAP, AND WHERE IN IT (TEST-PLAN N2, 2026-08-10 — this is the whole defect)
The original policy took **the free bearing closest to the goal**: the minimum
deviation that clears. Flown in SITL through 2500 trunks it hit 34 trees at
6.0 m/s and 41 at 2.0 m/s, and the post-mortem eliminated every other candidate —
the struck trunks were **dead ahead** (median 2.8-5.9 deg off yaw, 94 %/88 % inside
the FOV), the vehicle **did** turn for them (median 23-37 deg of course change),
and it almost never felt blocked (4/10 358 steps). It sees the tree, turns, and
hits it anyway.

Minimum deviation is why. It parks the commanded heading exactly on the edge of
the padded shadow, so:

* there is **no margin for the turn transient** — the airframe cannot change
  course instantly, so its real path cuts inside the commanded bearing and clips
  the trunk it was steering past;
* it will happily choose a one-ray slot between two trunks over a wide-open gap
  a few degrees further from the goal, and in a stand whose gaps are about one
  airframe wide that means dodging out of one trunk's shadow into its neighbour's.

Two fixes were proposed for that and **both were measured and rejected**, which is
why this module still ships the policy that collided:

* :attr:`AvoidanceConfig.gap_policy` ``"widest"`` — take the widest contiguous gap
  instead of the nearest-to-goal one. **3.3x worse** in the harness (243 vs 73
  strikes over six manifest x speed configurations). Taking the largest opening
  regardless of the goal walks the vehicle sideways across the stand, meeting more
  trunks per metre of progress than the cleaner dodge saves.
* :attr:`AvoidanceConfig.transient_margin_m` — hold the commanded heading clear of
  the gap edge so the turn transient has room. **A wash**: 35 -> 38 strikes.

Both knobs are kept, defaulted off, because this harness has twice predicted the
wrong *mechanism* for this layer (it forecast paralysis where the aircraft
collided, and predicted strikes would fall 27 -> 1-2 with speed where the flight
gave 34 -> 41). They are there to be flown, not to be trusted offline. **The real
defect is still open.**
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Literal

from droneflight.maneuver import ManeuverLimits, required_react_distance_m
from droneflight.perception import DepthScan


@dataclass(frozen=True)
class AvoidanceConfig:
    """Airframe + policy parameters for reactive avoidance."""

    drone_radius_m: float = 0.45  # half-width incl. props
    safety_margin_m: float = 0.5
    react_distance_override_m: float | None = None
    """Steer around obstacles within this range. **Leave None** — it is then
    derived from the flown speed and PX4's own accel/jerk limits
    (`maneuver.required_react_distance_m`).

    🔴 It used to be a constant 6.0 m, and TEST-PLAN B2 flew that into 19-37 tree
    strikes: at 6.0 m/s it buys 1.0 s, and the airframe needs 1.20 s just to graze
    past a trunk. Set this only to *reproduce* a historical run (the kinematic
    `simfly` sweeps were all measured at 6.0) — every such run is labelled.
    """

    obstacle_radius_m: float = 0.15
    """Radius of the thing being dodged, for the lateral-clearance budget. 0.15 m
    is the B2 trunk manifest's."""

    slowdown_distance_m: float = 5.0  # begin slowing when clearance drops below this
    cruise_speed_ms: float = 5.0
    min_speed_ms: float = 0.5
    limits: ManeuverLimits = field(default_factory=ManeuverLimits)

    gap_policy: Literal["widest", "nearest_goal"] = "nearest_goal"
    """Which free gap to steer into. See the module docstring.

    🔴 **Defaults to ``nearest_goal`` even though that is the policy N2 flew into
    34-41 tree strikes** — because ``widest`` was built to replace it and measured
    **3.3x worse** (243 vs 73 strikes over six manifest x speed configurations;
    dense at 6.0 m/s went 18 -> 74). Taking the largest opening regardless of the
    goal makes the vehicle traverse the stand sideways, so it meets more trunks per
    metre of progress than it saves by dodging cleanly. The knob is kept so the
    comparison can be re-run in the loop — the harness has been wrong about this
    layer's mechanism before — but nothing may be *shipped* on ``widest`` without
    a flight that beats ``nearest_goal``.
    """

    transient_margin_m: float = 0.0
    """Lateral room left between the commanded heading and the edge of the gap.

    The reasoning was sound: the airframe cannot turn instantly, so its real path
    cuts inside the commanded bearing, and aiming *at* the edge of a padded shadow
    should therefore clip the obstacle. **Measured, it is a wash** — 35 -> 38
    strikes under ``nearest_goal``. It rescues ``widest`` substantially (142 -> 101)
    but that policy is far behind regardless. Default 0.0 = off, pending a flight.

    Converted to an angle at the reaction distance, and always halved back if the
    gap is too narrow to afford it.
    """

    @property
    def safety_distance_m(self) -> float:
        return self.drone_radius_m + self.safety_margin_m

    @property
    def lateral_clearance_m(self) -> float:
        """Sideways offset needed to pass an obstacle: airframe + margin + its radius."""
        return self.drone_radius_m + self.safety_margin_m + self.obstacle_radius_m

    @property
    def react_distance_m(self) -> float:
        """Reaction distance actually used — derived unless explicitly overridden."""
        if self.react_distance_override_m is not None:
            return self.react_distance_override_m
        return required_react_distance_m(
            self.cruise_speed_ms, self.lateral_clearance_m, self.limits
        )

    @property
    def is_survivable(self) -> bool:
        """Whether the reaction distance in force can actually produce a miss.

        False means the configuration is geometrically unable to avoid a head-on
        obstacle, however good the steering is. That was true of every B2 flight.
        """
        return self.react_distance_m >= required_react_distance_m(
            self.cruise_speed_ms, self.lateral_clearance_m, replace(self.limits, safety_factor=1.0)
        )


@dataclass(frozen=True)
class AvoidanceCommand:
    """A reactive steering command."""

    desired_heading_rel: float  # radians relative to current heading
    speed_ms: float
    blocked: bool  # True if no free direction — hovering in place
    clearance_ahead_m: float


def _gaps(free: list[int], bearings: tuple[float, ...]) -> list[tuple[float, float]]:
    """Contiguous runs of free bearings, as ``(low, high)`` radians.

    Contiguity is by ray index, so two free sectors either side of a trunk are two
    gaps rather than one wide one — the distinction the whole policy turns on.
    """
    out: list[tuple[float, float]] = []
    start = prev = free[0]
    for j in [*free[1:], -1]:
        if j != prev + 1:
            out.append((bearings[start], bearings[prev]))
            start = j
        prev = j
    return out


def _distance_to_span(angle: float, span: tuple[float, float]) -> float:
    """Angular distance from ``angle`` to the nearest point of ``span`` (0 if inside)."""
    lo, hi = span
    if lo <= angle <= hi:
        return 0.0
    return min(abs(angle - lo), abs(angle - hi))


def avoid(scan: DepthScan, goal_bearing_rel: float, config: AvoidanceConfig) -> AvoidanceCommand:
    """Compute a steering command from a depth scan and the goal direction."""
    safety = config.safety_distance_m
    bearings = scan.bearings_rel
    n = len(bearings)

    # A bearing is to be avoided if an obstacle lies within the reaction distance
    # (lookahead) along it — steering early, not at the safety radius. Each such
    # ray masks the angular sector the airframe needs to pass it at that range.
    blocked = [r < config.react_distance_m for r in scan.ranges]
    padded = list(blocked)
    for i, r in enumerate(scan.ranges):
        if not blocked[i]:
            continue
        # ⚠️ Known under-pad, left in deliberately. This is airframe + margin
        # (0.95 m); passing a trunk actually needs `lateral_clearance_m` (1.10 m,
        # which adds the trunk's own radius), so every commanded heading aims
        # ~0.15 m too close — a 1.1 deg shortfall per side at the 7.8 m reaction
        # distance. Corrected to 1.10 m and measured, it is **worse**: 35 -> 40
        # strikes (sparse at 6.0 m/s improves 11 -> 5, but dense at 6.0 degrades
        # 18 -> 27). A wider pad masks more bearings, which narrows the gaps and
        # forces bigger deviations — and in this density deviation is the hazard.
        # Recorded rather than silently fixed; revisit if a flight ever shows the
        # harness is inverted here too.
        pad = math.atan2(config.drone_radius_m + config.safety_margin_m, max(r, 0.1))
        for j in range(n):
            if abs(bearings[j] - bearings[i]) <= pad:
                padded[j] = True

    free = [j for j in range(n) if not padded[j]]
    if not free:
        return AvoidanceCommand(
            desired_heading_rel=0.0, speed_ms=0.0, blocked=True, clearance_ahead_m=scan.min_range()
        )

    half_fov = scan.fov_rad / 2.0
    goal_clamped = max(-half_fov, min(half_fov, goal_bearing_rel))

    # Two independent decisions, deliberately separated so they can be A/B'd:
    # *which* gap to enter, and *where* in it to aim.
    gaps = _gaps(free, bearings)
    if config.gap_policy == "widest":
        lo, hi = max(gaps, key=lambda g: g[1] - g[0])
    else:
        lo, hi = min(gaps, key=lambda g: _distance_to_span(goal_clamped, g))

    # Stay clear of both edges so the turn transient has somewhere to overshoot
    # into: the airframe cuts inside the commanded bearing, so aiming *at* an edge
    # clips the obstacle that made it. A gap too narrow for the full margin gets
    # its centre, which is the best available compromise rather than an edge.
    margin = math.atan2(config.transient_margin_m, max(config.react_distance_m, 0.1))
    margin = min(margin, (hi - lo) / 2.0)
    chosen = max(lo + margin, min(hi - margin, goal_clamped))

    # Slow down in clutter: speed scales with the distance to the *nearest*
    # obstacle anywhere in view (not just ahead), so the drone eases through tight
    # spots even while steering around them.
    clearance = scan.min_range()
    frac = (clearance - safety) / (config.slowdown_distance_m - safety)
    frac = max(0.0, min(1.0, frac))
    speed = config.min_speed_ms + frac * (config.cruise_speed_ms - config.min_speed_ms)
    return AvoidanceCommand(
        desired_heading_rel=chosen, speed_ms=speed, blocked=False, clearance_ahead_m=clearance
    )
