"""Reactive obstacle avoidance (Technology-Selection §2.4, §3.3).

A local steering layer that sits on top of the global route: given a depth scan
and the direction to the next waypoint, it picks the freest heading that still
makes progress and slows (or hovers) when the way is tight. This is what keeps
the drone from hitting a tree the map missed or that drift nudged it toward — the
planner routes through navigable tubes, the avoidance layer handles what the plan
didn't see.

Method: a "follow-the-gap" / vector-field-histogram-lite policy — mask bearings
where an obstacle is within the safety distance (angularly padded by the airframe
half-width), then steer to the free bearing closest to the goal, with speed
proportional to the clearance ahead. Reactive layers have known local-minima
limitations; this is a safety layer, not a replacement for the global planner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

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
        pad = math.atan2(config.drone_radius_m + config.safety_margin_m, max(r, 0.1))
        for j in range(n):
            if abs(bearings[j] - bearings[i]) <= pad:
                padded[j] = True

    free = [j for j in range(n) if not padded[j]]
    if not free:
        return AvoidanceCommand(
            desired_heading_rel=0.0, speed_ms=0.0, blocked=True, clearance_ahead_m=scan.min_range()
        )

    # Steer to the free bearing closest to the goal (clamped into the FOV).
    half_fov = scan.fov_rad / 2.0
    goal_clamped = max(-half_fov, min(half_fov, goal_bearing_rel))
    best = min(free, key=lambda j: abs(bearings[j] - goal_clamped))
    chosen = bearings[best]

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
