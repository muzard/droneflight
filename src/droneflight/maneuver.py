"""How far ahead the aircraft must react to actually get out of the way.

WHY THIS EXISTS (TEST-PLAN B2, flown 2026-07-28 — ``sim-logs/b2/``)
``AvoidanceConfig.react_distance_m`` was a **constant 6.0 m**, tuned in a
kinematic harness where velocity changes instantly. Flown against real trunks in
Gazebo, the corridor executor hit **19 trees at 50 stems/ha and 37 at 107** while
reporting ``REACHED GOAL``, because at 6.0 m/s that constant buys 1.0 s of
warning and the airframe needs 1.20 s merely to *graze* past a trunk:

===========================================  ========
lateral offset to clear a trunk head-on       1.10 m
achievable in 6 m at 6 m/s                    0.66 m
===========================================  ========

A ratio of 0.60 is not a tuning miss — the vehicle was **geometrically incapable**
of avoiding a head-on trunk, and no amount of steering logic fixes that. The
reaction distance is not a free parameter: it is fixed by the speed you fly and
the acceleration the autopilot will give you.

WHAT THE MODEL IS
A multirotor cannot step its velocity. PX4 ramps acceleration at ``MPC_JERK_AUTO``
until it reaches ``MPC_ACC_HOR``, so lateral displacement from a standing start in
that axis is a jerk-limited profile, not ``½at²``. At the flown defaults
(3.0 m/s², 4.0 m/s³) the ramp alone consumes 0.75 s — which is why the aircraft
shed only 1.39 m/s over the final 6.4 m of an approach (~1.1 m/s², a third of the
3.0 nominally available).

WHAT THIS IS NOT
A guarantee, and not a claim about sensors. It answers *"given that I see a trunk
at range R, can I miss it?"*. Whether a real camera **detects** one at range R is
a separate question with a separate answer — see ``perception.py`` on angular
sampling and TEST-PLAN B4. Assuming a reaction distance the sensor cannot feed is
the same mistake in a new place.

Defaults are PX4's, read from the flown ULog rather than from documentation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

MPC_ACC_HOR = 3.0
"""PX4 default horizontal acceleration limit, m/s^2 (verified in `13_47_06.ulg`)."""

MPC_JERK_AUTO = 4.0
"""PX4 default auto-mode jerk limit, m/s^3 (verified in `13_47_06.ulg`)."""


@dataclass(frozen=True)
class ManeuverLimits:
    """The acceleration envelope the autopilot will actually deliver."""

    max_accel_ms2: float = MPC_ACC_HOR
    max_jerk_ms3: float = MPC_JERK_AUTO
    latency_s: float = 0.1
    """Sense -> command -> setpoint dead time. The executor's control period is
    10 Hz, and nothing moves during it."""

    safety_factor: float = 1.5
    """Applied to the *distance*, not the offset. 1.0 is a bare graze with the
    airframe's own safety margin and nothing else — one late detection, one gust
    or one under-performing axis and it is a strike."""

    def __post_init__(self) -> None:
        if self.max_accel_ms2 <= 0 or self.max_jerk_ms3 <= 0:
            raise ValueError("acceleration and jerk limits must be positive")
        if self.latency_s < 0 or self.safety_factor <= 0:
            raise ValueError("latency must be >= 0 and safety_factor > 0")


def lateral_offset_m(t_s: float, limits: ManeuverLimits | None = None) -> float:
    """Lateral displacement achievable ``t_s`` after committing to a dodge.

    Jerk-limited ramp to ``max_accel_ms2``, then constant acceleration. Starting
    lateral velocity is zero by construction: the aircraft is flying *along* the
    route when it detects something across it.
    """
    limits = limits or ManeuverLimits()
    if t_s <= 0.0:
        return 0.0
    t_ramp = limits.max_accel_ms2 / limits.max_jerk_ms3
    if t_s <= t_ramp:
        return limits.max_jerk_ms3 * t_s**3 / 6.0
    s_ramp = limits.max_jerk_ms3 * t_ramp**3 / 6.0
    v_ramp = limits.max_jerk_ms3 * t_ramp**2 / 2.0
    tau = t_s - t_ramp
    return s_ramp + v_ramp * tau + 0.5 * limits.max_accel_ms2 * tau**2


def time_to_offset_s(offset_m: float, limits: ManeuverLimits | None = None) -> float:
    """Time needed to displace ``offset_m`` sideways. Inverse of `lateral_offset_m`."""
    limits = limits or ManeuverLimits()
    if offset_m <= 0.0:
        return 0.0
    lo, hi = 0.0, 1.0
    while lateral_offset_m(hi, limits) < offset_m:
        hi *= 2.0
        if hi > 1e4:  # pragma: no cover - unreachable for any sane offset
            raise ValueError(f"offset {offset_m} m unreachable under {limits}")
    for _ in range(100):
        mid = (lo + hi) / 2.0
        if lateral_offset_m(mid, limits) < offset_m:
            lo = mid
        else:
            hi = mid
    return hi


def required_react_distance_m(
    speed_ms: float, lateral_clearance_m: float, limits: ManeuverLimits | None = None
) -> float:
    """How far ahead an obstacle must be seen for the dodge to complete in time.

    ``lateral_clearance_m`` is the *total* sideways offset needed to pass the
    obstacle: airframe half-width + safety margin + the obstacle's own radius.
    """
    limits = limits or ManeuverLimits()
    if speed_ms <= 0.0:
        return 0.0
    t = limits.latency_s + time_to_offset_s(lateral_clearance_m, limits)
    return speed_ms * t * limits.safety_factor


def max_speed_for_react_distance_ms(
    react_distance_m: float, lateral_clearance_m: float, limits: ManeuverLimits | None = None
) -> float:
    """The other lever: fastest speed at which ``react_distance_m`` is survivable.

    Use when the *sensor* caps how far ahead obstacles can be detected — then
    speed is the only remaining variable. Inverse of `required_react_distance_m`,
    which is linear in speed, so this is exact rather than iterative.
    """
    limits = limits or ManeuverLimits()
    if react_distance_m <= 0.0:
        return 0.0
    t = limits.latency_s + time_to_offset_s(lateral_clearance_m, limits)
    return react_distance_m / (t * limits.safety_factor)


def max_speed_for_spacing_ms(
    mean_spacing_m: float, lateral_clearance_m: float, limits: ManeuverLimits | None = None
) -> float:
    """Fastest speed at which *reactive* avoidance is coherent in a field of
    obstacles ``mean_spacing_m`` apart.

    THE POINT (TEST-PLAN B2, measured 2026-07-30)
    Required reaction distance grows with speed; mean stem spacing does not. Once
    the requirement exceeds the spacing there is never an instant with a clear
    gap, and a layer whose founding premise is "obstacles are isolated events you
    steer around" has nothing to steer into. Measured in the corrected harness on
    the real 107 stems/ha manifest, strikes track that ratio and nothing else:

    ====================  ==========  ==========  =======
    cruise                need        spacing/need  strikes
    ====================  ==========  ==========  =======
    6.0 m/s               7.82 m      0.59        27
    4.0 m/s               5.21 m      0.88         8
    3.0 m/s               3.91 m      1.18         9
    2.0 m/s               2.61 m      1.76         1
    ====================  ==========  ==========  =======

    Real managed forest is 400-1000 stems/ha = 1.8-2.8 m spacing, so this caps
    reactive flight there at roughly walking pace. Beyond that the answer is not a
    better reactive policy but routing *between* known stems.
    """
    limits = limits or ManeuverLimits(safety_factor=1.0)
    return max_speed_for_react_distance_ms(mean_spacing_m, lateral_clearance_m, limits)


def required_rays(fov_rad: float, range_m: float, obstacle_radius_m: float) -> int:
    """Rays needed so an obstacle of that radius cannot fall *between* two of them.

    The companion constraint to reaction distance, and the reason 6.0 m looked
    self-consistent: an obstacle subtends ``2*atan(r/range)``, so a scan sampling
    the FOV more coarsely than that simply misses it. At the flown 31 rays over
    90 deg (3.00 deg) a 0.15 m-radius trunk is resolved to ~6 m and detection
    falls to **60.9 %** by 15 m — measured against the B2 manifest along the flown
    track. Raising `react_distance_m` without raising this just moves the blind
    spot.
    """
    if range_m <= 0.0 or obstacle_radius_m <= 0.0 or fov_rad <= 0.0:
        raise ValueError("fov, range and obstacle radius must be positive")
    subtended = 2.0 * math.atan(obstacle_radius_m / range_m)
    return max(2, math.ceil(fov_rad / subtended) + 1)
