"""What PX4 *actually* does at a corner, measured in flight.

WHY THIS EXISTS (acceptance §3, 2026-07-26 — `sim-logs/4.7/`)
`tether.py` used to score a mission's corner loads at the **commanded** speed:
the speed policy's slowdown in one arm, flat `cruise_speed_ms` in the control arm.
That control arm does not exist. PX4's multicopter mission-mode trajectory
generator decelerates into every waypoint on its own, scaled by how sharp the
turn is — so "what the fiber would see if the drone never slowed" was scoring a
profile no autopilot will ever fly.

Measured directly: the acceptance §3 control arm was commanded **6.0 m/s at all
32 waypoints** (verified from the ULog's `position_setpoint_triplet.cruising_speed`,
not from the CLI flag) and still flew every corner slower, monotonically in bend
angle — 4.96 m/s at 20°, 1.75 m/s at 90°, 0.46 m/s at a 160° reversal.

The consequence for the offline model was large and ran **both ways**: scoring at
flat cruise over-predicted the corner turn transient by 7-18x at bends >=100°,
and *under*-predicted it by 2-5x at bends <=35°, where the real in-loop peak
actually sits. It predicted 6 fiber snaps on a course that broke nothing.

WHAT THIS IS NOT
A model of PX4's source. It is a **calibration from one flight**, and its limits
are real:

* One airframe (gz x500), one PX4 build (v1.18-beta), **default mission params** —
  `MPC_XY_CRUISE`, acceptance radius and the jerk/accel limits all move this curve.
* One cruise speed (6.0 m/s) and one leg length (120 m), so whether the corner cap
  is **absolute** (a lateral-acceleration limit) or a **fraction of cruise** is
  UNRESOLVED. It is applied here as an absolute cap, which is the more physical
  reading, but settling it needs a second flight at a different cruise speed.
* Flat world. Terrain and wind can only make the real corner slower.

So: use it as a *prior* that replaces a demonstrably wrong assumption, not as a
guarantee. It is deliberately the conservative (faster) envelope of the
measurements — see `_MEASURED`.
"""

from __future__ import annotations

import bisect
import math

# Per-bend corner behaviour from the acceptance §3 control arm
# (`sim-logs/4.7/s3-cruise.csv`, commanded 6.0 m/s at *every* waypoint), as
# (course change deg, speed m/s, lateral accel m/s^2). Each entry is the mean of
# the arm's repeats at that bend; repeats agreed to ~0.1 m/s and ~0.1 m/s^2.
#
# BOTH columns are needed, and this is the whole lesson of §3:
#  * **speed** falls monotonically with bend — that is PX4 decelerating into the
#    corner uncommanded, and it dominates the *drag* term.
#  * **lateral accel** does NOT. It peaks near 2.9 m/s^2 at 45-50 deg (just under
#    PX4's 3.0 m/s^2 `MPC_ACC_HOR` default) and collapses to ~0.9 at a reversal,
#    because PX4 nearly stops and pivots rather than carving a reversal. It
#    dominates the *turn* term, and it is why the tether transient peaks at
#    moderate bends rather than at reversals.
# Calibrating speed alone leaves the turn term ~4x low; calibrating both
# reproduces the flown transient to ~2%.
#
# The speed column is made monotone non-increasing by a running max from the
# sharp end (a sharper turn cannot be flown faster, and for a break-risk model
# the faster reading is conservative). Only the 100 deg point moved,
# 1.39 -> 1.45 m/s, less than the measurement spread. The accel column is left
# exactly as measured — it is genuinely non-monotone and that is the finding.
_MEASURED: tuple[tuple[float, float, float], ...] = (
    (0.0, 6.00, 0.12),
    (20.0, 4.96, 2.75),
    (30.0, 3.98, 2.70),
    (35.0, 3.71, 2.73),
    (45.0, 3.26, 2.87),
    (50.0, 3.07, 2.87),
    (60.0, 2.69, 2.75),
    (70.0, 2.46, 2.40),
    (75.0, 2.24, 2.35),
    (80.0, 2.09, 2.18),
    (90.0, 1.75, 1.66),
    (95.0, 1.72, 1.47),
    (100.0, 1.45, 1.16),
    (110.0, 1.45, 1.52),
    (120.0, 1.15, 1.20),
    (135.0, 0.95, 1.07),
    (150.0, 0.58, 0.94),
    (155.0, 0.49, 0.80),
    (160.0, 0.46, 0.88),
    # Extrapolated to a true reversal; the course topped out at 160 deg.
    (180.0, 0.30, 0.80),
)

CALIBRATION_CRUISE_MS = 6.0
"""The cruise speed the table was measured at. See the module docstring."""

_BENDS = [b for b, _, _ in _MEASURED]
_SPEEDS = [v for _, v, _ in _MEASURED]
_ACCELS = [a for _, _, a in _MEASURED]


def _interp(deflection_deg: float, table: list[float]) -> float:
    if deflection_deg <= _BENDS[0]:
        return table[0]
    if deflection_deg >= _BENDS[-1]:
        return table[-1]
    i = bisect.bisect_left(_BENDS, deflection_deg)
    if _BENDS[i] == deflection_deg:
        return table[i]
    frac = (deflection_deg - _BENDS[i - 1]) / (_BENDS[i] - _BENDS[i - 1])
    return table[i - 1] + (table[i] - table[i - 1]) * frac


def px4_corner_speed_ms(deflection_deg: float) -> float:
    """The speed PX4 flies through a corner of ``deflection_deg``, uncommanded.

    Linear interpolation over the measured table; clamped at both ends. This is a
    *cap the autopilot imposes on itself*, independent of what speed was commanded.
    """
    return _interp(deflection_deg, _SPEEDS)


def px4_corner_lateral_accel_ms2(deflection_deg: float) -> float:
    """Lateral acceleration PX4 uses through a corner of ``deflection_deg``."""
    return _interp(deflection_deg, _ACCELS)


def flown_speed_ms(commanded_ms: float, deflection_deg: float) -> float:
    """The speed actually flown: the commanded speed, capped by PX4's own corner limit.

    This is the one function callers want. Commanding a *slower* speed than PX4
    would have flown still slows the vehicle (the policy arm did); commanding a
    faster one does not (the control arm proved it).
    """
    return min(commanded_ms, px4_corner_speed_ms(deflection_deg))


def flown_deflection_deg(
    vertex_bend_deg: float, speed_ms: float, coupling_length_m: float
) -> float:
    """Course change the *payout* sees at a corner — not the plan's vertex angle.

    A waypoint model reads a turn as the angle between two legs. The fiber sees a
    continuous trajectory, where the same turn is a heading rate sustained over
    the coupling length: ``(a_lat / v) * (L / v)`` radians. This is the identical
    bridge the Gazebo plugin applies in the loop (``TurnDeflectionDeg`` in
    `sitl/tether_plugin/tether_math.hpp`), so scoring offline through it makes the
    two models structurally the same rather than merely similarly-named.

    It matters because the two disagree badly: a 20 deg vertex was flown as a
    **49 deg** per-sample deflection, and every bend >= 100 deg saturated at 180.
    Using the vertex angle under-reads shallow corners — which is exactly where
    the flown transient peaks.
    """
    if speed_ms <= 0.0 or coupling_length_m <= 0.0:
        return 0.0
    if vertex_bend_deg <= 0.0:
        return 0.0
    accel = px4_corner_lateral_accel_ms2(vertex_bend_deg)
    if accel <= 0.0:
        return 0.0
    deg = math.degrees(accel * coupling_length_m / (speed_ms * speed_ms))
    return min(deg, 180.0)
