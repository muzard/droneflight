"""Perception sensor-mode policy (Technology-Selection §3.3, covert tradeoff).

Sensor choice is a **concealment tradeoff**, not a fixed "lidar is best":

* **Passive optical** (global-shutter stereo for VIO/VTRN) — emits nothing, so
  it is covert, but needs ambient light and degrades badly at night.
* **Active lidar** (3D lidar for LIO / obstacle avoidance) — works day or night
  and is robust in low-texture, foliage-heavy nap-of-the-earth flight, **but it
  emits**: the laser is visible to night-vision devices and laser-warning
  receivers, so it can defeat the whole "not seen" mission.
* **Passive thermal** (uncooled LWIR) — passive and night-capable, so covert at
  night, but lower resolution and **export-controlled** (ITAR/EAR).

This module picks a mode from the light condition and the operator's covert
priority, and surfaces the tradeoff (never silently emitting in a covert night
mission). Lidar is a *build option*, not a default.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum


class SensorMode(StrEnum):
    PASSIVE_OPTICAL = "passive_optical"  # covert, needs light
    PASSIVE_THERMAL = "passive_thermal"  # covert at night, export-controlled
    ACTIVE_LIDAR = "active_lidar"  # night-capable but emits (detectable)


@dataclass(frozen=True)
class SensorSuite:
    """Which sensors the airframe actually carries."""

    optical: bool = True
    thermal: bool = False
    lidar: bool = False


@dataclass(frozen=True)
class SensorPolicy:
    """Operator policy for perception under the concealment mission."""

    prefer_covert: bool = True
    allow_active_emission: bool = False  # permit ACTIVE_LIDAR even when covert


@dataclass(frozen=True)
class SensorDecision:
    mode: SensorMode
    covert: bool  # True if the chosen mode emits nothing detectable
    warnings: tuple[str, ...] = field(default_factory=tuple)


def select_sensor_mode(
    is_night: bool, suite: SensorSuite, policy: SensorPolicy | None = None
) -> SensorDecision:
    """Choose the perception mode for the current conditions.

    Day → passive optical (covert + capable). Night → prefer passive thermal if
    carried; else fall back to active lidar only if emission is permitted,
    flagging that it is detectable; else report that no covert night sensor
    exists (do not silently emit).
    """
    policy = policy or SensorPolicy()

    if not is_night:
        if suite.optical:
            return SensorDecision(mode=SensorMode.PASSIVE_OPTICAL, covert=True)
        if suite.lidar and (policy.allow_active_emission or not policy.prefer_covert):
            return _active_lidar_decision()
        raise SensorSelectionError("no usable daytime sensor in the suite")

    # Night.
    if suite.thermal:
        return SensorDecision(
            mode=SensorMode.PASSIVE_THERMAL,
            covert=True,
            warnings=("thermal is export-controlled (ITAR/EAR) — confirm compliance",),
        )
    if suite.lidar and (policy.allow_active_emission or not policy.prefer_covert):
        return _active_lidar_decision()
    raise SensorSelectionError(
        "night mission with no covert night sensor: passive optical is night-blind, "
        "no passive thermal carried, and active lidar is disallowed for covert ops "
        "(it emits a laser detectable by night-vision devices). "
        "Add thermal, or accept detection by enabling active emission."
    )


def _active_lidar_decision() -> SensorDecision:
    return SensorDecision(
        mode=SensorMode.ACTIVE_LIDAR,
        covert=False,
        warnings=(
            "ACTIVE lidar emits: its laser is visible to night-vision devices and "
            "laser-warning receivers — this compromises concealment",
        ),
    )


class SensorSelectionError(RuntimeError):
    """Raised when no acceptable sensor mode exists for the conditions/policy."""
