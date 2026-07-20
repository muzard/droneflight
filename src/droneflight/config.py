"""Flight configuration for GPS-denied, low-altitude, tether-aware execution."""

from __future__ import annotations

from dataclasses import dataclass, field

from droneflight.sensors import SensorPolicy, SensorSuite
from droneflight.speed_policy import SpeedPolicy


@dataclass(frozen=True)
class FlightConfig:
    """Top-level flight-stack configuration."""

    # MAVLink connection to PX4 (SITL default: udp://:14540).
    connection_url: str = "udp://:14540"
    # Seconds to wait for the autopilot to report ready before aborting.
    ready_timeout_s: float = 30.0
    # Abort/RTL if the mission stalls (no progress) for this long.
    progress_timeout_s: float = 60.0
    # Return to launch on any in-flight error rather than loitering.
    rtl_on_error: bool = True

    speed: SpeedPolicy = field(default_factory=SpeedPolicy)
    sensors_suite: SensorSuite = field(default_factory=SensorSuite)
    sensors_policy: SensorPolicy = field(default_factory=SensorPolicy)
    night_operation: bool = False
