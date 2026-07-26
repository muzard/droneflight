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
    # Abort/RTL if the mission stalls (no progress) for this long. None = derive
    # it from the plan's longest leg (see executor.auto_progress_timeout_s); a
    # fixed value aborts healthy flights on long legs.
    progress_timeout_s: float | None = None
    # Return to launch on any in-flight error rather than loitering.
    rtl_on_error: bool = True

    speed: SpeedPolicy = field(default_factory=SpeedPolicy)
    # The tether-safe speed policy is on by default. Disabling it flies every
    # waypoint at cruise — the control arm for the in-loop speed-policy test
    # (RUNBOOK §4.7): with it off, the tether transient at corners is not damped.
    speed_policy_enabled: bool = True
    sensors_suite: SensorSuite = field(default_factory=SensorSuite)
    sensors_policy: SensorPolicy = field(default_factory=SensorPolicy)
    night_operation: bool = False
