"""Match PX4 SITL's simulated world to a planned mission's home.

PX4 SITL spawns the vehicle at a *default* home (Zurich, 47.397/8.546) unless
``PX4_HOME_LAT`` / ``PX4_HOME_LON`` / ``PX4_HOME_ALT`` are exported before
``make px4_sitl``. A droneplanner ``.plan`` places its waypoints at the real AO,
which is nowhere near Zurich — so without this the uploaded mission either flies
hundreds of km "home to target" or is rejected outright. This is the number-one
first-flight bring-up trap; extracting home from the plan removes it.

Usage (on the Ubuntu VM, see ``RUNBOOK.md``)::

    eval "$(droneflight sitl-env --plan out/mission.plan)"
    cd PX4-Autopilot && make px4_sitl gz_x500

The env is derived purely from the loaded mission, so it is fully testable
without a sim.
"""

from __future__ import annotations

from droneflight.model import FlightMission


def px4_home_env(mission: FlightMission) -> dict[str, str]:
    """PX4 SITL home-position environment variables for ``mission``."""
    return {
        "PX4_HOME_LAT": f"{mission.home_lat:.8f}",
        "PX4_HOME_LON": f"{mission.home_lon:.8f}",
        "PX4_HOME_ALT": f"{mission.home_alt_m:.2f}",
    }


def format_env_exports(env: dict[str, str]) -> str:
    """Render an env mapping as a shell ``export`` block (for ``eval``)."""
    return "\n".join(f"export {key}={value}" for key, value in env.items())
