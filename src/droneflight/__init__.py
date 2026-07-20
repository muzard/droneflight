"""droneflight — online flight stack for the fiber-laying drone.

Executes the offline planner's ``.plan`` (the ``droneplanner`` integration
boundary) on a PX4 autopilot over MAVLink/MAVSDK, for **GPS-denied, low-altitude,
tether-aware** flight. Adds a covert **sensor-mode policy** (passive optics vs
active lidar vs passive thermal) and a **tether-safe speed policy** (slow through
sharp turns/reversals so the paying-out fiber does not snag).

This is the hardware-dependent subsystem from ``Technology-Selection.md`` §3-4;
live runs target PX4 SITL + Gazebo (see ``RUNBOOK.md``). Milestone: **F0 — bridge
+ policies + executor scaffold** (SITL-ready; live loop pending the sim host).
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
