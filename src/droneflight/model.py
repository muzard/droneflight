"""Flight-mission data contracts.

A ``FlightMission`` is the flight stack's view of the planner's ``.plan``:
WGS84 waypoints with relative (AGL) altitude, plus the home position. Mission
items add the executed per-waypoint speed (from the tether-safe speed policy).
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class Waypoint(BaseModel):
    """A planned waypoint in WGS84, altitude relative to home (AGL)."""

    model_config = ConfigDict(frozen=True)

    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)
    alt_agl_m: float = Field(description="Altitude above the home/launch point (meters).")


class FlightMission(BaseModel):
    """A loaded, executable flight mission (from a QGC ``.plan``)."""

    model_config = ConfigDict(frozen=True)

    name: str
    home_lat: float = Field(ge=-90.0, le=90.0)
    home_lon: float = Field(ge=-180.0, le=180.0)
    home_alt_m: float = 0.0
    cruise_speed_ms: float = Field(gt=0)
    waypoints: tuple[Waypoint, ...]


class MissionItem(BaseModel):
    """A waypoint plus its executed speed (after the speed policy)."""

    model_config = ConfigDict(frozen=True)

    lat: float
    lon: float
    alt_agl_m: float
    speed_ms: float = Field(gt=0)
