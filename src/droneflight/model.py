"""Flight-mission data contracts.

A ``FlightMission`` is the flight stack's view of the planner's ``.plan``:
WGS84 waypoints with relative (AGL) altitude, plus the home position. Mission
items add the executed per-waypoint speed (from the tether-safe speed policy).
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class Waypoint(BaseModel):
    """A planned waypoint in WGS84, altitude **relative to the home point**.

    ⚠️ Frame matters: this is height above *home*, not above the terrain beneath
    the waypoint. They coincide only on flat ground. The planner thinks in
    terrain-relative AGL and converts on the way out (see droneplanner
    ``io/mavlink.py``); conflating the two flies the aircraft into hillsides.
    The ``alt_agl_m`` name is historical — read it as ``alt_rel_home_m``.
    """

    model_config = ConfigDict(frozen=True)

    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)
    alt_agl_m: float = Field(
        description="Altitude above the home/launch point (meters) — MAV_FRAME 3, not terrain AGL."
    )
    speed_ms: float | None = Field(
        default=None,
        gt=0,
        description=(
            "Planned ground speed for the leg LEAVING this waypoint, from the plan's "
            "MAV_CMD_DO_CHANGE_SPEED items (MAVSDK's `speed_m_s` has the same "
            "'after this item' meaning). None = the mission's cruise speed."
        ),
    )
    corridor_half_width_m: float | None = Field(
        default=None,
        ge=0,
        description=(
            "Lateral freedom the planner grants at this waypoint (its A1 concealment "
            "corridor), from the plan's `droneplanner` block. None = not stated."
        ),
    )


class FlightMission(BaseModel):
    """A loaded, executable flight mission (from a QGC ``.plan``)."""

    model_config = ConfigDict(frozen=True)

    name: str
    home_lat: float = Field(ge=-90.0, le=90.0)
    home_lon: float = Field(ge=-180.0, le=180.0)
    home_alt_m: float = 0.0
    cruise_speed_ms: float = Field(gt=0)
    waypoints: tuple[Waypoint, ...]

    def leg_speed_ms(self, index: int) -> float:
        """Planned speed on the leg leaving waypoint ``index`` (cruise if unstated)."""
        if 0 <= index < len(self.waypoints):
            v = self.waypoints[index].speed_ms
            if v is not None:
                return v
        return self.cruise_speed_ms


class MissionItem(BaseModel):
    """A waypoint plus its executed speed (after the speed policy)."""

    model_config = ConfigDict(frozen=True)

    lat: float
    lon: float
    alt_agl_m: float
    speed_ms: float = Field(gt=0)
