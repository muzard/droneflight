"""MAVSDK adapter implementing ``FlightConnector`` against a real PX4.

This is the live integration path (SITL or hardware); it is exercised in the sim
harness, not unit tests (no autopilot in CI), so it is excluded from coverage.
MAVSDK is imported lazily so the rest of the package works without it present.

GPS-denied note: PX4 flying on external vision (EKF2) reports position health via
local, not global, position. This adapter waits for connection + a local-position
fix; do not gate on ``is_global_position_ok`` in a GPS-denied setup.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from typing import Any

from droneflight.connector import ConnectorError, not_ready_message
from droneflight.model import MissionItem


class MavsdkConnector:  # pragma: no cover - requires a live autopilot
    """Drive a PX4 autopilot over MAVSDK."""

    def __init__(self) -> None:
        self._system: Any = None

    async def connect(self, url: str, ready_timeout_s: float) -> None:
        from mavsdk import System

        system = System()
        await system.connect(system_address=url)
        local_ok: bool | None = None
        home_ok: bool | None = None
        try:
            async with asyncio.timeout(ready_timeout_s):
                async for state in system.core.connection_state():
                    if state.is_connected:
                        break
                async for health in system.telemetry.health():
                    # GPS-denied: gate on local position, not global.
                    local_ok = health.is_local_position_ok
                    home_ok = health.is_home_position_ok
                    if local_ok and home_ok:
                        break
        except TimeoutError as exc:
            raise ConnectorError(not_ready_message(ready_timeout_s, local_ok, home_ok)) from exc
        self._system = system

    async def upload_mission(self, items: list[MissionItem]) -> None:
        from mavsdk.mission import MissionItem as MavItem
        from mavsdk.mission import MissionPlan

        system = self._require()
        mav_items = [
            MavItem(
                it.lat,
                it.lon,
                it.alt_agl_m,
                it.speed_ms,
                True,  # is_fly_through
                float("nan"),  # gimbal pitch
                float("nan"),  # gimbal yaw
                MavItem.CameraAction.NONE,
                float("nan"),  # loiter time
                float("nan"),  # camera photo interval
                float("nan"),  # acceptance radius
                float("nan"),  # yaw
                float("nan"),  # camera photo distance
                MavItem.VehicleAction.NONE,
            )
            for it in items
        ]
        try:
            # Clear first: PX4 keeps the previously-uploaded mission *and* its
            # current-item index, so `start_mission()` after a failed run resumes
            # mid-route (e.g. "11/77") instead of flying from the start. Clearing
            # makes a re-run idempotent without restarting the simulator.
            await system.mission.clear_mission()
            await system.mission.upload_mission(MissionPlan(mav_items))
            await system.mission.set_current_mission_item(0)
        except Exception as exc:
            raise ConnectorError(f"mission upload failed: {exc}") from exc

    async def arm(self) -> None:
        system = self._require()
        try:
            await system.action.arm()
        except Exception as exc:
            raise ConnectorError(f"arm failed: {exc}") from exc

    async def start_mission(self) -> None:
        system = self._require()
        try:
            await system.mission.start_mission()
        except Exception as exc:
            raise ConnectorError(f"start_mission failed: {exc}") from exc

    async def mission_progress(self) -> AsyncIterator[tuple[int, int]]:
        system = self._require()
        async for progress in system.mission.mission_progress():
            yield progress.current, progress.total
            if progress.current >= progress.total > 0:
                return

    async def return_to_launch(self) -> None:
        system = self._require()
        try:
            await system.action.return_to_launch()
        except Exception as exc:
            raise ConnectorError(f"RTL failed: {exc}") from exc

    async def disconnect(self) -> None:
        self._system = None

    def _require(self) -> Any:
        if self._system is None:
            raise ConnectorError("not connected")
        return self._system
