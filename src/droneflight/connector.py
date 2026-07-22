"""Autopilot connector interface + an in-memory fake for tests.

The executor drives the autopilot through this narrow async interface so its
orchestration/failsafe logic is testable without a live PX4 (the real adapter is
``MavsdkConnector``). Keeping the surface small also isolates the MAVSDK
dependency to one module.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from typing import Protocol, runtime_checkable

from droneflight.model import MissionItem


@runtime_checkable
class FlightConnector(Protocol):
    """Minimal async autopilot control surface."""

    async def connect(self, url: str, ready_timeout_s: float) -> None: ...
    async def upload_mission(self, items: list[MissionItem]) -> None: ...
    async def arm(self) -> None: ...
    async def start_mission(self) -> None: ...
    def mission_progress(self) -> AsyncIterator[tuple[int, int]]:
        """Yield (current_item, total_items) as the mission advances."""
        ...

    async def return_to_launch(self) -> None: ...
    async def disconnect(self) -> None: ...


class ConnectorError(RuntimeError):
    """A transport/command error from the autopilot connector."""


def not_ready_message(
    timeout_s: float, local_position_ok: bool | None, home_position_ok: bool | None
) -> str:
    """Explain a readiness timeout in terms of *which* health flag never came.

    "not ready within 30s" on its own sends you looking at the wrong thing. In a
    GPS-denied setup EKF2 has no position at all until an external-vision stream
    is fused, so a missing local position almost always means the EV bridge is
    not running — which is invisible from here, because the bridge fails before
    it ever reaches the autopilot.
    """
    if local_position_ok is None and home_position_ok is None:
        return (
            f"autopilot not ready within {timeout_s:.0f}s — no telemetry at all. "
            "Check the MAVLink URL and that PX4 is up."
        )
    detail = f"local_position_ok={local_position_ok}, home_position_ok={home_position_ok}"
    hint = ""
    if not local_position_ok:
        hint = (
            " GPS-denied: EKF2 has no position until external vision is fused — start "
            "`droneflight ev-bridge` first and wait for its est-pub readback to settle "
            "(RUNBOOK §3b.3). If the bridge exited, its error is on its own console."
        )
    elif not home_position_ok:
        hint = " Local position is good but home is not set; check the EKF2 global origin."
    return f"autopilot not ready within {timeout_s:.0f}s ({detail}).{hint}"


class FakeConnector:
    """Deterministic in-memory connector for testing the executor.

    Records the command sequence and simulates a mission that advances one item
    at a time. Configurable to fail at a chosen step to exercise the failsafe.
    """

    def __init__(
        self,
        fail_at: str | None = None,
        arm_raises: bool = False,
        stall_after: int | None = None,
    ) -> None:
        self.calls: list[str] = []
        self.uploaded: list[MissionItem] = []
        self._fail_at = fail_at
        self._arm_raises = arm_raises
        self._stall_after = stall_after

    def _step(self, name: str) -> None:
        self.calls.append(name)
        if self._fail_at == name:
            raise ConnectorError(f"simulated failure at {name}")

    async def connect(self, url: str, ready_timeout_s: float) -> None:
        self._step("connect")

    async def upload_mission(self, items: list[MissionItem]) -> None:
        self._step("upload_mission")
        self.uploaded = list(items)

    async def arm(self) -> None:
        self._step("arm")
        if self._arm_raises:
            raise ConnectorError("simulated arm failure")

    async def start_mission(self) -> None:
        self._step("start_mission")

    async def mission_progress(self) -> AsyncIterator[tuple[int, int]]:
        self._step("mission_progress")
        total = len(self.uploaded)
        for i in range(total):
            if self._fail_at == f"progress:{i}":
                raise ConnectorError(f"simulated failure during item {i}")
            if self._stall_after is not None and i >= self._stall_after:
                await asyncio.Event().wait()  # hang forever: exercises the watchdog
            yield i + 1, total

    async def return_to_launch(self) -> None:
        self._step("return_to_launch")

    async def disconnect(self) -> None:
        self._step("disconnect")
