"""Autopilot connector interface + an in-memory fake for tests.

The executor drives the autopilot through this narrow async interface so its
orchestration/failsafe logic is testable without a live PX4 (the real adapter is
``MavsdkConnector``). Keeping the surface small also isolates the MAVSDK
dependency to one module.
"""

from __future__ import annotations

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


class FakeConnector:
    """Deterministic in-memory connector for testing the executor.

    Records the command sequence and simulates a mission that advances one item
    at a time. Configurable to fail at a chosen step to exercise the failsafe.
    """

    def __init__(self, fail_at: str | None = None, arm_raises: bool = False) -> None:
        self.calls: list[str] = []
        self.uploaded: list[MissionItem] = []
        self._fail_at = fail_at
        self._arm_raises = arm_raises

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
            yield i + 1, total

    async def return_to_launch(self) -> None:
        self._step("return_to_launch")

    async def disconnect(self) -> None:
        self._step("disconnect")
