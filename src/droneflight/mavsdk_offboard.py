"""MAVSDK driver for the corridor executor — the untestable half (TEST-PLAN B2b).

Everything that can be decided without an autopilot lives in ``offboard.py`` and
is unit-tested. This file is only the adapter: telemetry in, setpoints out. It is
excluded from coverage for the same reason ``mavsdk_connector.py`` is — exercising
it needs a live PX4, so the honest place for logic is above this boundary, not in
it.

⚠️ **PX4 refuses to enter offboard mode without a setpoint stream already
running**, and drops out of it if the stream stalls for ~0.5 s. So the driver
sends a zero setpoint *before* asking for offboard, and the control loop must
keep up its rate or the vehicle falls back to the previous mode mid-flight.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import AsyncIterator

from droneflight.offboard import VelocityCommand

_log = logging.getLogger(__name__)


class MavsdkOffboardConnector:
    """Drives PX4 in offboard velocity mode via MAVSDK."""

    def __init__(self, control_hz: float = 10.0) -> None:
        self._system: object | None = None
        self._period_s = 1.0 / max(1.0, control_hz)

    @property
    def _drone(self):  # type: ignore[no-untyped-def]
        if self._system is None:
            raise RuntimeError("not connected")
        return self._system

    async def connect(self, url: str, ready_timeout_s: float) -> None:
        from mavsdk import System

        system = System()
        await system.connect(system_address=url)
        async for state in system.core.connection_state():
            if state.is_connected:
                break
        _log.info("connected to %s", url)

        async with asyncio.timeout(ready_timeout_s):
            async for health in system.telemetry.health():
                if health.is_local_position_ok and health.is_armable:
                    break
        self._system = system

    async def arm(self) -> None:
        await self._drone.action.arm()

    async def takeoff(self, altitude_m: float) -> None:
        await self._drone.action.set_takeoff_altitude(altitude_m)
        await self._drone.action.takeoff()
        # Climb out before handing over; offboard from the ground is a scramble.
        async for position in self._drone.telemetry.position():
            if position.relative_altitude_m >= altitude_m * 0.9:
                break

    async def start_offboard(self) -> None:
        from mavsdk.offboard import OffboardError, VelocityNedYaw

        # PX4 needs a setpoint stream BEFORE it will accept the mode change.
        await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await self._drone.offboard.start()
        except OffboardError as exc:
            raise RuntimeError(f"offboard rejected: {exc._result.result}") from exc
        _log.info("offboard engaged")

    async def set_velocity_ned(self, command: VelocityCommand) -> None:
        from mavsdk.offboard import VelocityNedYaw

        await self._drone.offboard.set_velocity_ned(
            VelocityNedYaw(command.north_ms, command.east_ms, command.down_ms, command.yaw_deg)
        )

    async def telemetry(self) -> AsyncIterator[tuple[float, float, float, float]]:
        """``(east_m, north_m, up_m, yaw_deg)`` at the control rate.

        Sampled rather than streamed: the loop must run at a fixed cadence to keep
        offboard alive, and PX4's telemetry topics arrive at their own rates.
        """
        while True:
            position = await anext(self._drone.telemetry.position_velocity_ned().__aiter__())
            attitude = await anext(self._drone.telemetry.attitude_euler().__aiter__())
            yield (
                position.position.east_m,
                position.position.north_m,
                -position.position.down_m,
                attitude.yaw_deg,
            )
            await asyncio.sleep(self._period_s)

    async def telemetry_full(
        self,
    ) -> AsyncIterator[tuple[float, float, float, float, float, float]]:
        """``(east, north, up, yaw_deg, v_east, v_north)`` — the local planner's feed.

        ``up`` is altitude relative to HOME (the plan's frame), not local NED z.

        Velocity lets it plan from where momentum is carrying the aircraft
        (`local_planner.LocalPlanner._plan_committed`).

        ⚠️ The streams are drained by background tasks that keep only the LATEST
        sample. Reading a persistent gRPC stream at 10 Hz while PX4 publishes
        faster lets it queue, and the planner would then be steering on a
        position seconds old — invisible in the numbers, fatal among trunks.
        """
        latest: dict[str, object] = {}

        async def drain(name: str, stream: AsyncIterator[object]) -> None:
            async for sample in stream:
                latest[name] = sample

        tasks = [
            asyncio.create_task(drain("pv", self._drone.telemetry.position_velocity_ned())),
            asyncio.create_task(drain("att", self._drone.telemetry.attitude_euler())),
            asyncio.create_task(drain("pos", self._drone.telemetry.position())),
        ]
        try:
            loop = asyncio.get_running_loop()
            while not {"pv", "att", "pos"} <= latest.keys():
                await asyncio.sleep(0.02)
            next_t = loop.time()
            while True:
                pv = latest["pv"]
                att = latest["att"]
                pos = latest["pos"]
                yield (
                    pv.position.east_m,  # type: ignore[attr-defined]
                    pv.position.north_m,  # type: ignore[attr-defined]
                    # 🔴 NOT -down_m. With external vision EKF2's local z sat
                    # 124.6 m below truth — exactly home AMSL, the origin the EV
                    # bridge sets — so the first GPS-denied fly-local climbed
                    # through 43 m chasing a 12 m target before it was aborted.
                    # The plan's altitudes are relative to home; so is this.
                    pos.relative_altitude_m,  # type: ignore[attr-defined]
                    att.yaw_deg,  # type: ignore[attr-defined]
                    pv.velocity.east_m_s,  # type: ignore[attr-defined]
                    pv.velocity.north_m_s,  # type: ignore[attr-defined]
                )
                next_t += self._period_s
                await asyncio.sleep(max(0.0, next_t - loop.time()))
        finally:
            for task in tasks:
                task.cancel()

    async def stop_offboard(self) -> None:
        with contextlib.suppress(Exception):
            await self._drone.offboard.stop()

    async def return_to_launch(self) -> None:
        await self._drone.action.return_to_launch()

    async def disconnect(self) -> None:
        self._system = None
