"""Mission executor: fly a planner ``.plan`` on PX4 (Technology-Selection §3.6).

Orchestrates connect → upload → arm → start → monitor, applying the sensor-mode
policy and the tether-safe speed policy first. On any in-flight error it triggers
return-to-launch (configurable) rather than loitering. The autopilot is driven
through the injected ``FlightConnector`` so this logic is testable without a live
PX4; ``MavsdkConnector`` is the real adapter used against SITL/hardware.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import AsyncIterator
from dataclasses import dataclass, replace

from droneflight.config import FlightConfig
from droneflight.connector import ConnectorError, FlightConnector
from droneflight.model import FlightMission, MissionItem
from droneflight.sensors import SensorDecision, select_sensor_mode
from droneflight.speed_policy import apply_speed_policy

_log = logging.getLogger(__name__)


async def _monitored_progress(
    progress: AsyncIterator[tuple[int, int]], timeout_s: float
) -> AsyncIterator[tuple[int, int]]:
    """Yield mission progress, aborting if no update arrives within ``timeout_s``.

    A live PX4 that stalls (never advancing an item) would otherwise hang the
    monitor loop forever. This watchdog converts a stall into a ``ConnectorError``
    so the executor's failsafe (RTL) fires — the safe outcome for a real flight.
    """
    ait = aiter(progress)
    while True:
        try:
            async with asyncio.timeout(timeout_s):
                item = await anext(ait)
        except StopAsyncIteration:
            return
        except TimeoutError as exc:
            raise ConnectorError(f"no mission progress within {timeout_s:.0f}s") from exc
        yield item


@dataclass(frozen=True)
class FlightResult:
    """Outcome of a mission run."""

    completed: bool
    items_flown: int
    items_total: int
    sensor: SensorDecision
    aborted_reason: str | None = None
    rtl_triggered: bool = False


def prepare_items(mission: FlightMission, config: FlightConfig) -> list[MissionItem]:
    """Apply the tether-safe speed policy to a mission (pure, testable).

    The policy's cruise speed is aligned to the mission's cruise speed so the
    plan's intended pace is honored while turns are still slowed.
    """
    policy = replace(config.speed, cruise_speed_ms=mission.cruise_speed_ms)
    return apply_speed_policy(mission, policy)


async def fly_mission(
    connector: FlightConnector, mission: FlightMission, config: FlightConfig
) -> FlightResult:
    """Execute a mission end-to-end through the connector."""
    sensor = select_sensor_mode(config.night_operation, config.sensors_suite, config.sensors_policy)
    for warning in sensor.warnings:
        _log.warning("sensor: %s", warning)
    if config.night_operation and not sensor.covert:
        _log.warning("night mission is flying a NON-covert (emitting) sensor mode")

    items = prepare_items(mission, config)
    flown = 0
    total = len(items)

    try:
        await connector.connect(config.connection_url, config.ready_timeout_s)
        await connector.upload_mission(items)
        await connector.arm()
        await connector.start_mission()
        async for current, total in _monitored_progress(
            connector.mission_progress(), config.progress_timeout_s
        ):
            flown = current
            _log.info("mission progress: %d/%d", current, total)
    except (ConnectorError, TimeoutError) as exc:
        reason = str(exc)
        _log.error("mission aborted: %s", reason)
        rtl = False
        if config.rtl_on_error:
            try:
                await connector.return_to_launch()
                rtl = True
            except ConnectorError as rtl_exc:  # pragma: no cover - defensive
                _log.error("RTL failed: %s", rtl_exc)
        await _safe_disconnect(connector)
        return FlightResult(
            completed=False,
            items_flown=flown,
            items_total=total,
            sensor=sensor,
            aborted_reason=reason,
            rtl_triggered=rtl,
        )

    await _safe_disconnect(connector)
    return FlightResult(
        completed=flown >= total, items_flown=flown, items_total=total, sensor=sensor
    )


async def _safe_disconnect(connector: FlightConnector) -> None:
    try:
        await connector.disconnect()
    except ConnectorError:  # pragma: no cover - defensive
        _log.warning("disconnect failed")
