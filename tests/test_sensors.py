"""Tests for the covert sensor-mode policy."""

from __future__ import annotations

import pytest

from droneflight.sensors import (
    SensorMode,
    SensorPolicy,
    SensorSelectionError,
    SensorSuite,
    select_sensor_mode,
)


def test_day_uses_passive_optical() -> None:
    d = select_sensor_mode(is_night=False, suite=SensorSuite(optical=True))
    assert d.mode is SensorMode.PASSIVE_OPTICAL
    assert d.covert is True
    assert d.warnings == ()


def test_night_prefers_passive_thermal() -> None:
    d = select_sensor_mode(is_night=True, suite=SensorSuite(optical=True, thermal=True))
    assert d.mode is SensorMode.PASSIVE_THERMAL
    assert d.covert is True
    assert any("export-controlled" in w for w in d.warnings)


def test_night_covert_without_thermal_or_active_is_error() -> None:
    # Passive optical is night-blind; active lidar disallowed for covert ops.
    with pytest.raises(SensorSelectionError, match="no covert night sensor"):
        select_sensor_mode(is_night=True, suite=SensorSuite(optical=True, lidar=True))


def test_night_active_lidar_when_emission_allowed() -> None:
    d = select_sensor_mode(
        is_night=True,
        suite=SensorSuite(optical=True, lidar=True),
        policy=SensorPolicy(allow_active_emission=True),
    )
    assert d.mode is SensorMode.ACTIVE_LIDAR
    assert d.covert is False
    assert any("emits" in w for w in d.warnings)


def test_non_covert_night_uses_lidar() -> None:
    d = select_sensor_mode(
        is_night=True,
        suite=SensorSuite(optical=True, lidar=True),
        policy=SensorPolicy(prefer_covert=False),
    )
    assert d.mode is SensorMode.ACTIVE_LIDAR


def test_day_no_optical_falls_back_to_lidar_if_allowed() -> None:
    d = select_sensor_mode(
        is_night=False,
        suite=SensorSuite(optical=False, lidar=True),
        policy=SensorPolicy(allow_active_emission=True),
    )
    assert d.mode is SensorMode.ACTIVE_LIDAR


def test_day_no_usable_sensor_errors() -> None:
    with pytest.raises(SensorSelectionError, match="no usable daytime sensor"):
        select_sensor_mode(is_night=False, suite=SensorSuite(optical=False, lidar=False))
