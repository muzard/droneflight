"""The GNSS gate in front of EKF2 (``ev-bridge --gnss``), exercised without a sim."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from droneflight.correction import CorrectionConfig
from droneflight.ev_bridge import (
    EvBridge,
    EvBridgeConfig,
    EvBridgeError,
    RecordingPublisher,
    ReplayTruthSource,
    TruthSample,
)
from droneflight.gnss_gate import (
    GnssGate,
    GnssGateConfig,
    RecordingGnssPublisher,
    gps_input_fields,
    project,
    reproject,
)
from droneflight.gnss_spoof import ConsistencyMonitor, SpoofAttack, simulate_spoof
from droneflight.nav import DriftConfig, ErrorInjector

HOME = (60.40, 25.30, 40.0)  # Askola-ish


def _truth(route_m: float = 3000.0, speed: float = 3.0, hz: float = 10.0) -> list[TruthSample]:
    n = int(route_m / speed * hz)
    return [
        TruthSample(t_s=i / hz, east_m=speed * i / hz, north_m=0.0, up_m=10.0) for i in range(n)
    ]


def _gate(attack: SpoofAttack | None, *, gated: bool = True) -> GnssGate:
    return GnssGate(
        GnssGateConfig(
            home_lat=HOME[0], home_lon=HOME[1], home_alt_m=HOME[2], attack=attack, gated=gated
        )
    )


async def _fly(gate: GnssGate, route_m: float = 3000.0) -> tuple[RecordingGnssPublisher, EvBridge]:
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=0), None)
    pub = RecordingGnssPublisher()
    bridge = EvBridge(injector, EvBridgeConfig(rate_hz=10.0), gnss=gate, gnss_publisher=pub)
    await bridge.run(ReplayTruthSource(_truth(route_m)), RecordingPublisher())
    return pub, bridge


def test_projection_is_pxs_and_round_trips() -> None:
    lat, lon = reproject(HOME[0], HOME[1], 1000.0, 0.0)
    # 1 km north on PX4's sphere (R = 6371 km) is 1000 / R radians of latitude.
    assert lat - HOME[0] == pytest.approx(math.degrees(1000.0 / 6371000.0), rel=1e-9)
    assert lon == pytest.approx(HOME[1], abs=1e-12)
    for n, e in ((0.0, 0.0), (2500.0, -3100.0), (-4000.0, 4000.0)):
        back = project(HOME[0], HOME[1], *reproject(HOME[0], HOME[1], n, e))
        assert back == pytest.approx((n, e), abs=1e-6)


def test_gps_input_is_a_healthy_3d_fix_in_mavlink_units() -> None:
    gate = _gate(None)
    fix = gate.step(0.0, (0.0, 0.0, 5.0), (0.0, 0.0), 0.0, (1.0, 2.0, 0.0))
    assert fix is not None
    doc = gps_input_fields(fix, 1.5)
    assert doc["fix_type"] == 3
    assert doc["lat"] == round(fix.lat * 1e7)
    assert doc["alt"] == pytest.approx(HOME[2] + 5.0)
    assert (doc["vn"], doc["ve"]) == (1.0, 2.0)
    assert doc["ignore_flags"] == 0


async def test_honest_gnss_flows_to_ekf2_for_the_whole_route() -> None:
    gate = _gate(None)
    pub, _ = await _fly(gate)
    assert gate.monitor.trusted
    assert gate.withheld == 0
    assert len(pub.fixes) == pytest.approx(1000 / 0.2 * 1.0, rel=0.01)  # 1000 s at 5 Hz


async def test_fast_carry_off_closes_the_gate_and_nothing_spoofed_gets_through() -> None:
    attack = SpoofAttack(onset_m=1000.0, pull_rate=0.5, bearing_deg=0.0)
    gate = _gate(attack)
    pub, _ = await _fly(gate)
    assert not gate.monitor.trusted
    assert 0.0 < gate.alarm_after_onset_m < 60.0
    assert gate.withheld > 0
    # Every fix EKF2 received carried at most the drag accrued before the alarm.
    worst = max(math.hypot(*f.spoof_en) for f in pub.fixes)
    assert worst <= 0.5 * gate.alarm_after_onset_m + 1e-9
    assert all(f.t_s <= gate.alarm_t_s for f in pub.fixes)


async def test_the_ungated_control_arm_keeps_publishing_but_still_records_the_alarm() -> None:
    attack = SpoofAttack(onset_m=1000.0, pull_rate=0.5)
    gate = _gate(attack, gated=False)
    pub, _ = await _fly(gate)
    assert not gate.monitor.trusted  # it WOULD have closed
    assert gate.withheld == 0
    assert max(math.hypot(*f.spoof_en) for f in pub.fixes) > 900.0


def test_spoof_bearing_is_a_compass_bearing_and_its_doppler_is_consistent() -> None:
    attack = SpoofAttack(onset_m=0.0, pull_rate=0.1, bearing_deg=90.0)
    gate = _gate(attack)
    fix = gate.step(0.0, (0.0, 0.0, 0.0), (0.0, 0.0), 100.0, (0.0, 3.0, 0.0))
    assert fix is not None
    assert fix.spoof_en == pytest.approx((10.0, 0.0), abs=1e-9)  # 10 m east
    # The spoofer's velocity drags east at pull_rate x ground speed.
    assert fix.vel_ned == pytest.approx((0.0, 3.3, 0.0), abs=1e-9)


def test_the_gate_refuses_vio_that_carries_vtrn_fixes() -> None:
    from droneflight.nav import route_reference
    from tests.test_ev_bridge import _mission

    reference = route_reference(
        _mission(), forest_frac=0.4, stale_frac=0.1, seed=1, home_relative=True
    )
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=0), reference, CorrectionConfig())
    with pytest.raises(EvBridgeError, match="raw VIO"):
        EvBridge(injector, gnss=_gate(None), gnss_publisher=RecordingGnssPublisher())


async def test_the_record_carries_the_gnss_state() -> None:
    gate = _gate(SpoofAttack(onset_m=100.0, pull_rate=0.5))
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=0), None)
    rows: list[dict[str, object]] = []
    bridge = EvBridge(
        injector,
        EvBridgeConfig(rate_hz=10.0),
        gnss=gate,
        gnss_publisher=RecordingGnssPublisher(),
        on_sample=lambda s: rows.append(s.to_dict()),
    )
    await bridge.run(ReplayTruthSource(_truth(600.0)), RecordingPublisher())
    assert any("gnss" in r for r in rows)
    assert rows[0]["gnss_trusted"] is True
    assert rows[-1]["gnss_trusted"] is False


def test_no_rewind_keeps_the_drag_accrued_inside_the_window() -> None:
    """EKF2 cannot be wound back, so its fallback carries the in-window drag.

    With drift-free VIO the end error is exactly the anchored offset (+ noise):
    the window-start anchor holds less of the spoof than the alarm-point one.
    (With drift the two can cancel by chance, so this is not a per-run bound.)
    """
    attack = SpoofAttack(onset_m=1000.0, pull_rate=0.1)
    for seed in range(5):
        rew = simulate_spoof(4000.0, attack, seed=seed, vio_rate=0.0)
        alarm = simulate_spoof(4000.0, attack, seed=seed, vio_rate=0.0, rewind=False)
        assert rew.detect_after_m == alarm.detect_after_m  # same test, same alarm
        assert alarm.error_at_end_m > rew.error_at_end_m + 5.0


def test_the_monitor_latches() -> None:
    mon = ConsistencyMonitor()
    for i in range(300):
        d = 5.0 * i
        mon.update(d, (0.0 if d < 500 else 2.0 * (d - 500), 0.0), (0.0, 0.0))
    assert not mon.trusted
    first = mon.alarm_at_m
    assert not mon.update(2000.0, (0.0, 0.0), (0.0, 0.0))
    assert mon.alarm_at_m == first


async def test_a_recorded_gate_run_reloads(tmp_path: Path) -> None:
    from droneflight.ev_bridge import TrackRecorder, load_track

    path = tmp_path / "gnss.jsonl"
    gate = _gate(SpoofAttack(onset_m=100.0, pull_rate=0.5))
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=0), None)
    with TrackRecorder(path) as rec:
        bridge = EvBridge(
            injector,
            EvBridgeConfig(rate_hz=10.0),
            gnss=gate,
            gnss_publisher=RecordingGnssPublisher(),
            recorder=rec,
        )
        await bridge.run(ReplayTruthSource(_truth(600.0)), RecordingPublisher())
    track = load_track(path)
    assert track[0].gnss_trusted is True
    assert track[-1].gnss_trusted is False
