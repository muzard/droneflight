"""Tests for the droneflight CLI (offline validate path)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from droneflight.cli import app


def _write_plan(path: Path) -> None:
    doc = {
        "fileType": "Plan",
        "version": 1,
        "mission": {
            "cruiseSpeed": 6.0,
            "plannedHomePosition": [46.95, 15.0, 30.0],
            "items": [
                {"type": "SimpleItem", "command": 16, "params": [0, 0, 0, None, 46.95, 15.0, 30.0]},
                {"type": "SimpleItem", "command": 16, "params": [0, 0, 0, None, 46.96, 15.0, 32.0]},
            ],
        },
    }
    path.write_text(json.dumps(doc))


def test_validate_ok(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["validate", "--plan", str(plan)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "sensor mode: passive_optical" in out
    assert "2 waypoints" in out


def test_validate_night_without_thermal_errors(tmp_path: Path) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["validate", "--plan", str(plan), "--night"])
    assert rc == 3  # no covert night sensor in the default suite


def test_validate_bad_plan(tmp_path: Path) -> None:
    bad = tmp_path / "bad.plan"
    bad.write_text("{ not json")
    assert app(["validate", "--plan", str(bad)]) == 2


def test_simfly_reports_avoidance(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["simfly", "--plan", str(plan), "--density", "10", "--seed", "1"])
    out = capsys.readouterr().out
    assert rc in (0, 1)
    assert "avoidance OFF" in out
    assert "avoidance ON" in out
    assert "trunks" in out


def test_nav_reports_drift(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["nav", "--plan", str(plan), "--vtrn-interval", "500"])
    out = capsys.readouterr().out
    assert rc == 0
    assert "VIO only" in out
    assert "VIO + VTRN (idealized" in out
    assert "VIO + VTRN (optical" in out
    assert "avail" in out


def test_sitl_env_prints_plan_home(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["sitl-env", "--plan", str(plan)])
    out = capsys.readouterr().out
    assert rc == 0
    # Home comes from the plan (46.95), not PX4's Zurich default.
    assert "export PX4_HOME_LAT=46.95" in out
    assert "export PX4_HOME_LON=15.00" in out


def test_sitl_env_bad_plan(tmp_path: Path) -> None:
    bad = tmp_path / "bad.plan"
    bad.write_text("{ not json")
    assert app(["sitl-env", "--plan", str(bad)]) == 2


def test_nav_fidelity_flags(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["nav", "--plan", str(plan), "--forest-frac", "0.7", "--stale-frac", "0.3"])
    out = capsys.readouterr().out
    assert rc == 0
    assert "70% forest / 30% stale basemap" in out


def _write_track(path: Path, points: list[tuple[float, float]], err: float = 0.0) -> None:
    """A recorded ev-bridge run: `points` are TRUE (Gazebo) ENU positions."""
    lines = []
    for i, (e, n) in enumerate(points):
        lines.append(
            json.dumps(
                {
                    "t": i * 0.5,
                    "truth": {"e": e, "n": n, "u": 30.0},
                    "err": {"e": err, "n": 0.0},
                    "pub": {"n": n, "e": e + err, "d": -30.0, "yaw": 0.0},
                    "fix": "accepted" if i % 10 == 0 else None,
                }
            )
        )
    path.write_text("\n".join(lines) + "\n")


def test_ev_report_track_on_route_is_within_corridor(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    plan, track = tmp_path / "m.plan", tmp_path / "run.jsonl"
    _write_plan(plan)
    _write_track(track, [(0.0, float(n)) for n in range(0, 1000, 50)])
    rc = app(["ev-report", "--plan", str(plan), "--record", str(track)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "WITHIN corridor" in out
    assert "TRUE track vs plan" in out


def test_ev_report_offset_track_leaves_corridor(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    plan, track = tmp_path / "m.plan", tmp_path / "run.jsonl"
    _write_plan(plan)
    _write_track(track, [(40.0, float(n)) for n in range(0, 1000, 50)], err=40.0)
    rc = app(["ev-report", "--plan", str(plan), "--record", str(track)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "LEFT the corridor" in out
    assert "max 40.0 m" in out
    assert "injected error: final 40.0 m" in out


def test_ev_report_empty_record_errors(tmp_path: Path) -> None:
    plan, track = tmp_path / "m.plan", tmp_path / "run.jsonl"
    _write_plan(plan)
    track.write_text("")
    assert app(["ev-report", "--plan", str(plan), "--record", str(track)]) == 2


def test_format_bridge_stats_reports_fix_breakdown() -> None:
    from droneflight.cli import _format_bridge_stats
    from droneflight.ev_bridge import EvBridgeStats

    stats = EvBridgeStats(
        samples=100,
        published=50,
        distance_m=1234.0,
        max_error_m=12.5,
        final_error_m=3.25,
        attempts=20,
        accepted=8,
        no_features=9,
        low_confidence=1,
        gated_out=2,
        corrupt_accepted=1,
    )
    lines = _format_bridge_stats(stats)
    assert "flown 1234 m" in lines[0]
    assert "max 12.5 m, final 3.2 m" in lines[1]
    assert "8/20 accepted (40% avail)" in lines[2]
    assert "1 blunders slipped through" in lines[2]

    assert len(_format_bridge_stats(EvBridgeStats())) == 2  # no attempts -> no fix line


def test_live_monitor_throttles_and_shows_ekf_delta(capsys: pytest.CaptureFixture[str]) -> None:
    from droneflight.cli import _live_monitor
    from droneflight.ev_bridge import BridgeSample, VisionPose

    def sample(t: float, est: tuple[float, float, float] | None) -> BridgeSample:
        return BridgeSample(
            t_s=t,
            truth_enu=(1.0, 2.0, 30.0),
            error_en=(3.0, 4.0),
            published=VisionPose(2.0, 4.0, -30.0, 0.0, 0.0, 0.0),
            estimate_ned=est,
        )

    monitor = _live_monitor(interval_s=5.0)
    monitor(sample(0.0, (2.5, 4.0, -30.0)))
    monitor(sample(1.0, (2.5, 4.0, -30.0)))  # throttled away
    monitor(sample(6.0, None))
    lines = capsys.readouterr().out.strip().splitlines()
    assert len(lines) == 2
    assert "injected err=   5.0 m" in lines[0]
    assert "est-pub= 0.50 m" in lines[0]
    assert "est-pub=  n/a" in lines[1]


def _ev_args(**over: object) -> argparse.Namespace:
    base = dict(
        drift_rate=0.02,
        seed=0,
        vtrn=True,
        attempt_interval=50.0,
        forest_frac=0.4,
        stale_frac=0.1,
        fidelity_seed=1,
    )
    return argparse.Namespace(**{**base, **over})


def test_build_injector_no_vtrn_is_the_dead_reckoning_arm(tmp_path: Path) -> None:
    from droneflight.cli import _build_injector
    from droneflight.plan_loader import load_qgc_plan

    plan = tmp_path / "m.plan"
    _write_plan(plan)
    mission = load_qgc_plan(plan)

    with_vtrn = _build_injector(_ev_args(), mission)
    without = _build_injector(_ev_args(vtrn=False), mission)
    assert with_vtrn.reference is not None
    assert without.reference is None

    for _ in range(200):
        with_vtrn.advance(10.0, 0.0, 0.0)
        without.advance(10.0, 0.0, 0.0)
    assert with_vtrn.attempts > 0
    assert without.attempts == 0
