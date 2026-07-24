"""Tests for the droneflight CLI (offline validate path)."""

from __future__ import annotations

import argparse
import json
import math
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


def _write_terrain_plan(path: Path) -> None:
    """A plan whose relative altitudes dip below home, as terrain-following does."""
    doc = {
        "fileType": "Plan",
        "version": 1,
        "mission": {
            "cruiseSpeed": 6.0,
            "plannedHomePosition": [46.95, 15.0, 124.7],
            "items": [
                {"type": "SimpleItem", "command": 16, "params": [0, 0, 0, None, 46.95, 15.0, 30.0]},
                {
                    "type": "SimpleItem",
                    "command": 16,
                    "params": [0, 0, 0, None, 46.96, 15.0, -41.9],
                },
                {"type": "SimpleItem", "command": 22, "params": [0, 0, 0, None, 46.95, 15.0, 5.0]},
            ],
        },
    }
    path.write_text(json.dumps(doc))


def test_validate_warns_about_waypoints_below_home(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    plan = tmp_path / "t.plan"
    _write_terrain_plan(plan)
    assert app(["validate", "--plan", str(plan)]) == 0
    out = capsys.readouterr().out
    assert "1/2 waypoints are BELOW the home point" in out
    assert "underground" in out


def test_flatten_plan_rewrites_only_nav_waypoints(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    from droneflight.plan_loader import load_qgc_plan

    plan, out = tmp_path / "t.plan", tmp_path / "flat.plan"
    _write_terrain_plan(plan)
    assert app(["flatten-plan", "--plan", str(plan), "--out", str(out), "--alt", "30"]) == 0
    assert "flattened 2 waypoints" in capsys.readouterr().out

    assert [w.alt_agl_m for w in load_qgc_plan(out).waypoints] == [30.0, 30.0]
    # The non-waypoint item (command 22 = TAKEOFF) is left alone.
    doc = json.loads(out.read_text())
    assert doc["mission"]["items"][2]["params"][6] == 5.0
    assert doc["mission"]["plannedHomePosition"] == [46.95, 15.0, 124.7]


def test_flatten_plan_rejects_a_plan_with_no_waypoints(tmp_path: Path) -> None:
    plan, out = tmp_path / "t.plan", tmp_path / "flat.plan"
    plan.write_text(json.dumps({"fileType": "Plan", "mission": {"items": []}}))
    assert app(["flatten-plan", "--plan", str(plan), "--out", str(out)]) == 2


def _write_track_with_modes(path: Path) -> None:
    """A run that flies the route, then RTLs straight home after an abort."""
    lines = []
    # On-route mission samples: hugging the plan (north from home).
    for i, n in enumerate(range(0, 1000, 50)):
        lines.append(
            json.dumps(
                {
                    "t": i * 0.5,
                    "truth": {"e": 2.0, "n": float(n), "u": 30.0},
                    "err": {"e": 1.0, "n": 0.0},
                    "pub": {"n": float(n), "e": 3.0, "d": -30.0, "yaw": 0.0},
                    "mode": "FlightMode.MISSION",
                }
            )
        )
    # Post-abort RTL: beelines home well off the planned polyline.
    for i, e in enumerate(range(0, 400, 50)):
        lines.append(
            json.dumps(
                {
                    "t": 100 + i * 0.5,
                    "truth": {"e": float(e), "n": 500.0, "u": 30.0},
                    "err": {"e": 1.0, "n": 0.0},
                    "pub": {"n": 500.0, "e": float(e), "d": -30.0, "yaw": 0.0},
                    "mode": "FlightMode.RETURN_TO_LAUNCH",
                }
            )
        )
    path.write_text("\n".join(lines) + "\n")


def test_ev_report_scores_the_mission_phase_not_the_rtl(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A post-abort RTL cuts across the route; scoring it buries the real result."""
    plan, track = tmp_path / "m.plan", tmp_path / "run.jsonl"
    _write_plan(plan)
    _write_track_with_modes(track)
    rc = app(["ev-report", "--plan", str(plan), "--record", str(track)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "[MISSION, 20]" in out
    assert "WITHIN corridor" in out  # 2 m off-route, not the 350 m RTL excursion
    assert "excluded samples in: FlightMode.RETURN_TO_LAUNCH" in out


def test_ev_report_warns_when_the_record_has_no_modes(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    plan, track = tmp_path / "m.plan", tmp_path / "run.jsonl"
    _write_plan(plan)
    _write_track(track, [(0.0, float(n)) for n in range(0, 1000, 50)])
    assert app(["ev-report", "--plan", str(plan), "--record", str(track)]) == 0
    out = capsys.readouterr().out
    assert "WHOLE RUN (no mode field)" in out
    assert "no flight-mode field" in out


def test_ev_report_compares_multiple_arms(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The A/B/C deliverable: one table, and a warning if the arms agree."""
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    good, bad = tmp_path / "vtrn.jsonl", tmp_path / "novtrn.jsonl"
    _write_track(good, [(1.0, float(n)) for n in range(0, 1000, 50)], err=1.0)
    _write_track(bad, [(80.0, float(n)) for n in range(0, 1000, 50)], err=80.0)

    rc = app(["ev-report", "--plan", str(plan), "--record", str(good), "--record", str(bad)])
    out = capsys.readouterr().out
    assert rc == 0
    lines = [ln for ln in out.splitlines() if ln.startswith(("vtrn", "novtrn"))]
    assert len(lines) == 2
    assert lines[0].endswith("WITHIN")
    assert lines[1].endswith("LEFT")
    assert "1 arm(s) held the corridor, 1 left it" in out


def test_ev_report_flags_arms_that_all_agree(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """All-pass or all-fail means the route decided it, not the navigation."""
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    a, b = tmp_path / "a.jsonl", tmp_path / "b.jsonl"
    for path in (a, b):
        _write_track(path, [(1.0, float(n)) for n in range(0, 1000, 50)], err=1.0)
    assert app(["ev-report", "--plan", str(plan), "--record", str(a), "--record", str(b)]) == 0
    assert "2 arm(s) held the corridor, 0 left it" in capsys.readouterr().out


def test_ev_report_reports_a_missing_record(tmp_path: Path) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    assert app(["ev-report", "--plan", str(plan), "--record", str(tmp_path / "nope.jsonl")]) == 2


def test_live_monitor_names_the_pre_origin_readback_instead_of_printing_it(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Guard the 5,421,242 m first sample seen in all four SITL runs.

    Before EKF2 has a global origin, `position_velocity_ned` reads out in another
    frame and the est-pub delta comes back in the millions. The RUNBOOK tells
    operators to watch this number, so printing it verbatim reliably starts a
    hunt for a bug that is not there.
    """
    from droneflight.cli import _live_monitor
    from droneflight.ev_bridge import BridgeSample, VisionPose

    def sample(t: float, est: tuple[float, float, float]) -> BridgeSample:
        return BridgeSample(
            t_s=t,
            truth_enu=(0.0, 0.0, 30.0),
            error_en=(0.0, 0.0),
            published=VisionPose(0.0, 0.0, -30.0, 0.0, 0.0, 0.0),
            estimate_ned=est,
        )

    monitor = _live_monitor(interval_s=5.0)
    monitor(sample(0.0, (-5089753.0, -1866623.125, -0.097)))  # the observed value
    monitor(sample(6.0, (0.4, 0.1, -30.0)))  # settled
    lines = capsys.readouterr().out.strip().splitlines()
    assert "no EKF2 origin yet" in lines[0]
    assert "5421242" not in lines[0]
    assert "est-pub= 0.41 m" in lines[1]


def test_live_monitor_flags_ev_dropout(capsys: pytest.CaptureFixture[str]) -> None:
    from droneflight.cli import _live_monitor
    from droneflight.ev_bridge import BridgeSample, VisionPose

    monitor = _live_monitor(interval_s=0.0)
    monitor(
        BridgeSample(
            t_s=1.0,
            truth_enu=(0.0, 0.0, 30.0),
            error_en=(1.0, 0.0),
            published=VisionPose(0.0, 0.0, -30.0, 0.0, 0.0, 0.0),
            ev_down=True,
        )
    )
    assert "[EV DOWN]" in capsys.readouterr().out


def test_format_bridge_stats_reports_dropout_and_yaw() -> None:
    from droneflight.cli import _format_bridge_stats
    from droneflight.ev_bridge import EvBridgeStats

    lines = _format_bridge_stats(
        EvBridgeStats(
            samples=100,
            published=90,
            dropout_events=3,
            dropout_suppressed=10,
            max_yaw_error_rad=math.radians(4.5),
        )
    )
    joined = "\n".join(lines)
    assert "3 lock losses, 10 publishes withheld (90.0% uptime)" in joined
    assert "yaw error: max 4.50" in joined

    # Silent when neither axis is enabled — the flown arms must not grow noise.
    assert len(_format_bridge_stats(EvBridgeStats(samples=10, published=10))) == 2


def test_build_injector_wires_the_yaw_axis(tmp_path: Path) -> None:
    from droneflight.cli import _build_injector
    from droneflight.plan_loader import load_qgc_plan

    plan = tmp_path / "m.plan"
    _write_plan(plan)
    mission = load_qgc_plan(plan)

    off = _build_injector(_ev_args(), mission)
    on = _build_injector(_ev_args(yaw_bias=7.0), mission)
    assert off.yaw_error_rad == 0.0
    assert on.yaw_error_rad == pytest.approx(math.radians(7.0))


def test_ev_bridge_parser_defaults_keep_both_new_axes_off() -> None:
    """The acceptance arms must be reproducible from the same command line."""
    from droneflight.cli import build_parser

    args = build_parser().parse_args(["ev-bridge", "--plan", "x.plan"])
    assert args.dropout_interval == 0.0
    assert args.yaw_bias == 0.0
    assert args.yaw_drift == 0.0


def test_ev_report_table_shows_uptime_and_yaw(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The comparison table must reveal whether each arm's axis actually engaged.

    Without these columns a dropout arm whose axis silently never fired looks
    identical to one that fired and changed nothing — which is exactly the
    ambiguity that made the first dropout/yaw result unreadable.
    """
    plan = tmp_path / "m.plan"
    _write_plan(plan)

    def record(name: str, ev_down: bool, yaw: float) -> Path:
        path = tmp_path / name
        rows = []
        for i in range(10):
            doc = {
                "t": i * 0.1,
                "truth": {"e": 0.0, "n": float(i), "u": 30.0},
                "err": {"e": 0.0, "n": 0.0},
                "pub": {"n": float(i), "e": 0.0, "d": -30.0, "yaw": 0.0},
                "mode": "MISSION",
            }
            if ev_down and i < 5:
                doc["ev"] = "down"
            if yaw:
                doc["yaw_err"] = yaw
            rows.append(json.dumps(doc))
        path.write_text("\n".join(rows) + "\n", encoding="utf-8")
        return path

    args = ["ev-report", "--plan", str(plan)]
    for name, down, yaw in (("clean", False, 0.0), ("dropped", True, 0.0), ("yawed", False, 0.1)):
        args += ["--record", str(record(f"{name}.jsonl", down, yaw))]

    assert app(args) == 0
    out = capsys.readouterr().out
    assert "EV up" in out and "yaw" in out
    assert "100.0%" in out  # the clean arm
    assert "50.0%" in out  # the dropout arm withheld half its publishes
    assert "5.7" in out  # 0.1 rad of yaw, in degrees
    # And the reader is warned not to read a flat corridor as "no effect".
    assert "near-blind to yaw" in out


def test_tether_check_reports_policy_validation(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    plan = tmp_path / "m.plan"
    _write_plan(plan)
    rc = app(["tether-check", "--plan", str(plan)])
    out = capsys.readouterr().out
    assert rc in (0, 1)
    assert "tether v0" in out
    assert "no policy (flat cruise)" in out
    assert "speed policy" in out


def test_tether_check_flags_a_break_and_exits_nonzero(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    # A reversal plan plus a tiny break budget: flat cruise must SNAP.
    doc = {
        "fileType": "Plan",
        "version": 1,
        "mission": {
            "cruiseSpeed": 6.0,
            "plannedHomePosition": [46.90, 15.0, 30.0],
            "items": [
                {"type": "SimpleItem", "command": 16, "params": [0, 0, 0, None, 46.90, 15.0, 30.0]},
                {
                    "type": "SimpleItem",
                    "command": 16,
                    "params": [0, 0, 0, None, 46.9018, 15.0, 30.0],
                },
                {
                    "type": "SimpleItem",
                    "command": 16,
                    "params": [0, 0, 0, None, 46.9002, 15.0, 30.0],
                },
            ],
        },
    }
    plan = tmp_path / "rev.plan"
    plan.write_text(json.dumps(doc))
    rc = app(["tether-check", "--plan", str(plan), "--break-tension", "1.0"])
    out = capsys.readouterr().out
    assert "SNAP" in out
    assert "prevents" in out  # the policy saves what flat cruise would break
    assert rc == 0  # policy arm stays within the limit even though flat cruise does not


def _write_tether_plugin_log(path: Path, *, enabled: int = 1, span: float = 40.0) -> None:
    """A CSV in the Gazebo plugin's log format — see sitl/tether_plugin/."""
    from droneflight.tether import TetherConfig, drag_tension_n

    config = TetherConfig(airborne_span_m=span)
    rows = []
    for i in range(20):
        drag = drag_tension_n(config, 6.0)
        tension = config.back_tension_n + drag
        rows.append(
            f"{i / 10!r},0.0,0.0,30.0,6.0,0.0,6.0,{drag!r},{tension!r},0.0,0.0,0.0,"
            f"{-tension!r},0.0,1"
        )
    path.write_text(
        "\n".join(
            [
                f"# droneflight tether plugin v0 enabled={enabled} link=x500/base_link",
                "# fiber_diameter_m=0.00025 drag_coefficient=1.2 air_density_kg_m3=1.225 "
                "airborne_span_m=40.0 back_tension_n=0.5 payout_inertia_kg=0.15 "
                "coupling_length_m=8.0 speed_floor_ms=0.3",
                "t_s,x_m,y_m,z_m,vx_ms,vy_ms,speed_ms,drag_n,tension_n,"
                "lateral_accel_ms2,deflection_deg,turn_n,fx_n,fy_n,moving",
                *rows,
            ]
        )
        + "\n"
    )


def test_tether_plugin_check_passes_a_faithful_log(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    log = tmp_path / "tether.csv"
    _write_tether_plugin_log(log)
    rc = app(["tether-plugin-check", "--log", str(log)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "EXACT" in out
    assert "20 samples" in out
    assert "0.7646" in out  # the applied peak, back-tension + drag at cruise


def test_tether_plugin_check_fails_a_divergent_log(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Built with a 60 m airborne span, logged a header claiming 40 m: the flight
    was disturbed by a model nobody analysed. Must exit non-zero."""
    log = tmp_path / "tether.csv"
    _write_tether_plugin_log(log, span=60.0)
    rc = app(["tether-plugin-check", "--log", str(log)])
    out = capsys.readouterr().out
    assert rc == 1
    assert "DIVERGED" in out


def test_tether_plugin_check_flags_a_disabled_control_arm(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    log = tmp_path / "tether.csv"
    _write_tether_plugin_log(log, enabled=0)
    app(["tether-plugin-check", "--log", str(log)])
    assert "DISABLED" in capsys.readouterr().out


def test_tether_plugin_check_reports_a_break(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    log = tmp_path / "tether.csv"
    _write_tether_plugin_log(log)
    app(["tether-plugin-check", "--log", str(log), "--break-tension", "0.5"])
    assert "would snap" in capsys.readouterr().out


def test_tether_plugin_check_missing_log(tmp_path: Path) -> None:
    assert app(["tether-plugin-check", "--log", str(tmp_path / "nope.csv")]) == 2
