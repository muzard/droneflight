"""Tests for the external-vision injection bridge.

None of this needs Gazebo or PX4: the truth source and the publisher are both
protocols, so the loop, the frame conversions, the message parsing and the
corridor analysis are all exercised on the Mac. What is *not* covered here is the
live MAVSDK publish and the real ``gz topic`` subprocess (RUNBOOK §3b).
"""

from __future__ import annotations

import asyncio
import json
import math
from itertools import pairwise
from pathlib import Path

import pytest

from droneflight.correction import CorrectionConfig
from droneflight.ev_bridge import (
    BridgeSample,
    EvBridge,
    EvBridgeConfig,
    EvBridgeError,
    GazeboTopic,
    RecordingPublisher,
    ReplayTruthSource,
    TrackRecorder,
    TruthSample,
    _JsonStreamSplitter,
    analyse_track,
    discover_gz_topics,
    enu_flu_to_ned_frd_euler,
    enu_to_ned,
    load_track,
    parse_truth,
    plan_track_enu,
    quat_to_euler,
    resolve_truth_topic,
)
from droneflight.model import FlightMission, Waypoint
from droneflight.nav import DriftConfig, ErrorInjector, _densify, route_reference

# --------------------------------------------------------------------------- #
# Frames
# --------------------------------------------------------------------------- #


def test_enu_to_ned_swaps_axes_and_flips_up() -> None:
    assert enu_to_ned(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)


def _yaw_quat(psi: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a rotation of ``psi`` about Z."""
    return 0.0, 0.0, math.sin(psi / 2.0), math.cos(psi / 2.0)


@pytest.mark.parametrize(
    ("enu_yaw", "expected_ned_yaw"),
    [
        (0.0, math.pi / 2),  # ENU heading east  -> NED 90 deg from north
        (math.pi / 2, 0.0),  # ENU heading north -> NED 0
        (math.pi, -math.pi / 2),  # ENU heading west  -> NED -90
        (-math.pi / 2, math.pi),  # ENU heading south -> NED 180
    ],
)
def test_yaw_enu_to_ned(enu_yaw: float, expected_ned_yaw: float) -> None:
    _r, _p, yaw = quat_to_euler(*_yaw_quat(enu_yaw))
    _, _, ned_yaw = enu_flu_to_ned_frd_euler(0.0, 0.0, yaw)
    assert math.isclose(abs(ned_yaw), abs(expected_ned_yaw), abs_tol=1e-9)


def test_pitch_flips_sign_roll_does_not() -> None:
    roll, pitch, _ = enu_flu_to_ned_frd_euler(0.3, 0.2, 0.0)
    assert roll == pytest.approx(0.3)
    assert pitch == pytest.approx(-0.2)


def test_quat_to_euler_normalises_and_handles_identity() -> None:
    assert quat_to_euler(0.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)  # degenerate -> identity
    assert quat_to_euler(0.0, 0.0, 0.0, 2.0) == (0.0, 0.0, 0.0)  # unnormalised


# --------------------------------------------------------------------------- #
# Reading gz topic output
# --------------------------------------------------------------------------- #


def test_json_splitter_compact_and_pretty() -> None:
    s = _JsonStreamSplitter()
    assert s.feed('{"a":1}{"b":2}') == ['{"a":1}', '{"b":2}']
    assert s.feed('{\n  "c": {\n    "d": 3\n  }\n}\n') == ['{\n  "c": {\n    "d": 3\n  }\n}']


def test_json_splitter_across_chunk_boundaries() -> None:
    s = _JsonStreamSplitter()
    assert s.feed('{"a":') == []
    assert s.feed('{"b":1}') == []
    out = s.feed("}")
    assert len(out) == 1
    assert json.loads(out[0]) == {"a": {"b": 1}}


def test_json_splitter_ignores_braces_inside_strings() -> None:
    s = _JsonStreamSplitter()
    out = s.feed('{"name":"x500{0}","esc":"a\\"}b"}')
    assert len(out) == 1
    assert json.loads(out[0])["name"] == "x500{0}"


_ODOM_TOPIC = GazeboTopic("/model/x500_0/odometry", "odometry", "x500_0")
_POSE_TOPIC = GazeboTopic("/world/default/pose/info", "pose_v", "x500_0")


def test_parse_odometry_with_omitted_default_fields() -> None:
    # Protobuf JSON drops zero-valued fields: a drone exactly on the world
    # x-axis has no "y" key at all. Reading that as missing (not as an error)
    # is the difference between a correct pose and a silent frame bug.
    doc = json.loads(
        '{"header":{"stamp":{"sec":"12","nsec":500000000}},'
        '"pose":{"position":{"x":3.5},"orientation":{"w":1}}}'
    )
    sample = parse_truth(doc, _ODOM_TOPIC)
    assert sample is not None
    assert sample.t_s == pytest.approx(12.5)
    assert (sample.east_m, sample.north_m, sample.up_m) == (3.5, 0.0, 0.0)
    assert sample.qw == 1.0


def test_parse_pose_v_picks_the_named_model() -> None:
    doc = {
        "header": {"stamp": {"sec": 4}},
        "pose": [
            {"name": "ground_plane", "position": {"x": 99.0}},
            {"name": "x500_0", "position": {"x": 1.0, "y": 2.0, "z": 3.0}},
        ],
    }
    sample = parse_truth(doc, _POSE_TOPIC)
    assert sample is not None
    assert (sample.east_m, sample.north_m, sample.up_m) == (1.0, 2.0, 3.0)
    assert sample.t_s == pytest.approx(4.0)


def test_parse_pose_v_returns_none_when_model_absent() -> None:
    assert parse_truth({"pose": [{"name": "other"}]}, _POSE_TOPIC) is None
    assert parse_truth({"nonsense": 1}, _ODOM_TOPIC) is None


def test_resolve_truth_topic_prefers_model_odometry() -> None:
    topic = resolve_truth_topic(
        available=["/world/default/pose/info", "/model/x500_0/odometry", "/clock"]
    )
    assert topic.topic == "/model/x500_0/odometry"
    assert topic.kind == "odometry"


def test_resolve_truth_topic_falls_back_to_world_pose() -> None:
    topic = resolve_truth_topic(available=["/clock", "/world/default/pose/info"])
    assert topic.kind == "pose_v"


def test_resolve_truth_topic_error_lists_what_it_saw() -> None:
    with pytest.raises(EvBridgeError, match="no ground-truth topic"):
        resolve_truth_topic(available=["/clock"])


# --------------------------------------------------------------------------- #
# The loop
# --------------------------------------------------------------------------- #


def _straight_truth(n: int = 200, step_m: float = 5.0, dt: float = 0.1) -> list[TruthSample]:
    """Fly due north from the origin at ground truth (ENU)."""
    return [
        TruthSample(t_s=i * dt, east_m=0.0, north_m=i * step_m, up_m=30.0, qw=1.0) for i in range(n)
    ]


def _mission() -> FlightMission:
    return FlightMission(
        name="t",
        home_lat=46.90,
        home_lon=15.0,
        cruise_speed_ms=6.0,
        waypoints=(
            Waypoint(lat=46.90, lon=15.0, alt_agl_m=30.0),
            Waypoint(lat=46.91, lon=15.0, alt_agl_m=30.0),
        ),
    )


async def _run(injector: ErrorInjector, samples: list[TruthSample], **kw: object) -> tuple:
    bridge = EvBridge(injector, EvBridgeConfig(**kw))  # type: ignore[arg-type]
    pub = RecordingPublisher()
    stats = await bridge.run(ReplayTruthSource(samples), pub)
    return stats, pub


async def test_published_pose_is_truth_plus_error_in_ned() -> None:
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=3))
    stats, pub = await _run(injector, _straight_truth(), rate_hz=0.0)
    assert stats.published == stats.samples == 200
    # First sample: nothing flown yet, so the pose is clean truth in NED.
    first = pub.poses[0]
    assert (first.north_m, first.east_m, first.down_m) == (0.0, 0.0, -30.0)
    # Last: north/east carry the injected error, altitude is still truth.
    err_e, err_n = injector.error
    last = pub.poses[-1]
    assert last.north_m == pytest.approx(199 * 5.0 + err_n)
    assert last.east_m == pytest.approx(err_e)
    assert last.down_m == pytest.approx(-30.0)
    assert math.hypot(err_e, err_n) > 1.0  # ~1 km at 2% drift


async def test_track_records_truth_not_the_corrupted_pose() -> None:
    injector = ErrorInjector(DriftConfig(rate=0.05, seed=1))
    stats, _ = await _run(injector, _straight_truth(), rate_hz=0.0)
    assert stats.track_enu[-1] == (0.0, 199 * 5.0)  # exactly the ground truth
    assert stats.final_error_m > 5.0  # ...while the injected error is large


async def test_rate_limit_downsamples_in_sim_time() -> None:
    # Truth at 10 Hz, publish at 2 Hz -> roughly every fifth sample.
    stats, pub = await _run(
        ErrorInjector(DriftConfig(seed=0)), _straight_truth(n=100, dt=0.1), rate_hz=2.0
    )
    assert stats.samples == 100
    assert stats.published == pytest.approx(20, abs=1)
    assert len(pub.poses) == stats.published


async def test_stop_event_and_duration_limit() -> None:
    stop = asyncio.Event()
    stop.set()
    bridge = EvBridge(ErrorInjector(DriftConfig()), EvBridgeConfig(rate_hz=0.0))
    stats = await bridge.run(ReplayTruthSource(_straight_truth()), RecordingPublisher(), stop=stop)
    assert stats.samples == 0

    stats2, _ = await _run(
        ErrorInjector(DriftConfig()), _straight_truth(n=200, dt=0.1), max_duration_s=2.0
    )
    assert stats2.samples == pytest.approx(21, abs=1)  # 2 s at 10 Hz


async def test_px4_estimate_never_influences_the_injected_error() -> None:
    """The trap: PX4's own position is EKF2's output, so it cannot be truth.

    A probe returning wild values must change nothing except the diagnostic
    field — if it ever fed the injector, these two runs would diverge.
    """
    samples = _straight_truth(n=60)
    clean, pub_clean = await _run(ErrorInjector(DriftConfig(seed=9)), samples, rate_hz=0.0)

    seen: list[BridgeSample] = []
    bridge = EvBridge(
        ErrorInjector(DriftConfig(seed=9)),
        EvBridgeConfig(rate_hz=0.0),
        estimate_probe=lambda: (1e6, -1e6, 42.0),
        on_sample=seen.append,
    )
    pub_probed = RecordingPublisher()
    probed = await bridge.run(ReplayTruthSource(samples), pub_probed)

    assert probed.final_error_m == clean.final_error_m
    assert probed.track_enu == clean.track_enu
    assert pub_probed.poses == pub_clean.poses
    assert seen[-1].estimate_ned == (1e6, -1e6, 42.0)  # recorded, and only recorded


async def test_vtrn_bounds_the_error_that_no_vtrn_lets_run() -> None:
    """The acceptance A/B in miniature, offline: fixes on vs fixes off."""
    mission = _mission()
    samples = _straight_truth(n=400, step_m=5.0)  # ~2 km
    drift = DriftConfig(rate=0.02, seed=5)
    config = CorrectionConfig(attempt_interval_m=50.0)

    off, _ = await _run(ErrorInjector(drift, None, config), samples, rate_hz=0.0)
    reference = route_reference(mission, forest_frac=0.2, stale_frac=0.0, seed=1)
    on, _ = await _run(ErrorInjector(drift, reference, config), samples, rate_hz=0.0)

    assert on.attempts > 0
    assert on.accepted > 0
    assert on.max_error_m < off.max_error_m
    assert off.final_error_m > 20.0


# --------------------------------------------------------------------------- #
# Shared error model
# --------------------------------------------------------------------------- #


def test_injector_matches_the_offline_simulation() -> None:
    """The bridge must inject the model `droneflight nav` measured, not a copy."""
    from droneflight.nav import _densify, _to_local, simulate_corrected_navigation

    mission = _mission()
    drift = DriftConfig(rate=0.02, seed=11)
    reference = route_reference(mission, seed=2)
    offline = simulate_corrected_navigation(mission, drift, reference)

    injector = ErrorInjector(drift, reference)
    local = [_to_local(w.lat, w.lon, mission.home_lat) for w in mission.waypoints]
    for (ax, ay), (bx, by) in pairwise(_densify(local, 10.0)):
        injector.advance(math.hypot(bx - ax, by - ay), bx, by)

    assert math.hypot(*injector.error) == pytest.approx(offline.endpoint_error_m)
    assert injector.attempts == offline.attempts
    assert injector.accepted == offline.accepted


def test_injector_without_reference_never_attempts_a_fix() -> None:
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=1))
    for _ in range(500):
        injector.advance(10.0, 0.0, 0.0)
    assert injector.attempts == 0
    assert injector.distance_m == pytest.approx(5000.0)
    assert math.hypot(*injector.error) > 10.0


def test_injector_ignores_zero_length_steps() -> None:
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=1))
    assert injector.advance(0.0, 0.0, 0.0) is None
    assert injector.error == (0.0, 0.0)
    assert injector.distance_m == 0.0


# --------------------------------------------------------------------------- #
# Corridor analysis
# --------------------------------------------------------------------------- #


def test_plan_track_is_relative_to_home() -> None:
    plan = plan_track_enu(_mission())
    assert plan[0] == pytest.approx((0.0, 0.0), abs=1e-6)
    assert plan[1][1] == pytest.approx(1112.0, rel=1e-3)  # 0.01 deg latitude
    assert plan[1][0] == pytest.approx(0.0, abs=1e-6)


def test_track_on_the_route_is_within_corridor() -> None:
    plan = plan_track_enu(_mission())
    track = [(0.0, y) for y in range(0, 1000, 50)]
    report = analyse_track(track, plan, tolerance_m=15.0)
    assert report.max_deviation_m < 1e-6
    assert report.within_corridor


def test_track_offset_from_the_route_leaves_the_corridor() -> None:
    plan = plan_track_enu(_mission())
    track = [(30.0, y) for y in range(0, 1000, 50)]
    report = analyse_track(track, plan, tolerance_m=15.0)
    assert report.max_deviation_m == pytest.approx(30.0, abs=1e-6)
    assert report.mean_deviation_m == pytest.approx(30.0, abs=1e-6)
    assert not report.within_corridor


def test_analyse_track_needs_both_sides() -> None:
    with pytest.raises(EvBridgeError):
        analyse_track([], plan_track_enu(_mission()))


# --------------------------------------------------------------------------- #
# Recording
# --------------------------------------------------------------------------- #


async def test_record_and_load_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "run.jsonl"
    injector = ErrorInjector(DriftConfig(rate=0.02, seed=4), route_reference(_mission(), seed=1))
    with TrackRecorder(path) as recorder:
        bridge = EvBridge(
            injector,
            EvBridgeConfig(rate_hz=0.0),
            recorder=recorder,
            estimate_probe=lambda: (1.0, 2.0, -30.0),
        )
        stats = await bridge.run(ReplayTruthSource(_straight_truth(n=100)), RecordingPublisher())

    loaded = load_track(path)
    assert len(loaded) == stats.published
    assert loaded[-1].truth_enu[:2] == pytest.approx(stats.track_enu[-1])
    assert loaded[-1].estimate_ned == (1.0, 2.0, -30.0)
    assert math.hypot(*loaded[-1].error_en) == pytest.approx(stats.final_error_m, abs=1e-3)
    assert any(s.fix for s in loaded)


# --------------------------------------------------------------------------- #
# The gz subprocess plumbing (fake `gz` binary — no Gazebo needed)
# --------------------------------------------------------------------------- #


def _fake_gz(tmp_path: Path, body: str, rc: int = 0) -> str:
    path = tmp_path / "gz"
    path.write_text(f'#!/bin/sh\nif [ "$2" = "-l" ]; then\n{body}\nfi\nexit {rc}\n')
    path.chmod(0o755)
    return str(path)


def test_discover_gz_topics_reads_the_listing(tmp_path: Path) -> None:
    gz = _fake_gz(tmp_path, "echo /clock\necho /model/x500_0/odometry")
    assert discover_gz_topics(gz) == ["/clock", "/model/x500_0/odometry"]


def test_discover_gz_topics_without_gazebo_says_so() -> None:
    with pytest.raises(EvBridgeError, match="not on PATH"):
        discover_gz_topics("definitely-not-a-real-binary")


def test_discover_gz_topics_propagates_failure(tmp_path: Path) -> None:
    with pytest.raises(EvBridgeError, match="failed"):
        discover_gz_topics(_fake_gz(tmp_path, "echo /clock", rc=1))


def _fake_gz_echo(tmp_path: Path, messages: list[str]) -> str:
    """A `gz` stand-in that streams JSON messages on `gz topic -e`."""
    path = tmp_path / "gz-echo"
    lines = "\n".join(f"printf '%s' '{m}'" for m in messages)
    path.write_text(f"#!/bin/sh\n{lines}\n")
    path.chmod(0o755)
    return str(path)


_MSG = (
    '{{"header":{{"stamp":{{"sec":{t}}}}},'
    '"pose":{{"position":{{"x":{x},"y":{y},"z":30}},"orientation":{{"w":1}}}}}}'
)


async def test_gazebo_truth_source_streams_parsed_samples(tmp_path: Path) -> None:
    from droneflight.ev_bridge import GazeboTruthSource

    msgs = [_MSG.format(t=i, x=i, y=2 * i) for i in range(3)]
    source = GazeboTruthSource(_ODOM_TOPIC, gz_binary=_fake_gz_echo(tmp_path, msgs))
    got = []
    with pytest.raises(EvBridgeError, match="ended"):
        async for sample in source.stream():
            got.append(sample)
    assert [(s.east_m, s.north_m) for s in got] == [(0.0, 0.0), (1.0, 2.0), (2.0, 4.0)]
    assert got[-1].up_m == 30.0


async def test_gazebo_truth_source_drives_the_bridge(tmp_path: Path) -> None:
    from droneflight.ev_bridge import GazeboTruthSource

    msgs = [_MSG.format(t=i, x=0, y=10 * i) for i in range(20)]
    source = GazeboTruthSource(_ODOM_TOPIC, gz_binary=_fake_gz_echo(tmp_path, msgs))
    bridge = EvBridge(ErrorInjector(DriftConfig(rate=0.02, seed=2)), EvBridgeConfig(rate_hz=0.0))
    with pytest.raises(EvBridgeError):
        await bridge.run(source, RecordingPublisher())


# --------------------------------------------------------------------------- #
# Edge cases
# --------------------------------------------------------------------------- #


def test_json_splitter_keeps_a_partial_object_after_a_complete_one() -> None:
    s = _JsonStreamSplitter()
    assert s.feed('{"a":1}{"b":') == ['{"a":1}']
    assert s.feed("2}") == ['{"b":2}']


def test_parse_tolerates_junk_and_wrong_shapes() -> None:
    assert parse_truth("not a dict", _POSE_TOPIC) is None
    assert parse_truth({"pose": "not a list"}, _POSE_TOPIC) is None
    # A non-numeric string where a number belongs reads as 0, not a crash.
    doc = {"pose": {"position": {"x": "nan-ish"}, "orientation": {"w": 1}}}
    sample = parse_truth(doc, _ODOM_TOPIC)
    assert sample is not None
    assert sample.east_m == 0.0


async def test_timestamp_modes() -> None:
    samples = [TruthSample(t_s=7.5, east_m=0.0, north_m=0.0, up_m=1.0)]

    _, pub_none = await _run(ErrorInjector(DriftConfig()), samples, timestamp_mode="none")
    assert pub_none.poses[0].time_usec == 0

    _, pub_sim = await _run(ErrorInjector(DriftConfig()), samples, timestamp_mode="sim")
    assert pub_sim.poses[0].time_usec == 7_500_000

    _, pub_epoch = await _run(ErrorInjector(DriftConfig()), samples, timestamp_mode="epoch")
    assert pub_epoch.poses[0].time_usec > 1_700_000_000_000_000  # wall-clock microseconds


def test_analyse_track_against_a_single_point_plan() -> None:
    report = analyse_track([(3.0, 4.0)], [(0.0, 0.0)], tolerance_m=15.0)
    assert report.max_deviation_m == pytest.approx(5.0)
    assert report.within_corridor


def test_load_track_skips_blank_lines(tmp_path: Path) -> None:
    path = tmp_path / "run.jsonl"
    path.write_text('{"t":1,"truth":{"e":1,"n":2,"u":3},"err":{"e":0,"n":0},"pub":{}}\n\n')
    assert len(load_track(path)) == 1


def test_global_origin_fields_scaling() -> None:
    from droneflight.ev_bridge import global_origin_fields

    fields = global_origin_fields(46.9012345, 15.1234567, 412.5)
    assert fields["latitude"] == 469012345  # degE7
    assert fields["longitude"] == 151234567
    assert fields["altitude"] == 412500  # millimetres
    assert fields["target_system"] == 1


def _zigzag_mission() -> FlightMission:
    """A multi-leg route — a straight line has no corridor to hold."""
    legs = [(46.90, 15.00), (46.92, 15.01), (46.93, 14.99), (46.95, 15.02), (46.97, 15.00)]
    return FlightMission(
        name="zigzag",
        home_lat=legs[0][0],
        home_lon=legs[0][1],
        cruise_speed_ms=6.0,
        waypoints=tuple(Waypoint(lat=la, lon=lo, alt_agl_m=30.0) for la, lo in legs),
    )


async def _closed_loop_corridor(reference: object) -> float:
    """Max deviation of the TRUE track when PX4 flies the corrupted estimate.

    The autopilot steers so its *estimate* follows the plan, so the true position
    is ``plan - error``. That is the mechanism the SITL acceptance test measures;
    running it through the real bridge here is its offline proxy.
    """
    mission = _zigzag_mission()
    plan = plan_track_enu(mission)
    drift = DriftConfig(rate=0.02, seed=7)
    config = CorrectionConfig(attempt_interval_m=50.0)

    steering = ErrorInjector(drift, reference, config)  # type: ignore[arg-type]
    truth = []
    for i, (e, n) in enumerate(_densify(list(plan), 5.0)):
        ex, ey = steering.error
        truth.append(TruthSample(t_s=i * 0.8, east_m=e - ex, north_m=n - ey, up_m=30.0, qw=1.0))
        steering.advance(5.0, e - ex, n - ey)

    stats, _ = await _run(
        ErrorInjector(drift, reference, config),  # type: ignore[arg-type]
        truth,
        rate_hz=0.0,
    )
    return analyse_track(stats.track_enu, plan, tolerance_m=15.0).max_deviation_m


async def test_acceptance_ab_offline_proxy() -> None:
    """Fixes on: the true track holds the corridor. Fixes off: it leaves."""
    on = await _closed_loop_corridor(route_reference(_zigzag_mission(), seed=1))
    off = await _closed_loop_corridor(None)
    assert on <= 15.0, f"VTRN should hold the corridor, got {on:.1f} m"
    assert off > 50.0, f"dead reckoning should wander well out, got {off:.1f} m"
    assert off > 5 * on
