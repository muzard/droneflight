"""Tests for the v0 trailing-fiber tether model.

Pure and deterministic — no Gazebo. Covers the tension terms, the payout-tension
profile, and the two findings the model surfaced: it separates the speed policy
from flat cruise on a reversal, and it is robust to the duplicate-waypoint
clusters that otherwise smear a turn to ~0°.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from droneflight.model import FlightMission, Waypoint
from droneflight.speed_policy import SpeedPolicy
from droneflight.tether import (
    TetherConfig,
    TetherLogError,
    analyse_tether,
    check_plugin_parity,
    drag_tension_n,
    parse_plugin_log,
    tension_n,
    turn_tension_n,
)


def _mission(coords: list[tuple[float, float]], cruise: float = 6.0) -> FlightMission:
    return FlightMission(
        name="t",
        home_lat=coords[0][0],
        home_lon=coords[0][1],
        cruise_speed_ms=cruise,
        waypoints=tuple(Waypoint(lat=la, lon=lo, alt_agl_m=30.0) for la, lo in coords),
    )


# --------------------------------------------------------------------------- #
# Tension terms
# --------------------------------------------------------------------------- #


def test_config_rejects_nonsense() -> None:
    with pytest.raises(ValueError, match="break_tension_n"):
        TetherConfig(break_tension_n=0.0)
    with pytest.raises(ValueError, match="coupling_length_m"):
        TetherConfig(coupling_length_m=0.0)
    with pytest.raises(ValueError, match="warn_fraction"):
        TetherConfig(warn_fraction=1.5)


def test_drag_scales_with_speed_squared_and_span() -> None:
    c = TetherConfig()
    assert drag_tension_n(c, 0.0) == 0.0
    d1 = drag_tension_n(c, 3.0)
    d2 = drag_tension_n(c, 6.0)
    assert d2 == pytest.approx(4.0 * d1)  # doubling speed quadruples drag
    wide = TetherConfig(airborne_span_m=80.0)
    assert drag_tension_n(wide, 6.0) == pytest.approx(2.0 * d2)  # linear in span


def test_turn_tension_zero_straight_max_at_reversal() -> None:
    c = TetherConfig()
    assert turn_tension_n(c, 6.0, 0.0) == 0.0  # straight leg: no turn load
    quarter = turn_tension_n(c, 6.0, 90.0)
    reversal = turn_tension_n(c, 6.0, 180.0)
    assert reversal > quarter > 0.0
    # (1 - cos): 90° -> 1, 180° -> 2, so a reversal is exactly twice a right angle.
    assert reversal == pytest.approx(2.0 * quarter)


def test_turn_tension_scales_with_speed_squared() -> None:
    c = TetherConfig()
    slow = turn_tension_n(c, 2.0, 180.0)
    fast = turn_tension_n(c, 4.0, 180.0)
    assert fast == pytest.approx(4.0 * slow)  # halving speed quarters the turn load


def test_squares_with_multiplication_not_pow_for_cpp_parity() -> None:
    """drag/turn must square speed with `*`, not `**2`.

    `v**2` routes through pow() and differs from `v*v` by 1 ULP on some values.
    The C++ plugin (tether_math.hpp) squares with a multiply, and the parity gate
    demands a bit-exact match on the applied force — a real span=4000 flight
    falsely "DIVERGED" by 3.55e-15 N until this was aligned. 4.989233542214098 is
    a speed from that flight where pow != mul.
    """
    v = 4.989233542214098
    c = TetherConfig()
    k = 0.5 * c.air_density_kg_m3 * c.drag_coefficient * c.fiber_diameter_m * c.airborne_span_m
    assert drag_tension_n(c, v) == k * (v * v)  # multiply, bit-exact
    # pow() and multiply disagree for this value on macOS's libm but agree on
    # glibc (CI), so the "not pow" half can only be checked where they differ.
    if v**2 != v * v:
        assert drag_tension_n(c, v) != k * v**2  # not the pow() value


def test_total_tension_is_the_sum() -> None:
    c = TetherConfig()
    total = tension_n(c, speed_ms=5.0, deflection_deg=120.0)
    assert total == pytest.approx(
        c.back_tension_n + drag_tension_n(c, 5.0) + turn_tension_n(c, 5.0, 120.0)
    )


# --------------------------------------------------------------------------- #
# The speed-policy validation — the whole point
# --------------------------------------------------------------------------- #


def test_uncalibrated_model_says_a_reversal_at_cruise_snaps() -> None:
    """What the model claimed before the PX4 corner prior — kept, and pinned OFF.

    This was the module's headline result and the justification for the speed
    policy. It is preserved only as the *uncalibrated* baseline: acceptance §3
    flew exactly this scenario and nothing snapped, because no vehicle flies a
    reversal at cruise. See ``test_px4_prior_is_what_survives_a_reversal``.
    """
    m = _mission([(46.90, 15.0), (46.9018, 15.0), (46.9002, 15.0)])  # out then reverse
    cfg = TetherConfig()
    no_policy = analyse_tether(m, cfg, policy=None, px4_corner_prior=False)
    with_policy = analyse_tether(
        m, cfg, policy=SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0), px4_corner_prior=False
    )

    assert no_policy.breaks
    assert no_policy.peak is not None and no_policy.peak.deflection_deg > 150.0
    assert with_policy.within_limit
    assert with_policy.max_tension_n < no_policy.max_tension_n


def test_px4_prior_is_what_survives_a_reversal_not_the_speed_policy() -> None:
    """The §3 finding, pinned: PX4's own corner deceleration keeps the fiber intact.

    Flown 2026-07-26 (`sim-logs/4.7/`): a control arm commanded 6.0 m/s at all 32
    waypoints still cornered at 0.46-4.96 m/s, the peak turn transient moved
    -0.4% versus the policy arm, and neither arm broke anything. With the prior
    on, the model must reproduce that null — if these two ever diverge again, the
    prior has stopped being applied.
    """
    m = _mission([(46.90, 15.0), (46.9018, 15.0), (46.9002, 15.0)])
    cfg = TetherConfig()
    no_policy = analyse_tether(m, cfg, policy=None)
    with_policy = analyse_tether(m, cfg, policy=SpeedPolicy(cruise_speed_ms=6.0, min_speed_ms=1.0))

    assert no_policy.within_limit, "PX4 will not fly a reversal at cruise"
    assert with_policy.within_limit
    assert with_policy.max_tension_n == pytest.approx(no_policy.max_tension_n)
    # And the uncalibrated model must still disagree, or the arms are not distinct.
    uncal = analyse_tether(m, cfg, policy=None, px4_corner_prior=False)
    assert uncal.max_tension_n > no_policy.max_tension_n * 2.0


def test_prior_uses_the_flown_deflection_not_the_vertex_angle() -> None:
    """A shallow vertex is a much larger course change to the payout.

    The waypoint model reads the angle between two legs; the fiber sees a heading
    rate sustained over the coupling length. Measured in flight: a 20 deg vertex
    was flown as a 49 deg per-sample deflection. Scoring the vertex angle
    under-reads shallow corners, which is where the flown transient peaks.
    """
    # A gentle bend: ~20 deg at the middle vertex.
    m = _mission([(46.9000, 15.0000), (46.9020, 15.0000), (46.9038, 15.0009)])
    load = analyse_tether(m, TetherConfig(), policy=None).loads[1]
    assert 0.0 < load.deflection_deg < 30.0
    assert load.flown_deflection_deg > load.deflection_deg * 2.0


def test_straight_route_never_breaks_and_policy_is_inactive() -> None:
    m = _mission([(46.90, 15.0), (46.91, 15.0), (46.92, 15.0)])
    rep = analyse_tether(m, TetherConfig(), policy=SpeedPolicy(cruise_speed_ms=6.0))
    assert rep.within_limit
    # Straight line: every vertex holds cruise, tension is just back + drag.
    assert all(load.speed_ms == pytest.approx(6.0) for load in rep.loads)


def test_report_flags_breaks_and_warnings() -> None:
    m = _mission([(46.90, 15.0), (46.9018, 15.0), (46.9002, 15.0)])
    # Exercises the report's break/warning classification, so the prior is off:
    # with it on this route no longer breaks at any sane budget (that is the §3
    # finding, covered by its own test — this one is about the plumbing).
    rep = analyse_tether(
        m,
        TetherConfig(break_tension_n=1.0, warn_fraction=0.5),
        policy=None,
        px4_corner_prior=False,
    )
    assert rep.breaks
    assert rep.peak is not None and rep.peak.exceeds_break
    assert rep.peak.load_fraction > 1.0


# --------------------------------------------------------------------------- #
# Robustness to the duplicate-waypoint cluster defect
# --------------------------------------------------------------------------- #


def test_duplicate_cluster_does_not_hide_a_turn() -> None:
    """A sharp turn smeared into sub-meter waypoints is still seen at full angle.

    The planner emits duplicate clusters (0.2-1.0 m apart); measuring bends on
    the raw waypoints would spread a 90° corner across several ~0° segments and
    the tether load — and the speed policy — would miss it entirely.
    """
    # A 90° corner where the corner vertex is split into a 0.5 m micro-cluster.
    m = _mission(
        [
            (46.9000, 15.0000),
            (46.9010, 15.0000),  # heading north
            (46.90100, 15.00001),  # ~0.8 m micro-step — the cluster
            (46.9010, 15.0010),  # now heading east: a 90° turn hidden in the cluster
        ]
    )
    rep = analyse_tether(m, TetherConfig(), policy=None)
    assert rep.peak is not None
    # After consolidation the corner shows as a real turn, not ~0°.
    assert max(load.deflection_deg for load in rep.loads) > 60.0


# --------------------------------------------------------------------------- #
# Reading back the Gazebo plugin's log (the in-the-loop parity gate)
# --------------------------------------------------------------------------- #
#
# These use hand-written CSVs so the parser and its failure modes are covered
# without a C++ compiler. `tests/test_tether_plugin.py` does the complementary
# half: logs written by the plugin's own C++ formatter, scored for exactness.


def _plugin_csv(rows: list[str], *, enabled: int = 1, back: float = 0.5) -> str:
    header = [
        f"# droneflight tether plugin v0 enabled={enabled} link=x500/base_link",
        f"# fiber_diameter_m=0.00025 drag_coefficient=1.2 air_density_kg_m3=1.225 "
        f"airborne_span_m=40.0 back_tension_n={back} payout_inertia_kg=0.15 "
        f"coupling_length_m=8.0 speed_floor_ms=0.3",
        "t_s,x_m,y_m,z_m,vx_ms,vy_ms,speed_ms,drag_n,tension_n,"
        "lateral_accel_ms2,deflection_deg,turn_n,fx_n,fy_n,moving",
    ]
    return "\n".join([*header, *rows]) + "\n"


def _cruise_row(t: float, speed: float) -> str:
    drag = drag_tension_n(TetherConfig(), speed)
    tension = 0.5 + drag
    return (
        f"{t!r},0.0,0.0,30.0,{speed!r},0.0,{speed!r},{drag!r},{tension!r},"
        f"0.0,0.0,0.0,{-tension!r},0.0,1"
    )


def test_plugin_log_carries_its_own_coefficients(tmp_path: Path) -> None:
    """A log has to be scorable months later without trusting anyone's memory of
    the command line — so the parameters come out of the file, not a default."""
    path = tmp_path / "t.csv"
    path.write_text(_plugin_csv([_cruise_row(0.0, 6.0), _cruise_row(0.1, 7.0)], back=0.9))
    log = parse_plugin_log(path)
    assert log.config.back_tension_n == 0.9
    assert log.config.airborne_span_m == 40.0
    assert log.speed_floor_ms == 0.3
    assert log.enabled and log.link == "x500/base_link"
    assert len(log.rows) == 2


def test_plugin_parity_is_exact_on_a_consistent_log(tmp_path: Path) -> None:
    path = tmp_path / "t.csv"
    path.write_text(_plugin_csv([_cruise_row(t / 10, 6.0) for t in range(10)]))
    parity = check_plugin_parity(parse_plugin_log(path))
    assert parity.exact
    assert parity.moving_samples == 10
    assert parity.peak_tension_n == pytest.approx(0.7646, abs=1e-4)
    assert parity.load_fraction(2.0) == pytest.approx(0.3823, abs=1e-4)


def test_plugin_parity_flags_a_drifted_coefficient(tmp_path: Path) -> None:
    """The failure this gate exists for: the plugin was built with a different
    airborne span than the header claims, so its drag is off. Without the check,
    the flight would be scored against a model it never ran."""
    stale = TetherConfig(airborne_span_m=60.0)
    drag = drag_tension_n(stale, 6.0)
    tension = 0.5 + drag
    rows = [
        f"{i / 10!r},0,0,30,6.0,0.0,6.0,{drag!r},{tension!r},0,0,0,{-tension!r},0,1"
        for i in range(5)
    ]
    path = tmp_path / "t.csv"
    path.write_text(_plugin_csv(rows))  # header still says span=40
    parity = check_plugin_parity(parse_plugin_log(path))
    assert not parity.exact
    assert parity.max_drag_error_n > 0.1


def test_plugin_parity_flags_a_bogus_static_sample(tmp_path: Path) -> None:
    """Below the speed floor the only legal tensions are 0 (never launched) and
    back-tension (hovering with the line out). Anything else means the floor
    branch is applying something it should not."""
    path = tmp_path / "t.csv"
    path.write_text(
        _plugin_csv([_cruise_row(0.0, 6.0), "0.1,0,0,30,0.0,0.0,0.0,0.0,0.31,0,0,0,-0.31,0,0"])
    )
    parity = check_plugin_parity(parse_plugin_log(path))
    assert parity.bad_static_samples == 1
    assert not parity.exact


def test_plugin_log_rejects_an_empty_or_malformed_file(tmp_path: Path) -> None:
    empty = tmp_path / "empty.csv"
    empty.write_text(_plugin_csv([]))
    with pytest.raises(TetherLogError, match="no data rows"):
        parse_plugin_log(empty)

    short = tmp_path / "short.csv"
    short.write_text(_plugin_csv(["0.0,1,2,3"]))
    with pytest.raises(TetherLogError, match="expected 15 columns"):
        parse_plugin_log(short)

    junk = tmp_path / "junk.csv"
    junk.write_text(_plugin_csv([",".join(["x"] * 15)]))
    with pytest.raises(TetherLogError, match="non-numeric"):
        parse_plugin_log(junk)


def test_plugin_log_records_a_disabled_control_arm(tmp_path: Path) -> None:
    path = tmp_path / "t.csv"
    path.write_text(_plugin_csv([_cruise_row(0.0, 6.0)], enabled=0))
    assert not parse_plugin_log(path).enabled


def test_the_plans_leg_speeds_are_what_gets_scored() -> None:
    """The ``.plan`` carries DO_CHANGE_SPEED since 2026-09-23 (3 m/s in forest,
    6 m/s in the open). Scoring every vertex at cruise ignored it; a vertex whose
    legs are both slow must be scored slow, and one next to a fast leg fast."""
    coords = [(46.90, 15.0), (46.9009, 15.0), (46.9018, 15.0), (46.9027, 15.0)]
    speeds = [3.0, 3.0, 6.0, None]
    m = FlightMission(
        name="t",
        home_lat=coords[0][0],
        home_lon=coords[0][1],
        cruise_speed_ms=6.0,
        waypoints=tuple(
            Waypoint(lat=la, lon=lo, alt_agl_m=30.0, speed_ms=v)
            for (la, lo), v in zip(coords, speeds, strict=True)
        ),
    )
    loads = analyse_tether(m, TetherConfig(), policy=None, px4_corner_prior=False).loads
    assert [load.speed_ms for load in loads] == [3.0, 3.0, 6.0, 6.0]
    capped = analyse_tether(
        m, TetherConfig(), policy=SpeedPolicy(cruise_speed_ms=6.0), px4_corner_prior=False
    )
    assert capped.loads[1].speed_ms == 3.0


def test_a_straight_track_past_trunks_has_no_span_contacts() -> None:
    from droneflight.perception import Trunk
    from droneflight.tether import airborne_span_contacts

    track = [(float(x), 0.0) for x in range(0, 100)]
    trunks = (Trunk(50.0, 1.0, 0.15), Trunk(70.0, -1.2, 0.15))
    assert airborne_span_contacts(track, trunks, 20.0).trunks == ()


def test_the_span_cuts_the_corner_the_aircraft_flew_round() -> None:
    """Round a trunk on a dogleg: the aircraft clears it, a long span crosses it."""
    from droneflight.perception import Trunk
    from droneflight.tether import airborne_span_contacts

    # East 30 m, then north 30 m; a trunk just inside the corner.
    track = [(float(x), 0.0) for x in range(31)] + [(30.0, float(y)) for y in range(1, 31)]
    trunks = (Trunk(28.0, 2.0, 0.3),)
    assert airborne_span_contacts(track, trunks, 2.0).trunks == ()
    long_span = airborne_span_contacts(track, trunks, 10.0)
    assert long_span.trunks == (0,)
    assert long_span.per_km > 0.0


def test_free_fall_lag_scales_with_height_and_speed() -> None:
    from droneflight.tether import free_fall_touchdown_lag_m

    cfg = TetherConfig()
    base = free_fall_touchdown_lag_m(9.0, 3.0, cfg)
    assert 8.0 < base < 20.0  # ~1.9 m/s terminal fall -> ~14 m
    assert free_fall_touchdown_lag_m(9.0, 1.5, cfg) == pytest.approx(base / 2)
    assert free_fall_touchdown_lag_m(18.0, 3.0, cfg) == pytest.approx(base * 2)


def test_a_free_falling_span_lowers_the_speed_ceiling() -> None:
    """At 12 m AGL the free-fall span is ~40 m at 6 m/s — the default — and grows
    with speed, so the ceiling sits well under the fixed-span one."""
    from droneflight.tether import free_fall_span_m, speed_ceiling_ms

    cfg = TetherConfig()
    assert 35.0 < free_fall_span_m(12.0, 6.0, cfg) < 45.0
    fixed = math.sqrt(
        (cfg.break_tension_n - cfg.back_tension_n) / drag_tension_n(cfg, 1.0)
    )  # fixed 40 m span, v^2 drag
    ceiling = speed_ceiling_ms(12.0, cfg)
    assert 6.0 < ceiling < fixed
    assert speed_ceiling_ms(24.0, cfg) < ceiling  # higher = longer span = slower
