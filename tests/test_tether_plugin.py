"""Parity tests for the Gazebo tether plugin's physics — run on the Mac.

The plugin (`sitl/tether_plugin/`) has to apply the *same* tension the offline
model computes, or the in-loop disturbance is a second, unvalidated model rather
than the one `tether-check` and the 40-seed studies were measured against. But
Gazebo Harmonic exists only on the Ubuntu VM, so the plugin itself cannot be
compiled here.

The split that makes this testable: all the physics lives in a Gazebo-free
header (`tether_math.hpp`), and `tether_probe.cc` evaluates it from a script on
stdin. These tests compile that probe with the host C++ compiler and diff every
number against `droneflight.tether`. What stays untested off the VM is only the
glue — entity lookup and `AddWorldForce` — mirroring how `ev_bridge`'s pure
parts are tested without a sim.

Equality here is **exact**, not approximate: the header mirrors the Python's
association order, and ``std::cos`` and ``math.cos`` are the same libm call on a
given host — so this is parity between the two implementations *on this machine*,
which is what "the in-loop force is the model" needs to mean. A tolerance would
hide exactly the drift these tests exist to catch.
"""

from __future__ import annotations

import math
import shutil
import subprocess
from pathlib import Path

import pytest

from droneflight.tether import (
    TetherConfig,
    drag_tension_n,
    tension_n,
    turn_tension_n,
)

PLUGIN_DIR = Path(__file__).resolve().parents[1] / "sitl" / "tether_plugin"
PROBE_SRC = PLUGIN_DIR / "tether_probe.cc"

# Must match CMakeLists.txt — see the module docstring on why contraction is off.
CXX_FLAGS = ["-std=c++17", "-O2", "-ffp-contract=off", "-I", str(PLUGIN_DIR)]

# Parameter sets exercised against the reference. The first is the shipped
# default; the rest are deliberately awkward so a coefficient transposed between
# the two implementations cannot hide behind a symmetric default.
PARAM_SETS = [
    TetherConfig(),
    TetherConfig(
        fiber_diameter_m=0.0009,
        drag_coefficient=0.7,
        air_density_kg_m3=1.07,
        airborne_span_m=13.5,
        back_tension_n=1.25,
        payout_inertia_kg=0.41,
        coupling_length_m=3.3,
    ),
    TetherConfig(airborne_span_m=0.0, back_tension_n=0.0, payout_inertia_kg=0.0),
]

SPEEDS = [0.0, 0.001, 0.3, 1.0, 2.5, 6.0, 7.3, 12.0, 25.0]
DEFLECTIONS = [0.0, 0.5, 7.0, 16.0, 30.0, 89.9, 90.0, 150.0, 179.9, 180.0]


def _compiler() -> str | None:
    for name in ("c++", "clang++", "g++"):
        found = shutil.which(name)
        if found:
            return found
    return None


@pytest.fixture(scope="module")
def probe(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Compile `tether_probe.cc` once for the module."""
    cxx = _compiler()
    if cxx is None:
        pytest.skip("no C++ compiler on PATH — cannot check plugin/model parity")
    out = tmp_path_factory.mktemp("tether_plugin") / "tether_probe"
    subprocess.run([cxx, *CXX_FLAGS, "-o", str(out), str(PROBE_SRC)], check=True)
    return out


def _run(probe: Path, script: list[str]) -> list[list[float]]:
    """Feed the probe a script; return one row of floats per command."""
    result = subprocess.run(
        [str(probe)],
        input="\n".join(script) + "\n",
        capture_output=True,
        text=True,
        check=True,
    )
    rows = []
    for line in result.stdout.splitlines():
        assert not line.startswith("error"), f"probe rejected a command: {line}"
        if not line.strip():
            continue
        rows.append([float(v) for v in line.split()])
    return rows


def _params_line(config: TetherConfig, speed_floor: float = 0.3) -> str:
    return (
        f"params {config.fiber_diameter_m!r} {config.drag_coefficient!r} "
        f"{config.air_density_kg_m3!r} {config.airborne_span_m!r} {config.back_tension_n!r} "
        f"{config.payout_inertia_kg!r} {config.coupling_length_m!r} {speed_floor!r}"
    )


# --------------------------------------------------------------------------- #
# The coefficients themselves
# --------------------------------------------------------------------------- #


def test_plugin_defaults_are_the_tether_config_defaults(probe: Path) -> None:
    """The strongest guard against divergence: the two default sets are one set.

    TETHER_PLUGIN.md's "do not re-derive the coefficients" is only enforceable if
    something checks. If a `tether.py` default is retuned and the header is not,
    this fails before anyone flies with the stale number.
    """
    (row,) = _run(probe, ["defaults"])
    config = TetherConfig()
    fields = [
        ("fiber_diameter_m", config.fiber_diameter_m),
        ("drag_coefficient", config.drag_coefficient),
        ("air_density_kg_m3", config.air_density_kg_m3),
        ("airborne_span_m", config.airborne_span_m),
        ("back_tension_n", config.back_tension_n),
        ("payout_inertia_kg", config.payout_inertia_kg),
        ("coupling_length_m", config.coupling_length_m),
    ]
    for (name, expected), got in zip(fields, row, strict=False):
        assert got == expected, f"{name}: plugin {got} != tether.py {expected}"
    assert row[7] == 0.3  # speed_floor_ms — plugin-only, no Python counterpart


# --------------------------------------------------------------------------- #
# Tension terms — bit-exact against droneflight.tether
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("config", PARAM_SETS)
def test_drag_matches_reference_exactly(probe: Path, config: TetherConfig) -> None:
    rows = _run(probe, [_params_line(config), *(f"drag {v!r}" for v in SPEEDS)])
    for speed, (got,) in zip(SPEEDS, rows, strict=True):
        assert got == drag_tension_n(config, speed), f"drag at {speed} m/s"


@pytest.mark.parametrize("config", PARAM_SETS)
def test_turn_matches_reference_exactly(probe: Path, config: TetherConfig) -> None:
    cases = [(v, d) for v in SPEEDS for d in DEFLECTIONS]
    rows = _run(probe, [_params_line(config), *(f"turn {v!r} {d!r}" for v, d in cases)])
    for (speed, deg), (got,) in zip(cases, rows, strict=True):
        assert got == turn_tension_n(config, speed, deg), f"turn at {speed} m/s, {deg} deg"


@pytest.mark.parametrize("config", PARAM_SETS)
def test_total_tension_matches_reference_exactly(probe: Path, config: TetherConfig) -> None:
    cases = [(v, d) for v in SPEEDS for d in DEFLECTIONS]
    rows = _run(probe, [_params_line(config), *(f"tension {v!r} {d!r}" for v, d in cases)])
    for (speed, deg), (got,) in zip(cases, rows, strict=True):
        expected = tension_n(config, speed_ms=speed, deflection_deg=deg)
        assert got == expected, f"tension at {speed} m/s, {deg} deg"


def test_cruise_tension_reproduces_the_flown_plan_number(probe: Path) -> None:
    """Sanity anchor: 6 m/s straight-leg cruise is the ~0.76 N the offline run reports.

    `tether-check` on the flown route peaks at 0.79 N (40 % of the 2 N budget) —
    a straight leg at cruise is the bulk of that, so a plugin producing anything
    far from it is wrong in a way the sweeps above might not make obvious.
    """
    config = TetherConfig()
    (row,) = _run(probe, [_params_line(config), "tension 6.0 0.0"])
    assert row[0] == tension_n(config, speed_ms=6.0, deflection_deg=0.0)
    assert row[0] == pytest.approx(0.7646, abs=1e-4)


# --------------------------------------------------------------------------- #
# The wrench: magnitude from the model, direction opposite to travel
# --------------------------------------------------------------------------- #


def test_back_pull_opposes_horizontal_velocity(probe: Path) -> None:
    config = TetherConfig()
    velocities = [(6.0, 0.0), (-6.0, 0.0), (0.0, 4.0), (3.0, -4.0), (-2.1, -1.4)]
    rows = _run(
        probe, [_params_line(config), *(f"pull {vx!r} {vy!r} 0 0" for vx, vy in velocities)]
    )
    for (vx, vy), row in zip(velocities, rows, strict=True):
        fx, fy, speed, drag, tension, dir_x, dir_y, moving = row
        assert moving == 1.0
        assert speed == math.hypot(vx, vy)
        assert drag == drag_tension_n(config, speed)
        assert tension == config.back_tension_n + drag
        # magnitude is the model's tension...
        assert math.hypot(fx, fy) == pytest.approx(tension, rel=1e-15)
        # ...and it points backwards along travel: the line trails the drone.
        assert fx * vx + fy * vy < 0.0
        assert dir_x == pytest.approx(vx / speed, rel=1e-15)
        assert dir_y == pytest.approx(vy / speed, rel=1e-15)


def test_back_pull_has_no_vertical_or_torque_component(probe: Path) -> None:
    """v0 is a horizontal force at the CoM. Vertical drag and the spool moment are
    out of scope (TETHER_PLUGIN.md), so the probe only ever reports fx/fy."""
    (row,) = _run(probe, [_params_line(TetherConfig()), "pull 6.0 0.0 0 0"])
    assert len(row) == 8  # fx fy speed drag tension dirx diry moving — no fz, no torque


def test_below_speed_floor_pulls_at_back_tension_along_last_direction(probe: Path) -> None:
    """A hovering drone still has a taut line, but no usable velocity heading.

    Drag is dropped below the floor (it is ~7e-7 N there), so the pull is pure
    back-tension along the last real travel direction — not a zero-velocity
    singularity that would shove the vehicle in an arbitrary bearing.
    """
    config = TetherConfig()
    rows = _run(
        probe,
        [
            _params_line(config),
            "pull 0.05 0.0 1 0",  # crawling east, last direction east
            "pull 0.0 0.0 0 1",  # stationary, last direction north
            "pull 0.0 0.0 3 4",  # last direction given un-normalised
        ],
    )
    east, north, diagonal = rows
    assert east[4] == config.back_tension_n and east[7] == 0.0
    assert (east[0], east[1]) == (-config.back_tension_n, -0.0)
    assert (north[0], north[1]) == (-0.0, -config.back_tension_n)
    assert math.hypot(diagonal[0], diagonal[1]) == pytest.approx(config.back_tension_n, rel=1e-15)
    assert diagonal[0] == pytest.approx(-config.back_tension_n * 0.6, rel=1e-15)


def test_vehicle_that_never_moved_gets_no_force(probe: Path) -> None:
    """On the pad, before launch, there is no line out — and no direction to
    invent. Applying back-tension in an arbitrary bearing here would nudge a
    parked vehicle and pollute the pre-arm state."""
    (row,) = _run(probe, [_params_line(TetherConfig()), "pull 0.0 0.0 0 0"])
    fx, fy, _speed, drag, tension, _dx, _dy, moving = row
    assert (fx, fy, drag, tension, moving) == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_speed_floor_is_configurable_and_switches_regimes(probe: Path) -> None:
    config = TetherConfig()
    rows = _run(
        probe,
        [
            _params_line(config, speed_floor=5.0),
            "pull 4.0 0.0 1 0",  # under a raised floor: back-tension only
            _params_line(config, speed_floor=0.3),
            "pull 4.0 0.0 1 0",  # over the default floor: back-tension + drag
        ],
    )
    under, over = rows[0], rows[1]
    assert under[4] == config.back_tension_n
    assert over[4] == config.back_tension_n + drag_tension_n(config, 4.0)


# --------------------------------------------------------------------------- #
# Turn diagnostic — the continuous-trajectory analogue of a waypoint deflection
# --------------------------------------------------------------------------- #


def test_deflection_from_lateral_acceleration_matches_the_geometry(probe: Path) -> None:
    """A steady turn of radius R has ``a_lat = v^2 / R``, so the course change over
    the coupling length L is exactly ``L / R`` radians — independent of speed.

    This is what lets the plugin score an in-flight corner with the same
    ``turn_tension_n`` the offline model applies at a waypoint.
    """
    config = TetherConfig()
    radius_m = 40.0
    speeds = [3.0, 6.0, 11.0]
    expected_deg = math.degrees(config.coupling_length_m / radius_m)
    rows = _run(
        probe,
        [_params_line(config), *(f"defl {v!r} {v * v / radius_m!r}" for v in speeds)],
    )
    for speed, (got,) in zip(speeds, rows, strict=True):
        assert got == pytest.approx(expected_deg, rel=1e-12), f"at {speed} m/s"


def test_deflection_is_zero_when_not_turning_and_clamps_at_reversal(probe: Path) -> None:
    rows = _run(
        probe,
        [
            _params_line(TetherConfig()),
            "defl 6.0 0.0",  # straight and level: no course change
            "defl 0.0 2.0",  # stationary: no course change
            "defl 0.5 60.0",  # absurd: clamped to a reversal, not wrapped
        ],
    )
    assert rows[0][0] == 0.0
    assert rows[1][0] == 0.0
    assert rows[2][0] == 180.0


# --------------------------------------------------------------------------- #
# The per-step evaluation, and the CSV that carries it off the VM
# --------------------------------------------------------------------------- #


def test_step_matches_its_parts(probe: Path) -> None:
    """`EvaluateStep` is what the plugin calls, so it must agree with the pieces
    the tests above pin down — otherwise the tested functions and the flown ones
    are different code."""
    config = TetherConfig()
    rows = _run(
        probe,
        [
            _params_line(config),
            "step 6.0 0.0 0.0 0.9 0 0",  # cruising east, turning left
            "pull 6.0 0.0 0 0",
            "defl 6.0 0.9",
        ],
    )
    step, pull, defl = rows
    assert step[:8] == pull  # the wrench half is exactly ComputeBackPull
    assert step[8] == 0.9  # lateral accel: all of it is perpendicular here
    assert step[9] == defl[0]
    assert step[10] == turn_tension_n(config, 6.0, defl[0])


def test_step_reports_no_turn_when_accelerating_straight_ahead(probe: Path) -> None:
    """Speeding up on a straight leg is not a course change: the payout momentum
    is not being redirected, so the turn term must stay zero."""
    (row,) = _run(probe, [_params_line(TetherConfig()), "step 6.0 0.0 2.0 0.0 0 0"])
    assert row[8] == 0.0  # no lateral component
    assert row[9] == 0.0
    assert row[10] == 0.0


def _synthetic_flight_csv(probe: Path, *, enabled: bool = True) -> str:
    """A short trajectory in the plugin's own log format, written by its own code.

    The C++ writes these rows with the same `WriteCsvRow` the plugin uses in
    flight, so parsing them here exercises the real format rather than a
    hand-rolled imitation of it.
    """
    script = [f"csvheader {1 if enabled else 0} x500/base_link"]
    # pad -> climb-out -> cruise east -> a left turn -> hover
    samples = [
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.1, 0.0, 0.0, 5.0, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0),
        (0.2, 0.1, 0.0, 30.0, 6.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        (0.3, 0.7, 0.0, 30.0, 5.6, 2.1, 0.1, 1.4, 1.0, 0.0),
        (0.4, 1.2, 0.2, 30.0, 4.0, 4.0, -0.9, 0.9, 0.94, 0.34),
        (0.5, 1.6, 0.6, 30.0, 0.05, 0.0, 0.0, 0.0, 0.71, 0.71),
    ]
    script += ["csvrow " + " ".join(repr(v) for v in sample) for sample in samples]
    script.append("csvfooter 0.7646 6.0 0.02")
    result = subprocess.run(
        [str(probe)], input="\n".join(script) + "\n", capture_output=True, text=True, check=True
    )
    return result.stdout


def test_plugin_csv_parses_and_scores_exact(probe: Path, tmp_path: Path) -> None:
    """The acceptance gate, end to end: C++ writes the log, Python re-derives every
    tension in it from `tether.py`, and the two agree exactly."""
    from droneflight.tether import check_plugin_parity, parse_plugin_log

    path = tmp_path / "tether.csv"
    path.write_text(_synthetic_flight_csv(probe))

    log = parse_plugin_log(path)
    assert log.enabled
    assert log.link == "x500/base_link"
    assert log.config == TetherConfig()  # header round-trips the coefficients
    assert log.speed_floor_ms == 0.3
    assert len(log.rows) == 6

    parity = check_plugin_parity(log)
    assert parity.exact
    assert parity.max_drag_error_n == 0.0
    assert parity.max_tension_error_n == 0.0
    assert parity.max_turn_error_n == 0.0
    assert parity.max_force_error_n < 1e-15  # vector magnitude vs scalar: ULP only
    assert parity.moving_samples == 3  # the three above the 0.3 m/s floor
    assert parity.peak_speed_ms == pytest.approx(6.0)
    assert parity.peak_tension_n == pytest.approx(
        TetherConfig().back_tension_n + drag_tension_n(TetherConfig(), 6.0)
    )


def test_parity_gate_catches_a_divergent_log(probe: Path, tmp_path: Path) -> None:
    """Proof the gate has teeth: nudge one logged tension by a single ULP — the
    kind of difference a quietly retuned coefficient would produce — and the
    check must stop calling the run exact."""
    from droneflight.tether import check_plugin_parity, parse_plugin_log

    lines = _synthetic_flight_csv(probe).splitlines()
    # The fastest sample — the one with the most tension to get wrong.
    data = [i for i, line in enumerate(lines) if not line.startswith(("#", "t_s,"))]
    cruise = max(data, key=lambda i: float(lines[i].split(",")[8]))
    parts = lines[cruise].split(",")
    parts[8] = repr(math.nextafter(float(parts[8]), math.inf))
    lines[cruise] = ",".join(parts)

    path = tmp_path / "tampered.csv"
    path.write_text("\n".join(lines) + "\n")

    parity = check_plugin_parity(parse_plugin_log(path))
    assert not parity.exact
    assert 0.0 < parity.max_tension_error_n < 1e-15


def test_disabled_run_is_recorded_as_such(probe: Path, tmp_path: Path) -> None:
    """A control arm must be identifiable from its log alone. An arm that applies
    no force and an arm whose plugin failed to load look identical otherwise —
    an ambiguity this program has already paid for once."""
    from droneflight.tether import parse_plugin_log

    path = tmp_path / "control.csv"
    path.write_text(_synthetic_flight_csv(probe, enabled=False))
    assert not parse_plugin_log(path).enabled


# --------------------------------------------------------------------------- #
# The harness itself
# --------------------------------------------------------------------------- #


def test_probe_fails_loudly_on_an_unknown_command(probe: Path) -> None:
    """A typo in a test script must fail, not silently skip a comparison."""
    result = subprocess.run(
        [str(probe)], input="drg 6.0\n", capture_output=True, text=True, check=False
    )
    assert result.returncode != 0
    assert result.stdout.startswith("error drg")
