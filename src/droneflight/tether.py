"""Trailing-fiber tether model (v0) — payout tension as a disturbance.

No standard sim models a trailing fiber (Technology-Selection §4.2, §7 risks), so
this is the offline, testable core of that required build item — the counterpart
to what a Gazebo wrench plugin will later apply *in the loop*. Its job is to turn
"the fiber snags/snaps if you whip through a turn or reverse at speed" from a
hand-wave into a number, so the **speed policy** can be validated against physics
instead of against three tuned angle thresholds.

The aircraft is assumed to carry the spool (**drone-side payout** — the §2.2
baseline: mass travels with you, back-tension is controlled locally), so the
paid-out length at a point is just the distance flown from the launch point.

Tension at the drone is modelled as three transparent, configurable terms::

    T(v, theta) = T_back  +  T_drag(v)  +  T_turn(v, theta)

* ``T_back``   — the spool's controlled back-tension. Roughly constant; it is
  what keeps the line from going slack (slack + re-tension = a snatch load).
* ``T_drag``   — aerodynamic drag on the **airborne span** only, ~
  ``0.5 rho Cd d S v**2``, where ``S`` is the length of fiber actually moving
  through air near the drone (drone → ground/canopy contact), NOT the total
  paid-out length.

  ⚠️ **This deviates deliberately from the spec's "∝ paid-out length".** That
  hand-wave (Tech-Selection §7) describes a *towed* line held taut behind the
  aircraft. This is a fiber-**laying** drone: the paid-out fiber is deposited on
  the ground and does not move through the air, so its drag is not dragged along.
  Taken literally, "∝ paid-out length" put the peak tension at the final
  waypoint on a straight leg (34 N over 5 km) — where the speed policy does
  nothing — which is the tell that the assumption was wrong for this platform.
  The airborne span is bounded by flight altitude, so drag is a modest, roughly
  constant baseline rather than a runaway.

* ``T_turn``   — the dynamic tension of redirecting the payout momentum through a
  course change: ``m_eff * v**2 * (1 - cos theta) / L_couple``. ``(1 - cos
  theta)`` is **zero on a straight leg and maximal at a reversal** (theta =
  180°), which is exactly the failure the speed policy exists to prevent, and it
  scales with ``v**2`` so slowing helps quadratically. ``m_eff`` is the effective
  moving mass of the **payout system** (spool feed + near fiber) reflected at the
  drone — not the fiber's own linear mass, which is far too light for its
  momentum to matter (see ``SHORTFALLS``).

This is an **engineering approximation, not cable dynamics** — no catenary, no
wave propagation, no slack/snatch transient, no snag. Every coefficient is a
config knob and the calibration is un-benched: treat absolute newtons as
order-of-magnitude and the *comparisons* (policy on vs off, leg vs reversal) as
the real output. Real numbers need payout-hardware bench data (Tech-Selection §7).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from itertools import pairwise
from pathlib import Path

from droneflight.model import FlightMission
from droneflight.speed_policy import (
    SpeedPolicy,
    _speed_for_bend,
    _to_local_m,
    consolidate_indices,
)

# Glass tensile break for ultra-thin G657A2 fiber is on the order of a few N at
# the proof strain used for payout; controlled back-tension is set well under it.
# These are deliberately conservative order-of-magnitude defaults, not a datasheet.


@dataclass(frozen=True)
class TetherConfig:
    """v0 payout-tension parameters. All SI; every term is switchable via a coefficient.

    ⚠️ Calibration is un-benched (Tech-Selection §7). The defaults are chosen so a
    fast reversal sits near a conservative break budget and the speed policy pulls
    it back under — enough to exercise the *mechanism*; the absolute newtons are
    illustrative until payout-hardware data replaces them.
    """

    fiber_diameter_m: float = 0.00025  # 0.25 mm coated G657A2
    drag_coefficient: float = 1.2  # long thin cylinder in crossflow, order 1
    air_density_kg_m3: float = 1.225
    airborne_span_m: float = 40.0  # fiber length in the airflow (drone → ground); ~ flight altitude
    back_tension_n: float = 0.5  # controlled spool back-tension (keeps line taut)
    payout_inertia_kg: float = 0.15  # effective reflected mass of the payout system at a corner
    coupling_length_m: float = 8.0  # length over which that inertia is redirected
    break_tension_n: float = 2.0  # fiber failure budget (conservative; un-benched)
    warn_fraction: float = 0.6  # fraction of break at which to warn before it snaps

    def __post_init__(self) -> None:
        for name in (
            "fiber_diameter_m",
            "air_density_kg_m3",
            "airborne_span_m",
            "payout_inertia_kg",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if self.coupling_length_m <= 0.0:
            raise ValueError("coupling_length_m must be positive")
        if self.break_tension_n <= 0.0:
            raise ValueError("break_tension_n must be positive")
        if not (0.0 < self.warn_fraction <= 1.0):
            raise ValueError("warn_fraction must be in (0, 1]")


def drag_tension_n(config: TetherConfig, speed_ms: float) -> float:
    """Aerodynamic drag on the airborne span — the steady baseline tension.

    This is also the magnitude of the steady back-pull disturbance force the fiber
    exerts on the airframe, which the Gazebo plugin will apply as a wrench. Note
    it depends on the airborne span (bounded by altitude), not total paid-out
    length: the laid fiber behind the drone is not in the airflow.
    """
    if speed_ms <= 0.0:
        return 0.0
    return (
        0.5
        * config.air_density_kg_m3
        * config.drag_coefficient
        * config.fiber_diameter_m
        * config.airborne_span_m
        * speed_ms**2
    )


def turn_tension_n(config: TetherConfig, speed_ms: float, deflection_deg: float) -> float:
    """Dynamic tension of redirecting the payout momentum through a course change.

    ``(1 - cos theta)`` is 0 on a straight leg and 2 at a full reversal, so this
    term is worst exactly where the speed policy slows down most, and it scales
    with ``v**2``.
    """
    if speed_ms <= 0.0 or deflection_deg <= 0.0:
        return 0.0
    theta = math.radians(deflection_deg)
    return (
        config.payout_inertia_kg * speed_ms**2 * (1.0 - math.cos(theta)) / config.coupling_length_m
    )


def tension_n(config: TetherConfig, *, speed_ms: float, deflection_deg: float) -> float:
    """Total payout tension at the drone: back-tension + drag + turn."""
    return (
        config.back_tension_n
        + drag_tension_n(config, speed_ms)
        + turn_tension_n(config, speed_ms, deflection_deg)
    )


@dataclass(frozen=True)
class TetherLoad:
    """Peak tension at one waypoint and how close it ran to failure."""

    index: int
    paid_out_length_m: float
    speed_ms: float
    deflection_deg: float
    tension_n: float
    break_tension_n: float

    @property
    def load_fraction(self) -> float:
        return self.tension_n / self.break_tension_n

    @property
    def exceeds_break(self) -> bool:
        return self.tension_n > self.break_tension_n


@dataclass(frozen=True)
class TetherReport:
    """Payout-tension profile over a whole mission."""

    loads: tuple[TetherLoad, ...]
    config: TetherConfig

    @property
    def max_tension_n(self) -> float:
        return max((load.tension_n for load in self.loads), default=0.0)

    @property
    def peak(self) -> TetherLoad | None:
        return max(self.loads, key=lambda load: load.tension_n, default=None)

    @property
    def breaks(self) -> tuple[TetherLoad, ...]:
        return tuple(load for load in self.loads if load.exceeds_break)

    @property
    def warnings(self) -> tuple[TetherLoad, ...]:
        thresh = self.config.warn_fraction
        return tuple(
            load for load in self.loads if not load.exceeds_break and load.load_fraction >= thresh
        )

    @property
    def within_limit(self) -> bool:
        return not self.breaks


def _deflection_deg(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]
) -> float:
    """Course change (deg) at ``b`` on the polyline a→b→c. 0 = straight, 180 = reversal."""
    u1 = (b[0] - a[0], b[1] - a[1])
    u2 = (c[0] - b[0], c[1] - b[1])
    l1 = math.hypot(*u1)
    l2 = math.hypot(*u2)
    if l1 == 0.0 or l2 == 0.0:
        return 0.0
    dot = (u1[0] * u2[0] + u1[1] * u2[1]) / (l1 * l2)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def analyse_tether(
    mission: FlightMission,
    config: TetherConfig,
    *,
    policy: SpeedPolicy | None = None,
) -> TetherReport:
    """Payout tension along a mission, at the speeds the policy would fly.

    With ``policy`` set, each vertex's speed is the tether-safe policy speed; with
    ``policy=None`` every vertex runs at ``cruise_speed_ms`` — the control that
    shows what the fiber would see if the drone never slowed. Tension is scored
    on the route **consolidated** to real vertices (duplicate-waypoint clusters
    merged), so a turn is seen at its true angle rather than smeared to ~0°.
    """
    lat0 = mission.home_lat
    raw = [_to_local_m(w.lat, w.lon, lat0) for w in mission.waypoints]
    consolidate_below = policy.consolidate_below_m if policy is not None else 2.0
    reps, _group = consolidate_indices(raw, consolidate_below)
    pts = [raw[i] for i in reps]

    # Cumulative flown distance = paid-out length at each vertex (drone-side).
    paid_out = [0.0]
    for p, q in pairwise(pts):
        paid_out.append(paid_out[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))

    loads: list[TetherLoad] = []
    for i in range(len(pts)):
        deflection = (
            0.0 if i == 0 or i == len(pts) - 1 else _deflection_deg(pts[i - 1], pts[i], pts[i + 1])
        )
        if policy is None:
            speed = mission.cruise_speed_ms
        elif i == 0 or i == len(pts) - 1:
            speed = policy.cruise_speed_ms
        else:
            speed = _speed_for_bend(deflection, policy)
        t = tension_n(config, speed_ms=speed, deflection_deg=deflection)
        loads.append(
            TetherLoad(
                index=reps[i],
                paid_out_length_m=paid_out[i],
                speed_ms=speed,
                deflection_deg=deflection,
                tension_n=t,
                break_tension_n=config.break_tension_n,
            )
        )
    return TetherReport(loads=tuple(loads), config=config)


# --------------------------------------------------------------------------- #
# In-the-loop half: reading back what the Gazebo plugin actually applied
# --------------------------------------------------------------------------- #
#
# ``sitl/tether_plugin/`` mirrors the model above in C++ so PX4 SITL can feel the
# payout disturbance. The mirror is only trustworthy if something checks it, so
# the plugin logs every tension it applies at full precision and the functions
# below re-derive those numbers from *this* module and diff them. That is the
# acceptance gate in TETHER_PLUGIN.md: the in-loop force must BE the model, not
# a second copy of it that quietly drifts.


class TetherLogError(ValueError):
    """A plugin CSV that cannot be read as one."""


@dataclass(frozen=True)
class PluginLogRow:
    """One logged physics sample from the Gazebo plugin."""

    t_s: float
    x_m: float
    y_m: float
    z_m: float
    vx_ms: float
    vy_ms: float
    speed_ms: float
    drag_n: float
    tension_n: float
    lateral_accel_ms2: float
    deflection_deg: float
    turn_n: float
    fx_n: float
    fy_n: float
    moving: bool


@dataclass(frozen=True)
class PluginLog:
    """A parsed plugin CSV, including the coefficients it recorded in its header.

    The header matters as much as the rows: it is what makes a log self-
    describing, so a run can be scored months later without trusting a shell
    script's memory of which parameters it passed.
    """

    rows: tuple[PluginLogRow, ...]
    config: TetherConfig
    speed_floor_ms: float
    enabled: bool
    link: str


def _parse_header_params(text: str) -> dict[str, str]:
    return dict(
        token.split("=", 1) for token in text.split() if "=" in token and not token.startswith("=")
    )


def parse_plugin_log(path: Path) -> PluginLog:
    """Read a ``log_csv`` written by the Gazebo tether plugin."""
    config_names = {f.name for f in fields(TetherConfig)}
    params: dict[str, str] = {}
    rows: list[PluginLogRow] = []
    header_seen = False

    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            params.update(_parse_header_params(line[1:]))
            continue
        if not header_seen:
            header_seen = True  # the column-name row
            continue
        parts = line.split(",")
        if len(parts) != 15:
            raise TetherLogError(f"{path}: expected 15 columns, got {len(parts)}: {line[:60]}")
        try:
            values = [float(p) for p in parts]
        except ValueError as exc:
            raise TetherLogError(f"{path}: non-numeric row: {line[:60]}") from exc
        rows.append(
            PluginLogRow(
                t_s=values[0],
                x_m=values[1],
                y_m=values[2],
                z_m=values[3],
                vx_ms=values[4],
                vy_ms=values[5],
                speed_ms=values[6],
                drag_n=values[7],
                tension_n=values[8],
                lateral_accel_ms2=values[9],
                deflection_deg=values[10],
                turn_n=values[11],
                fx_n=values[12],
                fy_n=values[13],
                moving=values[14] != 0.0,
            )
        )

    if not rows:
        raise TetherLogError(f"{path}: no data rows — did the plugin load? (look for '[tether]')")

    try:
        # Only the fields TetherConfig shares with the plugin; break_tension_n and
        # warn_fraction are analysis-only and the plugin never sees them.
        config = TetherConfig(
            **{k: float(v) for k, v in params.items() if k in config_names},
        )
    except (ValueError, TypeError) as exc:
        raise TetherLogError(f"{path}: bad parameter header: {exc}") from exc

    return PluginLog(
        rows=tuple(rows),
        config=config,
        speed_floor_ms=float(params.get("speed_floor_ms", 0.3)),
        enabled=params.get("enabled", "1") not in ("0", "false"),
        link=params.get("link", "?"),
    )


@dataclass(frozen=True)
class PluginParity:
    """How far the in-loop plugin drifted from this module, and what it applied.

    ``exact`` is the gate. The errors are not tolerances that happen to be small:
    the C++ mirrors this module's association order, so anything other than
    identically zero means the two implementations have genuinely diverged.

    ``max_force_error_n`` is deliberately **not** part of ``exact``: it compares
    a re-normalised vector magnitude against a scalar, so a few ULP is expected
    on any heading that is not axis-aligned. It catches a wrong-direction or
    wrong-magnitude wrench, not drift.
    """

    samples: int
    moving_samples: int
    max_drag_error_n: float
    max_tension_error_n: float
    max_turn_error_n: float
    max_force_error_n: float
    bad_static_samples: int
    peak_tension_n: float
    mean_tension_n: float
    peak_turn_n: float
    peak_speed_ms: float

    @property
    def exact(self) -> bool:
        return (
            self.max_drag_error_n == 0.0
            and self.max_tension_error_n == 0.0
            and self.max_turn_error_n == 0.0
            and self.bad_static_samples == 0
        )

    def load_fraction(self, break_tension_n: float) -> float:
        return self.peak_tension_n / break_tension_n


def check_plugin_parity(log: PluginLog) -> PluginParity:
    """Re-derive every logged tension from this module and diff it.

    Scored on the **moving** samples: those are the ones with a speed-dependent
    tension, and "at a commanded steady speed the logged tension equals
    ``drag_tension_n(speed)``" is the acceptance criterion. Below the speed floor
    the plugin has no velocity heading to work from, so it applies either pure
    back-tension (line out, hovering) or nothing at all (never left the pad) —
    anything else there is counted as a defect rather than diffed.
    """
    config = log.config
    drag_err = tension_err = turn_err = force_err = 0.0
    bad_static = 0
    moving = 0
    tension_sum = 0.0
    peak_tension = peak_turn = peak_speed = 0.0

    for row in log.rows:
        tension_sum += row.tension_n
        peak_tension = max(peak_tension, row.tension_n)
        peak_turn = max(peak_turn, row.turn_n)
        peak_speed = max(peak_speed, row.speed_ms)

        # The turn term is a logged diagnostic, not an applied force, but it is
        # still this module's function and must still agree.
        turn_err = max(
            turn_err,
            abs(row.turn_n - turn_tension_n(config, row.speed_ms, row.deflection_deg)),
        )

        if row.moving:
            moving += 1
            expected_drag = drag_tension_n(config, row.speed_ms)
            drag_err = max(drag_err, abs(row.drag_n - expected_drag))
            tension_err = max(
                tension_err, abs(row.tension_n - (config.back_tension_n + expected_drag))
            )
        elif row.tension_n not in (0.0, config.back_tension_n):
            bad_static += 1

        # The wrench magnitude must be the tension: a direction bug would leave
        # every scalar above correct while pulling the wrong way.
        force_err = max(force_err, abs(math.hypot(row.fx_n, row.fy_n) - row.tension_n))

    return PluginParity(
        samples=len(log.rows),
        moving_samples=moving,
        max_drag_error_n=drag_err,
        max_tension_error_n=tension_err,
        max_turn_error_n=turn_err,
        max_force_error_n=force_err,
        bad_static_samples=bad_static,
        peak_tension_n=peak_tension,
        mean_tension_n=tension_sum / len(log.rows),
        peak_turn_n=peak_turn,
        peak_speed_ms=peak_speed,
    )
