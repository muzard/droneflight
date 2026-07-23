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
from dataclasses import dataclass
from itertools import pairwise

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
