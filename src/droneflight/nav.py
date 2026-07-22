"""GPS-denied navigation-error emulation (Technology-Selection §2.1, §7).

This is **not** a state estimator — it is an emulation of the *error* a real
GPS-denied stack would produce, so we can quantify whether the drone can hold a
planned route before any perception code exists (and later feed PX4's
external-vision input in SITL with realistic error).

Two layers, matching the recommended architecture:

* **VIO/LIO drift** — relative odometry drifts ~0.5-3% of distance travelled
  (a slowly-wandering biased random walk); with this as the only source, error
  is unbounded over kilometers.
* **VTRN correction** — periodic absolute fixes (map-matching the downward camera
  to a georeferenced basemap) reset the accumulated error to a small bounded
  residual, keeping drift in check over the whole route.

Deterministic given the seed. All geometry is local ENU meters about the home
point.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from itertools import pairwise

from droneflight.correction import (
    CorrectionConfig,
    Fix,
    FixOutcome,
    OpticalCorrector,
    PatchyReference,
    ReferenceMap,
)
from droneflight.model import FlightMission

_EARTH_R = 6_371_000.0


@dataclass(frozen=True)
class DriftConfig:
    """VIO/LIO relative-odometry drift model."""

    rate: float = 0.02  # error accrued per meter travelled (fraction of distance)
    wander_deg_per_100m: float = 20.0  # how fast the drift direction meanders
    seed: int = 0


@dataclass(frozen=True)
class VtrnConfig:
    """Absolute-correction (map-matching) model."""

    fix_interval_m: float = 500.0  # distance between absolute fixes
    residual_error_m: float = 2.0  # bounded error left after a fix


class DriftWalker:
    """Incremental VIO/LIO drift — the stateful core of the drift model.

    Error accrues *coherently*: a slowly-wandering heading times a fixed fraction
    of the distance travelled, so it grows like a biased random walk rather than
    averaging out. Callers step it by distance, which is what makes the same
    model usable offline (over a densified route) and online (over a live truth
    stream — see :mod:`droneflight.ev_bridge`).

    ``rng`` is injectable so a caller that needs *other* draws interleaved with
    the walk (the idealised VTRN reset below) can share one stream and stay
    deterministic.
    """

    def __init__(self, config: DriftConfig, rng: random.Random | None = None) -> None:
        self.config = config
        self._rng = rng if rng is not None else random.Random(config.seed)
        self._heading = self._rng.uniform(0.0, 2.0 * math.pi)
        self._ex = 0.0
        self._ey = 0.0

    @property
    def error(self) -> tuple[float, float]:
        """Current estimate-minus-truth error vector, local-ENU meters."""
        return self._ex, self._ey

    def set_error(self, error: tuple[float, float]) -> None:
        """Overwrite the accumulated error (after an absolute fix)."""
        self._ex, self._ey = error

    def advance(self, distance_m: float) -> tuple[float, float]:
        """Accrue drift over ``distance_m`` and return the new error vector."""
        cfg = self.config
        self._heading += self._rng.gauss(
            0.0, math.radians(cfg.wander_deg_per_100m) * distance_m / 100.0
        )
        self._ex += cfg.rate * distance_m * math.cos(self._heading)
        self._ey += cfg.rate * distance_m * math.sin(self._heading)
        return self._ex, self._ey


class ErrorInjector:
    """Incremental GPS-denied error: coherent drift + gated optical fixes.

    The online counterpart of :func:`simulate_corrected_navigation` — the same
    two layers, but stepped by *travelled distance* instead of walked over a
    precomputed path, so it can corrupt a live ground-truth stream. Both share
    this implementation, so what SITL flies is the model the offline study
    measured, not a re-derivation of it.

    With ``reference=None`` the optical layer is off entirely: pure dead
    reckoning, unbounded error. That is the "VTRN disabled" arm of the A/B.
    """

    def __init__(
        self,
        drift: DriftConfig,
        reference: ReferenceMap | None = None,
        config: CorrectionConfig | None = None,
    ) -> None:
        self.config = config or CorrectionConfig()
        self.reference = reference
        self._walker = DriftWalker(drift)
        self._corrector = OpticalCorrector(self.config, seed=drift.seed)
        self._since_attempt = 0.0
        self.distance_m = 0.0
        self.counts = {outcome: 0 for outcome in FixOutcome}

    @property
    def error(self) -> tuple[float, float]:
        """Current estimate-minus-truth error vector, local-ENU meters."""
        return self._walker.error

    @property
    def attempts(self) -> int:
        return sum(self.counts.values())

    @property
    def accepted(self) -> int:
        return self.counts[FixOutcome.ACCEPTED] + self.counts[FixOutcome.ACCEPTED_CORRUPT]

    def advance(self, distance_m: float, x: float, y: float) -> Fix | None:
        """Step the error by ``distance_m``; maybe attempt a fix at true ``(x, y)``.

        Returns the :class:`~droneflight.correction.Fix` if a match was attempted
        at this step, else ``None``. ``(x, y)`` is the **true** position (the
        nadir camera looks at the real world, not at the estimate), so the caller
        must pass ground truth here.
        """
        if distance_m <= 0.0:
            return None
        self._walker.advance(distance_m)
        self.distance_m += distance_m
        if self.reference is None:
            return None

        self._corrector.propagate(distance_m)
        self._since_attempt += distance_m
        if self._since_attempt < self.config.attempt_interval_m:
            return None

        fix, error = self._corrector.attempt(self._walker.error, self.reference.sample(x, y))
        self._walker.set_error(error)
        self.counts[fix.outcome] += 1
        self._since_attempt = 0.0
        return fix


@dataclass(frozen=True)
class NavResult:
    """Emulated navigation performance over a route."""

    distance_m: float
    max_error_m: float
    endpoint_error_m: float
    mean_error_m: float
    fixes: int
    within_corridor: bool  # max_error <= corridor_tolerance_m


def _to_local(lat: float, lon: float, lat0: float) -> tuple[float, float]:
    x = math.radians(lon) * math.cos(math.radians(lat0)) * _EARTH_R
    y = math.radians(lat) * _EARTH_R
    return x, y


def local_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Local-ENU meters (east, north) of ``(lat, lon)`` about a reference point.

    Equirectangular about ``ref_lat`` — sub-meter over a 5 km route, which is far
    inside the corridor tolerances this is compared against. Used to put a
    ``.plan`` in the same frame as the simulator's world origin.
    """
    x, y = _to_local(lat, lon, ref_lat)
    x0, y0 = _to_local(ref_lat, ref_lon, ref_lat)
    return x - x0, y - y0


def _densify(pts: list[tuple[float, float]], step_m: float) -> list[tuple[float, float]]:
    out = [pts[0]]
    for (ax, ay), (bx, by) in pairwise(pts):
        seg = math.hypot(bx - ax, by - ay)
        n = max(1, int(seg / step_m))
        for i in range(1, n + 1):
            t = i / n
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return out


def simulate_navigation(
    mission: FlightMission,
    drift: DriftConfig,
    vtrn: VtrnConfig | None = None,
    *,
    corridor_tolerance_m: float = 15.0,
    sample_step_m: float = 10.0,
) -> NavResult:
    """Emulate GPS-denied navigation error along a mission's route."""
    lat0 = mission.home_lat
    local = [_to_local(w.lat, w.lon, lat0) for w in mission.waypoints]
    path = _densify(local, sample_step_m)

    rng = random.Random(drift.seed)
    walker = DriftWalker(drift, rng)
    dist_since_fix = 0.0
    total = 0.0
    fixes = 0
    max_err = 0.0
    err_sum = 0.0
    last_err = 0.0

    for (ax, ay), (bx, by) in pairwise(path):
        seg = math.hypot(bx - ax, by - ay)
        if seg == 0.0:
            continue
        total += seg
        ex, ey = walker.advance(seg)
        dist_since_fix += seg

        if vtrn is not None and dist_since_fix >= vtrn.fix_interval_m:
            ang = rng.uniform(0.0, 2.0 * math.pi)
            ex = vtrn.residual_error_m * math.cos(ang)
            ey = vtrn.residual_error_m * math.sin(ang)
            walker.set_error((ex, ey))
            dist_since_fix = 0.0
            fixes += 1

        err = math.hypot(ex, ey)
        last_err = err
        max_err = max(max_err, err)
        err_sum += err

    n = max(1, len(path) - 1)
    return NavResult(
        distance_m=total,
        max_error_m=max_err,
        endpoint_error_m=last_err,
        mean_error_m=err_sum / n,
        fixes=fixes,
        within_corridor=max_err <= corridor_tolerance_m,
    )


@dataclass(frozen=True)
class CorrectedNavResult:
    """Navigation error under *realistic*, fidelity-limited optical correction.

    Unlike :class:`NavResult`'s idealised VTRN (a scheduled reset), this reports
    what the :mod:`droneflight.correction` corrector actually did against a
    spatially varying reference: how many match attempts produced a usable fix,
    how many were declined for want of texture or confidence, how many blunders
    the gate caught, and how many slipped through (``corrupt_accepted``).
    """

    distance_m: float
    max_error_m: float
    endpoint_error_m: float
    mean_error_m: float
    attempts: int
    accepted: int
    no_features: int
    low_confidence: int
    gated_out: int
    corrupt_accepted: int
    within_corridor: bool

    @property
    def fix_availability(self) -> float:
        """Fraction of match attempts that yielded an accepted fix."""
        return self.accepted / self.attempts if self.attempts else 0.0


def route_reference(
    mission: FlightMission,
    *,
    forest_frac: float = 0.4,
    stale_frac: float = 0.1,
    seed: int = 0,
) -> PatchyReference:
    """Build a route-aligned :class:`PatchyReference` for ``mission``."""
    lat0 = mission.home_lat
    pts = [_to_local(w.lat, w.lon, lat0) for w in mission.waypoints]
    return PatchyReference.along_route(
        pts, forest_frac=forest_frac, stale_frac=stale_frac, seed=seed
    )


def simulate_corrected_navigation(
    mission: FlightMission,
    drift: DriftConfig,
    reference: ReferenceMap,
    config: CorrectionConfig | None = None,
    *,
    corridor_tolerance_m: float = 15.0,
    sample_step_m: float = 10.0,
) -> CorrectedNavResult:
    """Emulate GPS-denied error with data-driven optical correction.

    VIO/LIO drift accrues along the route exactly as in
    :func:`simulate_navigation`; the difference is the correction. At every
    ``config.attempt_interval_m`` a match is attempted against ``reference`` at
    the drone's true position, then robustly gated and fused. Where the
    reference is thin (forest canopy) or stale (conflict zone) fixes are sparse
    or rejected and the error is free to grow — the behaviour the idealised
    model hides.
    """
    config = config or CorrectionConfig()
    lat0 = mission.home_lat
    local = [_to_local(w.lat, w.lon, lat0) for w in mission.waypoints]
    path = _densify(local, sample_step_m)

    injector = ErrorInjector(drift, reference, config)
    max_err = 0.0
    err_sum = 0.0
    last_err = 0.0

    for (ax, ay), (bx, by) in pairwise(path):
        seg = math.hypot(bx - ax, by - ay)
        injector.advance(seg, bx, by)
        if seg == 0.0:
            continue
        err = math.hypot(*injector.error)
        last_err = err
        max_err = max(max_err, err)
        err_sum += err

    n = max(1, len(path) - 1)
    counts = injector.counts
    return CorrectedNavResult(
        distance_m=injector.distance_m,
        max_error_m=max_err,
        endpoint_error_m=last_err,
        mean_error_m=err_sum / n,
        attempts=injector.attempts,
        accepted=injector.accepted,
        no_features=counts[FixOutcome.NO_FEATURES],
        low_confidence=counts[FixOutcome.LOW_CONFIDENCE],
        gated_out=counts[FixOutcome.GATED_OUT],
        corrupt_accepted=counts[FixOutcome.ACCEPTED_CORRUPT],
        within_corridor=max_err <= corridor_tolerance_m,
    )
