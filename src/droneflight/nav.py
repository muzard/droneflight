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


@dataclass(frozen=True)
class VelocityErrorConfig:
    """Error of the ESTIMATED velocity — the one the planner predicts from and PX4
    closes its velocity loop on. An Ornstein-Uhlenbeck process per axis.

    MEASURED 2026-09-25 against Gazebo truth on the fly-local SITL records
    (sim-logs/mvp_local): GPS-denied (EV + VIO velocity, r3e/s3e/r4e/d400n)
    0.15-0.31 m/s per axis, 1/e correlation time 2.8-3.2 s, |e| p95 0.53-0.88;
    GPS-aided (gps1) ~0.1 m/s, 1.0 s. Not a lag (best-fit lag 0.00 s). The harness
    fed the planner the TRUE velocity and never reproduced a SITL contact.
    """

    sigma_ms: float = 0.22
    tau_s: float = 3.0
    seed: int = 0


@dataclass(frozen=True)
class YawDriftConfig:
    """Heading error of a GPS-denied estimator.

    Yaw is the weak axis of a visual-inertial estimate: gravity observes roll and
    pitch, but nothing observes heading, so it walks freely. Quoted per 100 m
    rather than per second so the model stays reproducible whatever rate the
    caller steps it at — the same reason :class:`DriftWalker` is distance-driven.

    All-zero by default, which makes the yaw layer an exact no-op: existing
    callers of :class:`ErrorInjector` keep their bit-identical results.
    """

    bias_deg: float = 0.0  # constant misalignment, present from the first sample
    wander_deg_per_100m: float = 0.0  # random-walk growth of the heading error
    seed: int = 0

    @property
    def active(self) -> bool:
        return self.bias_deg != 0.0 or self.wander_deg_per_100m != 0.0


@dataclass(frozen=True)
class DropoutConfig:
    """External-vision lock loss — the stream stops, rather than degrading.

    Real VIO does not fade out; it loses tracking and publishes nothing until it
    re-acquires. Onset is a Poisson process in **distance** (reproducible across
    publish rates); duration is in **seconds**, because re-acquisition time is a
    property of the estimator, not of how far the vehicle flew. The mixed units
    are deliberate.

    ``mean_interval_m = 0`` disables the layer entirely.
    """

    mean_interval_m: float = 0.0  # mean distance between lock losses (0 = never)
    duration_s: float = 2.0  # how long each loss lasts
    seed: int = 0

    @property
    def active(self) -> bool:
        return self.mean_interval_m > 0.0


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
        *,
        correction_ramp_m: float = 0.0,
        yaw: YawDriftConfig | None = None,
    ) -> None:
        self.config = config or CorrectionConfig()
        self.reference = reference
        # Heading error, off unless asked for. Its own RNG, so enabling it cannot
        # shift the horizontal drift draws (see YawWalker).
        self._yaw = YawWalker(yaw) if yaw is not None and yaw.active else None
        # Distance over which an accepted fix is bled into the reported error
        # instead of applied in one step. 0 = instant (the offline default).
        #
        # Instant application is wrong at the autopilot boundary: it teleports the
        # *published* pose by up to ~10 m, and EKF2's innovation gate rejects the
        # jump — measured in SITL as 13 momentary losses of EV fusion, each
        # ~1.1 s after an accepted fix, with test ratios to 311. A real pipeline
        # absorbs a map match over the filter's own dynamics; the pose handed to
        # the autopilot stays continuous. Ramping models that, and keeps EKF2
        # fusing the correction instead of discarding it.
        self.correction_ramp_m = correction_ramp_m
        self._pending = (0.0, 0.0)  # correction not yet bled into the error
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
    def yaw_error_rad(self) -> float:
        """Current heading error, radians. 0.0 when the yaw layer is off."""
        return self._yaw.error_rad if self._yaw is not None else 0.0

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
        if self._yaw is not None:
            self._yaw.advance(distance_m)
        self._bleed_pending(distance_m)
        self.distance_m += distance_m
        if self.reference is None:
            return None

        self._corrector.propagate(distance_m)
        self._since_attempt += distance_m
        if self._since_attempt < self.config.attempt_interval_m:
            return None

        # The corrector returns the error *after* fusing. With a ramp we hold the
        # delta back and bleed it in over the next `correction_ramp_m` metres, so
        # the pose the autopilot sees never steps.
        before = self._walker.error
        fix, error = self._corrector.attempt(before, self.reference.sample(x, y))
        if self.correction_ramp_m > 0.0:
            px, py = self._pending
            self._pending = (px + error[0] - before[0], py + error[1] - before[1])
        else:
            self._walker.set_error(error)
        self.counts[fix.outcome] += 1
        self._since_attempt = 0.0
        return fix

    def _bleed_pending(self, distance_m: float) -> None:
        """Apply a slice of any held-back correction, proportional to distance."""
        if self.correction_ramp_m <= 0.0:
            return
        px, py = self._pending
        if px == 0.0 and py == 0.0:
            return
        frac = min(1.0, distance_m / self.correction_ramp_m)
        ex, ey = self._walker.error
        self._walker.set_error((ex + px * frac, ey + py * frac))
        self._pending = (px * (1.0 - frac), py * (1.0 - frac))


class YawWalker:
    """Heading error that accrues with distance flown.

    Kept separate from :class:`DriftWalker` — and drawing from its **own** RNG —
    so that switching the yaw layer on cannot perturb the horizontal drift
    sequence. Without that separation, adding a yaw axis would silently change
    every existing position result, and the offline/online parity the whole
    experiment rests on would break.
    """

    def __init__(self, config: YawDriftConfig, rng: random.Random | None = None) -> None:
        self.config = config
        self._rng = rng if rng is not None else random.Random(config.seed)
        self._error = math.radians(config.bias_deg)
        self.distance_m = 0.0

    @property
    def error_rad(self) -> float:
        """Current heading error (estimate minus truth), radians, unwrapped."""
        return self._error

    def advance(self, distance_m: float) -> float:
        """Accrue heading drift over ``distance_m`` and return the new error.

        The per-step deviation scales with **sqrt** of distance, not distance.
        That is what makes a random walk step-size invariant: variance adds, so
        it must be variance that is proportional to distance. Scaling the
        *deviation* linearly instead makes total variance proportional to
        ``sum(d_i**2)``, which collapses toward zero as the caller steps more
        finely — the walk quietly stops walking.

        This is not hypothetical: :class:`DriftWalker` scales its heading wander
        linearly, and at the bridge's ~0.24 m steps its heading is effectively
        frozen. The flown control arm shows it — injected error of exactly
        1.98/3.99/5.99/8.00 m at 100/200/300/400 m on a constant 146.5 deg
        bearing, a straight line rather than the documented biased walk. That
        model is left alone because the acceptance was measured with it; this
        one is written correctly from the start.
        """
        if distance_m <= 0.0:
            return self._error
        self.distance_m += distance_m
        if self.config.wander_deg_per_100m:
            self._error += self._rng.gauss(
                0.0, math.radians(self.config.wander_deg_per_100m) * math.sqrt(distance_m / 100.0)
            )
        return self._error


class DropoutModel:
    """Whether the external-vision estimate is available at this instant.

    Stateful and deterministic. Onset is drawn as an exponential in travelled
    distance; once a dropout starts it runs for ``duration_s`` of *simulator*
    time, and the next onset is only scheduled after the current one clears (so
    dropouts never overlap, which is what a single estimator does).
    """

    def __init__(self, config: DropoutConfig, rng: random.Random | None = None) -> None:
        self.config = config
        self._rng = rng if rng is not None else random.Random(config.seed)
        self._distance_m = 0.0
        self._next_onset_m: float | None = None
        self._down_until_t: float | None = None
        self.events = 0
        self.samples_down = 0

    @property
    def down(self) -> bool:
        return self._down_until_t is not None

    def _schedule(self) -> None:
        self._next_onset_m = self._distance_m + self._rng.expovariate(
            1.0 / self.config.mean_interval_m
        )

    def step(self, distance_m: float, t_s: float) -> bool:
        """Advance by ``distance_m`` at sim time ``t_s``; return True if EV is down."""
        if not self.config.active:
            return False
        self._distance_m += max(0.0, distance_m)
        if self._next_onset_m is None:
            self._schedule()

        if self._down_until_t is not None:
            if t_s >= self._down_until_t:
                self._down_until_t = None
                self._schedule()
            else:
                self.samples_down += 1
                return True

        assert self._next_onset_m is not None
        if self._distance_m >= self._next_onset_m:
            self._down_until_t = t_s + self.config.duration_s
            self.events += 1
            self.samples_down += 1
            return True
        return False


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
    home_relative: bool = False,
) -> PatchyReference:
    """Build a route-aligned :class:`PatchyReference` for ``mission``.

    ⚠️ **Frame.** The returned map is a *spatial field*, so it only works if
    callers sample it in the frame its polyline was built in.
    :func:`simulate_corrected_navigation` walks the route in the absolute-ish
    local metres of :func:`_to_local`, which is the default here. The live bridge
    works in **home-relative ENU** (Gazebo's world frame) and must pass
    ``home_relative=True``.

    Getting this wrong is silent and ruinous: every sample lands millions of
    metres from the polyline, projects onto the same endpoint, and the whole
    varying field collapses to a single bucket — usually reporting a perfect map
    everywhere. :meth:`PatchyReference.sample` now raises instead.
    """
    if home_relative:
        pts = [
            local_enu(w.lat, w.lon, mission.home_lat, mission.home_lon) for w in mission.waypoints
        ]
    else:
        pts = [_to_local(w.lat, w.lon, mission.home_lat) for w in mission.waypoints]
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
