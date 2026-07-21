"""Optical terrain-relative position correction (VTRN) — the drift-bounding layer.

Technology-Selection §2.1 calls the absolute-correction layer "the single most
important architectural decision in the whole system": pure VIO/LIO drifts
0.5-3 % of distance, i.e. **25-150 m over 5 km**, so a GPS-denied route needs
periodic *absolute* fixes to stay in its concealment corridor. Those fixes come
from **visual terrain-relative navigation** — matching the downward camera to a
pre-loaded georeferenced orthophoto/DSM (§5.2). §9's residual risk is blunt:
"VTRN is only as good as its basemap."

`nav.py` originally modelled that layer as an *idealised reset*: every fixed
interval, error teleports back to a small constant residual. That silently
assumes a fix is (1) **available everywhere**, (2) **always accepted**, and
(3) **always correct**. All three are false in the field, and worst exactly
where this drone operates:

* **Availability** depends on matchable texture. A downward camera over uniform
  forest canopy, water, snow, or fresh sand sees few geo-registered features —
  and *forest is where this drone deliberately hides*. So the concealment
  objective and the navigation objective are in tension: the drone is most lost
  where it is most concealed.
* **Correctness** depends on the reference matching the present world. Over a
  conflict zone the basemap goes stale fast — new craters, rubble, defoliation,
  construction, seasonal canopy — so a confident-looking match can be **wrong**.

This module makes the reference's fidelity a first-class, spatially varying
input (:class:`ReferenceMap`) and turns the correction into a real measurement:
attempt a match, score its confidence, **gate** it against the propagated
estimate (rejecting blunders), and fuse it with a scalar Kalman update so error
*shrinks toward the fix accuracy* instead of teleporting. Where the map cannot
be trusted, the corrector declines and the stack falls back to dead-reckoning —
which is the honest behaviour, and the whole point.

Pure Python, deterministic given the seed. All geometry is local-ENU meters;
positions enter only as the estimate-minus-truth **error vector** ``(ex, ey)``,
so the corrector is independent of the absolute frame.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from enum import Enum
from itertools import pairwise
from typing import Protocol, runtime_checkable

__all__ = [
    "CorrectionConfig",
    "Fix",
    "FixOutcome",
    "MapSample",
    "OpticalCorrector",
    "PatchyReference",
    "ReferenceMap",
    "UniformReference",
]


@dataclass(frozen=True)
class MapSample:
    """The reference basemap's quality at one location.

    * ``matchability`` — density/quality of geo-registered features the nadir
      camera can lock onto here, in [0, 1]. High over textured open terrain,
      roads, buildings; low over uniform canopy, water, snow, sand. Drives
      whether a fix is *available* at all.
    * ``staleness`` — probability the reference disagrees with the present world
      here, in [0, 1]. Small in stable areas; large in conflict zones where the
      ground truth changes between basemap capture and the mission. Drives
      whether an available match is a **blunder**.
    * ``accuracy_m`` — georegistration accuracy of a *good* match here (≈ ortho
      GSD / DSM quality); the noise floor a clean fix leaves behind.
    """

    matchability: float
    staleness: float
    accuracy_m: float


@runtime_checkable
class ReferenceMap(Protocol):
    """A georeferenced reference the VTRN pipeline matches the nadir feed to."""

    def sample(self, x: float, y: float) -> MapSample:
        """Reference quality at local-ENU meters ``(x, y)``."""
        ...


@dataclass(frozen=True)
class UniformReference:
    """A reference of uniform quality everywhere — the optimistic baseline.

    Useful as a control: it is the closest analogue to the old idealised model,
    and shows how much rosier "the map is good everywhere" is than reality.
    """

    matchability: float = 0.85
    staleness: float = 0.02
    accuracy_m: float = 3.0

    def sample(self, x: float, y: float) -> MapSample:
        return MapSample(self.matchability, self.staleness, self.accuracy_m)


@dataclass(frozen=True)
class PatchyReference:
    """Reference quality that varies along a route: the realistic case.

    Built by :meth:`along_route`, which buckets the route by arc length and
    marks some buckets as **forest** (low matchability — the drone's own cover
    starves VTRN) and some as **stale** (high staleness — conflict-zone change
    that can produce confident but wrong matches). Everything else is decent
    open terrain. Queries project onto the polyline to find their bucket, so the
    field is defined over the whole plane, not just on the line.
    """

    _pts: tuple[tuple[float, float], ...]
    _cum: tuple[float, ...]
    _matchability: tuple[float, ...]
    _staleness: tuple[float, ...]
    _accuracy: tuple[float, ...]
    open_matchability: float
    forest_matchability: float
    open_staleness: float
    stale_staleness: float
    accuracy_m: float

    @classmethod
    def along_route(
        cls,
        pts: list[tuple[float, float]],
        *,
        n_buckets: int = 40,
        forest_frac: float = 0.4,
        stale_frac: float = 0.1,
        open_matchability: float = 0.85,
        forest_matchability: float = 0.10,
        open_staleness: float = 0.02,
        stale_staleness: float = 0.5,
        accuracy_m: float = 3.0,
        forest_accuracy_m: float = 6.0,
        seed: int = 0,
    ) -> PatchyReference:
        """Build a route-aligned reference with forest and stale patches.

        ``forest_frac`` / ``stale_frac`` are the fractions of route length that
        are (respectively) low-texture forest and stale conflict-zone; the
        patches are placed deterministically from ``seed``.
        """
        if len(pts) < 2:
            raise ValueError("need at least two route points")
        cum = [0.0]
        for a, b in pairwise(pts):
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

        n = max(1, n_buckets)
        rng = random.Random(seed)
        order = list(range(n))
        rng.shuffle(order)
        forest = set(order[: round(forest_frac * n)])
        rng.shuffle(order)
        stale = set(order[: round(stale_frac * n)])

        matchability = tuple(
            forest_matchability if i in forest else open_matchability for i in range(n)
        )
        staleness = tuple(stale_staleness if i in stale else open_staleness for i in range(n))
        accuracy = tuple(forest_accuracy_m if i in forest else accuracy_m for i in range(n))
        return cls(
            _pts=tuple(pts),
            _cum=tuple(cum),
            _matchability=matchability,
            _staleness=staleness,
            _accuracy=accuracy,
            open_matchability=open_matchability,
            forest_matchability=forest_matchability,
            open_staleness=open_staleness,
            stale_staleness=stale_staleness,
            accuracy_m=accuracy_m,
        )

    def _arclen_of_nearest(self, x: float, y: float) -> float:
        """Arc length of the closest point on the polyline to ``(x, y)``."""
        best_d2 = math.inf
        best_s = 0.0
        for i, (a, b) in enumerate(pairwise(self._pts)):
            ax, ay = a
            bx, by = b
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0.0 else ((x - ax) * dx + (y - ay) * dy) / seg2
            t = max(0.0, min(1.0, t))
            px, py = ax + t * dx, ay + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = self._cum[i] + t * math.sqrt(seg2)
        return best_s

    def sample(self, x: float, y: float) -> MapSample:
        total = self._cum[-1]
        n = len(self._matchability)
        if total <= 0.0:
            idx = 0
        else:
            frac = self._arclen_of_nearest(x, y) / total
            idx = min(n - 1, max(0, int(frac * n)))
        return MapSample(self._matchability[idx], self._staleness[idx], self._accuracy[idx])


@dataclass(frozen=True)
class CorrectionConfig:
    """Policy for accepting, rejecting, and fusing optical fixes."""

    attempt_interval_m: float = 50.0  # distance between VTRN match attempts
    min_matchability: float = 0.10  # below this the camera finds nothing to match
    confidence_floor: float = 0.25  # reject a candidate whose match confidence is below this
    gate_chi2: float = 9.21  # innovation gate (2-DOF chi-square, ~99%); reject outliers
    base_blunder_m: float = 60.0  # magnitude of a stale-map mismatch (blunder)
    q_var_per_m: float = 0.20  # estimator self-uncertainty growth, m^2 per m travelled
    init_var_m2: float = 4.0  # initial estimate variance (m^2)


class FixOutcome(Enum):
    """What happened at one match attempt."""

    NO_FEATURES = "no_features"  # too little matchable texture (matchability < floor)
    LOW_CONFIDENCE = "low_confidence"  # a candidate formed but its confidence was too low
    GATED_OUT = "gated_out"  # candidate rejected by the innovation gate (likely a blunder)
    ACCEPTED = "accepted"  # a good fix, fused
    ACCEPTED_CORRUPT = "accepted_corrupt"  # a *wrong* fix that slipped through the gate


@dataclass(frozen=True)
class Fix:
    """The record of one match attempt."""

    outcome: FixOutcome
    innovation_m: float  # |measurement - estimate|
    accuracy_m: float  # measurement noise std used
    confidence: float  # reported match confidence (feature strength only)
    correction: tuple[float, float]  # delta applied to the error vector (0,0 if none)

    @property
    def accepted(self) -> bool:
        return self.outcome in (FixOutcome.ACCEPTED, FixOutcome.ACCEPTED_CORRUPT)

    @property
    def corrupt(self) -> bool:
        return self.outcome is FixOutcome.ACCEPTED_CORRUPT


class OpticalCorrector:
    """Stateful VTRN corrector: robustly fuse optical fixes into the estimate.

    Holds the estimator's self-assessed position variance ``P`` (isotropic,
    m^2), which grows with distance (:meth:`propagate`) and shrinks on an
    accepted fix. The corrector only ever sees the estimate-minus-truth error
    vector; it does not know the absolute position, and — crucially — it does
    **not** know a match is stale. Its only defence against a stale/wrong match
    is the innovation gate, and that defence weakens precisely as drift (and
    thus ``P``) grows: the fix you most need, after a long dead-reckoning
    stretch, is the one you can least validate. That is the real failure mode.
    """

    def __init__(self, config: CorrectionConfig | None = None, seed: int = 0) -> None:
        self.config = config or CorrectionConfig()
        self._rng = random.Random(seed)
        self.variance = self.config.init_var_m2

    def propagate(self, distance_m: float) -> None:
        """Grow the self-assessed variance over ``distance_m`` of dead-reckoning."""
        self.variance += self.config.q_var_per_m * max(0.0, distance_m)

    def attempt(
        self, error: tuple[float, float], sample: MapSample
    ) -> tuple[Fix, tuple[float, float]]:
        """Attempt one match; return ``(fix_record, updated_error_vector)``.

        ``error`` is the current estimate-minus-truth vector; the returned vector
        is that error after fusing an accepted fix (unchanged if none is
        accepted). The confidence reported to the accept/reject logic reflects
        **feature strength only** (``matchability``): a stale map still looks
        confident, which is why staleness is caught by the gate, not the floor.
        """
        cfg = self.config
        ex, ey = error
        m = sample.matchability

        # (1) Availability: enough matchable texture, and a candidate this attempt.
        if m < cfg.min_matchability or self._rng.random() > m:
            return Fix(FixOutcome.NO_FEATURES, math.inf, math.inf, m, (0.0, 0.0)), error

        # (2) Measurement: nadir match of the true position, noisier as features
        #     thin out. A stale reference yields a plausible but wrong match (a
        #     blunder offset) with probability = staleness.
        r = sample.accuracy_m / max(m, cfg.min_matchability)
        nx = self._rng.gauss(0.0, r)
        ny = self._rng.gauss(0.0, r)
        corrupt = self._rng.random() < sample.staleness
        bx = by = 0.0
        if corrupt:
            ang = self._rng.uniform(0.0, 2.0 * math.pi)
            mag = cfg.base_blunder_m * (0.5 + self._rng.random())
            bx, by = mag * math.cos(ang), mag * math.sin(ang)

        # Innovation y = z - est = (true + n + b) - (true + e) = n + b - e.
        yx = nx + bx - ex
        yy = ny + by - ey
        innovation = math.hypot(yx, yy)
        var_meas = r * r

        # (3) Confidence floor — filters low-texture areas (but NOT staleness).
        confidence = m
        if confidence < cfg.confidence_floor:
            return Fix(FixOutcome.LOW_CONFIDENCE, innovation, r, confidence, (0.0, 0.0)), error

        # (4) Innovation gate — the only defence against a stale/wrong match.
        d2 = (yx * yx + yy * yy) / (self.variance + var_meas)
        if d2 > cfg.gate_chi2:
            return Fix(FixOutcome.GATED_OUT, innovation, r, confidence, (0.0, 0.0)), error

        # (5) Fuse: scalar Kalman update. Error shrinks toward the fix accuracy;
        #     variance drops. A blunder that slipped the gate corrupts both.
        k = self.variance / (self.variance + var_meas)
        cx, cy = k * yx, k * yy
        self.variance *= 1.0 - k
        outcome = FixOutcome.ACCEPTED_CORRUPT if corrupt else FixOutcome.ACCEPTED
        return Fix(outcome, innovation, r, confidence, (cx, cy)), (ex + cx, ey + cy)
