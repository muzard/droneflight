"""Localize against a stem map — a pose fix from the trees themselves.

WHY THIS EXISTS
The stem-routed plan flew 330/330 through 2500 trunks with **0 strikes**, but it
flew **GPS-aided** (`EKF2_GPS_CTRL=7`), and this programme is GPS-denied. The
margins say that matters: the route was planned with 1.387 m of clearance and flew
with **0.708 m**, because PX4's own tracking consumed 0.679 m with a near-perfect
position estimate. Any navigation error adds on top of that. GPS-denied VTRN holds
**11.8 m** on a good seed — about 17x too coarse for the route it would be flying.

So below-canopy flight needs a position source good to well under a metre, in
forest, without GPS. The stem map is already there for routing, and a sensor that
sees stems is already required to build it: **match the trunks you can see against
the trunks you expected, and the disagreement is your pose error.** One instrument,
both jobs — and it works exactly where optical VTRN starves, since the conflict arm
of the Slice-3 acceptance failed by fix starvation (76.5 % of attempts found no
texture) in the forest that is supposed to be hiding the aircraft.

🔴 **THE MAP IS A PRIOR, NOT TRUTH, AND THAT DICTATES THE ALGORITHM.** Per the user
(2026-09-15): a government forest survey may be used to *validate* stem mapping,
but *"the drone cannot rely on that during a conflict, as the battlespace is
modified and changes"* — felled, shelled, bulldozed, tracked through. A large and
unknown fraction of the map will be wrong by the time it is flown, in both
directions: mapped stems that are gone, and standing stems that were never mapped.

That rules out least-squares or nearest-neighbour fitting, which are dragged off by
outliers. What survives is **consensus**: hypothesise a transform from a minimal
sample, count how many stems agree, and keep the hypothesis with the largest
self-consistent support. Everything that disagrees is simply not evidence. The
inter-stem distance used to seed the hypotheses is invariant to the very thing
being solved for, which is what makes two correspondences enough.

🔑 **A wrong fix is far worse than no fix.** Dead reckoning degrades predictably and
the operator can budget for it; a confident pose that is 3 m out steers the aircraft
into a trunk it believes it has cleared. This is the same asymmetry as a *merged*
stem detection, which puts a phantom trunk in the middle of the gap the aircraft is
aiming for. So :class:`PoseFix` carries ``accepted`` and a reason, and the caller
must be able to decline the fix.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

Point = tuple[float, float]


@dataclass(frozen=True)
class PoseFix:
    """A correction to the vehicle's believed pose, and whether to trust it."""

    dx: float
    dy: float
    dyaw_deg: float
    """Rotation about the vehicle's believed position, degrees counter-clockwise."""

    inliers: int
    observed: int
    rmse_m: float
    accepted: bool
    reason: str
    """Why it was accepted or refused — so a declined fix is diagnosable rather
    than just absent."""

    @property
    def inlier_fraction(self) -> float:
        return self.inliers / self.observed if self.observed else 0.0

    @property
    def shift_m(self) -> float:
        return math.hypot(self.dx, self.dy)

    def apply(self, x: float, y: float) -> Point:
        """Map a point in the believed frame into the corrected frame."""
        a = math.radians(self.dyaw_deg)
        ca, sa = math.cos(a), math.sin(a)
        return (ca * x - sa * y + self.dx, sa * x + ca * y + self.dy)


NO_FIX = PoseFix(0.0, 0.0, 0.0, 0, 0, float("nan"), False, "no fix attempted")


def _transform_from_pair(p1: Point, p2: Point, q1: Point, q2: Point) -> tuple[float, float, float]:
    """Rigid transform taking observed pair (p1, p2) onto map pair (q1, q2).

    Anchored on the pair's midpoint rather than on one of its points: an error in
    either endpoint then contributes half as much to the translation.
    """
    ang = math.atan2(q2[1] - q1[1], q2[0] - q1[0]) - math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    ca, sa = math.cos(ang), math.sin(ang)
    pmx, pmy = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
    qmx, qmy = (q1[0] + q2[0]) / 2.0, (q1[1] + q2[1]) / 2.0
    return qmx - (ca * pmx - sa * pmy), qmy - (sa * pmx + ca * pmy), ang


def _score(
    observed: Sequence[Point],
    map_stems: Sequence[Point],
    dx: float,
    dy: float,
    ang: float,
    tol: float,
) -> tuple[int, float, list[tuple[int, int]]]:
    """Inlier count, sum of squared residuals, and the correspondences."""
    ca, sa = math.cos(ang), math.sin(ang)
    tol2 = tol * tol
    inliers: list[tuple[int, int]] = []
    sse = 0.0
    used: set[int] = set()
    for i, (px, py) in enumerate(observed):
        tx = ca * px - sa * py + dx
        ty = sa * px + ca * py + dy
        best_j, best_d2 = -1, tol2
        for j, (qx, qy) in enumerate(map_stems):
            if j in used:
                continue
            d2 = (tx - qx) ** 2 + (ty - qy) ** 2
            if d2 < best_d2:
                best_j, best_d2 = j, d2
        if best_j >= 0:
            used.add(best_j)
            inliers.append((i, best_j))
            sse += best_d2
    return len(inliers), sse, inliers


def _refine(
    observed: Sequence[Point], map_stems: Sequence[Point], pairs: list[tuple[int, int]]
) -> tuple[float, float, float]:
    """Least-squares rigid fit (Kabsch, 2D) over correspondences already agreed.

    Least squares is safe *here* and nowhere else in this module: by this point the
    outliers have been voted out, so what remains is noise.
    """
    n = len(pairs)
    pcx = sum(observed[i][0] for i, _ in pairs) / n
    pcy = sum(observed[i][1] for i, _ in pairs) / n
    qcx = sum(map_stems[j][0] for _, j in pairs) / n
    qcy = sum(map_stems[j][1] for _, j in pairs) / n
    sxy = syx = sxx = syy = 0.0
    for i, j in pairs:
        ax, ay = observed[i][0] - pcx, observed[i][1] - pcy
        bx, by = map_stems[j][0] - qcx, map_stems[j][1] - qcy
        sxx += ax * bx
        syy += ay * by
        sxy += ax * by
        syx += ay * bx
    ang = math.atan2(sxy - syx, sxx + syy)
    ca, sa = math.cos(ang), math.sin(ang)
    return qcx - (ca * pcx - sa * pcy), qcy - (sa * pcx + ca * pcy), ang


def localize(
    observed: Sequence[Point],
    map_stems: Sequence[Point],
    *,
    search_radius_m: float = 20.0,
    search_yaw_deg: float = 20.0,
    match_tol_m: float = 1.0,
    min_inliers: int = 4,
    min_inlier_fraction: float = 0.25,
    pair_tol_m: float = 0.5,
    min_pair_separation_m: float = 4.0,
    ambiguity_ratio: float = 1.5,
    distinct_m: float = 3.0,
) -> PoseFix:
    """Fix the vehicle's pose by matching seen stems to mapped ones.

    ``observed`` are stems as currently sensed, already projected into the frame
    the vehicle *believes* it is in; ``map_stems`` is the prior, in the same frame.
    The returned transform is therefore the correction to that belief.

    ``min_pair_separation_m`` is a quality floor on the seed, not just an
    efficiency knob: a short baseline fixes the heading poorly, so short pairs
    generate a spray of slightly-wrong transforms that are expensive to score and
    useless as evidence.

    ``search_radius_m`` / ``search_yaw_deg`` bound the hypothesis to the drift the
    navigation solution could plausibly have accumulated. Bounding it is not just an
    optimisation: an unbounded search over a repetitive pattern like a plantation
    will eventually find a confident alignment one row over, and being exactly one
    row wrong is the failure mode that flies into a tree.

    Returns :data:`NO_FIX`-style refusal rather than a weak answer whenever support
    is thin — see the module docstring on why a wrong fix is worse than none.
    """
    n_obs = len(observed)
    if n_obs < 2 or len(map_stems) < 2:
        return PoseFix(0.0, 0.0, 0.0, 0, n_obs, float("nan"), False, "too few stems to match")

    # Map pairs indexed by separation, so a candidate correspondence is a lookup
    # rather than a scan. Separation is invariant to the pose being solved for.
    max_sep = _max_separation(observed) + 2.0 * pair_tol_m
    map_pairs: list[tuple[float, int, int]] = []
    for a in range(len(map_stems)):
        for b in range(a + 1, len(map_stems)):
            d = math.dist(map_stems[a], map_stems[b])
            if min_pair_separation_m <= d <= max_sep:
                map_pairs.append((d, a, b))
    if not map_pairs:
        return PoseFix(0.0, 0.0, 0.0, 0, n_obs, float("nan"), False, "no comparable map pairs")
    map_pairs.sort(key=lambda t: t[0])
    seps = [t[0] for t in map_pairs]

    import bisect

    # Top hypotheses, deduplicated by transform. Keeping the best RIVAL — not just
    # the best — is what catches the dangerous case: in a repetitive stand a second
    # alignment one row over can explain almost as many stems as the true one, and
    # picking between them on inlier count alone is a coin flip that ends in a tree.
    top: list[tuple[int, float, list[tuple[int, int]], float, float, float]] = []
    yaw_limit = math.radians(search_yaw_deg)
    for i in range(n_obs):
        for j in range(i + 1, n_obs):
            d = math.dist(observed[i], observed[j])
            if d < min_pair_separation_m:
                continue
            lo = bisect.bisect_left(seps, d - pair_tol_m)
            hi = bisect.bisect_right(seps, d + pair_tol_m)
            for k in range(lo, hi):
                _, a, b = map_pairs[k]
                for q1, q2 in ((map_stems[a], map_stems[b]), (map_stems[b], map_stems[a])):
                    dx, dy, ang = _transform_from_pair(observed[i], observed[j], q1, q2)
                    if math.hypot(dx, dy) > search_radius_m:
                        continue
                    if abs(_wrap(ang)) > yaw_limit:
                        continue
                    count, sse, pairs = _score(observed, map_stems, dx, dy, ang, match_tol_m)
                    _insert(top, (count, sse, pairs, dx, dy, ang), distinct_m)

    if not top:
        return PoseFix(0.0, 0.0, 0.0, 0, n_obs, float("nan"), False, "no admissible hypothesis")

    count, sse, pairs, dx, dy, ang = top[0]
    rival = top[1][0] if len(top) > 1 else 0
    if count >= 2:
        dx, dy, ang = _refine(observed, map_stems, pairs)
        count, sse, pairs = _score(observed, map_stems, dx, dy, ang, match_tol_m)
    rmse = math.sqrt(sse / count) if count else float("nan")

    if count < min_inliers:
        reason = f"only {count} stems agree (need {min_inliers})"
        return PoseFix(dx, dy, math.degrees(_wrap(ang)), count, n_obs, rmse, False, reason)
    if count / n_obs < min_inlier_fraction:
        reason = f"only {count}/{n_obs} seen stems agree (need {min_inlier_fraction:.0%})"
        return PoseFix(dx, dy, math.degrees(_wrap(ang)), count, n_obs, rmse, False, reason)
    if rival * ambiguity_ratio > count:
        # A materially different transform explains nearly as much. Measured on a
        # map with 1.5 m of position error this check is the difference between
        # declining and confidently reporting a pose **20.8 m** wrong.
        reason = f"ambiguous: a different alignment explains {rival}/{n_obs} against {count}"
        return PoseFix(dx, dy, math.degrees(_wrap(ang)), count, n_obs, rmse, False, reason)
    return PoseFix(
        dx,
        dy,
        math.degrees(_wrap(ang)),
        count,
        n_obs,
        rmse,
        True,
        f"{count}/{n_obs} stems agree at {rmse:.2f} m",
    )


def _insert(
    top: list[tuple[int, float, list[tuple[int, int]], float, float, float]],
    cand: tuple[int, float, list[tuple[int, int]], float, float, float],
    distinct_m: float,
    keep: int = 6,
) -> None:
    """Keep the best few *genuinely distinct* hypotheses, best first.

    🔑 ``distinct_m`` must be several metres, NOT the match tolerance. Two seed
    pairs describing the *same* alignment do not agree exactly: a 4 m pair carrying
    0.05 m of sensor noise fixes the heading to only ~0.7 deg, which over a 25 m
    lever arm scatters the translation by half a metre or more. Deduplicating at
    the 1 m match tolerance therefore kept those near-duplicates as separate
    entries, the ambiguity test compared the best solution against a noisier copy
    of *itself*, and good fixes were refused — availability on a pristine map fell
    to 7/12. The rival that actually matters is an alignment a whole stem away,
    which is metres, so the threshold belongs at the stem-spacing scale.
    """
    for idx, existing in enumerate(top):
        same_xy = math.hypot(cand[3] - existing[3], cand[4] - existing[4]) <= distinct_m
        same_yaw = abs(_wrap(cand[5] - existing[5])) <= math.radians(5.0)
        if same_xy and same_yaw:
            if cand[0] > existing[0] or (cand[0] == existing[0] and cand[1] < existing[1]):
                top[idx] = cand
                top.sort(key=lambda t: (-t[0], t[1]))
            return
    top.append(cand)
    top.sort(key=lambda t: (-t[0], t[1]))
    del top[keep:]


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _max_separation(points: Sequence[Point]) -> float:
    best = 0.0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            best = max(best, math.dist(points[i], points[j]))
    return best
