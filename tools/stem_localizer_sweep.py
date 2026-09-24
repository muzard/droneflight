"""Acceptance sweep: how well does the pose fix survive a STALE map?

Per the user (2026-09-15) the drone cannot rely on a survey during a conflict,
because the battlespace is modified and changes. So degradation against map
staleness is a first-class acceptance test, not a robustness footnote:

    python tools/stem_localizer_sweep.py

⚠️ Synthetic stands with uniformly-scattered stems. A real plantation is regular,
which is the HARD case — an alignment one row over fits almost as well, and the
ambiguity check exists for exactly that (pinned by
`test_a_regular_plantation_is_reported_as_AMBIGUOUS`).
"""

import math

import numpy as np

from droneflight.stem_localizer import localize


def scene(
    n_map=120,
    area=60.0,
    seen_radius=25.0,
    felled=0.0,
    unmapped=0.0,
    drift=(6.0, -4.0),
    dyaw_deg=8.0,
    sensor_noise=0.05,
    map_noise=0.05,
    seed=0,
):
    rng = np.random.default_rng(seed)
    truth = rng.uniform(-area, area, size=(n_map, 2))
    keep = rng.random(n_map) >= felled
    map_stems = truth[keep] + rng.normal(0, map_noise, size=(keep.sum(), 2))
    in_range = np.hypot(truth[:, 0], truth[:, 1]) <= seen_radius
    seen_truth = truth[in_range]
    n_new = round(unmapped * len(seen_truth))
    if n_new:
        extra = rng.uniform(-seen_radius, seen_radius, size=(n_new * 3, 2))
        extra = extra[np.hypot(extra[:, 0], extra[:, 1]) <= seen_radius][:n_new]
        seen_truth = np.vstack([seen_truth, extra])
    seen = seen_truth + rng.normal(0, sensor_noise, size=seen_truth.shape)
    a = math.radians(-dyaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    bx = ca * (seen[:, 0] - drift[0]) - sa * (seen[:, 1] - drift[1])
    by = sa * (seen[:, 0] - drift[0]) + ca * (seen[:, 1] - drift[1])
    return [tuple(p) for p in np.c_[bx, by]], [tuple(p) for p in map_stems], drift, dyaw_deg


def run(label, **kw):
    errs, yaws, acc, inl, worst_silent = [], [], 0, [], 0.0
    for s in range(12):
        obs, mp, drift, dyaw = scene(seed=s, **kw)
        fix = localize(obs, mp)
        if fix.accepted:
            acc += 1
            e = math.hypot(fix.dx - drift[0], fix.dy - drift[1])
            errs.append(e)
            yaws.append(abs(fix.dyaw_deg - dyaw))
            inl.append(fix.inlier_fraction)
            worst_silent = max(worst_silent, e)
    e = np.array(errs) if errs else np.array([np.nan])
    y = np.array(yaws) if yaws else np.array([np.nan])
    print(
        f"{label:<34}{acc:>3}/12{np.mean(e):>9.3f}{np.max(e):>9.3f}{np.mean(y):>8.2f}"
        f"{(np.mean(inl) * 100 if inl else float('nan')):>8.0f}%"
    )


if __name__ == "__main__":
    print("Pose fix from stems. Truth drift = (6.0, -4.0) m, 8.0 deg yaw.")
    print(f"{'condition':<34}{'fixed':>6}{'mean err':>9}{'max err':>9}{'yaw err':>8}{'inliers':>9}")
    print("-" * 76)
    run("pristine map", felled=0.0, unmapped=0.0)
    for f in (0.2, 0.4, 0.6, 0.8):
        run(f"{int(f * 100)}% of mapped stems FELLED", felled=f, unmapped=0.0)
    for u in (0.3, 1.0, 2.0):
        run(f"{int(u * 100)}% unmapped stems appear", felled=0.0, unmapped=u)
    run("50% felled + 100% unmapped", felled=0.5, unmapped=1.0)
    run("70% felled + 100% unmapped", felled=0.7, unmapped=1.0)
    run("survey 0.5 m position error", map_noise=0.5)
    run("survey 1.5 m position error", map_noise=1.5)
