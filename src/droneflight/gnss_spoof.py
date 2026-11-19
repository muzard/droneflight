"""GNSS as an UNTRUSTED input: spoofing emulation and a VIO-consistency monitor.

WHY THIS EXISTS (roadmap P4)
The stack flies GPS-denied by design (``EKF2_GPS_CTRL=0``) because over the
front GNSS is jammed or spoofed. That throws away GNSS in the many places where
it is still honest. The navigation research (2026-11-18) recommends the middle
course: **keep GNSS as an untrusted input** and use it only while it agrees with
what the aircraft knows independently. The independent source here is VIO, whose
error grows at a bounded *rate* (~2 % of distance, :mod:`droneflight.nav`).

This module asks the question that decides whether that is safe: *how far can a
spoofer drag the estimate before a consistency check notices?*

THE ATTACK — carry-off
At ``onset_m`` along the route the spoofer captures the receiver (seamlessly:
the false position starts at the true one) and then drags the reported position
sideways at ``pull_rate`` metres of offset per metre flown. A step attack is
trivially caught by any innovation gate; carry-off is the one that matters.

THE MONITOR
GNSS minus VIO, ``o = p_gnss - p_vio``, can only change at the VIO drift rate
when GNSS is honest. So the monitor fits the change of ``o`` over the last
``window_m`` of flight and alarms when it exceeds ``k_rate * vio_rate * window``
plus a noise allowance. While GNSS is trusted the estimate follows it; after an
alarm the estimate falls back to VIO from the last trusted point.

🔑 **Measured (``tools/gnss_spoof_sweep.py``: 5 km, onset at 1 km, true VIO
drift 2 %, monitor bound 3 %, 20 seeds):**

| pull (m per m) | caught | median after | worst error | VIO alone |
|---|---|---|---|---|
| 0 (honest) | 0 false alarms | — | **~1.5 m at the end** | 98 m |
| 0.005-0.01 | never | — | 24-44 m | 98 m |
| 0.02 | 20 % | 1348 m | 84 m | 98 m |
| **0.03-0.04** | 45-60 % | 760-910 m | **124-163 m** | 98 m |
| 0.06-0.1 | 100 % | 430-690 m | 94-95 m | 98 m |
| ≥ 0.2 | 100 % | ≤ 80 m | ~83 m | 98 m |

Honest GNSS bounds the error at its own noise instead of VIO's 98 m, and a
spoofer slower than true VIO drift does less harm than flying VIO-only. The
cost is a **gap between the true drift and the monitor's bound**: a carry-off
in it is caught late or not at all and reaches **~1.7x the VIO-only error**.
The gap is the uncertainty in the VIO rate, so a tighter, measured bound on
the real VIO is what closes it.

⚠️ Offline, 2-D, a straight route at constant speed. Not modelled: a spoofer that
also feeds consistent velocity (Doppler) — this monitor uses positions only,
which is the weaker case for the defender; correlated VIO error (turns, low
texture); time-of-arrival or multi-antenna checks; jamming (that is just GNSS
loss, already covered by GPS-denied flight).
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class SpoofAttack:
    onset_m: float = 1000.0
    """Distance along the route at which the spoofer captures the receiver."""
    pull_rate: float = 0.05
    """Offset dragged per metre flown (0.05 = 5 cm of false offset per metre)."""
    bearing_deg: float = 90.0
    """Direction of the drag, relative to the direction of flight."""


@dataclass(frozen=True)
class MonitorConfig:
    windows_m: tuple[float, ...] = (100.0, 1000.0)
    """Flight distances over which the offset change is tested. The short one
    catches fast carry-off quickly; the long one shrinks the noise allowance per
    metre and catches slow carry-off."""
    vio_rate: float = 0.03
    """Bound on the VIO drift rate. Drift accrues ``rate * step`` per step in some
    direction, so ``|Δ vio| <= rate * span`` exactly; the bound must cover the
    worst VIO in service (Tech-Selection §2.1 gives 0.5-3 %), not the typical."""
    k_rate: float = 1.0
    """Alarm at this multiple of the drift bound (1.0: the bound is a bound)."""
    gnss_sigma_m: float = 1.5
    """Per-sample GNSS noise (horizontal, per axis)."""
    k_noise: float = 6.0
    """Noise allowance on the windowed change, in sigmas of a two-point difference.
    The test runs every step on every window, so it must be far out in the tail:
    at 4 sigma half the honest 5 km runs false-alarmed."""


@dataclass(frozen=True)
class SpoofRun:
    detected: bool
    detect_after_m: float
    """Distance flown from onset to alarm (inf if never)."""
    max_error_m: float
    """Worst estimate error over the run."""
    error_at_end_m: float
    vio_only_error_at_end_m: float
    false_alarm: bool
    """An alarm before the attack started."""


def simulate_spoof(
    route_m: float,
    attack: SpoofAttack | None,
    monitor: MonitorConfig | None = None,
    *,
    step_m: float = 5.0,
    vio_rate: float = 0.02,
    vio_wander_deg_per_100m: float = 20.0,
    seed: int = 0,
) -> SpoofRun:
    """Fly ``route_m``; return how the gated GNSS estimate fared.

    ``vio_rate`` is the TRUE drift of this run; ``monitor.vio_rate`` is the bound
    the monitor assumes. Keeping them separate is the point: the monitor is only
    as good as its bound on VIO.
    """
    cfg = monitor or MonitorConfig()
    rng = random.Random(seed)
    heading = rng.uniform(0.0, 2.0 * math.pi)
    vio = [0.0, 0.0]  # VIO error vector
    n = max(1, int(route_m / step_m))
    hist: deque[tuple[float, float, float]] = deque()
    trusted = True
    est_err_at_alarm = (0.0, 0.0)
    vio_at_alarm = (0.0, 0.0)
    detected_at = math.inf
    false_alarm = False
    max_err = 0.0
    err = (0.0, 0.0)
    longest = max(cfg.windows_m)
    thresh_noise = cfg.k_noise * cfg.gnss_sigma_m * math.sqrt(2.0)
    for i in range(1, n + 1):
        d = i * step_m
        heading += rng.gauss(0.0, math.radians(vio_wander_deg_per_100m) * step_m / 100.0)
        vio[0] += vio_rate * step_m * math.cos(heading)
        vio[1] += vio_rate * step_m * math.sin(heading)
        spoof = (0.0, 0.0)
        if attack is not None and d > attack.onset_m:
            mag = attack.pull_rate * (d - attack.onset_m)
            b = math.radians(attack.bearing_deg)
            spoof = (mag * math.cos(b), mag * math.sin(b))
        gn = (
            spoof[0] + rng.gauss(0.0, cfg.gnss_sigma_m),
            spoof[1] + rng.gauss(0.0, cfg.gnss_sigma_m),
        )
        if trusted:
            o = (gn[0] - vio[0], gn[1] - vio[1])
            hist.append((d, o[0], o[1]))
            while hist and d - hist[0][0] > longest:
                hist.popleft()
            for w in cfg.windows_m:
                d0, ox, oy = next(h for h in hist if d - h[0] <= w)
                span = d - d0
                if span < 0.5 * w:
                    continue
                change = math.hypot(o[0] - ox, o[1] - oy)
                if change > cfg.k_rate * cfg.vio_rate * span + thresh_noise:
                    trusted = False
                    if attack is None or d <= attack.onset_m:
                        false_alarm = True
                    else:
                        detected_at = d - attack.onset_m
                    # Fall back to VIO from the start of the window that fired:
                    # the GNSS error there is the offset plus the VIO error then.
                    est_err_at_alarm = (ox + vio[0], oy + vio[1])
                    vio_at_alarm = (vio[0], vio[1])
                    break
            err = (
                gn
                if trusted
                else (
                    est_err_at_alarm[0] + vio[0] - vio_at_alarm[0],
                    est_err_at_alarm[1] + vio[1] - vio_at_alarm[1],
                )
            )
        else:
            err = (
                est_err_at_alarm[0] + vio[0] - vio_at_alarm[0],
                est_err_at_alarm[1] + vio[1] - vio_at_alarm[1],
            )
        max_err = max(max_err, math.hypot(*err))
    return SpoofRun(
        detected=math.isfinite(detected_at),
        detect_after_m=detected_at,
        max_error_m=max_err,
        error_at_end_m=math.hypot(*err),
        vio_only_error_at_end_m=math.hypot(*vio),
        false_alarm=false_alarm,
    )


@dataclass(frozen=True)
class PullRateResult:
    pull_rate: float
    detected_fraction: float
    median_detect_after_m: float
    worst_max_error_m: float
    median_error_at_end_m: float
    median_vio_only_end_m: float
    false_alarms: int


def sweep_pull_rates(
    pull_rates: tuple[float, ...],
    *,
    route_m: float = 5000.0,
    onset_m: float = 1000.0,
    seeds: int = 20,
    monitor: MonitorConfig | None = None,
) -> list[PullRateResult]:
    """Monte-Carlo each carry-off rate over ``seeds`` VIO/GNSS noise draws."""
    out = []
    for rate in pull_rates:
        runs = [
            simulate_spoof(route_m, SpoofAttack(onset_m=onset_m, pull_rate=rate), monitor, seed=s)
            for s in range(seeds)
        ]
        hits = [r.detect_after_m for r in runs if r.detected]
        out.append(
            PullRateResult(
                pull_rate=rate,
                detected_fraction=len(hits) / len(runs),
                median_detect_after_m=_median(hits) if hits else math.inf,
                worst_max_error_m=max(r.max_error_m for r in runs),
                median_error_at_end_m=_median([r.error_at_end_m for r in runs]),
                median_vio_only_end_m=_median([r.vio_only_error_at_end_m for r in runs]),
                false_alarms=sum(r.false_alarm for r in runs),
            )
        )
    return out


def _median(xs: list[float]) -> float:
    s = sorted(xs)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])
