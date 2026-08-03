"""Did the aircraft actually hit anything? Scored from a flown track.

WHY THIS EXISTS (TEST-PLAN B2, 2026-07-28)
Two corridor flights reported ``REACHED GOAL`` and had in fact struck 19 and 37
trees. Nothing in the executor could see it: its ``min_clearance_m`` is *forward
perception*, so a trunk clipped off-axis never entered the FOV, and the run's own
log printed ``closest obstacle 0.0 m`` beside the success line. The strikes were
only found by scoring the ULog afterwards, by hand. This module makes that
scoring a first-class, tested operation so the re-fly is judged the same way.

THE METHOD, AND WHY IT IS NOT A PROXIMITY TEST
A manifest says where a trunk is; only the airframe's own motion says whether
physics resolved a *contact*. Proximity cannot separate "avoided narrowly" from
"hit it" — the closest miss in the sparse arm was 0.075 m and was not a strike,
while a 0.09 m approach in the dense arm was. So a strike is an abrupt
**horizontal deceleration** coincident with being near a trunk the vehicle is
vertically level with. The measured signature: speed collapsing 4.85 -> 1.20 m/s
in one 20 ms sample, the velocity component reversing, then a fall.

Deceleration is measured on the **velocity vector** (``hypot(dvE, dvN)``), not on
the speed, so a bounce counts — a rebound barely changes speed while reversing
direction entirely.

WHY THE THRESHOLD IS DEFENSIBLE
It was calibrated against a negative control, not chosen. B1 flew the same route,
same vehicle, over a byte-identical DEM with **no trees**; scored against the
dense manifest it yields zero episodes and never exceeds **0.50 g** across 54 k
airborne samples — including 1164 within 1 m of where a trunk would have been. The
2.0 g default therefore has 4x margin over everything free flight does on this
trajectory. **Re-run that control if the airframe, route or controller changes**;
the number is a property of the flight envelope, not a universal constant.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from droneflight.trunk_manifest import ForestManifest

_G = 9.80665


@dataclass(frozen=True)
class StrikeConfig:
    """Detection thresholds. Defaults are the B2-calibrated ones."""

    accel_threshold_g: float = 2.0
    """4x the 0.50 g ceiling the tree-free negative control ever reached."""

    near_obstacle_m: float = 1.0
    """How close to a trunk surface the transient must be to be attributed to it.
    Generous on purpose: the ejection tail of a real contact carries the vehicle
    away from the trunk while it is still decelerating."""

    ground_band_m: float = 2.0
    """AGL below which a transient is attributed to terrain instead."""

    min_speed_ms: float = 0.5
    """Ignore transients that were not *entered* with speed — landing, and the
    post-crash tumble, are not strikes.

    Tested against the speed **before** the sample, never after: an impact that
    stops the vehicle dead leaves it at ~0 m/s, so gating on the post-impact speed
    silently discards the most severe contacts. B2's vehicle happened to rebound
    to 1.2 m/s and so was detected either way — the flaw only showed up under a
    synthetic dead-stop."""

    episode_gap_s: float = 0.5
    """Samples closer together than this belong to one impact. Without it a single
    contact spanning several samples counts as several strikes."""

    airborne_agl_m: float = 1.0
    rotor_radius_m: float = 0.45
    """Airframe half-width. Used only for the reported clearance margin."""


@dataclass(frozen=True)
class StrikeEpisode:
    """One impact."""

    t_s: float
    kind: str
    """``tree``, ``ground``, or ``unknown`` — the last usually being the ejection
    tail of a contact already counted, once the vehicle has been thrown clear."""

    peak_g: float
    speed_in_ms: float
    speed_out_ms: float
    clearance_m: float
    east_m: float
    north_m: float
    up_m: float


@dataclass(frozen=True)
class StrikeReport:
    samples: int
    airborne_samples: int
    episodes: tuple[StrikeEpisode, ...]
    min_clearance_m: float
    samples_in_trunk_span: int
    samples_inside_trunk: int
    min_agl_m: float | None

    @property
    def tree_strikes(self) -> int:
        return sum(1 for e in self.episodes if e.kind == "tree")

    @property
    def ground_impacts(self) -> int:
        return sum(1 for e in self.episodes if e.kind == "ground")

    @property
    def clean(self) -> bool:
        """No contact of any kind. The success criterion B2 was missing."""
        return not self.episodes


def score_track(
    t_s: Sequence[float],
    east_m: Sequence[float],
    north_m: Sequence[float],
    up_m: Sequence[float],
    vel_e_ms: Sequence[float],
    vel_n_ms: Sequence[float],
    manifest: ForestManifest,
    config: StrikeConfig | None = None,
) -> StrikeReport:
    """Score a flown track against the manifest the world was built from.

    Positions are ENU metres in the Gazebo world frame (origin = plan home), which
    is what ``vehicle_local_position_groundtruth`` gives once NED is converted.
    Pass ground truth, never the estimate: the question is where the airframe
    *was*, not where it believed it was.
    """
    config = config or StrikeConfig()
    n = len(t_s)
    if not (n == len(east_m) == len(north_m) == len(up_m) == len(vel_e_ms) == len(vel_n_ms)):
        raise ValueError("track arrays must be the same length")
    if n < 2:
        return StrikeReport(n, 0, (), math.inf, 0, 0, None)

    hits: list[tuple[int, str, float, float]] = []  # index, kind, g, clearance
    min_clearance = math.inf
    in_span = inside = 0
    min_agl: float | None = None
    airborne = 0

    for i in range(1, n):
        dt = t_s[i] - t_s[i - 1]
        if dt <= 0.0:
            continue
        speed = math.hypot(vel_e_ms[i], vel_n_ms[i])
        speed_before = math.hypot(vel_e_ms[i - 1], vel_n_ms[i - 1])
        ground = manifest.ground_z_m(east_m[i], north_m[i])
        agl = None if ground is None else up_m[i] - ground
        if agl is not None:
            min_agl = agl if min_agl is None else min(min_agl, agl)
        if max(speed, speed_before) <= 0.3 and (agl is None or agl <= config.airborne_agl_m):
            continue
        airborne += 1

        clearance = manifest.surface_clearance_m(east_m[i], north_m[i], up_m[i])
        if math.isfinite(clearance):
            in_span += 1
            min_clearance = min(min_clearance, clearance)
            if clearance < 0.0:
                inside += 1

        accel_g = math.hypot(vel_e_ms[i] - vel_e_ms[i - 1], vel_n_ms[i] - vel_n_ms[i - 1]) / dt / _G
        if accel_g <= config.accel_threshold_g or speed_before <= config.min_speed_ms:
            continue
        if math.isfinite(clearance) and clearance < config.near_obstacle_m:
            kind = "tree"
        elif agl is not None and agl < config.ground_band_m:
            kind = "ground"
        else:
            kind = "unknown"
        hits.append((i, kind, accel_g, clearance))

    return StrikeReport(
        samples=n,
        airborne_samples=airborne,
        episodes=_group(hits, t_s, east_m, north_m, up_m, vel_e_ms, vel_n_ms, config),
        min_clearance_m=min_clearance,
        samples_in_trunk_span=in_span,
        samples_inside_trunk=inside,
        min_agl_m=min_agl,
    )


def _group(
    hits: list[tuple[int, str, float, float]],
    t_s: Sequence[float],
    east_m: Sequence[float],
    north_m: Sequence[float],
    up_m: Sequence[float],
    vel_e_ms: Sequence[float],
    vel_n_ms: Sequence[float],
    config: StrikeConfig,
) -> tuple[StrikeEpisode, ...]:
    """Collapse consecutive hit samples into one episode per impact."""
    episodes: list[StrikeEpisode] = []
    group: list[tuple[int, str, float, float]] = []

    def flush() -> None:
        if not group:
            return
        # The episode's identity comes from its most severe sample, and its kind
        # from the first attributable one: by the peak the vehicle may already
        # have been thrown clear of the trunk that stopped it.
        peak = max(group, key=lambda h: h[2])
        kind = next((h[1] for h in group if h[1] != "unknown"), "unknown")
        first, last = group[0][0], group[-1][0]
        pre = max(0, first - 3)
        post = min(len(t_s) - 1, last + 3)
        episodes.append(
            StrikeEpisode(
                t_s=t_s[first],
                kind=kind,
                peak_g=peak[2],
                speed_in_ms=math.hypot(vel_e_ms[pre], vel_n_ms[pre]),
                speed_out_ms=math.hypot(vel_e_ms[post], vel_n_ms[post]),
                clearance_m=peak[3],
                east_m=east_m[first],
                north_m=north_m[first],
                up_m=up_m[first],
            )
        )
        group.clear()

    for hit in hits:
        if group and t_s[hit[0]] - t_s[group[-1][0]] > config.episode_gap_s:
            flush()
        group.append(hit)
    flush()
    return tuple(episodes)
