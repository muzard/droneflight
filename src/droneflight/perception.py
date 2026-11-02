"""Optical / depth perception of nearby obstacles (Technology-Selection §3.3).

Emulates the forward depth sensing the drone uses to *see* trees at close range —
a passive global-shutter stereo pair (covert, needs light) or, when permitted, an
active lidar (wider/longer but emitting). The sensor is modelled as a set of rays
over a horizontal field of view returning the distance to the nearest obstacle,
which is what a depth camera or 2D lidar slice provides to the avoidance layer.

Obstacles are vertical cylinders (tree trunks) in the local ENU plane at the
flight altitude — forest avoidance is primarily horizontal, since the planner's
flyability model already put the drone in a navigable vertical band. Pure Python
(no heavy deps) so it stays testable without hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Trunk:
    """A tree trunk: a vertical cylinder in the local ENU plane (meters)."""

    x: float
    y: float
    radius: float = 0.15
    top_z: float = math.inf
    """World z of the crown top. A sensor or airframe above it (by
    ``OVERFLY_MARGIN_M``) cannot see or touch it — which is what lets a plan fly
    OVER a stand (droneplanner ``over_canopy_when_unpassable``) without the 2D
    oracle dodging trunks far beneath it. ``inf`` = infinitely tall (the old model)."""


OVERFLY_MARGIN_M = 0.5
"""Height above a trunk's top at which the airframe is clear of it."""


@dataclass
class ObstacleField:
    """The trees around the drone (the sim's ground truth for perception).

    A uniform spatial-grid index makes local queries O(nearby trunks), so the
    closed-loop sim stays fast even with thousands of trunks over a km-scale route.
    """

    trunks: tuple[Trunk, ...] = field(default_factory=tuple)
    cell_size_m: float = 10.0
    _index: dict[tuple[int, int], list[Trunk]] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        for t in self.trunks:
            self._index.setdefault(self._cell(t.x, t.y), []).append(t)

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return math.floor(x / self.cell_size_m), math.floor(y / self.cell_size_m)

    def _nearby(self, x: float, y: float, radius: float) -> list[Trunk]:
        r_cells = math.ceil(radius / self.cell_size_m)
        cx, cy = self._cell(x, y)
        out: list[Trunk] = []
        for i in range(cx - r_cells, cx + r_cells + 1):
            for j in range(cy - r_cells, cy + r_cells + 1):
                out.extend(self._index.get((i, j), ()))
        return out

    def clearance(self, x: float, y: float, z: float | None = None) -> float:
        """Distance from (x, y) to the nearest trunk *surface* (negative if inside).

        With ``z`` (world), trunks whose top the airframe is above are ignored."""
        near = [t for t in self._nearby(x, y, self.cell_size_m) if _reaches(t, z)]
        if not near:
            return math.inf
        return min(math.hypot(x - t.x, y - t.y) - t.radius for t in near)

    def ray_range(
        self, ox: float, oy: float, bearing: float, max_range: float, z: float | None = None
    ) -> float:
        """Distance to the first trunk hit along ``bearing`` from (ox, oy).

        Returns ``max_range`` if nothing is hit within range. Accounts for trunk
        radius (ray-circle intersection).
        """
        dx, dy = math.cos(bearing), math.sin(bearing)
        best = max_range
        for t in self._nearby(ox, oy, max_range + 0.5):
            if not _reaches(t, z):
                continue
            fx, fy = ox - t.x, oy - t.y
            # |O + s*D - C|^2 = r^2  ->  s^2 + 2 b s + c = 0  (D is unit)
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - t.radius * t.radius
            disc = b * b - c
            if disc < 0.0:
                continue
            sqrt_disc = math.sqrt(disc)
            s = -b - sqrt_disc
            if s < 0.0:
                s = -b + sqrt_disc  # origin inside/behind the near surface
            if 0.0 <= s < best:
                best = s
        return best


@dataclass(frozen=True)
class DepthScan:
    """A horizontal depth scan: per-bearing ranges relative to the drone heading."""

    bearings_rel: tuple[float, ...]  # radians, relative to heading (0 = straight ahead)
    ranges: tuple[float, ...]  # meters; == max_range where clear
    max_range: float
    fov_rad: float

    def min_range(self) -> float:
        return min(self.ranges) if self.ranges else self.max_range


def sense(
    field: ObstacleField,
    x: float,
    y: float,
    heading: float,
    fov_rad: float,
    n_rays: int,
    max_range: float,
    z: float | None = None,
) -> DepthScan:
    """Cast ``n_rays`` across the FOV centred on ``heading`` and return the scan.

    ``z`` (world) is the sensor's height: a horizontal scan does not see trunks
    whose tops are below it."""
    n_rays = max(2, n_rays)
    bearings = [(-fov_rad / 2.0) + fov_rad * i / (n_rays - 1) for i in range(n_rays)]
    ranges = tuple(field.ray_range(x, y, heading + rel, max_range, z) for rel in bearings)
    return DepthScan(
        bearings_rel=tuple(bearings), ranges=ranges, max_range=max_range, fov_rad=fov_rad
    )


def _reaches(t: Trunk, z: float | None) -> bool:
    """Does trunk ``t`` rise to height ``z`` (within the overfly margin)?"""
    return z is None or t.top_z + OVERFLY_MARGIN_M > z
