"""Load the trunk manifest the Gazebo world was built from (TEST-PLAN B2).

The manifest is written by ``droneplanner gz-world --forest-dsm`` next to the
world SDF. Both consumers read this one file:

* **Gazebo** got static collision cylinders from it, so a contact is
  ground-truth "you hit a tree";
* **``perception.sense()``** gets the same trunks from it, so the avoidance layer
  is dodging exactly what the physics engine will collide with.

That single-source property is the whole design. Regenerating the trees on this
side — even from the same seed — would be a second implementation and a second
chance to disagree, and a disagreement would show up as avoidance that "works"
against trees that are not there.

**This is oracle perception, deliberately.** It hands the avoidance layer perfect
range to every trunk in view: no noise, no dropout, no occlusion, no missed
detections. That is not a claim about cameras — it isolates *"can the aircraft
avoid a tree it knows about, with real dynamics and a real tether"* from *"can a
sensor see one"*, which needs hardware this project does not have (Track B).
TEST-PLAN B4 attacks the second question by degrading this oracle until
avoidance breaks, which turns it into a sensor requirement.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

from droneflight.perception import ObstacleField, Trunk

_SEARCH_RADIUS_M = 30.0
"""How far to look for a vertically-overlapping trunk. Wider than any clearance
worth reporting, so the answer is never an artifact of the search."""

_GROUND_RADIUS_M = 40.0
"""How far a trunk base may be and still stand in for the terrain under a sample."""


class TrunkManifestError(RuntimeError):
    """The manifest is missing, malformed, or empty."""


@dataclass(frozen=True)
class ForestManifest:
    """Trunks in the Gazebo world frame (ENU metres, origin = plan home)."""

    trunks: tuple[Trunk, ...]
    heights_m: tuple[float, ...]
    """Trunk heights, parallel to ``trunks`` — the 2D avoidance layer ignores
    these, but a vertical check needs them and dropping them here would lose
    information the manifest carries."""

    base_z_m: tuple[float, ...]
    """World z of each trunk's base, parallel to ``trunks``. These are terrain
    elevations by construction (the generator plants trunks on the DTM), which
    makes them a frame-exact terrain reference — no DEM and no georeferencing
    needed to ask "was the vehicle at ground level here". Validated against the
    DEM to 0.107 m mean / 0.545 m RMS over 2500 trunks (TEST-PLAN B2)."""

    _grid_cache: tuple[float, dict[tuple[int, int], list[int]]] | None = field(
        default=None, repr=False, compare=False
    )

    def obstacle_field(self, cell_size_m: float = 10.0) -> ObstacleField:
        """The indexed field ``perception.sense()`` queries — each trunk with its
        crown-top height, so a sensor flying over the canopy sees past it."""
        trunks = tuple(replace(t, top_z=self.top_z_m(i)) for i, t in enumerate(self.trunks))
        return ObstacleField(trunks=trunks, cell_size_m=cell_size_m)

    def top_z_m(self, index: int) -> float:
        """World z of a trunk's crown."""
        return self.base_z_m[index] + self.heights_m[index]

    def _cell(self, x: float, y: float, cell_m: float) -> tuple[int, int]:
        return math.floor(x / cell_m), math.floor(y / cell_m)

    def _index(self, cell_m: float) -> dict[tuple[int, int], list[int]]:
        if self._grid_cache is None or self._grid_cache[0] != cell_m:
            grid: dict[tuple[int, int], list[int]] = {}
            for i, t in enumerate(self.trunks):
                grid.setdefault(self._cell(t.x, t.y, cell_m), []).append(i)
            object.__setattr__(self, "_grid_cache", (cell_m, grid))
        assert self._grid_cache is not None
        return self._grid_cache[1]

    def nearby(self, east_m: float, north_m: float, radius_m: float) -> list[int]:
        """Indices of trunks whose *centre* is within ``radius_m``, via a grid index.

        Pure-Python and O(local), which is what makes whole-flight scoring (70 k
        samples x 2500 trunks) tractable without numpy or scipy.
        """
        cell_m = max(radius_m, 1.0)
        grid = self._index(cell_m)
        cx, cy = self._cell(east_m, north_m, cell_m)
        out: list[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for i in grid.get((cx + dx, cy + dy), ()):
                    t = self.trunks[i]
                    if math.hypot(t.x - east_m, t.y - north_m) <= radius_m:
                        out.append(i)
        return out

    def surface_clearance_m(self, east_m: float, north_m: float, up_m: float) -> float:
        """Distance to the nearest trunk *surface*, counting only trunks the
        sample is vertically level with. ``inf`` when above every crown or below
        every base; negative when inside a trunk.

        The vertical filter is the whole point: a 2D clearance says the vehicle
        was next to a trunk when it may have been 20 m above it.
        """
        best = math.inf
        for i in self.nearby(east_m, north_m, _SEARCH_RADIUS_M):
            if not self.base_z_m[i] <= up_m <= self.top_z_m(i):
                continue
            t = self.trunks[i]
            best = min(best, math.hypot(t.x - east_m, t.y - north_m) - t.radius)
        return best

    def ground_z_m(self, east_m: float, north_m: float) -> float | None:
        """Terrain elevation from the nearest trunk base, or None if none is near.

        A proxy, deliberately: trunks exist only where the planner found canopy,
        so this is defined along a forested route and nowhere else. Good enough to
        answer "did that impact happen at ground level"; not a DEM.
        """
        near = self.nearby(east_m, north_m, _GROUND_RADIUS_M)
        if not near:
            return None

        def distance(i: int) -> float:
            return math.hypot(self.trunks[i].x - east_m, self.trunks[i].y - north_m)

        return self.base_z_m[min(near, key=distance)]


def load_trunk_manifest(path: str | Path) -> ForestManifest:
    """Read ``<world>_trunks.csv`` into trunks the perception layer can sense."""
    file = Path(path)
    try:
        text = file.read_text(encoding="utf-8")
    except OSError as exc:
        raise TrunkManifestError(f"cannot read trunk manifest {file}: {exc}") from exc

    rows = list(csv.DictReader(text.splitlines()))
    if not rows:
        raise TrunkManifestError(
            f"{file} has no trunks — the world was built without --forest-dsm, or over "
            "terrain the planner found no canopy on. Flying it would test nothing."
        )

    trunks: list[Trunk] = []
    heights: list[float] = []
    bases: list[float] = []
    for line, row in enumerate(rows, start=2):
        try:
            trunks.append(
                Trunk(
                    x=float(row["east_m"]),
                    y=float(row["north_m"]),
                    radius=float(row["radius_m"]),
                )
            )
            heights.append(float(row["height_m"]))
            bases.append(float(row["base_z_m"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise TrunkManifestError(f"{file}:{line} is malformed: {exc}") from exc

    return ForestManifest(trunks=tuple(trunks), heights_m=tuple(heights), base_z_m=tuple(bases))
