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
from dataclasses import dataclass
from pathlib import Path

from droneflight.perception import ObstacleField, Trunk


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

    def obstacle_field(self, cell_size_m: float = 10.0) -> ObstacleField:
        """The indexed field ``perception.sense()`` queries."""
        return ObstacleField(trunks=self.trunks, cell_size_m=cell_size_m)


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
        except (KeyError, TypeError, ValueError) as exc:
            raise TrunkManifestError(f"{file}:{line} is malformed: {exc}") from exc

    return ForestManifest(trunks=tuple(trunks), heights_m=tuple(heights))
