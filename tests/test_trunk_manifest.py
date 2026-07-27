"""Unit tests for the trunk manifest loader (TEST-PLAN B2)."""

from __future__ import annotations

from pathlib import Path

import pytest

from droneflight.trunk_manifest import TrunkManifestError, load_trunk_manifest

_CSV = """east_m,north_m,base_z_m,radius_m,height_m
100.000,200.000,-5.000,0.150,18.000
110.500,205.250,-5.500,0.200,16.000
"""


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "world_trunks.csv"
    path.write_text(text, encoding="utf-8")
    return path


def test_loads_trunks_in_the_world_frame(tmp_path: Path) -> None:
    manifest = load_trunk_manifest(_write(tmp_path, _CSV))
    assert len(manifest.trunks) == 2
    assert manifest.trunks[0].x == 100.0
    assert manifest.trunks[0].y == 200.0
    assert manifest.trunks[1].radius == 0.2
    assert manifest.heights_m == (18.0, 16.0)


def test_builds_the_field_perception_queries(tmp_path: Path) -> None:
    field = load_trunk_manifest(_write(tmp_path, _CSV)).obstacle_field()
    assert len(field.trunks) == 2
    assert field.cell_size_m == 10.0


def test_an_empty_manifest_is_an_error_not_an_empty_forest(tmp_path: Path) -> None:
    """Silently flying a treeless 'forest' test would look like a pass."""
    with pytest.raises(TrunkManifestError, match="no trunks"):
        load_trunk_manifest(_write(tmp_path, "east_m,north_m,base_z_m,radius_m,height_m\n"))


def test_missing_file_is_reported_clearly(tmp_path: Path) -> None:
    with pytest.raises(TrunkManifestError, match="cannot read"):
        load_trunk_manifest(tmp_path / "absent.csv")


def test_malformed_rows_name_the_line(tmp_path: Path) -> None:
    bad = "east_m,north_m,base_z_m,radius_m,height_m\n1.0,2.0,3.0,notanumber,5.0\n"
    with pytest.raises(TrunkManifestError, match=":2 is malformed"):
        load_trunk_manifest(_write(tmp_path, bad))
