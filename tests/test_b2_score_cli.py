"""`droneflight b2-score` — the ULog adapter around `strikes.score_track`.

The scoring logic itself is tested in `test_strikes.py` without pyulog; these
cover the CLI's own contract: exit codes, error handling, and the manifest it is
handed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from droneflight.cli import app

MANIFEST = "east_m,north_m,base_z_m,radius_m,height_m\n10.0,0.0,0.0,0.150,15.0\n"


def _manifest(tmp_path: Path, text: str = MANIFEST) -> Path:
    path = tmp_path / "trunks.csv"
    path.write_text(text, encoding="utf-8")
    return path


def test_missing_ulog_exits_2(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    manifest = _manifest(tmp_path)
    rc = app(["b2-score", "--ulog", str(tmp_path / "nope.ulg"), "--trunks", str(manifest)])
    assert rc == 2
    assert "cannot read" in capsys.readouterr().out


def test_empty_manifest_exits_2(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A world built without --forest-dsm would score every flight as clean."""
    empty = _manifest(tmp_path, "east_m,north_m,base_z_m,radius_m,height_m\n")
    rc = app(["b2-score", "--ulog", str(tmp_path / "x.ulg"), "--trunks", str(empty)])
    assert rc == 2
    out = capsys.readouterr().out
    assert "no trunks" in out and "Flying it would test nothing" in out


def test_manifest_missing_base_z_is_rejected(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """base_z_m is what makes the vertical span check possible; without it a
    flight 20 m above a stand would score as scraping it."""
    old = _manifest(tmp_path, "east_m,north_m,radius_m,height_m\n10.0,0.0,0.150,15.0\n")
    rc = app(["b2-score", "--ulog", str(tmp_path / "x.ulg"), "--trunks", str(old)])
    assert rc == 2
    assert "malformed" in capsys.readouterr().out


def test_clean_track_exits_0(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A flight that passes 5 m from the trunk at constant speed is clean."""
    import droneflight.cli as cli

    n = 200
    track = (
        [i * 0.02 for i in range(n)],
        [6.0 * i * 0.02 for i in range(n)],
        [5.0] * n,
        [10.0] * n,
        [6.0] * n,
        [0.0] * n,
    )
    monkeypatch.setattr(cli, "read_groundtruth_track", lambda _p: track)
    rc = app(["b2-score", "--ulog", "any.ulg", "--trunks", str(_manifest(tmp_path))])
    assert rc == 0


def test_a_strike_exits_1_and_says_so(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """Arriving is not succeeding — the whole point of the command."""
    import droneflight.cli as cli

    n = 200
    east = [min(6.0 * i * 0.02, 10.0) for i in range(n)]
    vel_e = [6.0 if 6.0 * i * 0.02 < 10.0 else -1.0 for i in range(n)]
    track = (
        [i * 0.02 for i in range(n)],
        east,
        [0.0] * n,
        [10.0] * n,
        vel_e,
        [0.0] * n,
    )
    monkeypatch.setattr(cli, "read_groundtruth_track", lambda _p: track)
    rc = app(["b2-score", "--ulog", "any.ulg", "--trunks", str(_manifest(tmp_path)), "--verbose"])
    assert rc == 1
    out = capsys.readouterr().out
    assert "TREE STRIKES   1" in out
    assert "NOT CLEAN" in out


def test_threshold_flag_is_honoured(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    import droneflight.cli as cli

    n = 200
    east = [min(6.0 * i * 0.02, 10.0) for i in range(n)]
    vel_e = [6.0 if 6.0 * i * 0.02 < 10.0 else -1.0 for i in range(n)]
    track = ([i * 0.02 for i in range(n)], east, [0.0] * n, [10.0] * n, vel_e, [0.0] * n)
    monkeypatch.setattr(cli, "read_groundtruth_track", lambda _p: track)
    rc = app(
        [
            "b2-score",
            "--ulog",
            "any.ulg",
            "--trunks",
            str(_manifest(tmp_path)),
            "--threshold-g",
            "10000",
        ]
    )
    assert rc == 0
    assert "VERDICT: CLEAN" in capsys.readouterr().out
