"""Match PX4 SITL's simulated world to a planned mission's home.

PX4 SITL spawns the vehicle at a *default* home (Zurich, 47.397/8.546) unless
``PX4_HOME_LAT`` / ``PX4_HOME_LON`` / ``PX4_HOME_ALT`` are exported before
``make px4_sitl``. A droneplanner ``.plan`` places its waypoints at the real AO,
which is nowhere near Zurich — so without this the uploaded mission either flies
hundreds of km "home to target" or is rejected outright. This is the number-one
first-flight bring-up trap; extracting home from the plan removes it.

Usage (on the Ubuntu VM, see ``RUNBOOK.md``)::

    eval "$(droneflight sitl-env --plan out/mission.plan)"
    cd PX4-Autopilot && make px4_sitl gz_x500

The env is derived purely from the loaded mission, so it is fully testable
without a sim.
"""

from __future__ import annotations

from droneflight.model import FlightMission


def px4_home_env(mission: FlightMission) -> dict[str, str]:
    """PX4 SITL home-position environment variables for ``mission``."""
    return {
        "PX4_HOME_LAT": f"{mission.home_lat:.8f}",
        "PX4_HOME_LON": f"{mission.home_lon:.8f}",
        "PX4_HOME_ALT": f"{mission.home_alt_m:.2f}",
    }


def format_env_exports(env: dict[str, str]) -> str:
    """Render an env mapping as a shell ``export`` block (for ``eval``)."""
    return "\n".join(f"export {key}={value}" for key, value in env.items())


class ParamFileError(ValueError):
    """A ``.params`` file could not be parsed."""


_PARAM_TYPE_INT = 6
_PARAM_TYPE_FLOAT = 9


def parse_qgc_params(text: str) -> list[tuple[str, str]]:
    """Parse a QGroundControl ``.params`` file into ``(name, value)`` pairs.

    The format is ``<vehicle_id> <component_id> <name> <value> <type>``, tab or
    space separated, with ``#`` comments.
    """
    out: list[tuple[str, str]] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ParamFileError(f"line {lineno}: expected 5 fields, got {len(fields)}: {raw!r}")
        _vid, _cid, name, value, type_str = fields
        try:
            ptype = int(type_str)
            out.append(
                (name, str(int(float(value))) if ptype == _PARAM_TYPE_INT else str(float(value)))
            )
        except ValueError as exc:
            raise ParamFileError(f"line {lineno}: bad value/type in {raw!r}") from exc
    if not out:
        raise ParamFileError("no parameters found")
    return out


def format_pxh_param_sets(params: list[tuple[str, str]]) -> str:
    """Render parameters as ``param set`` lines for PX4's ``pxh>`` console.

    PX4's own ``param load`` / ``param import`` read **BSON**, not this text
    format — pointing them at a ``.params`` file fails with a ``tinybson`` node
    name overflow as it tries to parse the comment header. QGroundControl reads
    the text format, but a headless VM has no QGC, so the console path is
    ``param set``. Generating those lines from the same file keeps one source of
    truth instead of a hand-copied list that drifts.
    """
    lines = [f"param set {name} {value}" for name, value in params]
    lines.append("param save")
    return "\n".join(lines)
