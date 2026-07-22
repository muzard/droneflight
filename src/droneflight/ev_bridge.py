"""External-vision injection bridge — where GPS-denied stops being an offline model.

`nav.py` and `correction.py` measure what a GPS-denied estimator's *error* does to
a route. This module makes PX4 actually fly on that error: take the simulator's
**ground truth**, corrupt it with the same drift + fidelity-limited VTRN model,
and stream the result into EKF2 as `VISION_POSITION_ESTIMATE`. The autopilot then
believes the corrupted pose and steers to make *it* follow the mission — so the
**true** track wanders exactly as much as the navigation error says it should.
That is the whole experiment: fixes on, the true track holds the corridor; fixes
off, it leaves.

⚠️ **Ground truth must come from the simulator, never from PX4.**
PX4's `local_position` / `position_velocity_ned` is the EKF2 *output* — the very
thing being corrupted. Feeding it back through the injector closes a loop on
itself: the error would be measured against an estimate that already contains it,
drift would silently cancel, and the run would look beautiful and mean nothing.
So the only live truth source here is Gazebo's model odometry
(:class:`GazeboTruthSource`). PX4's estimate is accepted *only* through
``estimate_probe``, is written to the track log as a diagnostic, and never
reaches :class:`~droneflight.nav.ErrorInjector`.

**Frames.** Gazebo's world is ENU (x=east, y=north, z=up) with an FLU body;
MAVLink vision is NED (x=north, y=east, z=down) with an FRD body. The conversion
is small and easy to get backwards, so it lives in one tested place
(:func:`enu_to_ned`, :func:`enu_flu_to_ned_frd_euler`) — a swapped axis here would
show up as a plausible-looking flight that curves the wrong way.

**What is injected, and what is not.** The error model is horizontal (2D), so
altitude and attitude are passed through as truth: EV height and yaw are perfect.
That keeps the experiment about the thing being studied — horizontal drift and
whether optical fixes bound it — but it means this bridge is *optimistic* about
height and heading, which a real VIO stack is not.
"""

from __future__ import annotations

import asyncio
import json
import math
import shutil
import subprocess
import time
from collections.abc import AsyncIterator, Callable, Iterable, Sequence
from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path
from typing import Any, Literal, Protocol, runtime_checkable

from droneflight.correction import Fix
from droneflight.model import FlightMission
from droneflight.nav import ErrorInjector, local_enu

__all__ = [
    "BridgeSample",
    "CorridorReport",
    "EvBridge",
    "EvBridgeConfig",
    "EvBridgeError",
    "EvBridgeStats",
    "GazeboTopic",
    "GazeboTruthSource",
    "RecordingPublisher",
    "TrackRecorder",
    "TruthSample",
    "VisionPose",
    "analyse_track",
    "discover_gz_topics",
    "enu_flu_to_ned_frd_euler",
    "enu_to_ned",
    "global_origin_fields",
    "load_track",
    "plan_track_enu",
    "quat_to_euler",
    "resolve_truth_topic",
]

TimestampMode = Literal["none", "epoch", "sim"]


class EvBridgeError(RuntimeError):
    """The bridge could not source truth or publish vision."""


# --------------------------------------------------------------------------- #
# Frames
# --------------------------------------------------------------------------- #


def enu_to_ned(east_m: float, north_m: float, up_m: float) -> tuple[float, float, float]:
    """Position from Gazebo's ENU world frame to MAVLink's local NED."""
    return north_m, east_m, -up_m


def quat_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Quaternion to intrinsic Z-Y-X Euler angles ``(roll, pitch, yaw)`` in radians."""
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def enu_flu_to_ned_frd_euler(
    roll_enu: float, pitch_enu: float, yaw_enu: float
) -> tuple[float, float, float]:
    """Attitude from Gazebo (ENU world, FLU body) to MAVLink (NED world, FRD body).

    Roll is the same physical motion in both conventions (left wing up); pitch
    flips sign (FLU's +y is left, FRD's is right); yaw is measured clockwise from
    north instead of counter-clockwise from east.
    """
    return roll_enu, -pitch_enu, _wrap_pi(math.pi / 2.0 - yaw_enu)


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# --------------------------------------------------------------------------- #
# Ground truth
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class TruthSample:
    """Simulator ground truth: local-ENU meters about the world origin.

    ``t_s`` is *simulator* time, not wall time — SITL can run faster or slower
    than realtime, and the vision stream should be paced in the clock EKF2 lives
    in.
    """

    t_s: float
    east_m: float
    north_m: float
    up_m: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def euler_enu(self) -> tuple[float, float, float]:
        return quat_to_euler(self.qx, self.qy, self.qz, self.qw)


@runtime_checkable
class TruthSource(Protocol):
    """A source of simulator ground truth. **Not** the autopilot's estimate."""

    def stream(self) -> AsyncIterator[TruthSample]: ...


@dataclass(frozen=True)
class GazeboTopic:
    """A Gazebo transport topic carrying pose truth, and how to read it."""

    topic: str
    kind: Literal["odometry", "pose_v"]
    model: str


_PREFERRED_KINDS: tuple[tuple[str, Literal["odometry", "pose_v"]], ...] = (
    ("/model/{model}/odometry", "odometry"),
    ("/model/{model}/pose", "pose_v"),
    ("/world/{world}/dynamic_pose/info", "pose_v"),
    ("/world/{world}/pose/info", "pose_v"),
)


def discover_gz_topics(gz_binary: str = "gz", timeout_s: float = 10.0) -> list[str]:
    """List Gazebo transport topics (``gz topic -l``)."""
    if shutil.which(gz_binary) is None:
        raise EvBridgeError(
            f"{gz_binary!r} not on PATH — the EV bridge needs Gazebo's ground truth "
            "and must run on the sim host (see RUNBOOK §3b)"
        )
    try:
        proc = subprocess.run(  # fixed binary, no shell
            [gz_binary, "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise EvBridgeError(
            f"'{gz_binary} topic -l' timed out — is the simulator running?"
        ) from exc
    if proc.returncode != 0:
        raise EvBridgeError(f"'{gz_binary} topic -l' failed: {proc.stderr.strip()}")
    return [line.strip() for line in proc.stdout.splitlines() if line.strip()]


def resolve_truth_topic(
    world: str = "default",
    model: str = "x500_0",
    *,
    available: Sequence[str] | None = None,
    gz_binary: str = "gz",
) -> GazeboTopic:
    """Pick the best available ground-truth topic for ``model``.

    Prefers the model's own odometry (pose *and* twist, published at a steady
    rate) and falls back to the world's pose broadcast, which is always present
    because the scene-broadcaster plugin is part of every PX4 world.
    """
    topics = list(available) if available is not None else discover_gz_topics(gz_binary)
    for pattern, kind in _PREFERRED_KINDS:
        candidate = pattern.format(model=model, world=world)
        if candidate in topics:
            return GazeboTopic(topic=candidate, kind=kind, model=model)
    raise EvBridgeError(
        f"no ground-truth topic for model {model!r} in world {world!r}. "
        f"Tried: {', '.join(p.format(model=model, world=world) for p, _ in _PREFERRED_KINDS)}. "
        f"Available: {', '.join(topics) or '(none — is the simulator running?)'}"
    )


def wait_for_truth_topic(
    world: str = "default",
    model: str = "x500_0",
    *,
    timeout_s: float = 30.0,
    poll_s: float = 1.0,
    gz_binary: str = "gz",
    lister: Callable[[], list[str]] | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> GazeboTopic:
    """Wait for the ground-truth topic to appear, then resolve it.

    Topic discovery is a startup *race*: Gazebo advertises the vehicle's odometry
    only once the model is spawned, which is well after `gz sim` itself is up. A
    one-shot check that exits on the first miss makes the bridge fail instantly
    if it is started a few seconds early — and since it dies before connecting to
    PX4, the visible symptom is `fly` timing out much later with no clue why.
    Polling turns that race into a wait.
    """
    list_topics = lister if lister is not None else lambda: discover_gz_topics(gz_binary)
    deadline = timeout_s
    waited = 0.0
    last: EvBridgeError | None = None
    while True:
        try:
            return resolve_truth_topic(world, model, available=list_topics())
        except EvBridgeError as exc:
            last = exc
        if waited >= deadline:
            raise last
        sleep(poll_s)
        waited += poll_s


class _JsonStreamSplitter:
    """Split a character stream into top-level JSON objects.

    ``gz topic -e --json-output`` gives one JSON object per message, but whether
    it is compact or pretty-printed varies with the Gazebo build, so neither
    line-splitting nor a fixed shape is safe. Tracking brace depth (outside
    strings, respecting escapes) handles both.
    """

    def __init__(self) -> None:
        self._buf = ""
        self._depth = 0
        self._start = -1
        self._in_string = False
        self._escaped = False

    def feed(self, text: str) -> list[str]:
        out: list[str] = []
        base = len(self._buf)
        self._buf += text
        for offset, ch in enumerate(text):
            i = base + offset
            if self._in_string:
                if self._escaped:
                    self._escaped = False
                elif ch == "\\":
                    self._escaped = True
                elif ch == '"':
                    self._in_string = False
                continue
            if ch == '"':
                self._in_string = True
            elif ch == "{":
                if self._depth == 0:
                    self._start = i
                self._depth += 1
            elif ch == "}":
                self._depth -= 1
                if self._depth == 0 and self._start >= 0:
                    out.append(self._buf[self._start : i + 1])
                    self._start = -1
        if self._depth == 0:
            self._buf = ""
        elif self._start > 0:
            self._buf = self._buf[self._start :]
            self._start = 0
        return out


def _num(node: Any, *keys: str) -> float:
    """Read a nested numeric field, tolerating protobuf-JSON quirks.

    Protobuf JSON omits fields at their default value (so a drone exactly on the
    world x-axis has *no* ``y`` key, not ``"y": 0``) and renders 64-bit ints as
    strings. Both would otherwise show up as a silent frame error.
    """
    cur = node
    for key in keys:
        if not isinstance(cur, dict):
            return 0.0
        cur = cur.get(key)
    if isinstance(cur, bool) or cur is None:
        return 0.0
    if isinstance(cur, int | float):
        return float(cur)
    if isinstance(cur, str):
        try:
            return float(cur)
        except ValueError:
            return 0.0
    return 0.0


def _stamp_s(node: Any) -> float:
    return _num(node, "header", "stamp", "sec") + _num(node, "header", "stamp", "nsec") * 1e-9


def _sample_from_odometry(doc: Any) -> TruthSample | None:
    if not isinstance(doc, dict) or "pose" not in doc:
        return None
    return TruthSample(
        t_s=_stamp_s(doc),
        east_m=_num(doc, "pose", "position", "x"),
        north_m=_num(doc, "pose", "position", "y"),
        up_m=_num(doc, "pose", "position", "z"),
        qx=_num(doc, "pose", "orientation", "x"),
        qy=_num(doc, "pose", "orientation", "y"),
        qz=_num(doc, "pose", "orientation", "z"),
        qw=_num(doc, "pose", "orientation", "w"),
    )


def _sample_from_pose_v(doc: Any, model: str) -> TruthSample | None:
    if not isinstance(doc, dict):
        return None
    poses = doc.get("pose")
    if not isinstance(poses, list):
        return None
    for entry in poses:
        if isinstance(entry, dict) and entry.get("name") == model:
            stamp = _stamp_s(doc) or _stamp_s(entry)
            return TruthSample(
                t_s=stamp,
                east_m=_num(entry, "position", "x"),
                north_m=_num(entry, "position", "y"),
                up_m=_num(entry, "position", "z"),
                qx=_num(entry, "orientation", "x"),
                qy=_num(entry, "orientation", "y"),
                qz=_num(entry, "orientation", "z"),
                qw=_num(entry, "orientation", "w"),
            )
    return None


def parse_truth(doc: Any, topic: GazeboTopic) -> TruthSample | None:
    """Decode one ``gz topic`` JSON message into a :class:`TruthSample`."""
    if topic.kind == "odometry":
        return _sample_from_odometry(doc)
    return _sample_from_pose_v(doc, topic.model)


class GazeboTruthSource:
    """Ground truth from Gazebo, over ``gz topic -e --json-output``.

    Shelling out to the ``gz`` CLI rather than binding ``gz-transport``'s Python
    module keeps the bridge installable in the same venv as the rest of the
    stack: the transport bindings are a system package tied to the Gazebo build,
    and are not always present even where the simulator is.
    """

    def __init__(self, topic: GazeboTopic, *, gz_binary: str = "gz") -> None:
        self.topic = topic
        self._gz = gz_binary
        self._proc: asyncio.subprocess.Process | None = None

    async def stream(self) -> AsyncIterator[TruthSample]:
        proc = await asyncio.create_subprocess_exec(
            self._gz,
            "topic",
            "-e",
            "-t",
            self.topic.topic,
            "--json-output",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        self._proc = proc
        assert proc.stdout is not None
        splitter = _JsonStreamSplitter()
        try:
            while True:
                chunk = await proc.stdout.read(8192)
                if not chunk:
                    break
                for blob in splitter.feed(chunk.decode("utf-8", errors="replace")):
                    try:
                        doc = json.loads(blob)
                    except json.JSONDecodeError:  # pragma: no cover - defensive
                        continue
                    sample = parse_truth(doc, self.topic)
                    if sample is not None:
                        yield sample
            stderr = b""
            if proc.stderr is not None:
                stderr = await proc.stderr.read()
            raise EvBridgeError(
                f"ground-truth stream on {self.topic.topic} ended "
                f"(rc={proc.returncode}): {stderr.decode(errors='replace').strip()}"
            )
        finally:
            await self.close()

    async def close(self) -> None:
        proc, self._proc = self._proc, None
        if proc is None or proc.returncode is not None:
            return
        proc.terminate()
        try:
            async with asyncio.timeout(5.0):
                await proc.wait()
        except TimeoutError:  # pragma: no cover - defensive
            proc.kill()


class ReplayTruthSource:
    """Replay a recorded truth track — offline reruns, and tests without Gazebo."""

    def __init__(self, samples: Iterable[TruthSample]) -> None:
        self._samples = list(samples)

    async def stream(self) -> AsyncIterator[TruthSample]:
        for sample in self._samples:
            yield sample


# --------------------------------------------------------------------------- #
# Global origin
# --------------------------------------------------------------------------- #


def global_origin_fields(lat: float, lon: float, alt_m: float) -> dict[str, int]:
    """Payload for MAVLink ``SET_GPS_GLOBAL_ORIGIN`` (degE7 and millimetres)."""
    return {
        "target_system": 1,
        "latitude": round(lat * 1e7),
        "longitude": round(lon * 1e7),
        "altitude": round(alt_m * 1000.0),
        "time_usec": 0,
    }


async def set_global_origin(  # pragma: no cover - requires a live autopilot
    system: Any, lat: float, lon: float, alt_m: float
) -> None:
    """Anchor EKF2's local frame to a WGS84 origin.

    A ``.plan`` is lat/lon, so PX4 can only fly it in AUTO if the estimator can
    convert global to local. With GPS fusion off, nothing supplies that: EKF2
    takes its origin from the GNSS fix it is no longer allowed to use, so the
    local frame stays purely relative, ``is_home_position_ok`` never goes true,
    and the mission is rejected before it starts. Sending the plan's home as an
    explicit origin is the GPS-denied stand-in for what the georeferenced
    basemap does on the aircraft: it is VTRN, not GPS, that ties this vehicle's
    local frame to the world.
    """
    from mavsdk.mavlink_direct import MavlinkMessage

    try:
        await system.mavlink_direct.send_message(
            MavlinkMessage(
                message_name="SET_GPS_GLOBAL_ORIGIN",
                system_id=0,
                component_id=0,
                target_system_id=1,
                target_component_id=1,
                fields_json=json.dumps(global_origin_fields(lat, lon, alt_m)),
            )
        )
    except Exception as exc:
        raise EvBridgeError(
            f"could not set the EKF2 global origin ({exc}). Without it a lat/lon "
            "mission cannot be flown GPS-denied — see RUNBOOK §3b."
        ) from exc


# --------------------------------------------------------------------------- #
# Vision output
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class VisionPose:
    """A pose to publish as MAVLink ``VISION_POSITION_ESTIMATE`` (local NED/FRD)."""

    north_m: float
    east_m: float
    down_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    time_usec: int = 0


@runtime_checkable
class VisionPublisher(Protocol):
    async def publish(self, pose: VisionPose) -> None: ...


class RecordingPublisher:
    """In-memory publisher for tests and ``--dry-run``."""

    def __init__(self) -> None:
        self.poses: list[VisionPose] = []

    async def publish(self, pose: VisionPose) -> None:
        self.poses.append(pose)


class MavsdkMocapPublisher:  # pragma: no cover - requires a live autopilot
    """Publish vision poses through MAVSDK's ``mocap`` plugin."""

    def __init__(self, system: Any) -> None:
        self._system = system

    async def publish(self, pose: VisionPose) -> None:
        from mavsdk.mocap import AngleBody, Covariance, PositionBody, VisionPositionEstimate

        try:
            await self._system.mocap.set_vision_position_estimate(
                VisionPositionEstimate(
                    time_usec=pose.time_usec,
                    position_body=PositionBody(pose.north_m, pose.east_m, pose.down_m),
                    angle_body=AngleBody(pose.roll_rad, pose.pitch_rad, pose.yaw_rad),
                    # First element NaN = "covariance unknown" (MAVLink convention).
                    pose_covariance=Covariance([float("nan")]),
                )
            )
        except Exception as exc:
            raise EvBridgeError(f"vision publish failed: {exc}") from exc


# --------------------------------------------------------------------------- #
# The bridge
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class EvBridgeConfig:
    """Bridge plumbing (the error model itself lives in ``ErrorInjector``)."""

    rate_hz: float = 30.0  # vision publish rate, paced in simulator time
    corridor_tolerance_m: float = 15.0
    timestamp_mode: TimestampMode = "none"
    max_duration_s: float = 0.0  # 0 = run until the truth stream ends or stop is set


@dataclass(frozen=True)
class BridgeSample:
    """One step of the bridge: truth in, corrupted pose out, estimate observed."""

    t_s: float
    truth_enu: tuple[float, float, float]
    error_en: tuple[float, float]
    published: VisionPose
    estimate_ned: tuple[float, float, float] | None = None
    fix: str | None = None
    # PX4's flight mode, so analysis can separate the mission from an RTL or
    # loiter. Without it a post-abort return-to-home — which beelines across the
    # planned route — is scored as navigation error and dominates the result.
    mode: str | None = None

    def to_dict(self) -> dict[str, Any]:
        doc: dict[str, Any] = {
            "t": round(self.t_s, 4),
            "truth": {
                "e": round(self.truth_enu[0], 3),
                "n": round(self.truth_enu[1], 3),
                "u": round(self.truth_enu[2], 3),
            },
            "err": {"e": round(self.error_en[0], 3), "n": round(self.error_en[1], 3)},
            "pub": {
                "n": round(self.published.north_m, 3),
                "e": round(self.published.east_m, 3),
                "d": round(self.published.down_m, 3),
                "yaw": round(self.published.yaw_rad, 4),
            },
        }
        if self.estimate_ned is not None:
            n, e, d = self.estimate_ned
            doc["est"] = {"n": round(n, 3), "e": round(e, 3), "d": round(d, 3)}
        if self.fix is not None:
            doc["fix"] = self.fix
        if self.mode is not None:
            doc["mode"] = self.mode
        return doc


@dataclass
class EvBridgeStats:
    """What the bridge did — and what the injected error grew to."""

    samples: int = 0
    published: int = 0
    distance_m: float = 0.0
    max_error_m: float = 0.0
    final_error_m: float = 0.0
    attempts: int = 0
    accepted: int = 0
    no_features: int = 0
    low_confidence: int = 0
    gated_out: int = 0
    corrupt_accepted: int = 0
    track_enu: list[tuple[float, float]] = field(default_factory=list)

    @property
    def fix_availability(self) -> float:
        return self.accepted / self.attempts if self.attempts else 0.0


class TrackRecorder:
    """Append bridge samples to a JSONL file for post-flight analysis."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._fh: Any = None

    def __enter__(self) -> TrackRecorder:
        self._fh = self.path.open("w", encoding="utf-8")
        return self

    def __exit__(self, *exc: object) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def write(self, sample: BridgeSample) -> None:
        if self._fh is None:  # pragma: no cover - defensive
            return
        self._fh.write(json.dumps(sample.to_dict(), separators=(",", ":")) + "\n")


class EvBridge:
    """Stream corrupted ground truth into EKF2 as external vision.

    The loop is deliberately dumb: every truth sample advances the error model by
    the distance actually flown, and the published pose is truth + that error.
    Nothing here decides *how* the error behaves — that is
    :class:`~droneflight.nav.ErrorInjector`, shared with the offline study, so a
    SITL run and a `droneflight nav` run are the same model.
    """

    def __init__(
        self,
        injector: ErrorInjector,
        config: EvBridgeConfig | None = None,
        *,
        recorder: TrackRecorder | None = None,
        estimate_probe: Callable[[], tuple[float, float, float] | None] | None = None,
        mode_probe: Callable[[], str | None] | None = None,
        on_sample: Callable[[BridgeSample], None] | None = None,
    ) -> None:
        self.injector = injector
        self.config = config or EvBridgeConfig()
        self.recorder = recorder
        # Diagnostic only: PX4's own estimate, logged so a run can be checked for
        # "is EKF2 actually fusing this?". It must never feed the injector.
        self.estimate_probe = estimate_probe
        self.mode_probe = mode_probe
        self.on_sample = on_sample

    async def run(
        self,
        truth: TruthSource,
        publisher: VisionPublisher,
        *,
        stop: asyncio.Event | None = None,
    ) -> EvBridgeStats:
        stats = EvBridgeStats()
        interval = 1.0 / self.config.rate_hz if self.config.rate_hz > 0 else 0.0
        last_pos: tuple[float, float] | None = None
        # Publish against a rolling *deadline*, not "time since last publish".
        # The latter silently undershoots whenever the truth rate is not a
        # multiple of the target: 50 Hz truth asked for 30 Hz publishes every
        # other sample and yields 25 Hz. Accumulating the deadline keeps the
        # long-run average honest.
        next_pub_t = -math.inf
        t0: float | None = None

        async for sample in truth.stream():
            if stop is not None and stop.is_set():
                break
            stats.samples += 1
            if t0 is None:
                t0 = sample.t_s
            if self.config.max_duration_s > 0.0 and sample.t_s - t0 >= self.config.max_duration_s:
                break

            # Advance the error model by the horizontal distance actually flown.
            # The fix attempt is made at the TRUE position: a nadir camera sees
            # the real world, not the estimate.
            fix: Fix | None = None
            if last_pos is not None:
                step = math.hypot(sample.east_m - last_pos[0], sample.north_m - last_pos[1])
                fix = self.injector.advance(step, sample.east_m, sample.north_m)
            last_pos = (sample.east_m, sample.north_m)

            err_e, err_n = self.injector.error
            err = math.hypot(err_e, err_n)
            stats.max_error_m = max(stats.max_error_m, err)
            stats.final_error_m = err
            stats.track_enu.append((sample.east_m, sample.north_m))

            if sample.t_s < next_pub_t:
                continue
            next_pub_t = sample.t_s + interval if next_pub_t == -math.inf else next_pub_t + interval

            pose = self._pose_for(sample, (err_e, err_n))
            await publisher.publish(pose)
            stats.published += 1

            record = BridgeSample(
                t_s=sample.t_s,
                truth_enu=(sample.east_m, sample.north_m, sample.up_m),
                error_en=(err_e, err_n),
                published=pose,
                estimate_ned=self.estimate_probe() if self.estimate_probe else None,
                fix=fix.outcome.value if fix else None,
                mode=self.mode_probe() if self.mode_probe else None,
            )
            if self.recorder is not None:
                self.recorder.write(record)
            if self.on_sample is not None:
                self.on_sample(record)

        self._finalise(stats)
        return stats

    def _pose_for(self, sample: TruthSample, error: tuple[float, float]) -> VisionPose:
        err_e, err_n = error
        # Estimate = truth + error, horizontally. Height and attitude are truth:
        # the error model is 2D (see the module docstring).
        north, east, down = enu_to_ned(sample.east_m + err_e, sample.north_m + err_n, sample.up_m)
        roll, pitch, yaw = enu_flu_to_ned_frd_euler(*sample.euler_enu())
        return VisionPose(
            north_m=north,
            east_m=east,
            down_m=down,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            time_usec=self._timestamp_usec(sample),
        )

    def _timestamp_usec(self, sample: TruthSample) -> int:
        """Timestamp for the vision message — see RUNBOOK §3b if EKF2 rejects it.

        ``none`` (0) lets PX4 stamp on receipt. That is right when MAVLink
        timesync has not converged and wrong when it has (PX4 adds its offset to
        whatever it is given), which is why this is switchable rather than
        assumed: the estimate probe tells you within seconds which one this build
        wants.
        """
        mode = self.config.timestamp_mode
        if mode == "epoch":
            return int(time.time() * 1e6)
        if mode == "sim":
            return int(sample.t_s * 1e6)
        return 0

    def _finalise(self, stats: EvBridgeStats) -> None:
        from droneflight.correction import FixOutcome

        counts = self.injector.counts
        stats.distance_m = self.injector.distance_m
        stats.attempts = self.injector.attempts
        stats.accepted = self.injector.accepted
        stats.no_features = counts[FixOutcome.NO_FEATURES]
        stats.low_confidence = counts[FixOutcome.LOW_CONFIDENCE]
        stats.gated_out = counts[FixOutcome.GATED_OUT]
        stats.corrupt_accepted = counts[FixOutcome.ACCEPTED_CORRUPT]


# --------------------------------------------------------------------------- #
# Corridor analysis — the acceptance test
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class CorridorReport:
    """How far the **true** track strayed from the planned route."""

    samples: int
    max_deviation_m: float
    mean_deviation_m: float
    endpoint_deviation_m: float
    tolerance_m: float

    @property
    def within_corridor(self) -> bool:
        return self.max_deviation_m <= self.tolerance_m


def plan_track_enu(mission: FlightMission) -> tuple[tuple[float, float], ...]:
    """The mission's waypoints as local-ENU meters about its home point.

    PX4 SITL is launched with home set from the same plan (``droneflight
    sitl-env``), so this shares an origin with Gazebo's world frame.
    """
    return tuple(
        local_enu(w.lat, w.lon, mission.home_lat, mission.home_lon) for w in mission.waypoints
    )


def _distance_to_polyline(
    point: tuple[float, float], polyline: Sequence[tuple[float, float]]
) -> float:
    px, py = point
    if len(polyline) == 1:
        return math.hypot(px - polyline[0][0], py - polyline[0][1])
    best = math.inf
    for (ax, ay), (bx, by) in pairwise(polyline):
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 == 0.0 else ((px - ax) * dx + (py - ay) * dy) / seg2
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def analyse_track(
    track: Sequence[tuple[float, float]],
    plan: Sequence[tuple[float, float]],
    *,
    tolerance_m: float = 15.0,
) -> CorridorReport:
    """Cross-track deviation of a flown ENU track against the planned polyline."""
    if not track or not plan:
        raise EvBridgeError("need both a flown track and a planned route to compare")
    deviations = [_distance_to_polyline(p, plan) for p in track]
    return CorridorReport(
        samples=len(deviations),
        max_deviation_m=max(deviations),
        mean_deviation_m=sum(deviations) / len(deviations),
        endpoint_deviation_m=math.hypot(track[-1][0] - plan[-1][0], track[-1][1] - plan[-1][1]),
        tolerance_m=tolerance_m,
    )


def load_track(path: Path) -> list[BridgeSample]:
    """Read a JSONL track written by :class:`TrackRecorder`."""
    samples: list[BridgeSample] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        doc = json.loads(line)
        pub = doc.get("pub", {})
        est = doc.get("est")
        samples.append(
            BridgeSample(
                t_s=_num(doc, "t"),
                truth_enu=(
                    _num(doc, "truth", "e"),
                    _num(doc, "truth", "n"),
                    _num(doc, "truth", "u"),
                ),
                error_en=(_num(doc, "err", "e"), _num(doc, "err", "n")),
                published=VisionPose(
                    north_m=_num(pub, "n"),
                    east_m=_num(pub, "e"),
                    down_m=_num(pub, "d"),
                    roll_rad=0.0,
                    pitch_rad=0.0,
                    yaw_rad=_num(pub, "yaw"),
                ),
                estimate_ned=(
                    (_num(est, "n"), _num(est, "e"), _num(est, "d")) if est is not None else None
                ),
                fix=doc.get("fix"),
                mode=doc.get("mode"),
            )
        )
    return samples
