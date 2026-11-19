"""GNSS into EKF2 *through the consistency gate* — the SITL half of roadmap P4.

:mod:`droneflight.gnss_spoof` answers offline how far a carry-off spoofer can drag
a VIO-consistency-gated estimate. This module puts that gate where it would live
on the aircraft: on the companion computer, **in front of** EKF2.

WHY IN FRONT OF EKF2, NOT INSIDE IT
EKF2 already gates every GNSS sample on its innovation (``EKF2_GPS_P_GATE``). That
test is blind to carry-off by construction: the spoofer starts at the true
position and moves the fix by centimetres per sample, so each innovation is small
and EKF2 follows it — the attack exists precisely to pass that gate. What catches
it is the *rate* of GNSS-minus-VIO over hundreds of metres, which is a companion
check (:class:`~droneflight.gnss_spoof.ConsistencyMonitor`). So the bridge
synthesises the receiver output (Gazebo truth + spoof + noise), runs the monitor
against the VIO odometry it is already publishing as external vision, and sends
the fix to PX4 as MAVLink ``GPS_INPUT`` **only while the monitor trusts it**. On
an alarm the stream stops for good and EKF2 times GNSS out onto vision alone.

WHAT THE SITL RUN ADDS OVER THE OFFLINE SWEEP
* EKF2 fuses GNSS *and* EV together while GNSS is trusted. Whether it absorbs the
  VIO drift as an EV position bias (as PX4's ``ev_pos`` control is meant to when
  GNSS is active) or fights it, the sweep cannot say.
* There is no rewind. By the time the gate closes EKF2 has followed the spoofer
  for the whole window, so the fallback starts from the alarm point —
  ``tools/gnss_spoof_sweep.py --no-rewind`` is the matching offline prediction.
* The GNSS timeout and the hand-over to vision-only are PX4's, not a model's.

THE SPOOFER IS THE STRONG ONE
Its velocity is *consistent* with its position (Doppler follows the drag), so
EKF2's GNSS velocity fusion cannot separate it either. The monitor uses positions
only, as offline.

⚠️ Requires raw VIO: VTRN fixes must be off in the bridge (``--no-vtrn``) —
a fix moves odometry by metres and would read as spoofing. Enforced.
⚠️ GNSS noise is white per sample (real receivers are time-correlated, which is
*easier* for a rate test over 100-1000 m windows, not harder).
⚠️ Positions are projected from the plan home with PX4's own azimuthal-equidistant
map projection, so honest GNSS and Gazebo truth agree to millimetres over a 5 km
AO — provided Gazebo's origin is that home (``droneflight sitl-env``).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from droneflight.gnss_spoof import ConsistencyMonitor, MonitorConfig, SpoofAttack, spoof_offset

EARTH_RADIUS_M = 6371000.0
"""PX4's ``CONSTANTS_RADIUS_OF_EARTH`` — the projection must be PX4's, not WGS84's."""


def reproject(
    lat0_deg: float, lon0_deg: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Local NED metres → lat/lon, exactly as PX4's ``MapProjection::reproject``."""
    lat0 = math.radians(lat0_deg)
    x = north_m / EARTH_RADIUS_M
    y = east_m / EARTH_RADIUS_M
    c = math.hypot(x, y)
    if c == 0.0:
        return lat0_deg, lon0_deg
    sin_c, cos_c = math.sin(c), math.cos(c)
    lat = math.asin(cos_c * math.sin(lat0) + x * sin_c * math.cos(lat0) / c)
    lon = math.radians(lon0_deg) + math.atan2(
        y * sin_c, c * math.cos(lat0) * cos_c - x * math.sin(lat0) * sin_c
    )
    return math.degrees(lat), math.degrees(lon)


def project(
    lat0_deg: float, lon0_deg: float, lat_deg: float, lon_deg: float
) -> tuple[float, float]:
    """lat/lon → local (north, east) metres; inverse of :func:`reproject` (PX4's)."""
    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    cos_d = math.cos(lon - lon0)
    arg = math.sin(lat0) * math.sin(lat) + math.cos(lat0) * math.cos(lat) * cos_d
    c = math.acos(max(-1.0, min(1.0, arg)))
    k = c / math.sin(c) if c > 0.0 else 1.0
    north = k * (math.cos(lat0) * math.sin(lat) - math.sin(lat0) * math.cos(lat) * cos_d)
    east = k * math.cos(lat) * math.sin(lon - lon0)
    return north * EARTH_RADIUS_M, east * EARTH_RADIUS_M


@dataclass(frozen=True)
class GnssGateConfig:
    home_lat: float
    home_lon: float
    home_alt_m: float
    attack: SpoofAttack | None = None
    """``bearing_deg`` is a COMPASS bearing here (0 = north, 90 = east)."""
    monitor: MonitorConfig = field(default_factory=MonitorConfig)
    gated: bool = True
    """False = the control arm: publish every fix, spoofed or not. The monitor
    still runs, so the record says when it *would* have closed the gate."""
    rate_hz: float = 5.0
    """Receiver output rate (sim time). EKF2 times GNSS out after ~0.5 s of
    silence at defaults, so keep this well above 2 Hz."""
    monitor_step_m: float = 5.0
    """Distance between monitor tests — the step the offline sweep was run at."""
    seed: int = 4


@dataclass(frozen=True)
class GnssFix:
    """One receiver output, in both the local frame and WGS84."""

    t_s: float
    east_m: float
    north_m: float
    up_m: float
    lat: float
    lon: float
    alt_m: float
    vel_ned: tuple[float, float, float]
    spoof_en: tuple[float, float]
    published: bool
    """False when the gate is closed (the fix was computed and withheld)."""


def gps_input_fields(fix: GnssFix, sigma_m: float) -> dict[str, Any]:
    """Payload for MAVLink ``GPS_INPUT`` (#232) — a healthy 3D fix."""
    return {
        "time_usec": 0,
        "gps_id": 0,
        "ignore_flags": 0,
        "time_week_ms": 0,
        "time_week": 0,
        "fix_type": 3,
        "lat": round(fix.lat * 1e7),
        "lon": round(fix.lon * 1e7),
        "alt": fix.alt_m,
        "hdop": 0.7,
        "vdop": 1.0,
        "vn": fix.vel_ned[0],
        "ve": fix.vel_ned[1],
        "vd": fix.vel_ned[2],
        "speed_accuracy": 0.3,
        "horiz_accuracy": sigma_m * math.sqrt(2.0),
        "vert_accuracy": 3.0,
        "satellites_visible": 14,
        "yaw": 0,  # 0 = no GNSS heading
    }


@runtime_checkable
class GnssPublisher(Protocol):
    async def publish(self, fix: GnssFix) -> None: ...


class RecordingGnssPublisher:
    """In-memory publisher for tests and ``--dry-run``."""

    def __init__(self) -> None:
        self.fixes: list[GnssFix] = []

    async def publish(self, fix: GnssFix) -> None:
        self.fixes.append(fix)


class MavsdkGpsInputPublisher:  # pragma: no cover - requires a live autopilot
    """Send fixes as ``GPS_INPUT`` through MAVSDK's ``mavlink_direct``."""

    def __init__(self, system: Any, sigma_m: float) -> None:
        self._system = system
        self._sigma = sigma_m

    async def publish(self, fix: GnssFix) -> None:
        import json

        from mavsdk.mavlink_direct import MavlinkMessage

        from droneflight.ev_bridge import EvBridgeError

        try:
            await self._system.mavlink_direct.send_message(
                MavlinkMessage(
                    message_name="GPS_INPUT",
                    system_id=0,
                    component_id=0,
                    target_system_id=1,
                    target_component_id=1,
                    fields_json=json.dumps(gps_input_fields(fix, self._sigma)),
                )
            )
        except Exception as exc:
            raise EvBridgeError(f"GPS_INPUT publish failed: {exc}") from exc


class GnssGate:
    """Synthesise the receiver, test it against VIO, decide whether EKF2 gets it."""

    def __init__(self, config: GnssGateConfig) -> None:
        self.config = config
        self.monitor = ConsistencyMonitor(config.monitor, rewind=False)
        self._rng = random.Random(config.seed)
        self._next_t = -math.inf
        self._next_test_m = 0.0
        self.alarm_t_s = math.inf
        self.published = 0
        self.withheld = 0

    @property
    def open(self) -> bool:
        """True while fixes reach EKF2."""
        return self.monitor.trusted or not self.config.gated

    def step(
        self,
        t_s: float,
        truth_enu: tuple[float, float, float],
        vio_en: tuple[float, float],
        distance_m: float,
        vel_ned: tuple[float, float, float] | None,
    ) -> GnssFix | None:
        """Advance to one truth sample; return a fix when one is due (else None).

        ``distance_m`` is horizontal distance flown (the bridge's injector total),
        ``vio_en`` the raw odometry position (truth + VIO error) — never PX4's
        estimate, which would contain the very GNSS being tested.
        """
        cfg = self.config
        if t_s < self._next_t:
            return None
        interval = 1.0 / cfg.rate_hz if cfg.rate_hz > 0.0 else 0.0
        self._next_t = t_s + interval if self._next_t == -math.inf else self._next_t + interval

        sigma = cfg.monitor.gnss_sigma_m
        # Compass bearing: spoof_offset's (cos, sin) pair is (north, east).
        s_n, s_e = spoof_offset(cfg.attack, distance_m)
        east = truth_enu[0] + s_e + self._rng.gauss(0.0, sigma)
        north = truth_enu[1] + s_n + self._rng.gauss(0.0, sigma)
        up = truth_enu[2]

        vn, ve, vd = vel_ned if vel_ned is not None else (0.0, 0.0, 0.0)
        if cfg.attack is not None and distance_m > cfg.attack.onset_m:
            # A consistent spoofer: Doppler carries the drag rate too.
            ground_speed = math.hypot(vn, ve)
            b = math.radians(cfg.attack.bearing_deg)
            vn += cfg.attack.pull_rate * ground_speed * math.cos(b)
            ve += cfg.attack.pull_rate * ground_speed * math.sin(b)

        if distance_m >= self._next_test_m:
            self._next_test_m = distance_m + cfg.monitor_step_m
            if self.monitor.update(distance_m, (east, north), vio_en):
                self.alarm_t_s = t_s

        lat, lon = reproject(cfg.home_lat, cfg.home_lon, north, east)
        publish = self.open
        if publish:
            self.published += 1
        else:
            self.withheld += 1
        return GnssFix(
            t_s=t_s,
            east_m=east,
            north_m=north,
            up_m=up,
            lat=lat,
            lon=lon,
            alt_m=cfg.home_alt_m + up,
            vel_ned=(vn, ve, vd),
            spoof_en=(s_e, s_n),
            published=publish,
        )

    @property
    def alarm_after_onset_m(self) -> float:
        """Distance from spoof onset to alarm (inf if none; negative = false alarm)."""
        a = self.config.attack
        onset = a.onset_m if a is not None else math.inf
        return (
            self.monitor.alarm_at_m - onset if math.isfinite(self.monitor.alarm_at_m) else math.inf
        )
