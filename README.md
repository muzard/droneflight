# droneflight

Online **flight stack** for the autonomous fiber-laying drone — the subsystem
that *executes* the offline planner's route on a PX4 autopilot. It consumes the
`droneplanner` `.plan` (the integration boundary), applies a **tether-safe speed
policy** and a **covert sensor-mode policy**, and drives PX4 over MAVLink/MAVSDK
for GPS-denied, low-altitude flight.

This is the hardware-dependent subsystem from `../Technology-Selection.md` §3–4.
Live runs target **PX4 SITL + Gazebo Harmonic** (see [RUNBOOK.md](RUNBOOK.md));
the orchestration/policy logic is unit-tested against a fake autopilot so it is
verifiable without hardware.

## Status — F0 (bridge + policies + executor scaffold)

| Piece | State |
|---|---|
| QGC `.plan` loader (`plan_loader.py`) | ✅ consumes droneplanner output |
| Tether-safe speed policy (`speed_policy.py`) | ✅ slows through turns/reversals |
| Covert sensor-mode policy (`sensors.py`) | ✅ passive-optical / thermal / active-lidar tradeoff |
| Mission executor + failsafe (`executor.py`) | ✅ tested against a fake connector |
| MAVSDK adapter (`mavsdk_connector.py`) | ⚠️ live path — runs against SITL, not unit-tested |
| PX4 SITL + Gazebo tether model | ⏳ RUNBOOK + config; needs the Ubuntu sim host |
| Onboard perception (VIO/VTRN/LIO, avoidance) | later — separate effort |

## Usage

```bash
droneflight validate --plan out/mission.plan          # offline: policies + sensor mode
droneflight validate --plan out/mission.plan --night  # night: exercises the sensor tradeoff
droneflight fly --plan out/mission.plan --url udp://:14540   # live: execute on PX4/SITL
```

`validate` is fully offline (no autopilot); `fly` needs a running PX4 (SITL or
hardware) reachable at `--url`.

## The sensor tradeoff (why lidar is not the default)

Sensor choice is a **concealment decision**, not "lidar is best" (see
`sensors.py`):

- **Passive optical** — emits nothing (covert), but night-blind.
- **Active lidar** — works day/night and is robust in foliage, **but it emits**:
  the laser is visible to night-vision devices and laser-warning receivers, which
  defeats the "not seen" mission. It's a build option, gated behind
  `allow_active_emission`.
- **Passive thermal** — passive and night-capable (covert at night), but
  export-controlled (ITAR/EAR).

For a covert **night** mission with only passive optics, the policy **refuses**
rather than silently emitting — you must add thermal or explicitly accept
detection.

## Design

The executor drives the autopilot through a narrow async `FlightConnector`
interface (`connect → upload → arm → start → monitor`, RTL on error). `FakeConnector`
makes the flow testable; `MavsdkConnector` is the live adapter. GPS-denied note:
PX4 on external vision reports position health via *local*, not global, position —
the adapter gates on that.

## Development

```bash
pip install -e ".[dev]"
ruff check . && ruff format --check .
mypy --strict src
pytest --cov=droneflight --cov-fail-under=85
```

## Scope

This package is flight *execution* + policy. It is **not** the onboard state
estimator (VIO/VTRN/LIO), obstacle avoidance, or the payout-hardware controller —
those are separate efforts. No targeting/munitions logic (out of scope, per the
program spec).
