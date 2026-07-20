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

## Status — F1 (bridge + policies + perception/avoidance + nav emulation)

| Piece | State |
|---|---|
| QGC `.plan` loader (`plan_loader.py`) | ✅ consumes droneplanner output |
| Tether-safe speed policy (`speed_policy.py`) | ✅ slows through turns/reversals |
| Covert sensor-mode policy (`sensors.py`) | ✅ passive-optical / thermal / active-lidar tradeoff |
| Mission executor + failsafe (`executor.py`) | ✅ tested against a fake connector |
| **Optical detection + avoidance** (`perception.py`, `avoidance.py`) | ✅ depth sensing + follow-the-gap steering |
| **Closed-loop forest sim** (`reactive.py`) | ✅ flies the route through trees; avoidance ON vs OFF |
| **GPS-denied nav emulation** (`nav.py`) | ✅ VIO drift + VTRN correction |
| MAVSDK adapter (`mavsdk_connector.py`) | ⚠️ live path — runs against SITL, not unit-tested |
| PX4 SITL + Gazebo tether model | ⏳ RUNBOOK + config; needs the Ubuntu sim host |
| Real onboard perception (VIO/VTRN/LIO estimator) | later — separate effort |

## Usage

```bash
droneflight validate --plan out/mission.plan          # offline: policies + sensor mode
droneflight validate --plan out/mission.plan --night  # night: exercises the sensor tradeoff
droneflight nav      --plan out/mission.plan          # GPS-denied drift: VIO vs VIO+VTRN
droneflight simfly   --plan out/mission.plan --density 25          # forest flight: avoidance ON vs OFF
droneflight simfly   --plan out/mission.plan --drift-rate 0.02 --vtrn-interval 150
droneflight fly      --plan out/mission.plan --url udp://:14540    # live: execute on PX4/SITL
```

`validate`, `nav`, and `simfly` are fully offline (kinematic, no autopilot);
`fly` needs a running PX4 (SITL or hardware) reachable at `--url`.

## Optical detection + avoidance (won't crash into trees)

`perception.py` emulates the forward depth sensor (a passive stereo pair or a
lidar slice) as rays returning the range to the nearest trunk; `avoidance.py` is
a follow-the-gap / VFH-lite controller that steers to the freest bearing toward
the goal and slows in clutter. `reactive.py` closes the loop: it flies a planned
`.plan` through a scattered forest and checks it reaches the goal without a
collision.

What the closed-loop sim shows (deterministic, runs on any machine):

- **With known position**, the avoidance layer reliably dodges obstacles the plan
  didn't map: flying the waypoints *blindly* collides, the reactive layer reaches
  the goal (down to very dense stands, where reactive flight is genuinely
  marginal — matching the program's own honest limits).
- **Under GPS-denied drift**, avoidance senses *real* trees in the body frame, so
  it keeps dodging — but drift (tens of meters over km without correction, per
  `nav.py`) wanders the *true* path out of the planned tube into dense forest.
  So collision-free forest flight needs **both** reactive avoidance **and**
  bounded navigation (VTRN), not either alone.

This is a **kinematic** sim (2D horizontal, no physics engine) — a fast design
tool. Fidelity comes later from PX4 SITL + Gazebo (RUNBOOK).

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
