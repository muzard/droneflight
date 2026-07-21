# CLAUDE.md — droneflight (online flight stack)

Executes the planner's `.plan` on PX4 (GPS-denied, low-altitude, tether-aware),
with a covert **sensor-mode policy**, a tether-safe **speed policy**, an **optical
detection + avoidance** layer, and a **GPS-denied navigation-error emulation**.
Hardware-dependent subsystem (Technology-Selection §3–4); live runs target PX4
SITL + Gazebo on the Ubuntu VM (`RUNBOOK.md`). See `README.md` for usage.

## Status — F1 ✅ (all modules complete + tested; nothing half-built)
| Module | What |
|---|---|
| `plan_loader.py` | QGC `.plan` → `FlightMission` (the droneplanner boundary) |
| `sensors.py` | passive-optical / passive-thermal / active-lidar tradeoff |
| `speed_policy.py` | slow through turns/reversals (protect the fiber) |
| `perception.py` | forward depth sensing (rays → nearest-trunk range; stereo or lidar slice) |
| `avoidance.py` | follow-the-gap / VFH-lite reactive steering + slow-in-clutter |
| `reactive.py` | closed-loop **kinematic** forest-flight sim (avoidance ON vs OFF, drift, VTRN) |
| `nav.py` | **the simulated error** — VIO drift + VTRN correction (idealised reset *and* fidelity-limited optical driver) |
| `correction.py` | **optical terrain-relative correction** — data-driven fixes over a spatially varying basemap; confidence floor + innovation gate + Kalman fuse |
| `executor.py` + `connector.py` | connect→upload→arm→start→monitor, RTL on error (fake connector for tests) |
| `mavsdk_connector.py` | live PX4 adapter (⚠️ needs SITL; excluded from coverage, not unit-tested) |
| `config.py`, `cli.py` | config + `validate` / `nav` / `simfly` / `fly` commands |

## Run / test
```bash
cd /Users/arttu/Claude/Projects/DroneAutomation/droneflight
. /Users/arttu/Claude/Projects/DroneAutomation/.venv/bin/activate
droneflight validate --plan ../droneplanner/out/mission.plan            # offline: policies + sensor mode
droneflight nav      --plan .../mission.plan                            # VIO vs idealised vs optical VTRN
droneflight nav      --plan .../mission.plan --forest-frac 0.7 --stale-frac 0.3  # conflict-zone basemap
droneflight simfly   --plan .../mission.plan --density 25               # forest flight: avoidance ON vs OFF
droneflight simfly   --plan .../mission.plan --drift-rate 0.02 --vtrn-interval 150
droneflight fly      --plan .../mission.plan --url udp://:14540         # LIVE — needs PX4 SITL (Ubuntu VM)
ruff check . && ruff format --check . && mypy --strict src && pytest -q # 49 tests, ~95% cov
```
`validate`/`nav`/`simfly` are pure kinematic/offline (run anywhere); `fly` needs
a live PX4.

## Key facts (some from user feedback — don't regress)
- **Sensor choice is a concealment tradeoff, not "lidar is best"** (user P.S.):
  active lidar *emits* → visible to night-vision devices/laser-warning receivers,
  so it's gated behind `allow_active_emission`; passive optical is covert but
  night-blind; passive thermal is covert-at-night but export-controlled. A covert
  night mission with only optics **refuses** rather than silently emitting.
- **Reversals allowed at low speed** — the speed policy slows through them.
- **The simulated error (`nav.py`) is complete:** VIO drift accumulates
  *coherently* (~2% → tens of m over km, per Tech-Selection §2.1); VTRN fixes
  shrink it toward a bounded residual. Wired into `reactive.py` (drift + vtrn args).
- **VTRN is now data-driven, not magic (`correction.py`):** the old fix was an
  *idealised reset* (assumes a fix is available, accepted, and correct
  everywhere). `correction.py` makes basemap fidelity a spatially varying input
  (matchability + staleness + accuracy) and gates fixes on confidence + an
  innovation test. Two findings (see [NAV-FIDELITY.md](NAV-FIDELITY.md)): **forest
  starves fixes** (canopy = low texture — and forest is the drone's own cover, so
  it's most lost where most concealed), and a **stale conflict-zone basemap** can
  inject *confident-but-wrong* fixes — the innovation gate is the only defence,
  and it weakens as drift grows. `nav.py`'s idealised path is kept as the
  optimistic baseline for the A/B.

## The central finding (from the closed-loop sim)
- With **known position**, reactive avoidance reliably dodges unmapped obstacles:
  blind flight collides, reactive reaches the goal (verify: `simfly` shows
  OFF=COLLIDED wp1, ON=REACHED all).
- Under **GPS-denied drift**, avoidance senses *real* trees so it keeps dodging,
  but drift wanders the *true* path out of the planned tube into dense forest.
  **Safe forest flight needs BOTH reactive avoidance AND bounded navigation
  (VTRN)** — neither alone. `reactive.py`'s VTRN must *shrink* error, not teleport
  it (an earlier bug); avoidance must react at a *lookahead* distance, not the
  bare safety radius (another earlier bug) — both fixed.

## Caveats / what's NOT done
- `reactive.py` is a **2D horizontal kinematic** sim (no physics, no vertical
  avoidance) — a fast design tool, not fidelity. Real fidelity = PX4 SITL + Gazebo
  (RUNBOOK) on the Ubuntu VM.
- No real onboard state estimator (VIO/VTRN/LIO) — `nav.py` emulates its *error*,
  it is not an estimator. That + the Gazebo tether plugin are the next big items.
- `correction.py` is a **fidelity/error emulator, not a matcher** — no real
  orthophoto/DSM, no feature extraction; the matchability/staleness fields are
  synthetic. Real numbers need the §6.3 bench (replay nadir imagery over real
  reference maps). Its top mitigation — feed a basemap **matchability/currency**
  layer into the *planner* cost surface so routes don't dive into fix-deserts — is
  unbuilt and lives on the droneplanner side (see its SHORTFALLS cross-cutting note).
- `mavsdk_connector.py` live path is untested without an autopilot.
