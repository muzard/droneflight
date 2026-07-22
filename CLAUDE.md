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
| `ev_bridge.py` | **external-vision injection** — Gazebo truth + injected error → PX4 `VISION_POSITION_ESTIMATE`; ENU→NED frames, track recording, corridor analysis |
| `executor.py` + `connector.py` | connect→upload→arm→start→monitor, RTL on error (+ progress-stall watchdog); fake connector for tests |
| `mavsdk_connector.py` | live PX4 adapter (⚠️ needs SITL; excluded from coverage, not unit-tested) |
| `sitl_launch.py` | inject the plan's home into PX4 SITL (`PX4_HOME_*`) so a mission lands at the real AO, not Zurich |
| `config.py`, `cli.py` | config + `validate` / `nav` / `simfly` / `sitl-env` / `ev-bridge` / `ev-report` / `fly` commands |

## Run / test
```bash
cd /Users/arttu/Claude/Projects/DroneAutomation/droneflight
. /Users/arttu/Claude/Projects/DroneAutomation/.venv/bin/activate
droneflight validate --plan ../droneplanner/out/mission.plan            # offline: policies + sensor mode
droneflight nav      --plan .../mission.plan                            # VIO vs idealised vs optical VTRN
droneflight nav      --plan .../mission.plan --forest-frac 0.7 --stale-frac 0.3  # conflict-zone basemap
droneflight simfly   --plan .../mission.plan --density 25               # forest flight: avoidance ON vs OFF
droneflight simfly   --plan .../mission.plan --drift-rate 0.02 --vtrn-interval 150
eval "$(droneflight sitl-env --plan .../mission.plan)"                  # PX4 SITL home = plan home
droneflight ev-report --plan .../mission.plan --record run.jsonl        # corridor stats of a recorded EV run
droneflight fly      --plan .../mission.plan --url udp://:14540         # LIVE — needs PX4 SITL (Ubuntu VM)
droneflight ev-bridge --plan .../mission.plan --url udp://:14541        # LIVE — needs Gazebo + PX4 (RUNBOOK §3b)
ruff check . && ruff format --check . && mypy --strict src && pytest -q # 116 tests, ~96% cov
```
`validate`/`nav`/`simfly`/`sitl-env`/`ev-report` are pure offline (run anywhere);
`fly` needs a live PX4 and `ev-bridge` needs Gazebo **and** PX4.

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
- No real onboard state estimator (VIO/VTRN/LIO) — `nav.py` emulates its *error*
  and `ev_bridge.py` injects that error into EKF2; neither is an estimator. Real
  VIO/VTRN needs the x86+RTX box (Track B).
- `ev_bridge.py`'s **live** paths are unexercised: the MAVSDK `mocap` publish, the
  `SET_GPS_GLOBAL_ORIGIN` send, and the real `gz topic` subprocess (a fake `gz`
  binary covers the parsing and process plumbing, not Gazebo's actual output
  format). Injected error is **horizontal only** — EV height and yaw are truth.
- `correction.py` is a **fidelity/error emulator, not a matcher** — no real
  orthophoto/DSM, no feature extraction; the matchability/staleness fields are
  synthetic. Real numbers need the §6.3 bench (replay nadir imagery over real
  reference maps). Its top mitigation — feed a basemap **matchability/currency**
  layer into the *planner* cost surface so routes don't dive into fix-deserts — is
  unbuilt and lives on the droneplanner side (see its SHORTFALLS cross-cutting note).
- `mavsdk_connector.py` live path is untested without an autopilot.

## SITL bring-up plan (in progress — phased, per user)
Goal: an MVP on the **Mac's Ubuntu VM** — PX4 SITL flies a planner `.plan`,
GPS-denied via *injected* external-vision error (not real perception). Real
VIO/VTRN/avoidance (Isaac Sim + RTX) is a **later phase** — the M4 can't run it.
- ✅ **Slice 1 prep (done, testable on the Mac):** progress-stall watchdog
  (hung SITL → RTL, not a hang); `sitl-env`/`sitl_launch.py` + `sitl/run_sitl.sh`
  inject the plan's home so a mission lands at the AO, not Zurich; RUNBOOK
  sequences the first flight **GPS-enabled first** (isolate the executor/MAVSDK
  path), then GPS-denied.
- ✅ **Slice 1 DONE (2026-07-21, first live flight):** PX4 v1.18-beta SITL +
  Gazebo on the Ubuntu VM flew a planner `.plan` end-to-end, GPS-enabled.
  `droneflight fly` reported **COMPLETED 38/38**; `mavsdk_connector.py` needed
  **no fixes** — altitude framing (`alt_agl_m` → MAVSDK `relative_altitude_m`),
  mission encoding, arming and RTL all worked first time. ~10.5 min flight,
  ~5.8 m/s average (plan cruise 6.0).
- ⚠️ **…but the route was degenerate — don't over-trust that result.** The plan
  came from `configs/mission.example.yaml`, which has **no terrain**, so the
  planner emitted its straight-line stub: tortuosity **1.0000**, max turn
  **0.001°**, **constant 30 m** altitude, report `analysis_not_performed`.
  Consequences: the tether-safe **speed policy never engaged** (it slows through
  turns; there were none), the **altitude band/climb limits were never
  exercised**, and the measured cross-track (max **0.48 m**, mean 0.08 m) is
  meaningless as corridor-holding — perfect GPS, no wind, straight line. **A
  non-degenerate `.plan` is a precondition for the next SITL results to mean
  anything** (droneplanner has a procedural demo AO with terrain/forest, but no
  CLI path to emit a `.plan` from it — that gap is the cheap fix).
- 🔎 **Failsafe finding (real, program-level):** post-mission, link loss + SITL
  battery tripped `RTL: start return at 61 m`. An RTL that **climbs to 61 m and
  beelines home** discards the nap-of-the-earth concealment profile *and* drags
  the fiber off the laydown route. This aircraft is designed to fly with no GCS,
  so `NAV_DLL_ACT` and RTL altitude must be deliberate design choices, not
  defaults. `configs/px4/sitl_bringup.params` neutralises them for testing.
- ⏸️ **Slice 2 PARKED (2026-07-21) — terrain world, not on the critical path.**
  `droneplanner gz-world` injects the planned DTM into PX4's `default.sdf`
  (RUNBOOK §2b). It **loads and flies**, but the vehicle **tumbles once it
  translates**. Parked deliberately: it only adds *in-the-loop* confirmation of
  terrain-following, and the prize it was chasing — the altitude-frame CFIT bug —
  is already fixed and regression-tested. **The EV bridge does not need it.**

  *Established by measurement (don't re-investigate — see RUNBOOK §2b table):*
  heightmap loads (`terrain` in `gz model --list`, 160×160 is fine); terrain
  placement correct near home; hover control sound (18 s climb at 0.2°); physics
  timing clean (4.00 ms median step, 0 gaps >50 ms); world config now inherited
  from PX4. Bisection: a **flat** DEM tumbles too (557 m) — so not slope/normals.
  Datum is **unverified** (the earlier "confirmed" claim was circular — PX4 zeroes
  local z at spawn); ODE reports `AABB max z = 150.32` (DEM max elevation) vs the
  110.32 relief passed as `<size>`, which would put the surface ~40 m high and is
  the most promising thread.

  *Resume point:* fly `planner_nocol` (visual-only terrain + ground_plane, built
  and pushed). Currently blocked on `ekf2 missing data` — first check whether the
  sim is stepping at all (`listener sensor_combined -n 1`, and
  `gz topic -e -t /world/planner_nocol/clock -n 3`).

- 🎯 **Slice 3 — external-vision injection bridge: BUILT, not yet flown.**
  `ev_bridge.py` + `droneflight ev-bridge` / `ev-report` are written, linted,
  typed and tested (116 tests, ~96% cov) — but **everything below the unit tests
  is unverified until a VM session runs it.** Procedure: RUNBOOK §3b.
  - **Truth is Gazebo, never PX4.** `GazeboTruthSource` shells out to
    `gz topic -e --json-output` (model odometry, falling back to the world pose
    broadcast). PX4's own position is read *only* as a diagnostic readback
    (`estimate_probe`) and is structurally unable to reach the error model — the
    circularity trap has a dedicated regression test.
  - **One error model, two callers.** `nav.py` gained `DriftWalker` +
    `ErrorInjector` (stateful, distance-driven); `simulate_navigation` and
    `simulate_corrected_navigation` were refactored onto them and verified
    **bit-identical** over 180 configs. So SITL injects the model the offline
    study measured, not a second copy of it.
  - **Frames** ENU/FLU → NED/FRD in one tested place (`enu_to_ned`,
    `enu_flu_to_ned_frd_euler`): yaw = π/2 − yaw_enu, pitch flips, roll does not.
  - **Injected error is horizontal only** — EV height and yaw are ground truth,
    so this bridge is optimistic about both. Deliberate: it keeps the experiment
    about horizontal drift and whether optical fixes bound it.
  - ⚠️ **Global origin — expect this to bite first.** With `EKF2_GPS_CTRL=0`,
    EKF2 never gets a global origin (it normally takes one from the GNSS fix it
    may no longer use), so `is_home_position_ok` stays false and a lat/lon
    mission is rejected *before it starts*. The bridge sends
    `SET_GPS_GLOBAL_ORIGIN` (plan home) via MAVSDK `mavlink_direct`. Not a sim
    hack — on the aircraft it is VTRN, not GPS, that ties the local frame to the
    world.
  - ⚠️ **Two MAVLink endpoints.** `fly` binds 14540, so the bridge needs its own
    (`mavlink start -x -u 14581 -r 4000000 -f -m onboard -o 14541`), and must be
    started **before** `fly` — EKF2 has no position until vision arrives.
  - ⚠️ **Message timestamping is unresolved** (`--timestamp none|epoch|sim`,
    default `none`). Whether PX4 wants 0, companion epoch, or sim time depends on
    whether MAVLink timesync has converged, which I could not settle from the
    Mac. The live `est-pub` readback answers it in ~30 s; RUNBOOK §3b tabulates
    the symptom.
  - `EKF2_EV_CTRL` changed **15 → 11**: `VISION_POSITION_ESTIMATE` has no
    velocity field, so bit 2 asked EKF2 to fuse something never sent. (Two stale
    comments in that params file were wrong too — `EKF2_HGT_REF` 3 = vision, not 1.)

  *Acceptance (unrun):* mission completes GPS-denied; the **true** (Gazebo) track
  stays within corridor tolerance with fixes enabled and demonstrably wanders out
  with `--no-vtrn` — i.e. reproduce `nav.py`'s offline A/B against real flight.
  Both arms print the corridor verdict; `ev-report` recomputes it from a
  `--record` JSONL. Only meaningful on a **non-degenerate plan**.

- ⚠️ **Workflow: the VM syncs by `git pull`** from `github.com/muzard/{droneplanner,droneflight}`.
  Both repos are cloned in the guest — droneflight is installed, droneplanner is
  clone-only (it just delivers `demo_ao/` DEMs + Gazebo worlds; don't pip-install
  its geospatial stack there). The `.plan` is gitignored and still needs `scp`.
  **A stale VM has caused three false "bugs"** — pull after every push, and if
  behaviour contradicts the code, check `git log --oneline -1` on both ends
  before debugging. See RUNBOOK §1.6.
- 🐛 **Known planner defect (unfixed):** the kinematic fillet pass emits
  **duplicate waypoint clusters** (0.2–1.0 m apart) — 77 waypoints, only ~25
  distinct. Wastes autopilot mission slots and makes logs noisy. Needs a
  minimum-spacing filter on the emitted waypoints.
- ⏭ **Next build items:** **fly the EV bridge on the VM** (RUNBOOK §3b — it is
  built but unflown, and that is the only thing that turns Slice 3 green), then
  the **v0 tether disturbance plugin** (`sitl/`) to validate the speed policy
  against payout physics.
