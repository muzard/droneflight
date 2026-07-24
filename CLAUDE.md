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
| `speed_policy.py` | slow through turns/reversals (protect the fiber); consolidates duplicate-waypoint clusters so a turn is seen at its true angle |
| `tether.py` | **v0 payout-tension model** — back-tension + airborne-span drag + turn/reversal load; validates the speed policy against a break budget (`tether-check`); also **parses + scores the Gazebo plugin's log** (`tether-plugin-check`) |
| `perception.py` | forward depth sensing (rays → nearest-trunk range; stereo or lidar slice) |
| `avoidance.py` | follow-the-gap / VFH-lite reactive steering + slow-in-clutter |
| `reactive.py` | closed-loop **kinematic** forest-flight sim (avoidance ON vs OFF, drift, VTRN) |
| `nav.py` | **the simulated error** — VIO drift + VTRN correction (idealised reset *and* fidelity-limited optical driver), plus yaw drift and EV lock loss |
| `correction.py` | **optical terrain-relative correction** — data-driven fixes over a spatially varying basemap; confidence floor + innovation gate + Kalman fuse |
| `ev_bridge.py` | **external-vision injection** — Gazebo truth + injected error → PX4 `VISION_POSITION_ESTIMATE`; ENU→NED frames, EV dropout, track recording, corridor analysis |
| `executor.py` + `connector.py` | connect→upload→arm→start→monitor, RTL on error (+ progress-stall watchdog); fake connector for tests |
| `mavsdk_connector.py` | live PX4 adapter (⚠️ needs SITL; excluded from coverage, not unit-tested) |
| `sitl_launch.py` | inject the plan's home into PX4 SITL (`PX4_HOME_*`) so a mission lands at the real AO, not Zurich |
| `config.py`, `cli.py` | config + `validate` / `nav` / `tether-check` / `tether-plugin-check` / `simfly` / `sitl-env` / `px4-params` / `flatten-plan` / `ev-bridge` / `ev-report` / `fly` commands |
| `sitl/tether_plugin/` | **Gazebo C++ wrench plugin (v0)** — physics bit-exact vs `tether.py`; ✍️ written, **not yet compiled or flown** |

## Run / test
```bash
cd /Users/arttu/Claude/Projects/DroneAutomation/droneflight
. /Users/arttu/Claude/Projects/DroneAutomation/.venv/bin/activate
droneflight validate --plan ../droneplanner/out/mission.plan            # offline: policies + sensor mode
droneflight nav      --plan .../mission.plan                            # VIO vs idealised vs optical VTRN
droneflight nav      --plan .../mission.plan --forest-frac 0.7 --stale-frac 0.3  # conflict-zone basemap
droneflight tether-check --plan .../mission.plan                        # payout tension: speed policy vs flat cruise
droneflight tether-plugin-check --log tether-on.csv                     # Gazebo plugin log vs the model (exact-parity gate)
droneflight simfly   --plan .../mission.plan --density 25               # forest flight: avoidance ON vs OFF
droneflight simfly   --plan .../mission.plan --drift-rate 0.02 --vtrn-interval 150
eval "$(droneflight sitl-env --plan .../mission.plan)"                  # PX4 SITL home = plan home
droneflight flatten-plan --plan .../mission.plan --out ev_test.plan      # constant alt for PX4's FLAT world
droneflight ev-report --plan .../mission.plan --record run.jsonl        # corridor stats of a recorded EV run
droneflight fly      --plan .../mission.plan --url udpin://0.0.0.0:14540         # LIVE — needs PX4 SITL (Ubuntu VM)
droneflight ev-bridge --plan .../mission.plan --url udpin://0.0.0.0:14541        # LIVE — needs Gazebo + PX4 (RUNBOOK §3b)
ruff check . && ruff format --check . && mypy --strict src && pytest -q # 185 tests, ~96% cov
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
  format). Injected position error is **horizontal only** — EV height is always
  truth. Yaw is truth *by default* but can be corrupted (`--yaw-bias` /
  `--yaw-drift`); flown 2026-07-22 and it barely moves cross-track — see Slice 3.
- `correction.py` is a **fidelity/error emulator, not a matcher** — no real
  orthophoto/DSM, no feature extraction; the matchability/staleness fields are
  synthetic. Real numbers need the §6.3 bench (replay nadir imagery over real
  reference maps). Its top mitigation — feed a basemap **matchability/currency**
  layer into the *planner* cost surface so routes don't dive into fix-deserts — is
  unbuilt and lives on the droneplanner side (see its SHORTFALLS cross-cutting note).
- `mavsdk_connector.py` live path is untested without an autopilot.

## ⚠️ Every SITL result so far is a FLAT-WORLD "happy path" (read this before quoting any of them)
All of it — Slice 1, the three-arm acceptance, dropout/yaw, mag-denied
`dropout-long`, the failsafe-tolerance run — flew **`plans/ev_test.plan`: a
constant-30 m plan over PX4's default *flat ground plane*** (`default.sdf`), with a
**synthetic** VTRN basemap (`correction.py`'s matchability/staleness are generated
fields, not real orthophoto/DSM). The world has **no terrain, no obstacles, no
features**. So what the campaign has actually validated is the **plumbing and the
error models' effects** — EV injection, EKF2 fusion, ENU↔NED frames, the global-
origin handshake, the progress watchdog, failsafe behaviour, and how injected
drift/dropout/yaw move the estimate — *not* flight through a real environment.

What remains **completely unflown** because the world is benign:
- **Terrain-following** and the altitude-frame CFIT fix — regression-tested
  offline only; the Gazebo terrain world (Slice 2) is parked (tumbles on
  translation), so no relief has ever been flown.
- **Obstacle avoidance in the loop** — `reactive.py` is a *kinematic* dodge; PX4
  SITL has had no trees to avoid. Perception + avoidance vs real geometry is
  untouched (and real perception needs the RTX box, Track B).
- **VTRN over real features** — texture/forest starvation is *modelled*, never
  matched against real imagery; the §6.3 bench is unbuilt.
- **The tether speed policy over real turns** — the flown route is too smooth
  (≤16° bends), so the policy has never engaged in flight (see `tether.py`).
- **Concealment/viewshed** — no line-of-sight or cover exists in an empty world.

**Bottom line: "it flies GPS-denied" means "the injection pipeline flies a smooth
route over flat ground." The hard parts — terrain, obstacles, features — are the
next campaign, and they need a non-degenerate world (unblock Slice 2's terrain
tumble, add obstacle models, feed a feature-rich basemap). Treat every corridor
number to date as a floor that real terrain can only make worse.**

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
  **A second instance of this same class surfaced 2026-07-22** — the
  *position-loss* failsafe descends at land speed during a vision dropout (Slice 3,
  `dropout-long`). Both are autopilot defaults that abandon the NOE profile; the
  program needs a deliberate failsafe policy, not just these patches. The dropout
  case has a prepared fix (`configs/px4/dropout_tolerant.params`, RUNBOOK §3f):
  raise the no-aiding timeout so a brief gap is coasted through, not failsafed —
  because without a position estimate the vehicle cannot hold, only come down.
- 🔧 **Slice 2 — terrain world: root-caused and FIXED in code 2026-07-23, awaiting
  VM verification.** `droneplanner gz-world` injects the planned DTM into PX4's
  `default.sdf` (RUNBOOK §2b). It loaded and flew but **tumbled once it
  translated**.

  **Cause: the vehicle spawned BURIED.** Gazebo uses a DEM's **raw elevation
  values**, not a rescale into `<size>`'s z — ODE's `AABB max z = 150.32` was the
  demo DEM's max *elevation*, not the `110.32` relief we passed. With the old
  offset `-(home_elev − min_elev)` the terrain surface under home landed at world
  `z = min_elev` instead of 0:

  | world | home | DEM min | surface at home | vehicle |
  |---|---|---|---|---|
  | real DEM | 124.68 | 40.00 | **+40.0 m** | buried 40 m |
  | `--flat` diagnostic | 124.68 | 124.68 | **+124.7 m** | buried 125 m |

  PX4 spawns at world z = 0, so the vehicle sat inside the heightmap in permanent
  collision penetration — which reads as a *stable hover* and then flings it the
  instant it translates. ⚠️ **This also invalidates the old `--flat` bisection**
  ("a flat DEM tumbles too → not slope/normals → it's the heightmap itself"): the
  flat world was buried *deeper*, so that run measured the same bug twice, not a
  second one.

  **Fix (`droneplanner/io/gzworld.py`): hand Gazebo a zero-based DEM.**
  `normalise_dem()` subtracts the minimum elevation, so the written DEM spans
  `0 .. relief` and the raw-vs-rescaled interpretations become numerically
  identical — the datum no longer depends on a Gazebo behaviour we cannot pin
  down from the Mac. `gz-world` now emits `<out>_dem.tif` next to the world and
  points the SDF at it; `size.z` stays the relief; the pose offset is unchanged.
  Also floors a degenerate `relief = 0` (a truly flat DEM gave `<size … 0>`).
  Guarded by `surface_z_at_home()` == 0.0 across four home/DEM combinations.

  ✅ **Datum CONFIRMED on the VM (2026-07-24) — the vehicle now rests on the
  terrain.** The proof is the *attitude*, not the height: a vehicle resting on a
  heightmap takes the terrain's local slope, and the demo AO's DEM gradient at
  home (dz/dy = 0.0197 → **1.13°**) matches the measured vehicle roll
  (−0.019224 rad → **1.10°**) to 2 %. It settled at z = −0.007 m (≈7 mm of ODE
  contact penetration).
  ⚠️ **The z value is NOT a discriminator** — PX4 spawns at the world origin, so
  `z ≈ 0` either way, resting or buried. An earlier note here predicted "z ≈ +0.2"
  from a guess about the x500's model frame; that was wrong (its origin is at
  ground level, not the body). Score this on roll vs DEM gradient.

  ✅ **SLICE 2 IS DONE — the terrain world flies (2026-07-24,
  `sim-logs/terrain/17_12_10.ulg`).** World confirmed loaded from the console
  (`Starting gazebo with world: …/planner_terrain.sdf`, no URI errors). The
  vehicle flew **51/77 waypoints (~2/3 of a 5 km route) over real relief** with
  altitude tracking terrain from +29 m to −42 m — where it previously **tumbled
  on the first translation**. The tumble is gone; the buried-spawn diagnosis was
  right.

  🔴 **…and on its very first real-terrain flight it found a planner CFIT defect
  that flat-world SITL could never have found.** The run ended at 51/77: the
  aircraft flew into the ground at t=482 s and the watchdog aborted 565 s later.
  Not a tumble, not the world — a **descent-profile mismatch**. The plan is
  correct at every vertex *and* along every leg under linear altitude
  interpolation (min 22.4 m AGL), but PX4 descends toward the next waypoint's
  altitude **at its own vertical rate**, front-loading it:

  | leg wp50→51, 765 m, −40.5 m | at 17 % along the leg |
  |---|---|
  | linear interp (planner's assumption) | +14.4 m → **27.0 m AGL** |
  | PX4 actual | −11.9 m → **0.8 m AGL** → impact |

  Full analysis, the worst-case re-scoring (3 legs underground) and fix options
  are in **`droneplanner/SHORTFALLS.md` item 0** — it is a planner defect, not a
  flight-stack one. This is the clearest possible vindication of the flat-world
  caveat: the first flight over relief broke the aircraft in a way 8 flat
  campaigns could not.

- ✅ **Slice 3 — external-vision injection bridge: FLOWN, acceptance complete
  (2026-07-22).** `ev_bridge.py` + `droneflight ev-bridge` / `ev-report`, tested
  offline (172 tests, ~96 % cov) **and now exercised in flight** — the three-arm
  GPS-denied acceptance ran on the VM and the arms separated (table below).
  **The MVP flies GPS-denied.** Procedure: RUNBOOK §3b, arms: §3c.
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

  - 🐛 **Watchdog false-positive, found and fixed (2026-07-22).** The first
    GPS-denied flight aborted at **41/77**: the progress watchdog measures time
    between *waypoint arrivals*, and leg 41→42 is **1130.8 m = 188 s** at cruise
    against a fixed `progress_timeout_s=60`. ULog confirms RTL at arrival+60 s
    exactly. `progress_timeout_s` is now **None = derive from the plan's longest
    leg** (`executor.auto_progress_timeout_s`, 3× margin → 565 s for this route);
    an explicit value still overrides. Any fixed constant here is wrong — the safe
    value is a property of the route.

  - 📊 **First GPS-denied flight, 2026-07-22 (`sim-logs/13_17_18.ulg`,
    `ev-vtrn.jsonl`).** Armed on injected vision, flew **41/77 waypoints (~4 km)**
    before the watchdog aborted. Measured, VTRN arm:

    | Quantity | Value |
    |---|---|
    | EV publish rate | 30.0 Hz, `cs_ev_pos` on continuously 62→475 s |
    | EKF2 lock (est − published) | mean **0.42 m**, max 10.1 m |
    | Injected error | max **9.0 m**, mean 3.6 m (22/27 fixes accepted) |
    | **True track vs plan, MISSION phase** | max **10.4 m**, mean 3.4 m — **0/10139 samples outside the 15 m corridor** |

    So the VTRN arm **holds the corridor in real flight** — but this run predates
    the degenerate-basemap and fix-teleport fixes below, so it is superseded by
    the three-arm acceptance. Kept for the watchdog/EKF2-lock numbers only.
  - ⚠️ **Score the MISSION phase only.** The whole-run number for the same flight
    is max 122.6 m — all of it the post-abort RTL, which beelines home across a
    tortuous route. `ev-bridge` now records PX4's flight mode per sample and
    `ev-report` segments on it (older records get a warning); endpoint deviation
    is flagged as meaningless when the route was not completed.

  - ✅ **77/77 COMPLETED GPS-denied (2026-07-22, `sim-logs/13_40_08.ulg`).**
    AUTO_MISSION 25→937 s, `finished=True`, EV 30.0 Hz, EKF2 lock (est−published)
    mean 0.42 m. True track vs plan: **max 11.5 m, mean 5.3 m — within the 15 m
    corridor.** The MVP flies.
  - 🔴 **…but that run's basemap was degenerate — the result does NOT validate
    VTRN.** `route_reference()` builds its polyline in `_to_local` absolute metres
    (~5.2e6); the bridge samples at Gazebo **home-relative ENU** (~0). Every query
    landed **5344 km** off-route, projected onto the same endpoint, and collapsed
    the spatially varying field to **one bucket** — which happened to be open
    terrain. So the flight ran against "perfect map everywhere", exactly the
    optimistic assumption `correction.py` exists to remove. Fixed:
    `route_reference(..., home_relative=True)` for the bridge, and
    `PatchyReference.sample()` now **raises** on an off-route query instead of
    silently degenerating (it immediately caught four tests making the same
    mistake). **The VTRN arm must be re-flown.**
  - 🔴 **Fixes were teleported into the published pose.** `correction.py` shrinks
    error rather than jumping it — but the bridge applied the result to the
    *published* pose in one sample (up to **10.1 m**, median 3.5 m). EKF2's
    outlier gate rejected the step: 13 momentary losses of EV fusion, each
    ~1.1 s after an accepted fix (7/13 vs 1.3 expected by chance), `ev_hpos` test
    ratios to 311, 4.6 % of mission samples rejected. Fixed with
    `ErrorInjector(correction_ramp_m=...)` (`ev-bridge --fix-ramp`, default 10 m):
    7.8 m → 1.5 m max step with identical corrector decisions.
  - 📉 **Monte Carlo, 40 seeds, corrected frame — the flight was the lucky case:**

    | Scenario | median | p90 | worst | outside 15 m | fix avail | blunders |
    |---|---|---|---|---|---|---|
    | VTRN, 40 % forest / 10 % stale | 10.6 m | 15.6 m | 17.0 m | **4/40** | 48 % | 0 |
    | VTRN, 70 % forest / 30 % stale | 17.4 m | 55.0 m | 114.0 m | **25/40** | 20 % | 7 |
    | no VTRN (control) | 102.1 m | 106.4 m | 106.8 m | 40/40 | — | — |

    So VTRN is **not** a reliable corridor guarantee: it fails 1 seed in 10 on a
    benign map and most of the time in a conflict zone. One flight is one seed.

  - ✅ **ACCEPTANCE FLOWN — all three arms, 2026-07-22. The arms separate.**
    `sitl/run_ev_arm.sh` on the VM, one plan (`plans/ev_test.plan`, 77 wp,
    5.13 km, flat 30 m), `--seed 0 --fidelity-seed 1` pinned. All three
    **completed** (`finished=True`, 153/154 items, ~5.16 km flown) — no watchdog
    aborts, so `auto_progress_timeout_s` held. MISSION phase, 15 m corridor:

    | Arm | ULog | max dev | mean | endpoint | fixes | blund | predicted | verdict |
    |---|---|---|---|---|---|---|---|---|
    | A `vtrn` 40/10 | `sim-logs/14_53_15.ulg` | **11.8 m** | 4.0 m | 3.2 m | 28/62 (45 %) | 0 | 15.0 m | **WITHIN** |
    | B `novtrn` control | `sim-logs/15_16_47.ulg` | **103.2 m** | 44.1 m | 103.2 m | — | — | 106.4 m | LEFT |
    | C `conflict` 70/30 | `sim-logs/15_33_55.ulg` | **25.6 m** | 9.3 m | 8.8 m | 10/68 (15 %) | 0 | 21.9 m | LEFT |

    Records `sim-logs/ev-{novtrn,conflict}.jsonl` — but ⚠️ **`ev-vtrn.jsonl` (arm A)
    was later overwritten by arm G and is gone**; score arm A from its ULog
    (`14_53_15.ulg`) via `scratchpad/ulg_arms.py`, which reproduces it to 0.1 m
    (11.7 vs 11.8). `ev-report` for B and C still works:
    `droneflight ev-report --plan plans/ev_test.plan
    --record sim-logs/ev-novtrn.jsonl --record sim-logs/ev-conflict.jsonl`.

  - 📐 **The offline Monte Carlo predicted the flights** (15.0/11.8, 106.4/103.2,
    21.9/25.6 — all within ~4 m). So `nav.py` + `correction.py` are *predictive of
    in-the-loop SITL*, not just self-consistent, which is what lets the 40-seed
    study stand in for flights we cannot afford to run.
  - 🔬 **Arm C failed by fix STARVATION, not by poisoning — the mechanism matters.**
    Decline breakdown: **52/68 (76.5 %) `no_features`**, 3 `gated_out`,
    3 `low_confidence`, **0 blunders accepted**. The innovation gate only had to
    fire 3 times; the corridor breach came from canopy leaving nothing to match
    against. So on this seed the `--forest-frac 0.7` knob did all the damage and
    `--stale-frac 0.3` did almost none. This is the in-the-loop confirmation of
    NAV-FIDELITY's "forest starves fixes" finding. **Next experiment should
    separate the knobs** (70/10 vs 40/30) — they are currently varied together and
    only one of them is demonstrably dangerous.
  - ⚠️ **Seeds verified pinned, on evidence.** All three arms trace an identical
    injected-error vector to 50 m — (−0.831, +0.53) m, bearing 147°, matching to
    3 decimals — so the confound this experiment was designed to avoid is ruled
    out by measurement, not by trusting the script.
  - ⚠️ **Arm A is still one seed, and a lucky one.** 11.8 m nearly reproduces the
    *invalidated* degenerate-basemap run (11.5 m), which is coincidence, not
    vindication: the MC says 4/40 seeds breach at 40/10. **Do not quote arm A as
    "VTRN holds the corridor."** It says: this route, this seed, this basemap held.
  - 🐛 **Residual, unfixed: the fix-ramp did not eliminate EKF2 gate trips.**
    Max published-pose step is down to **0.45 m** (A) / **0.75 m** (C) from 7.8 m,
    but EV fusion is still rejected in **20 (A) / 22 (C) episodes, ~1.9 % of
    samples** — against **1 episode / 2 samples** in arm B, which applies no fixes
    at all. So the trips are still fix-induced and scale with correction size.
    Not currently harmful (arm A held with 3 m margin); the ramp is an improvement,
    not a cure.
  - 🐛 **Residual, unfixed: an early fix can be worse than no fix.** Arm A's first
    accepted fix drove error **0.99 m → 3.28 m** at ~100 m while the control sat at
    1.98 m — the corrector fuses fixes whose own uncertainty exceeds current drift.
    Net strongly positive over 5 km, but gating on *"does this fix beat my current
    drift estimate?"* looks like cheap headroom.
  - 📌 **Two traps when reading these ULogs.** `estimator_aid_src_ev_pos.test_ratio`
    maxes at ~9.4e12 in **all three arms including the no-fix control** — a garbage
    first sample, not an innovation; quote p99 (4.16 / 0.01 / 5.94). And all three
    published at exactly **25.0 Hz**, not the 30 Hz `--rate` default or the 30.0 Hz
    in the earlier table — dt is exactly 0.0400 s, so it is quantised somewhere
    (Gazebo's odometry rate?), not VM starvation. Consistent across arms, so it
    confounds nothing, but it is **unexplained**.
  - 🛠 **ULogs can be scored without the bridge's JSONL.** PX4 logs
    `vehicle_local_position_groundtruth` (= Gazebo truth), so feeding it through
    `plan_track_enu`/`analyse_track` reproduces `ev-report` to **0.1 m** (arm A:
    11.7 m from ULog vs 11.8 m from JSONL). Useful when a record is lost — but the
    ULog cannot show fix availability or blunders, which live only in the JSONL.

- ✅ **EV dropout + yaw-error injection: BUILT AND FLOWN (2026-07-22).**
  The two axes the acceptance never touched. Both **default to off**, so the
  three flown arms reproduce from the same command line — there is a test that
  asserts exactly that.
  - `nav.DropoutModel` (`--dropout-interval <m> --dropout-duration <s>`) — lock
    loss as **silence**, not as a degraded pose: the bridge withholds the publish
    entirely, because that is what a VIO that has lost tracking does. Onset is a
    Poisson process in **distance** (reproducible across publish rates), duration
    in **seconds** (re-acquisition is an estimator property). The sample is still
    *recorded* with `ev:"down"` — dropping the record too would make a dropout
    indistinguishable from the bridge dying. Drift keeps accruing while down.
  - `nav.YawWalker` (`--yaw-bias <deg> --yaw-drift <deg/100 m>`) — heading error
    added in the NED/FRD frame and wrapped; roll and pitch stay truth, since
    gravity observes them. Its **own RNG**, so enabling it cannot shift the
    position-drift draws (asserted bit-for-bit).
  - Sized against the real flown truth track (open loop — this picks parameters,
    it does **not** predict corridor deviation): `--dropout-interval 800` → 4
    losses / 99.2 % uptime over 5.13 km; `500` → 6 / 98.2 %; `250 --dropout-duration 5`
    → 15 / 92.7 %. `--yaw-drift 1.0` → 7.7° max heading error, `2.0` → 15.4°.
  - ✅ **BOTH FLOWN 2026-07-22 — and the headline is that the corridor did not
    move.** `dropout` (`sim-logs/19_30_04.ulg`, `ev-dropout.jsonl`) and `yaw`
    (`19_51_48.ulg`, `ev-yaw.jsonl`), each one axis on top of the `vtrn` basemap,
    seeds pinned, scored against the flown `vtrn` arm.

    | Arm | max dev | mean | EV uptime | injected yaw | verdict |
    |---|---|---|---|---|---|
    | `vtrn` (control) | 11.8 m | 4.0 m | 100 % | — | WITHIN |
    | `dropout` 500 m / 3 s | **12.2 m** | 4.1 m | 98.5 % | — | WITHIN |
    | `yaw` 1.0 °/100 m | **11.9 m** | 4.0 m | 100 % | 6.3° | WITHIN |

  - ✅ **Both axes verifiably engaged — this is a real null, not a broken run.**
    PX4 itself saw **exactly 6 EV gaps of 3.0–3.1 s** (`vehicle_visual_odometry`
    holes, independent of what the bridge claims), 543 publishes withheld. And
    EKF2's heading **tracked the injected yaw**: correlation **+0.970**, range
    ratio 0.90×, against a control heading-noise floor of 2.07°.
  - 🔴 **The corridor metric is near-blind to heading error — and that is a
    finding about the acceptance criterion, not about these arms.** A 6.1°
    injected heading error moved max deviation by 0.1 m; a 12.4° dropout-induced
    one moved it by 0.4 m. A multirotor flying **position setpoints** does not
    steer with its nose: the position controller closes on EV position, which
    neither axis perturbs. **CLAUDE.md's previous claim that yaw "couples into
    position through the controller" is measured FALSE** at these magnitudes for
    this vehicle in AUTO_MISSION. "WITHIN corridor" ≠ "navigation healthy".
  - 🔬 **Dropout's real cost is heading, not position.** MISSION phase, in the
    gap windows (+3 s recovery) vs outside them: heading error **12.40° vs 2.07°
    max** (mean 1.03° vs 0.15°) — 6× — while EKF2 position error is **11.71 m vs
    11.05 m**, i.e. unchanged. `EKF2_EV_CTRL=11` sources position *and yaw* from
    vision, so a lock loss removes the only heading observation while 3 s of
    inertial dead reckoning barely dents position. **The dropout arm was a bigger
    yaw disturbance (12.4°) than the deliberate yaw arm (6.1°).**
  - ⚠️ **I read that backwards — the magnetometer was the *cause* of the 12.4°,
    not the cure. Corrected by the mag-denied run below.** During gaps EKF2 falls
    back to 3D mag fusion (`cs_mag_3d` 0.0 % → 39.4 %), and I wrote that 12.4° was
    therefore a *best case* that mag denial would worsen. The flown `dropout-long`
    arm — **longer** 6 s gaps, mag **denied** — instead held heading to **1.88°**.
    Doubling the gap while removing the mag made heading 6× *better*, so the 12.4°
    excursion was the EV↔mag handoff, not the loss of EV heading. See the
    dropout-long entry for the measured reversal and its caveats.
  - 🎯 **Where heading error actually hurts this system is camera pointing, which
    nothing models.** The nadir VTRN camera and the forward avoidance FOV are both
    body-fixed, so 12° of heading error swings both. `correction.py` has no
    pointing-error term (matchability is purely spatial) and `perception.py`'s FOV
    is already flagged as too narrow. That coupling — not cross-track — is the
    reason to care about yaw here.
  - 🧰 **`ev-report`'s comparison table now carries EV-uptime and yaw columns**
    plus a note that a heading-only arm can be significant with a flat corridor.
    It previously showed them only in single-record mode, so these two arms first
    read as an ambiguous null — an arm whose axis never fired was indistinguishable
    from one that fired and changed nothing.
  - ✅ **`dropout-long` (6 s) + magnetometer DENIED FLOWN 2026-07-22
    (`sim-logs/21_16_45.ulg`, `ev-dropout-long.jsonl`, mag-denied).** Configuration
    verified from the ULog: **6 gaps of exactly 6.0–6.1 s**, `cs_mag_3d` and
    `cs_mag_hdg` **0.0 % everywhere** (mag genuinely off). Three findings, in order
    of importance:
    - 🟢 **The position-loss failsafe fired at every gap — and the aircraft rode
      through it.** Each 6 s gap crossed `EKF2_NOAID_TOUT`; `xy_valid` went false
      (~10 % of samples), PX4 entered **DESCEND** for ~2.7 s, then **recovered to
      AUTO_MISSION the instant vision returned**. Mission **COMPLETED 77/77**. So
      the failsafe is transient-tolerant: a dropout shorter than the descent-to-
      ground is survivable, not an abort.
    - 🟠 **This is the first arm to move the corridor: max dev 19.6 m, LEFT**
      (vs 12.2 m for the 3 s dropout that stayed WITHIN). During each invalid
      window PX4 stops holding cross-track and the true track drifts out — so
      cross-track responds to *position-validity loss*, not to the vision
      degradation the 3 s and yaw arms applied. Consistent with "the metric is
      near-blind to everything except an actual position-hold break".
    - 🔴 **…but the failsafe action is DESCEND at `MPC_LAND_SPEED` (0.7 m/s), which
      is the wrong direction for this aircraft.** Each event gave up **~2.2 m of
      altitude (~13 m cumulative over 6)** — toward the canopy/ground for a
      nap-of-the-earth fiber-layer. This is the **same class of program-level
      failsafe defect as the Slice-1 RTL-to-61 m finding**: an autopilot default
      that discards the concealment/CFIT profile. The action needs to be a
      deliberate choice (hold heading + altitude and coast on IMU, not sink).
      Flagged, **not** silently changed — it is a design decision, like `NAV_DLL_ACT`.
    - 🔵 **The heading reversal — now CONFIRMED single-variable by arms F and G
      (flown 2026-07-22).** MISSION-phase in-gap heading error, holding the 6 s
      gap fixed and toggling only the magnetometer:

      | Run | ULog | dropout | mag | in-gap heading |
      |---|---|---|---|---|
      | dropout (3 s) | `19_30_04` | 3 s | ON | 12.40° |
      | dropout-long | `21_16_45` | 6 s | **DENIED** | **1.88°** |
      | **F** dropout-long | `22_32_36` | 6 s | **ON** | **11.77°** |

      F vs `21_16_45` is the clean test — identical 6 s gaps, mag the only
      difference — and the mag-on run is **6× worse** (11.77° vs 1.88°). So the
      excursion is the **EV↔mag handoff**, not the loss of EV heading: when EV
      drops, EKF2 transiently fuses a magnetometer whose inferred heading
      disagrees with the EV frame. With the mag denied it coasts on the gyro and
      barely moves. **This overturns last session's "the mag is the anchor / 12.4°
      is a best case" claim outright.**
      - **Arm G (`vtrn`, mag denied, `22_54_38`) supplies the null baseline:**
        no dropout, heading **2.19°** — indistinguishable from arm A's mag-on
        ~2.3°. So the magnetometer is *harmless when EV is continuous* and only
        hurts at a dropout. That is the whole shape of the finding: mag denial
        costs nothing except where it removes the bad handoff, where it helps.
      - ⚠️ **Residual caveat:** SITL gyro is low-drift, so the 1.88° coast is a
        gyro best-case; real gyro bias over a 6 s coast could erode it. The mag
        attribution, though, is now solid.
    - 📐 **Corridor confirms the failsafe (not heading) owns cross-track.** F
      (mag-on) left the corridor at **20.3 m**, right beside the mag-denied
      dropout-long's 19.6 m — mag state moved cross-track by <1 m. Both 6 s arms
      leave (~20 m) and both shorter/no-dropout arms stay (≤12.2 m), so the
      corridor responds to the **position-loss failsafe**, cleanly independent of
      the heading axis. The two effects are fully separated: **heading ↔ mag,
      corridor ↔ failsafe.**
    - ✅ **Arming was NOT the first failure (predicted it would be).** With EV yaw
      as the sole heading source I expected yaw-alignment to block arming; it did
      not — `cs_yaw_align` reached 100 % and the vehicle armed and flew. The two
      tiny logs in the folder (`21_16_24.ulg` 9 s, `21_42_23.ulg` 30 s, both
      never-armed) are SITL boot restarts bracketing the real run, not failed
      arming attempts.
    - ⚠️ **Log provenance — the F/G session scrambled two filenames (verified by
      ULog params, not trusted by name):**
      - **F = `22_32_36.ulg`** (mag ON, 6×6 s gaps, 6 DESCENDs, 77/77). Its
        *bridge record is gone* — `ev-dropout-long.jsonl` is a 15 s / 452-sample
        stub from a superseded 22:55 bridge start. **Score F from the ULog only.**
      - **G = `22_54_38.ulg`** (`MAG_TYPE=5`, mag 0 %, no injected dropout,
        11.7 m WITHIN). The 57.6 s EV gap in it is the bridge-restart outage and
        is **pre-mission** (@45 s, mission starts 120 s), so it does not touch the
        score.
      - 🔴 **`ev-vtrn.jsonl` was OVERWRITTEN by arm G** (now 31 651 samples /
        1055 s / t0=103, was arm A's 30 923 / 1031 s / t0=348). The arm-A
        acceptance *record* is gone locally — recoverable only as `14_53_15.ulg`.
        **Any future `ev-report --record ev-vtrn.jsonl` now silently scores G as
        arm A.** Rename the local copy (e.g. `ev-G-vtrn-magdenied.jsonl`) before
        re-running the acceptance table.
    - ✅ **Failsafe-tolerance experiment FLOWN 2026-07-23 — it worked**
      (`sim-logs/failsafe/15_02_03.ulg`, `EKF2_NOAID_TOUT=8 s` via
      `configs/px4/dropout_tolerant.params`). Same 6×6 s gaps, but **0 DESCEND
      episodes** (was 6), **`xy_valid` never false** (was ~10 %), altitude flat,
      77/77. Raising the no-aiding timeout above the gap let EKF2 coast on IMU
      without declaring position invalid, so the failsafe never armed. **The fix
      is the TIMEOUT, not the action** — a multirotor with no position estimate
      cannot hold XY, so "descend vs hold" was a false choice; the lever is to
      stop treating a *brief* gap as position loss.
      - ⚠️ **Measured tradeoff:** coasting the full 6 s grew EKF2 error to
        **22.8 m in-gap** and the corridor to **24.5 m LEFT** — *worse* than the
        descent baseline's 20.3 m. So it removes the CFIT/altitude hazard (the
        point) but does **not** hold the corridor: a 6 s dropout exceeds bounded
        nav's 15 m whichever failsafe you pick. Not descending toward the canopy
        is still the right call for a NOE layer, and the excursion recovers when
        EV returns. Set the timeout to just cover the transient (8 s), no more —
        it also delays the failsafe on a *genuine* permanent loss.
  - 🔑 **The dropout pair is designed around EKF2's 5 s `EKF2_NOAID_TOUT`** (PX4
    default; *not* set in `gps_denied_ekf2.params`, so confirm with
    `param show EKF2_NOAID_TOUT` before flying). 3 s = under it → EKF2 should ride
    through the gap on IMU. 6 s = over it → horizontal position goes invalid and
    the **position-loss failsafe** fires — that failsafe is still live (only
    `NAV_DLL_ACT` and `SIM_BAT_DRAIN` are neutralised), and it is the genuinely
    untested path. For `dropout-long` the *failsafe behaviour is the result*, not
    the corridor number.
  - ⚠️ **The yaw arm cannot be judged from the console *or* the corridor.**
    `est-pub` stays near zero by construction — and, as flown, so does the
    corridor. Score it on **EKF2 heading error vs Gazebo truth** from the ULog
    (`vehicle_attitude` vs `vehicle_attitude_groundtruth`, MISSION phase only:
    the post-mission HOLD phase adds ~11° of parked-vehicle heading error and
    will make any whole-log number meaningless).

- 🐛 **Found while building it: `DriftWalker`'s heading wander is a no-op at
  bridge granularity.** Its per-step deviation scales with distance, but a random
  walk needs **√distance** (variance adds, not deviation). At the bridge's ~0.24 m
  steps the accumulated wander collapses to ~0, so injected drift is a **straight
  line, not the "biased random walk" the docstring claims**. The flown control arm
  proves it: exactly 1.98 / 3.99 / 5.99 / 8.00 m at 100/200/300/400 m on a constant
  146.5° bearing. Position *magnitude* is barely affected (39.9 m vs 40.0 m over
  2 km, since a frozen heading still accrues `rate × distance`) — it is the
  *shape* that is wrong, and a coherent straight-line offset is far easier for
  VTRN to correct than a wandering one. **Deliberately NOT fixed**: the three arms
  and the 40-seed MC were all measured with it, and silently correcting it would
  invalidate them. `YawWalker` uses the correct √ scaling; both behaviours are
  pinned by tests (`test_yaw_walk_is_step_size_invariant`,
  `test_drift_walker_heading_wander_is_step_size_dependent`). **Decide before the
  next MC run**, since it changes what "2 % drift" means.

- 🧹 **Adjacent fix from the logs:** the `est-pub` readback printed
  **5421242.25 m** on the first sample of all four SITL runs (EKF2 has no origin
  yet, so `position_velocity_ned` reads out in another frame). The RUNBOOK tells
  operators to watch that number, so it now prints `(no EKF2 origin yet)` instead.
  Also: `run_ev_arm.sh`'s arm/seed banner goes to the terminal, **not** into the
  tee'd log, so the arm logs never recorded which seed produced them — the bridge
  now prints seeds and both new axes into its own banner, which does get tee'd.

  Full procedure and pitfalls for re-running the arms: **RUNBOOK §3c**.

- ⚠️ **Workflow: the VM syncs by `git pull`** from `github.com/muzard/{droneplanner,droneflight}`.
  Both repos are cloned in the guest — droneflight is installed, droneplanner is
  clone-only (it just delivers `demo_ao/` DEMs + Gazebo worlds; don't pip-install
  its geospatial stack there). The `.plan` is gitignored and still needs `scp`.
  **A stale VM has caused three false "bugs"** — pull after every push, and if
  behaviour contradicts the code, check `git log --oneline -1` on both ends
  before debugging. See RUNBOOK §1.6.
- 🐛 **Known planner defect (unfixed upstream) — now with a measured teeth:** the
  kinematic fillet pass emits **duplicate waypoint clusters** (0.2–1.0 m apart) —
  77 waypoints, only ~25 distinct. Beyond wasting mission slots, it **silently
  defeated per-waypoint turn analysis**: on `ev_test.plan` a real turn spread
  across the cluster reads as **8° raw vs 16° after 2 m consolidation**, so the
  tether-safe speed policy (which keys off per-waypoint deflection) under-fired.
  `speed_policy.py` and `tether.py` now **consolidate to ≥2 m vertices first**; the
  planner still wants a minimum-spacing filter on emission.

- ✅ **Tether v0 model BUILT (`tether.py`, `tether-check`) — the offline half of
  the required tether build item.** Payout tension = back-tension + **airborne-span
  drag** + turn/reversal load; validates the speed policy against a break budget
  (a reversal at cruise = 2.11 N snaps a 2 N fiber, the policy's slowdown → 0.76 N
  survives). Three things worth knowing:
  - 🔧 **The spec's "drag ∝ paid-out length" is wrong for a LAYING drone** and I
    did not implement it literally. Taken at face value it put peak tension at the
    final waypoint (34 N over 5 km, straight leg) where the policy does nothing —
    the tell. The laid fiber is deposited, not towed, so only the **airborne span**
    (~flight altitude) drags. Documented in the module header; a refinement of the
    Tech-Selection §7 hand-wave.
  - 📊 **On the flown plan the tether is SLACK, not binding:** peak **0.79 N =
    40 % of a conservative 2 N budget**, at cruise, and the route is too smooth
    (max 16° bend) to need slowing. So on these concealment routes the speed
    policy is *inactive and unneeded* — the fiber constraint isn't what shapes
    them. The policy earns its keep only on reversal-heavy routes (proven in the
    unit test, not on this plan).
  - ⚠️ **Calibration is un-benched** (Tech-Selection §7). Coefficients are
    order-of-magnitude; trust the *comparisons* (policy on/off, leg vs reversal),
    not the absolute newtons. And a real fiber-break mechanism is **still
    unmodeled**: a snag that couples the **drone's** mass (kg) into the line
    dwarfs the fiber's own momentum and breaks it at any flight speed — that is a
    planning/avoidance problem, not a speed one (see [[tether-v0]]).

- ⏭ **Next build items (updated):**
  - **🌍 A non-flat world — the biggest gap.** Every SITL result is flat-world
    happy path (see the caveat section above). Unblock Slice 2's terrain tumble,
    add obstacle models, feed a feature-rich basemap — then re-fly the campaign
    over relief + obstacles + features. This is the real next campaign, not a
    tweak.
  - ✍️ **Gazebo C++ tether wrench plugin — WRITTEN 2026-07-24, NOT COMPILED, NOT
    FLOWN.** Sources in [`sitl/tether_plugin/`](sitl/tether_plugin/README.md);
    requirements/acceptance in [`sitl/TETHER_PLUGIN.md`](sitl/TETHER_PLUGIN.md);
    VM procedure in RUNBOOK §4. What is and isn't verified:
    - ✅ **The physics is bit-exact against `tether.py`, proven on the Mac.**
      `tether_math.hpp` is Gazebo-free and mirrors the Python's association
      order; `tether_probe.cc` evaluates it from stdin and
      `tests/test_tether_plugin.py` diffs 3 parameter sets × 9 speeds × 10
      deflections with **`==`, not a tolerance**. A test also asserts the two
      *default* coefficient sets are identical, so retuning one side fails the
      build. A ULP-tampering test proves the gate has teeth.
    - ✅ **The log format is testable off the VM too.** `tether_log.hpp` holds the
      CSV writer, so the C++ produces byte-real logs on the Mac that
      `tether.parse_plugin_log` / `check_plugin_parity` score. Acceptance §1 is
      therefore **runnable now**: `droneflight tether-plugin-check --log <csv>`,
      exit 1 on any divergence.
    - ⚠️ **`TetherPlugin.cc` has never seen a compiler** — written against the
      gz-sim8 API from docs, no `gz-sim8` headers on the M4. Expect the first VM
      build to surface signature errors. Everything behind that interface is
      tested; the glue is entity lookup + `AddWorldForce` and nothing else, by
      design (logic that migrates into it becomes untestable).
    - 🔧 **Corrected a requirement while building:** TETHER_PLUGIN.md said to put
      the `<plugin>` block in the **world** SDF. Wrong — `make px4_sitl gz_x500`
      spawns the vehicle into the world at runtime, so no world file contains an
      x500 to attach to, and a block placed there **loads nothing and reports
      nothing**. It goes in the x500 *model* SDF;
      `sitl/tether_plugin/install_tether_plugin.sh` patches it idempotently
      (`--remove` reverts to byte-identical).
    - ⚠️ **Expect acceptance §2 to be near-null, and don't read that as failure.**
      0.76 N on a ~2 kg x500 is ~0.04 g / ~2.2° of tilt, which PX4 trims out.
      Parity (§1) is what proves the plugin works; §2 measures whether the fiber
      *matters*. Sweep `airborne_span_m` (40 → 400) so the evidence is a scaling
      law, not one A/B at a magnitude the controller can hide.
    - ⛔ **Acceptance §3 (speed policy in the loop) is blocked on a route, not on
      the plugin** — `plans/ev_test.plan` bends ≤16°, so the policy never fires
      and `tether-check` already calls the tether slack there. Needs a turn-rich
      plan; same gap as the flat-world caveat.
    - 🧪 **A/B is one env var, not an SDF edit:** `DRONEFLIGHT_TETHER_ENABLED=0|1`
      and `DRONEFLIGHT_TETHER_CSV=<path>`. A disabled run still loads and still
      logs `enabled=0`, so a control arm is identifiable from its own artifacts —
      the lesson from the F/G filename scramble. **No `[tether]` line in the
      console = the plugin did not load; do not score that run.**
  - ✅ Failsafe-tolerance run — flown (RUNBOOK §3f).
  - Cheap offline wins: re-run the MC with forest-frac and stale-frac varied
    *separately* (the flown conflict arm showed only forest-frac doing damage),
    and settle the `DriftWalker` √distance question above.
