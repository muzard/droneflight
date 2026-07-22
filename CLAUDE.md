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
| `nav.py` | **the simulated error** — VIO drift + VTRN correction (idealised reset *and* fidelity-limited optical driver), plus yaw drift and EV lock loss |
| `correction.py` | **optical terrain-relative correction** — data-driven fixes over a spatially varying basemap; confidence floor + innovation gate + Kalman fuse |
| `ev_bridge.py` | **external-vision injection** — Gazebo truth + injected error → PX4 `VISION_POSITION_ESTIMATE`; ENU→NED frames, EV dropout, track recording, corridor analysis |
| `executor.py` + `connector.py` | connect→upload→arm→start→monitor, RTL on error (+ progress-stall watchdog); fake connector for tests |
| `mavsdk_connector.py` | live PX4 adapter (⚠️ needs SITL; excluded from coverage, not unit-tested) |
| `sitl_launch.py` | inject the plan's home into PX4 SITL (`PX4_HOME_*`) so a mission lands at the real AO, not Zurich |
| `config.py`, `cli.py` | config + `validate` / `nav` / `simfly` / `sitl-env` / `px4-params` / `flatten-plan` / `ev-bridge` / `ev-report` / `fly` commands |

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
droneflight flatten-plan --plan .../mission.plan --out ev_test.plan      # constant alt for PX4's FLAT world
droneflight ev-report --plan .../mission.plan --record run.jsonl        # corridor stats of a recorded EV run
droneflight fly      --plan .../mission.plan --url udpin://0.0.0.0:14540         # LIVE — needs PX4 SITL (Ubuntu VM)
droneflight ev-bridge --plan .../mission.plan --url udpin://0.0.0.0:14541        # LIVE — needs Gazebo + PX4 (RUNBOOK §3b)
ruff check . && ruff format --check . && mypy --strict src && pytest -q # 169 tests, ~96% cov
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
  truth. Yaw is truth *by default* but can now be corrupted (`--yaw-bias` /
  `--yaw-drift`); the yaw axis is built and unit-tested but **never flown**.
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

- ✅ **Slice 3 — external-vision injection bridge: FLOWN, acceptance complete
  (2026-07-22).** `ev_bridge.py` + `droneflight ev-bridge` / `ev-report`, tested
  offline (169 tests, ~96 % cov) **and now exercised in flight** — the three-arm
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

    Records `sim-logs/ev-{vtrn,novtrn,conflict}.jsonl`; reproduce with
    `droneflight ev-report --plan plans/ev_test.plan --record sim-logs/ev-vtrn.jsonl
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

- 🆕 **EV dropout + yaw-error injection: BUILT (2026-07-22), not yet flown.**
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
  - **⏭ Arms are wired and ready to fly — `run_ev_arm.sh dropout | yaw` (and the
    follow-on `dropout-long`). Procedure, predictions and what to watch:
    RUNBOOK §3d.** Each adds one axis on top of the `vtrn` basemap, so the flown
    `vtrn` arm (11.8 m) is their control and **no control re-flight is needed**.
    Run them one at a time.
  - 🔑 **The dropout pair is designed around EKF2's 5 s `EKF2_NOAID_TOUT`** (PX4
    default; *not* set in `gps_denied_ekf2.params`, so confirm with
    `param show EKF2_NOAID_TOUT` before flying). 3 s = under it → EKF2 should ride
    through the gap on IMU. 6 s = over it → horizontal position goes invalid and
    the **position-loss failsafe** fires — that failsafe is still live (only
    `NAV_DLL_ACT` and `SIM_BAT_DRAIN` are neutralised), and it is the genuinely
    untested path. For `dropout-long` the *failsafe behaviour is the result*, not
    the corridor number.
  - ⚠️ **The yaw arm cannot be judged from the console.** A heading error makes
    the vehicle fly a rotated frame while still believing it is on route, so
    `est-pub` stays near zero while the true track bows away. Only the corridor
    number catches it — a clean-looking run means nothing here.

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
- 🐛 **Known planner defect (unfixed):** the kinematic fillet pass emits
  **duplicate waypoint clusters** (0.2–1.0 m apart) — 77 waypoints, only ~25
  distinct. Wastes autopilot mission slots and makes logs noisy. Needs a
  minimum-spacing filter on the emitted waypoints.
- ⏭ **Next build items:** **fly the dropout and yaw arms on the VM** (both are
  built and unit-tested but unflown — the failsafe path and the yaw→position
  coupling can only be seen in the loop), then the **v0 tether disturbance
  plugin** (`sitl/`) to validate the speed policy against payout physics. Cheap
  analysis wins meanwhile: re-run the MC with forest-frac and stale-frac varied
  *separately* (the flown conflict arm showed only forest-frac doing damage), and
  settle the `DriftWalker` √distance question above.
