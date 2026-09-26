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
| `px4_profile.py` | **the measured PX4 corner prior** — bend → corner speed *and* lateral accel, calibrated from the acceptance §3 control arm. `tether.py` scores through it, because the "flat cruise" arm it used to assume is a profile no autopilot flies |
| `perception.py` | forward depth sensing (rays → nearest-trunk range; stereo or lidar slice) |
| `avoidance.py` | follow-the-gap / VFH-lite reactive steering + slow-in-clutter. 🔴 **FLOWN 2026-07-28 and it COLLIDES — 19 strikes at 50 stems/ha, 37 at 107.** `react_distance_m=6.0` gives 0.66 m of lateral offset against 1.10 m needed (0.60×) — geometrically cannot miss. The earlier "hovers at ≥100 stems/ha" limit was a *kinematic-harness artifact* and is wrong: in flight it blocks 0.17 % of steps and hits trees instead. See TEST-PLAN B2 |
| `offboard.py` + `mavsdk_offboard.py` | **corridor executor** — carrot guidance (projection onto the polyline, so a detour cannot leave it chasing a passed vertex), avoidance in the loop, altitude commanded by us not interpolated by PX4 (what A2 says a corridor needs). Logs progress every 5 s and aborts after 90 s without route progress — a silent loop got killed by hand once, mistaken for a hang. `droneflight fly-corridor` |
| `local_planner.py` | **the onboard local planner (2026-09-23)** — remembers every trunk the depth sensor returns (odometry frame), A* through the gaps with a clearance cost, speed from curvature/sight/gap/dynamic feasibility. Replaces reactive `avoid()`; **0 strikes GPS-denied in SITL**. See the section below |
| `local_offboard.py` | the live executor for it — truth-pose sensing, believed-pose planning, home-relative altitude with a datum check, per-leg speed from the `.plan`. `droneflight fly-local` |
| `maneuver.py` | **the reaction-distance budget** — how far ahead an obstacle must be seen for the airframe to actually miss it, from the flown speed and PX4's `MPC_ACC_HOR`/`MPC_JERK_AUTO` (jerk-limited, so *not* ½at²). `react_distance_m` is now **derived from this, not a constant**; also `required_rays`, the angular-sampling companion. Built after B2 flew the constant into 19–37 tree strikes |
| `trunk_manifest.py` | loads the trunk CSV the Gazebo world was built from, so perception senses what physics will hit |
| `strikes.py` | **did the flight hit anything?** Scores a flown track against its trunk manifest by *dynamic signature* (a >2 g transient at a trunk it is vertically level with), not by proximity. `droneflight b2-score`. Threshold calibrated against the tree-free B1 control, which never exceeds 0.50 g |
| `reactive.py` | closed-loop **accel- and jerk-limited** forest-flight sim (avoidance ON vs OFF, drift, VTRN). 🔴 Was purely kinematic until B2; that one simplification made it predict the *wrong failure mode* |
| `nav.py` | **the simulated error** — VIO drift + VTRN correction (idealised reset *and* fidelity-limited optical driver), plus yaw drift and EV lock loss |
| `correction.py` | **optical terrain-relative correction** — data-driven fixes over a spatially varying basemap; confidence floor + innovation gate + Kalman fuse |
| `ev_bridge.py` | **external-vision injection** — Gazebo truth + injected error → PX4 `VISION_POSITION_ESTIMATE`; ENU→NED frames, EV dropout, track recording, corridor analysis |
| `executor.py` + `connector.py` | connect→upload→arm→start→monitor, RTL on error (+ progress-stall watchdog); fake connector for tests |
| `mavsdk_connector.py` | live PX4 adapter (⚠️ needs SITL; excluded from coverage, not unit-tested) |
| `sitl_launch.py` | inject the plan's home into PX4 SITL (`PX4_HOME_*`) so a mission lands at the real AO, not Zurich |
| `config.py`, `cli.py` | config + `validate` / `nav` / `tether-check` / `tether-plugin-check` / `simfly` / `sitl-env` / `px4-params` / `flatten-plan` / `ev-bridge` / `ev-report` / `fly` commands |
| `sitl/tether_plugin/` | **Gazebo C++ wrench plugin (v0)** — physics bit-exact vs `tether.py`; ✅ compiled, flown, §1 parity + §2 disturbance passed (§3 ready) |

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
droneflight b2-score --ulog sim-logs/b2/13_47_06.ulg \
                     --trunks ../droneplanner/worlds/forest_dense_trunks.csv   # tree strikes from a flown ULog
# needs the optional analysis extra:  pip install -e '.[analysis]'   (pyulog)
# exit 0 only if CLEAN. Score an arm against the OTHER manifest too — impacts
# align with the world that actually loaded, which catches a stale gz server.
droneflight simfly   --plan .../mission.plan --drift-rate 0.02 --vtrn-interval 150
eval "$(droneflight sitl-env --plan .../mission.plan)"                  # PX4 SITL home = plan home
droneflight flatten-plan --plan .../mission.plan --out ev_test.plan      # constant alt for PX4's FLAT world
droneflight ev-report --plan .../mission.plan --record run.jsonl        # corridor stats of a recorded EV run
droneflight fly      --plan .../mission.plan --url udpin://0.0.0.0:14540         # LIVE — needs PX4 SITL (Ubuntu VM)
droneflight ev-bridge --plan .../mission.plan --url udpin://0.0.0.0:14541        # LIVE — needs Gazebo + PX4 (RUNBOOK §3b)
ruff check . && ruff format --check . && mypy --strict src && pytest -q # 337 tests
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

## 🟢 ONBOARD LOCAL PLANNER (2026-09-23) — avoidance without a prior map
`local_planner.py` (the planner), `reactive.fly_with_local_planner` (harness),
`local_offboard.py` + `droneflight fly-local` (live). **Why:** the user ruled out
lidar ("assume no lidar unless absolutely necessary"), which removes the prior
stem map the 0-strike 2026-09-14 flight depended on, and reactive avoidance is
dead in flight (N2). So the aircraft now does the stem routing itself: it
**remembers** every trunk its depth sensor has returned, **plans** an A* path
through the gaps with a clearance cost, and **replans** every 0.4 s.

**Offline, same harness and manifests that gave reactive avoidance 27 strikes:**

| case | strikes | min gap | mean speed |
|---|---|---|---|
| dense 107/ha, whole 5.2 km route, 6 m/s | **0** | +0.81 m | 5.9 m/s |
| sparse 50/ha, 6 m/s | 0 | +1.31 m | 6.0 |
| dense, 2 % drift **no fixes** (3 seeds) | 0 | ≥ +0.67 m | 5.8 |
| random 200/ha — 3.0 / 6.0 m/s | 0 / 1 per 1.2 km | +0.92 m | — |
| random 400/ha — 2.0 / 3.0 / 6.0 m/s | 0 / 0 (+0.05 m) / 2 | +0.55 m | — |
| random 800/ha — 2, 3, 6 m/s | **stalls or strikes at all** | — | — |
| **MVP route + MVP forest (200/ha)**, truth | **0** | +0.43 m | 4.8 (18.1 min) |
| MVP, 2 % drift + VTRN fixes (4 seeds) | **0** | +0.06 … +0.96 m | 4.6 |
| MVP, VTRN fixes bled in over 10 m like the bridge (4 seeds) | **0** | +0.31 … +0.59 m | 4.8 |

That envelope is now the planner's speed model
(`droneplanner/fields/stem_density.ONBOARD_PLANNING_ENVELOPE`).

**Every limit in it exists because of a specific strike, and the order matters:**
1. **Strict hard margin, relaxed only to escape.** A* forbids cells inside
   0.35 m of air gap except within 1.5 m of the aircraft.
2. **String-pull without giving back clearance.** The raw 8-connected path made the
   commanded speed flicker 2.8↔6.0 m/s per replan; the airframe averaged "fast".
3. **Brake FOR the turn** (`v² ≤ v_turn² + 2as`). The first rule let it reach a 45°
   bend at cruise and swing into a known trunk at 4.7 m/s.
4. **Commit to momentum** — replan from where 0.4 s of velocity carries it. A late
   south→north side-switch 4.5 m from a trunk at 5.5 m/s was unflyable.
5. **Dynamic feasibility** — each (path, speed) candidate is flown 2.5 s forward
   under PX4's `MPC_ACC_HOR`/`MPC_JERK_AUTO`; slower or the previous path win if
   the fresh one would pass too close. At 800/ha a 70° replan a metre from a trunk
   hit it at 2 m/s **with the trunk mapped to 0.2 m** — a clear path is not a
   followable one. This took 400/ha 11 → 2 strikes at 6 m/s.
6. **Sight limit** — never faster than can stop before the path's swept width
   leaves *observed* space. Occlusion made trunks appear too close to stop for.

🔑 **GPS-denied property, and the correction trap.** Scans are swept from the TRUE
pose and placed from the BELIEVED pose, so slow drift moves trunks and aircraft
together — avoidance is immune to it (0 strikes with 100 m of unfixed drift). But
a **VTRN fix** moves the aircraft in the believed frame without moving the
remembered trunks, and the EV bridge bleeds a fix in over ~10 m, so no step looks
like a jump. `observe()` therefore predicts each position from velocity and shifts
the whole map by the unexplained part — memory lives in the continuous odometry
frame, as a real VIO stack would hold it.

**IN THE LOOP (PX4 SITL + Gazebo, forest_mvp 200/ha, RUNBOOK §4.14):**

| flight | `b2-score` strikes | min air gap | time |
|---|---|---|---|
| GPS-aided | **0** | +0.54 m | 21.5 min |
| **GPS-denied** (EV + VTRN, nav error up to **11.8 m**) | **0** | +0.33 m | 18.6 min |

Harness predicted 0 strikes / +0.43 m / 18.1 min — the right answer this time, and
the first time the harness has been confirmed in the loop for an avoidance layer.

🐛 **Three defects the loop found that the harness could not:**
1. **EV altitude datum** — `position_velocity_ned` z sat 124.6 m (home AMSL) below
   truth under external vision. Now `relative_altitude_m`, plus a refuse-to-fly
   check (>25 m disagreement at offboard start).
2. **Clearing observed space on every correction** — bled-in VTRN fixes made every
   step a correction; the sight limit then crawled at 0.8 m/s in open ground. The
   grid now shifts by an accumulated offset.
3. **The correction detector missed slow bleeds** — at a 0.05 m/step threshold a
   fix under ~1.7 m was never detected. Threshold is now **0**: any displacement
   velocity does not explain *is* an estimate correction. (Harness, 6 seeds at
   400/ha GPS-denied: 0.05 → 2 grazes, 0 → 1; 200/ha 4/4 clean.)
The harness now bleeds fixes in like the bridge (`fix_ramp_m`), which is what
exposed 2 and 3 offline.

🔴 **2026-09-24: the loop found four more defects, and the model behind them.**
Chasing the envelope past 400/ha, then two GPS-denied SITL contacts:

| # | found by | defect | fix |
|---|---|---|---|
| 4 | harness, 500-600/ha grazes | feasibility predicted from zero acceleration | (superseded by 6) |
| 5 | harness, 800/ha: stationary 700 s | fixed 2.5 m pursuit chord cuts every bend by the same amount at any speed, so nothing was predicted clear and "stop" won forever | lookahead = 1.2 s of travel (0.8-2.5 m) + a creep candidate |
| 6 | **SITL fix2**: contact, −0.17 m | the prediction model was PX4's AUTO-mode jerk generator; **offboard velocity setpoints bypass it** | measured response: **0.3 s dead time + 0.4 s lag** (`px4_profile.OFFBOARD_*`, fitted on 3225 SITL windows, 1 s position error 0.05 m mean / 0.18 p95; the jerk model 0.17 / 0.40). Predictor flies the setpoints already in flight; margin 0.15 → 0.30; harness plant = same model, drawn per seed from inside the fit |
| 7 | **SITL fix3 + fix4**: contact at the SAME trunk | heading at a trunk at 2.9 m/s from 1.6 m when stopping takes ~2 m; replans kept going round it, then reversed | **momentum limit** `v ≤ (room along current velocity − margin) / (delay + lag)`; a moving candidate must also leave a clean stop after one replan |

Replay of the SITL records (the planner stepped through the flown inputs)
showed the **map was right every time** (trunk within 0.0-0.15 m) — the failures
were all prediction/dynamics, the same class as [[px4-vs-offline-models]].
Ruled out by measurement: telemetry staleness (est vs truth lag slope ≈ 0.00 s),
the tether plugin (fix4 tether off, same spot), CPU starvation (fix4 on a quiet
host). ⚠️ **The harness never reproduced 6 or 7** — 16/16 GPS-denied seeds clean
before and after, even with 10.5 m nav error. SITL is the arbiter for this class.

| SITL, GPS-denied, forest_mvp 200/ha | code | `b2-score` |
|---|---|---|
| denied3 (09-24 night) | pre-fixes | 0 strikes, +0.78 m |
| fix2 | + accel seed + lookahead | **contact −0.17 m** (tether on) |
| fix3 | + measured response model | **contact −0.23 m** (tether on) |
| fix4 | same as fix3 | **2 contacts** (tether off, quiet host) |
| **fix5** | + momentum limit + stop-after | **0 strikes, +0.676 m, 18.6 min** — +0.47 m at the fix3/4 trunk |
| mvp2 | fix5 code, forest legs at the harness's **5.0 m/s** | **1 strike** (waypoint 71) |
| mvp3 | + stopping distance with saturated braking | **1 strike** (waypoint 54) |

**Then a seed sweep on the final stack** (`sim-logs/mvp_local/s*`, `q2f`; RUNBOOK
§4.14): the EV noise is seeded, so each run is one draw. Two more stack changes
came out of it — **fixes are bled over 30 m, not 10** (at 10 m a 4-5 m fix moved
the estimate ~1.4 m/s and EKF2's velocity took part of it: same seeds 2/3 at 3 m/s
grazed twice at 10 m, clean at 30 m with the same nav error), and the bridge now
also publishes the **VIO velocity** (`EKF2_EV_CTRL 15`; measured no effect on the
contacts, kept because it is what the sensor provides). Results, 200/ha GPS-denied:

| speed | clean (seed set) | contact (seed set) |
|---|---|---|
| 4 m/s | 0/1 | 2/3 (at 10 m and at 30 m bleed) |
| 3 m/s | 0/1, 2/3, 6/7 | **4/5, twice** |
| **2 m/s** | **4/5 — +1.05 m of air** | — |
| 400/ha, 2 m/s | 4/5, +0.34 m | — |

Every struck trunk had been in the sensor's view from ~30 m, 6-12 s before
contact, nearly dead ahead — not sensing. So the `fly-local` record now logs what
the planner believed (path, feasibility candidates, memory within 5 m), and the
next reproduction of seed set 4/5 (r3e) showed exactly what happened: **the path
cleared the trunk by 0.9-1.4 m, the aircraft tracked it to 0.02 m, the map was
right to 0.1 m — and the momentum limit braked 3 → 0.8 m/s mid-curve.** Braking
and turning share `MPC_ACC_HOR`; the aircraft slid 0.72 m off its own path into
the trunk, and every candidate on offer was slower. 🔧 **Steer, don't brake**:
when every slower candidate fails, the feasibility check also tries the current
speed along the path. Then, four seed sets per case:

| after "steer, don't brake" | clean | contact |
|---|---|---|
| **200/ha, 3 m/s** | **4/4** (+0.63-0.87 m surface) | — |
| 200/ha, 4 m/s | 4/5 | 0/1 |
| 400/ha, 3 m/s | — | 4/5 (2) |

**Planning anchors: 200/ha → 3.0 m/s, 400/ha → 2.0 m/s.** (400/ha at 2 m/s was
flown clean on seed set 4/5 before this fix; its four-seed re-validation was cut
short by host CPU contention — pending.)

🔑 **The harness envelope is optimistic above ~2 m/s — SITL is the arbiter.** With
all fixes the harness flies 200/ha at 5 m/s and 400/ha at 4 m/s clean over 4 seeds
(`tools/envelope_sweep.py`); SITL hit a trunk at 5 m/s at 200/ha in both tries. At
5 m/s PX4 shed speed at ~1.4 m/s² against the 3 demanded, and the aircraft curved
onto a trunk that had sat just off its velocity ray. So the planner now prices on
a **SITL-anchored** envelope (`droneplanner/fields/stem_density.py`): 200 → 3.0 and
400 → 2.0 as flown over four seed sets, the rest discounted by the measured ratio. Raising an anchor
takes a SITL flight, ~20 min each.

Also fixed: the **EV bridge exited on one refused publish** (MAVSDK NO_SYSTEM
after a heartbeat timeout under a starved VM, fix2) and left EKF2 with no aiding
— the estimate ran 181 m off. It now counts and rides failures out.

🔴 **2026-09-25: the estimate's VELOCITY is wrong GPS-denied — and slow flight smears the map.**
Fitted against Gazebo truth on six `fly-local` records (`sim-logs/mvp_local`):

* **Braking is not the dynamics.** With GPS the offboard model predicts braking with
  ~0 bias (1166 windows); GPS-denied the aircraft ran **0.63 m further per second
  of braking** than predicted. The estimated velocity is off by 0.15-0.31 m/s per
  axis (1/e ≈ 3 s, p95 |e| 0.53-0.88) against ~0.1 GPS-aided — an error, not a lag
  (best-fit lag 0.00 s). It is mostly **consistent**: it correlates 0.73-0.79 with
  the rate the position estimate drifts, i.e. EKF position integrates it.
* **Harness now models it** (`reactive.fly_with_local_planner(velocity_error=
  nav.VelocityErrorConfig)`: planner sees true + e, plant tracks setpoint − e,
  believed position integrates e). At 0.15 m/s it produces braking-type contacts
  (400/ha at 3 m/s: 2/4 seeds) — **the first harness to reproduce a SITL contact
  class** — and the smear stall below. It is somewhat pessimistic (grazes −0.01 m
  on 200/ha at 3 m/s, which SITL flew 4/4 clean) — and in DENSE forest badly so:
  **calibrated 2026-09-26 on Askola's ~1000/ha stand at 1.0 m/s, which SITL flew 4/4
  clean: with σ 0.15 the harness blocks for 1928 s of 2000 and grazes 28 times;
  without it, clean (+0.44 m).** So a velocity-error harness failure at ≥ 1000/ha
  means nothing; use it for braking contacts at moderate density only.
* **`predict_velocity_error_ms`** (opt-in, PR #3): margin grows with look-ahead
  time. **Uncapped it stalled SITL 2/2** (v3e 400/ha 3 m/s, v4a 200/ha 4 m/s — 0
  contacts, stopped 0.97 m from a trunk for good); growth now capped at 1 s, and
  capped it **still stalled 1/2** (w3e, wp 83) with the other at +0.04 m (w3a).
  **Decision: stays 0 (off).** It trades contacts for stalls, not for a clean
  envelope.
* **Memory smear — the measured-density stall.** Askola measured zone (~1000
  stems/ha, 1.5 m median spacing, 0.5 m/s): **m3e, m3a and c3e all stopped for
  good at waypoint 4**, 0 contacts, nearest real trunk 2.5 m away. Consistent drift
  (1-3 cm/s) is never detected as a correction, and at 0.5 m/s the aircraft sits
  among the same trunks for minutes — each trunk was laid down 1-3 m apart, memory
  reached 2200-2700 points, every gap closed. **Fix: `clear_range_m`** — delete
  remembered points the current scan sees through (in the cone, ≤ 8 m where a 1°
  fan cannot miss a 0.16 m trunk, nearest ray and both neighbours past it by
  0.3 m); occluded/out-of-cone points stay. **k3e (same plan, same seeds as m3e):
  reached the goal, 0 strikes, +1.014 m, forest stretch at 0.49 m/s.** 21.4 min
  armed against 17.5 planned — unexplained, the forest stretch itself was on pace.

🧪 **B4 sensor ladder (`tools/sensor_ladder.py`, 2026-09-24) — the oracle is not
load-bearing.** MVP forest, 200/ha, 3 m/s, GPS-denied, 2 seeds per rung: **0 strikes
at every rung** — range noise 0.1/0.3/0.6 m at 10 m (growing with range), 30 % and
60 % dropped returns, max range 15 m and 8 m, and a stereo-like combination (0.3 m
@ 10 m, 30 % dropout, 15 m). The only failure: 0.6 m noise **stalled** one seed
(noise smeared the map shut) — the safe direction. Untested: phantom returns and
systematically missed trunks; and only at 200/ha / 3 m/s.
⚠️ **400 stems/ha is MARGINAL GPS-denied** — 1 graze (−0.016 m) in 6 harness seeds,
clean on truth. The 1500 m zone decision rests on 400/ha at 2.0 m/s.
⚠️ **Still oracle** (`perception.sense`: perfect ranges, no noise/dropout, but real
occlusion). ⚠️ 800/ha is outside the envelope, and the canopy-height model puts
the demo AO's real stands at 520–1290/ha.

## ⬆️ IN-FLIGHT ESCAPE CLIMB (2026-09-25) — flown in SITL, **4/4 clean**
`fly-local --escape-after` (default 20 s): blocked among trunks, climb straight up
to the tallest trunk top within 15 m + 3 m, hold 50 m of route, descend (sensing at
the target height). Test world `askolam230w`: Askola's measured-zone forest + an
**unmapped 90 m wall of trunks 1 m apart across the route at 150 m**
(`droneplanner/tools/add_trunk_wall.py`), plan unchanged. Two defects found on the way:

| run | defect | fix |
|---|---|---|
| x4e | **escaped 20 s after take-off at route 0** while flying normally — the trigger watched the vertex index, and a 25 m leg at 0.5 m/s shows no index progress for 50 s | trigger on along-route distance (`CarrotGuidance.along_m`, 1 m gain) |
| x5e | escaped at the wall, then **stalled at height 90 s** (1130 failed plans) — memory still held every trunk mapped from below | drop the memory on reaching the escape height; anything that really reaches it is sensed afresh |
| **x6e** | — | **reached the goal: 1 escape at route 5 (19.8 → 27.6 m), released at route 12, `b2-score` CLEAN +1.036 m, 0 samples inside a trunk** |

**All four seed sets:** x6e +1.036, x6a +0.848, x6b +0.913, x6g +0.672 m (`b2-score`
CLEAN, 0 samples inside a trunk), each with exactly 1 escape at route 5 (19.8 → 26-28 m)
released at route 12, and each reaching the goal. (A first x6a at RTF 0.08 was voided.)
**The offline harness now has altitude** (`reactive.fly_with_local_planner(vertical=
LocalOffboardConfig())`) and runs the same `Escape` state machine as `fly_local`. It
reproduces the walled-world escape offline (1 escape to 26.2 m, clean — SITL 26.2-27.6 m)
and flies the over-canopy Askola plan clean on two seeds, in ~1.5 min instead of ~25.
It found a third defect at once: the planner stops ~21 m short of a blockage it can see,
so in the open in front of an impassable stand no trunk was within the 15 m escape
radius and the escape never fired → **`escape_radius_m` 30 m** (the planner's report
mirrors it). A 7 s unit test now pins "escape over a wall, reach the goal clean".
⚠️ The escape height comes from the manifest's trunk tops (oracle); a real aircraft
needs its own canopy-top sensing or the plan's over-canopy altitude. And the
climb-through-crowns caveat applies: SITL trunks have no crowns.

**The escape fires on its own in dense natural forest (2026-09-26).** No wall — just
Poisson stands on the MVP route, GPS-denied, four seed sets each:

| world | speed | `b2-score` | air (surface − 0.45 m) | escapes |
|---|---|---|---|---|
| `forest_mvp1200n` (1200/ha) | 1.0 m/s | 4/4 CLEAN | +0.33 / +0.50 / +0.48 / **+0.07** | **1 in 4** (h1g, route 52) |
| `forest_mvp1500n` (1500/ha) | 0.5 m/s | **4/4 CLEAN** | +0.52 / +0.78 / +0.58 / +0.52 | **1 in 4** (h5e, route 40) |

Each escape followed the local planner's speed cap going to 0 (no gap it could thread)
and was cleared by the climb, 20 s later, with 0 contacts; h1g's thin +0.07 m was on the
approach to that dead end, at 0.95 m/s and normal flight height. At ~1000/ha: 0 escapes
in 4. So **passability, not speed, fails first past ~1000 stems/ha**, and one traverse
in four there relies on the escape. Planner anchors unchanged (bar: >0.15 m air on
every seed set) for 1.0 at 1200; 1500/ha at 0.5 is 4/4 (h5g re-flown after a run at RTF
0.11 with the host loaded was voided) → planner 1200/ha anchor 0.4 → 0.5.

**Why the dead ends? Not measurable from the old record (2026-09-26, PR #9).** In h1g every
replan returned *no path* for 20 s. Replayed offline against the TRUE trunks shifted into
the odometry frame — plus symmetric smear up to σ 0.5 m, plus a two-offset memory
mimicking the 1.1 m of navigation drift in those 13 s, plus windows up to 25 m — the
planner found a path every time. Remembered trunks near the aircraft sat 0.4-0.9 m from
where they now were, and memory held 1027 points against ~190 real trunks within 30 m,
but the record kept only the nearest few. So `fly_local` now dumps the **whole memory and
the corridor on the first step of each blockage** (`mem_dump`), and
`tools/replay_blocked.py` re-runs `plan_local` on exactly that, plus the variants a fix
would change (no corridor / wider window / merged memory).

## 🧵 TETHER ON OVER-CANOPY LEGS (2026-09-25)
Flying over the canopy puts cruise legs at 25-35 m AGL (canopy + clearance), where
the fiber's free-falling span is longer: the straight-leg ceiling is **warn 5.6 /
break 8.2 m/s at 28 m** (7.5 / 10.9 at 12 m). `tether-check --trajectory
<planner trajectory.geojson>` now scores every leg at its own AGL. Askola plans
(600 m zone; measured zone with the over-canopy fallback): **2 legs above warn, 0
above break** — 6.0 m/s at 32-33 m AGL = 77 % of the break speed, ~46 % of break
tension (drag ~v³). Accepted; a planner speed cap at high AGL is the lever if bench
data says otherwise. And where the fiber lands matters more than its tension there:
over closed canopy it settles on the crowns (droneplanner `cable_on_crowns`).

## 🧵 TETHER AT FULL SPEED AND UNDER THE CANOPY (2026-09-24)
User: *"Now that full-speed flight is in use, remember to check and simulate the
tether drag."* The 1500 m zone means 6 m/s over open ground and weaving flight
among trunks — neither existed when the tether was last flown (§3, a synthetic
turn course in mission mode).

**1. `tether-check` now scores the plan's own speeds.** It scored every vertex at
flat cruise and ignored the `.plan`'s `DO_CHANGE_SPEED`; now each vertex takes the
faster of its two legs, and the plan speed caps the policy arm (as
`executor.prepare_items` does). MVP plan: **0.78 N peak, 39 % of the 2 N break.**

**2. Drag at full speed is not v² — it is closer to v³.** Paid-out fiber is at rest
over the ground and falls broadside at ~1.9 m/s (250 µm, 0.069 g/m), so it touches
down `height / 1.9 × speed` behind the aircraft: **the airborne span grows with
speed**. At 12 m AGL and 6 m/s that is ~39 m — which is why the model's 40 m
default is about right today — but it doubles at 12 m/s. `speed_ceiling_ms`:

| fall height | warn (60 % of break) | break | span at 6 m/s |
|---|---|---|---|
| 5 m | 11.3 m/s | 14.6 | 16 m |
| 12 m (open-ground cruise) | **8.4 m/s** | **10.9** | 39 m |
| 25 m | 6.5 m/s | 8.5 | 82 m |

**Today's 6 m/s is inside it. Do not raise open-ground cruise past ~8 m/s, or fly
higher, without payout-bench data** — the fixed-span model would have said 14 m/s.
⚠️ Free fall ignores back-tension (lengthens the span) and rotor downwash
(shortens it); both un-benched.

**3. Under the canopy the risk is contact, not tension.**
`tether.airborne_span_contacts` / `tools/fiber_contact.py`: on a flown track, the
straight span from touchdown to aircraft cuts the corners the aircraft flew round.
On the three SITL forest flights (true tracks, flown manifests):

| touchdown behind aircraft | 200/ha GPS | 200/ha GPS-denied | 400/ha GPS-denied |
|---|---|---|---|
| ≤ 5 m | **0** | **0** | **0** |
| 10 m | 0 | 2.3 / km | 5.9 / km |
| 20 m | 9.3 / km | 14.4 / km | 38.9 / km |
| 40 m | 37 / km | 43 / km | 108 / km |

Free fall from 9 m puts touchdown **7 m behind at 1.5 m/s, 14 m at 3 m/s** — right on
the knee. 🔑 **This is a payout requirement: below the canopy the fiber must reach
the ground within ~5 m of the aircraft**, or slow flight is what gets it there.
Slower forest speeds help the fiber as well as the avoidance. Whether a contact
snags, abrades or just slides is unmodelled.

**4. FLOWN in the loop — tether ON, full MVP route, ✅ CLEAN** (`sim-logs/mvp_local/tether1_07_11_36.ulg`
+ `tether1.csv` + `tether1.jsonl`, RUNBOOK §4.14 flight 4). forest_mvp 200/ha,
GPS-aided, `fly-local`, Gazebo tether plugin on (span 40 m). `b2-score`: **0
strikes, 0 ground impacts, +1.28 m** min trunk-surface clearance; reached the goal
in 18.2 min. Plugin parity vs `tether.py`: **EXACT**.

| stretch | peak tension | mean | peak turn transient | max speed |
|---|---|---|---|---|
| forest, weaving at ≤3 m/s | 0.67 N | 0.57 N | **0.28 N** (p99 0.19) | 4.9 m/s |
| open ground, full speed | **0.78 N (39 % of break)** | 0.76 N | 0.05 N | 6.1 m/s |

**At full speed the load is drag on the open legs, not turns among the trunks** —
offboard weaving produces turn transients about the size of §3's mission-mode
corners (0.315 N). The 0.6 N drag disturbance did not affect avoidance. So the
tether is within budget at today's speeds; what limits it is (2) the ceiling if
cruise rises and (3) trunk contact under the canopy — neither of which the
plugin models (fixed span, no trees).

## ✅ STEM-MAP LOCALISATION BUILT (2026-09-15) — `stem_localizer.py`
**The gap this closes.** The stem-routed plan flew **330/330 with 0 strikes** —
but **GPS-aided** (`EKF2_GPS_CTRL=7`, `EKF2_EV_CTRL=0`, fix type 3 throughout),
and this programme is GPS-denied. The margins make that decisive: the route was
planned with **1.387 m** of clearance and flew with **0.708 m**, so PX4's own
tracking consumed **0.679 m** with a near-perfect position estimate. Any
navigation error lands on top. GPS-denied VTRN holds **11.8 m** on a good seed —
**~17x too coarse**.

So: match the trunks you can see against the trunks you expected, and the
disagreement is your pose error. One instrument does routing *and* localisation,
and it works exactly where optical VTRN starves (the conflict arm declined
**76.5 %** of fixes for `no_features` — in the forest that is supposed to be
hiding the aircraft).

**Measured against a known offset (6.0, −4.0) m and 8° of yaw:**

| condition | fixes accepted | mean error | max error | yaw error |
|---|---|---|---|---|
| pristine map | 12/12 | **0.034 m** | 0.062 m | 0.04° |
| 40 % of mapped stems **felled** | 12/12 | 0.025 m | 0.039 m | 0.07° |
| 60 % felled | 12/12 | 0.050 m | 0.109 m | 0.09° |
| 80 % felled | **4/12** | 0.043 m | 0.067 m | 0.10° |
| 100 % **unmapped** stems appear | 12/12 | 0.033 m | 0.067 m | 0.07° |
| **200 %** unmapped (2/3 of what it sees is not on the map) | 12/12 | 0.032 m | 0.048 m | 0.04° |
| 50 % felled + 100 % unmapped | 5/12 | 0.019 m | 0.026 m | 0.05° |
| survey **0.5 m** position error | 11/12 | 0.220 m | 0.429 m | 0.37° |
| survey **1.5 m** position error | 4/12 | 1.304 m | **1.907 m** | 1.58° |

🟢 **0.03 m against VTRN's 11.8 m — roughly 350x better, and it degrades the right
way.** Up to 60 % of the map being wrong costs nothing; past that it **declines**
rather than guessing (80 % felled: 4/12 accepted, and the 4 it accepts are still
good to 0.067 m). `tools/stem_localizer_sweep.py` reproduces the table.

**Why RANSAC consensus and not least squares.** Per the user (2026-09-15) a
government survey may *validate* stem mapping but **the drone cannot rely on one in
flight, because the battlespace is modified and changes**. A large, unknown share
of the map will be wrong in both directions — felled stems missing, unmapped ones
present. Least-squares and nearest-neighbour fitting are dragged off by exactly
that; consensus simply does not count it as evidence. Inter-stem *distance* seeds
the hypotheses because it is invariant to the pose being solved for, which is what
makes two correspondences enough.

🔴 **The failure mode that matters, and it nearly shipped.** On a map with **1.5 m**
of position error the matcher first accepted **11/12** fixes with a **20.8 m**
maximum error — confidently, silently wrong, which is far worse than no fix (dead
reckoning degrades predictably; a 20 m "fix" steers into a trunk). The defence is
an **ambiguity test**: keep the best *rival* hypothesis as well as the best one and
refuse when a materially different alignment explains nearly as much. That took it
to 4/12 accepted at 1.9 m max.
🔑 **And `distinct_m` must be several metres, not the match tolerance.** Set to
1.0 m it compared the best solution against a *noisier copy of itself* — two seed
pairs describing the same alignment do not agree exactly, since a 4 m baseline
carrying 0.05 m of sensor noise fixes heading to only ~0.7°, which over a 25 m lever
arm scatters the translation half a metre. Availability on a **pristine** map fell
to 7/12 before that was corrected. The rival that matters is an alignment a whole
stem away — metres.

➡️ **Requirement this establishes: stem-map position accuracy must be ≲0.5 m.**
At 0.5 m the fix still works (0.22 m mean) but eats a third of the 0.708 m flown
clearance; at 1.5 m it is unusable *and only partly self-detecting*. The detector
(`droneplanner/forest/stem_detect.py`) measures **0.017–0.029 m** on synthetic
scans, so a UAV-lidar map qualifies and **ALS crown-centroid inference (1–2 m) does
not** — it fails localisation as well as clearance.

⚠️ **Not yet in the loop.** This is scored offline against synthetic stands. It has
not been wired to `perception.py`, not run against detections from a real scan, and
not flown — the GPS-denied stem-routed flight is the test it exists for. A regular
plantation is the hard case (an alignment one row over fits almost as well) and is
pinned by `test_a_regular_plantation_is_reported_as_AMBIGUOUS`, which asserts it
**declines**.

## 🔴 The central finding is now PARTLY FALSIFIED IN THE LOOP (B2, 2026-07-28)
**"Reactive avoidance reliably dodges obstacles" was measured in a kinematic
harness and does not survive real dynamics.** Flown in Gazebo with live trunk
collisions, the corridor executor reached the goal in both forest arms **by
hitting trees** — 19 strikes at 50 stems/ha, **37** at 107 — while reporting
`REACHED GOAL`. Cause: `react_distance_m` = 6.0 m at 6.0 m/s yields **0.66 m** of
achievable lateral offset against **1.10 m** needed, under PX4's own
`MPC_ACC_HOR`/`MPC_JERK_AUTO`. The harness predicted *paralysis* (79 % blocked
steps); the aircraft delivered *collision* (0.17 % blocked). Full method, the
clean negative control, and the sized fix: **TEST-PLAN.md § B2**. Everything
below this line was measured in the kinematic sim and must be re-read in that
light.

### ✅ Fix landed on the Mac 2026-07-28 (unflown) — `maneuver.py`
`react_distance_m` is now **derived**, not a constant: 6.00 m → **11.73 m** at
6.0 m/s cruise, and it scales linearly with speed. `n_rays` 31 → **91** (3.00° →
1.00°) so the range is actually observable. `OffboardResult` gained **`strikes`**
and a `succeeded` property (`reached_goal and strikes == 0`) — B2 could not
express "arrived, wrecked". `OffboardConfig.sensing_shortfall()` refuses a
configuration that cannot geometrically miss; fed the flown B2 settings it says
so precisely:

> *react distance 6.0 m is below the 7.8 m needed to dodge a 0.15 m obstacle at
> 6.0 m/s — the airframe cannot miss it. Slow to 3.1 m/s or raise the reaction
> distance.*

(3.1 m/s is the *with-margin* recommendation at `safety_factor=1.5`; the
bare-graze speed is 4.60 m/s. Both are pinned by tests.)

⚠️ **This invalidates every `simfly`/`reactive.py` number measured at the old
constant** — `reactive.py` shares `AvoidanceConfig`, so its reaction distance
moved 6.0 → 9.78 m at its 5.0 m/s cruise. Deliberate: the constant is *measured
wrong*. `AvoidanceConfig(react_distance_override_m=6.0)` reproduces the old runs
and is documented as historical-only, mirroring `tether-check --no-px4-prior`.
### ✅ `reactive.py` now has PX4's accel + jerk limits (2026-07-30) — and it reproduces B2
The last prerequisite. Velocity is a state slewed under `MPC_ACC_HOR`/
`MPC_JERK_AUTO`; a contact is resolved as a **sliding stop** and counted rather
than aborting the run, because a harness that stops at the first collision cannot
be compared with a flight that had 36. Scored on the **real 282-wp route and the
real manifests**:

| arm | old harness | corrected harness | **flown** |
|---|---|---|---|
| sparse, 50 stems/ha | 0 collisions, "reaches goal" | **15** strikes | **19** |
| dense, 107 stems/ha | 0 collisions, "❌ 79 % blocked" | **22** strikes | **36** |

So it now produces the right *phenomenon* and the right *ordering*, at 60–80 % of
the measured count. That is the first time this harness has been comparable with
a flight at all. Both halves of the defect are pinned by tests: instant response
**hides a collision** (single trunk: +0.064 m clean vs −0.001 m graze) and
**predicts paralysis** where the vehicle makes progress (trunk wall: stuck vs
reached).

🔴 **But it does NOT show the fix working, and that is a real result.** Re-run with
the derived reaction distance and 91 rays, strikes go **15 → 16** (sparse) and
**22 → 27** (dense) — no better, slightly worse. **Do not fly the fix expecting a
clean run.** The likely reason is architectural and is the open question below.

🐛 **Found while wiring the validation up: `reactive.py` projected to *absolute*
metres (~1.1e6, 5.2e6) while every trunk manifest is home-relative (~0..3500).**
A real forest therefore sat a million metres from the aircraft and every run came
back clean with `min_clearance` of exactly 0.000 m. **This is the third instance
of that frame mismatch** (after `ev_bridge.route_reference`'s degenerate basemap),
and it is why the first validation attempt reported 0 strikes on both arms.
`_to_local` is now home-relative and a test pins home to the origin.
⚠️ It also means **every historical `simfly` result flew an emptier forest than it
claimed** whenever a manifest was involved — the `scatter_forest` path was
self-consistent and is unaffected.

⚠️ Two harness bugs my own tests caught, worth knowing if you touch it: removing
the *whole* velocity on contact pins the vehicle against the trunk forever (it
reads as a stall, not a strike), and without release hysteresis a grind along one
trunk counted **39 892 strikes**.

All green offline: ruff, `mypy --strict`, **296 tests**.

## The central finding (from the closed-loop **kinematic** sim — see caveat above)
- With **known position**, reactive avoidance reliably dodges unmapped obstacles:
  blind flight collides, reactive reaches the goal (verify: `simfly` shows
  OFF=COLLIDED wp1, ON=REACHED all). ⚠️ **In-loop this is false at 6 m/s** — the
  kinematic drone stops dead when blocked; the real one carries momentum through.
- Under **GPS-denied drift**, avoidance senses *real* trees so it keeps dodging,
  but drift wanders the *true* path out of the planned tube into dense forest.
  **Safe forest flight needs BOTH reactive avoidance AND bounded navigation
  (VTRN)** — neither alone. `reactive.py`'s VTRN must *shrink* error, not teleport
  it (an earlier bug); avoidance must react at a *lookahead* distance, not the
  bare safety radius (another earlier bug) — both fixed.

## 🔴 N2 follow-up 2026-08-11 — three candidate fixes built, measured, and all REJECTED
`avoidance.py` gained two knobs (`gap_policy`, `transient_margin_m`; `fly-corridor
--gap-policy/--transient-margin`) and **both default to the existing behaviour**,
because on the real route and real manifests every proposed change is worse:
`nearest_goal`+no margin **35** strikes over six manifest×speed configs, +margin
38, `widest` **142**, `widest`+margin 101, and correcting a real under-pad
(0.95 → 1.10 m) 40. Gap policy alone: **73 vs 243, 3.3× worse**.
🔑 **The three negatives share one mechanism and that is the finding: every change
that makes the vehicle deviate MORE makes it hit more trees**, worst in the dense
manifest. At ~4.6 m spacing the room needed to miss one trunk is comparable to the
distance to the next, so **the dodge is the hazard** — which is why
minimum-deviation steering keeps winning despite hitting 34 trees.
🐛 Known under-pad left in deliberately at `avoidance.py:173` (0.95 m where
`lateral_clearance_m` = 1.10 m is needed, omitting the trunk radius); corrected it
measures worse, so it is recorded in a comment rather than silently changed.
⚠️ Nothing shipped on harness evidence — it has twice predicted the wrong
*mechanism* here. The knobs exist to be flown. **The defect is still open.**

## 🔴 N2 attempt 2026-08-04 — VOID, wrong plan file. And it exposed a 2D-perception defect.
The three speed arms flew (`sim-logs/b2/new_tests/`, identified from `p90 |v_h|` =
**6.01 / 3.51 / 2.01 m/s** — their console logs were lost, so the flag is not
evidence). **All three flew `~/drone/plans/mission.plan`, a stale flat-30 m AGL
route**, not the 282-wp below-canopy `canopy.plan`. Consequences: **22–25 m above
the canopy**, 0 of 71 775 samples below crown level, cross-track vs the real plan
**mean 55–74 m / max 119.6 m**, and a meaningless **0 strikes** in all three arms
against *both* manifests. None finished (18.5 / 30.0 / 30.0 % of route). Two of
the five returned ULogs never armed at all; one is 1.07 GB of a parked vehicle
loitering for 4533 s.
**The cheap tell is `b2-score`'s `min AGL (from trunk bases)`** — it read
30.15 / 29.85 / 29.87 m, dead flat 30, against ~12 m for the real route. Check it
on every arm. Fixed path + verification checklist: RUNBOOK §4.11. The worlds and
trunk manifests are still valid (max 46–54 m from the current route), so **only
the `.plan` needs re-`scp`ing.**

🐛 **Found in passing, real, and NOT fixed: `perception.Trunk` is 2D — it has
`x, y, radius` and no vertical extent**, and `trunk_manifest` says outright that
"the 2D avoidance layer ignores heights". So `perception.sense()` reacts to every
trunk horizontally **regardless of the drone's altitude**: flying 15–25 m over the
canopy, the executor still dodged the forest beneath it, which is the likely
reason all three arms wandered off-route and stalled. This did **not** affect the
original B2 arms (they flew below canopy, where every sensed trunk really is at
flight altitude), so it is not a blocker for a correct re-fly — but it means
avoidance can never use "climb over it", and it will keep reacting to obstacles it
has already cleared vertically. `ForestManifest` already carries `base_z_m` /
`top_z_m` and a vertical containment test, so the data is there; only `sense()`
ignores it.

## Caveats / what's NOT done
- `reactive.py` is a **2D horizontal kinematic** sim (no physics, no vertical
  avoidance) — a fast design tool, not fidelity. Real fidelity = PX4 SITL + Gazebo
  (RUNBOOK) on the Ubuntu VM. 🔴 **B2 proved it is not merely low-fidelity but
  predicts the WRONG FAILURE MODE**: with instant velocity response a blocked
  drone hovers, so the harness forecasts paralysis where the aircraft collides.
  **It needs `MPC_ACC_HOR`/`MPC_JERK_AUTO` limits before any avoidance number it
  produces is worth quoting** — including the `--density` sweep.
- `perception.py`'s 31 rays over a 90° FOV is **3.00° spacing**, which resolves a
  0.3 m trunk only to ~6 m (100 % → **60.9 %** detection by 15 m). That, not
  `sensor_range_m` (30 m), is why `react_distance_m` could not simply be raised.
  It is a limit of the ray model, not of optics: 1280 px over 90° is 0.070°/px.
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
- **The tether speed policy over real turns** — every *planned* route is too
  smooth (≤16° bends) for the policy to engage. It has now been flown on a
  **synthetic** 20–160° course (acceptance §3, `sim-logs/4.7/`) and **did nothing
  measurable**: −0.4 % on the peak turn transient for 2.19× flight time, because
  PX4 already decelerates into corners on its own. So the policy is unexercised on
  real routes *and* unvalidated as a benefit on any route (see `tether.py`).
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
    them.
  - ✅ **RECALIBRATED 2026-07-27 — `tether.py` now scores through the measured PX4
    corner prior (`px4_profile.py`), and the corrected model reproduces the
    flight to ~1 %.** Two columns, both measured from the §3 control arm, and
    **both are needed**:
    * **corner speed** vs bend (6.00 m/s at 0° → 0.46 at 160°) — PX4 decelerating
      into the corner uncommanded. Dominates the *drag* term.
    * **lateral accel** vs bend — which is *not* monotone: it peaks at 2.87 m/s²
      near 45–50° (just under PX4's 3.0 `MPC_ACC_HOR` default) and collapses to
      ~0.9 at a reversal, because PX4 nearly stops and pivots rather than carving
      one. Dominates the *turn* term, and it is why the tether transient peaks at
      **moderate** bends while reversals are the safest corners.

    Calibrating speed alone leaves the turn term ~4× *low*; calibrating both:

    | quantity | old model | recalibrated | **flown (`sim-logs/4.7/`)** |
    |---|---|---|---|
    | peak applied \|F\| | 2.07 N | **0.765 N** | 0.7719 / 0.7732 N |
    | peak turn transient | 1.31 N | **0.311 N** | 0.3150 / 0.3164 N |
    | fiber snaps predicted | 6 | **0** | 0 |
    | policy vs no-policy | 2.07 vs 0.88 N | **identical** | −0.4 % |

    So the corrected model **predicts the §3 null that falsified the old one** —
    the strongest validation available short of a new flight. Also fixed: the turn
    term now uses the deflection the *payout* sees (`a_lat·L/v²`, the same bridge
    the Gazebo plugin applies in the loop) instead of the plan's vertex angle —
    they differ hugely, a 20° vertex was flown as a **49°** per-sample deflection.
    `tether-check --no-px4-prior` reproduces the old numbers and says so in its
    banner. ⚠️ **Calibrated at one cruise speed (6.0 m/s), one airframe, default
    PX4 params, flat world** — whether the corner cap is absolute or scales with
    cruise is **UNRESOLVED**; RUNBOOK §4.8 is the experiment that settles it.
    ⚠️ The parity gate is untouched and still EXACT on all real logs (it re-derives
    from *logged* speed/deflection, so it never goes through the prior).
  - 🔴 **"The policy earns its keep on reversal-heavy routes" is now MEASURED
    FALSE in the loop.** That claim rested on the unit test and the offline model.
    Acceptance §3 flew a deliberately reversal-heavy course (20–160°) and the
    policy moved the peak turn transient by **−0.4 %** (0.3164 → 0.3150 N) for
    **2.19× flight time** — because PX4 decelerates into corners on its own, and
    because slowing makes it corner *tighter*, which cancels the `v²` gain. Worse,
    the offline model over-predicts 7–18× at sharp bends and **under-predicts
    2–5× at shallow ones**, where the real in-loop peak actually is. Full result
    and both mechanisms: the §3 sub-entry under the tether plugin below
    (`sim-logs/4.7/`). **`tether-check`'s corner numbers should not be quoted as
    flight-representative until the model carries PX4's corner deceleration.**
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
  - ✅ **Gazebo C++ tether wrench plugin — DONE, all three acceptance items flown
    (2026-07-24 → 26).** §1 parity EXACT, §2 disturbance confirmed with a scaling
    law, §3 flown 2026-07-26. Sources in
    [`sitl/tether_plugin/`](sitl/tether_plugin/README.md); requirements/acceptance
    in [`sitl/TETHER_PLUGIN.md`](sitl/TETHER_PLUGIN.md); VM procedure in RUNBOOK §4.
    Built against gz-sim8, loaded, flew 77/77.
    🔴 **§3 returned a NEGATIVE: the speed policy does nothing measurable in the
    loop** (peak turn transient 0.3150 vs 0.3164 N, −0.4 %, for 2.19× flight
    time). The *plugin* is sound — the finding is about the **policy** and the
    offline model behind it. See the §3 sub-entry below.
    - ✅ **§2 A/B + scaling PASSED, and it is NOT null (`sim-logs/abtest/`).**
      Clean arms this time — `ab-off` genuinely recorded `enabled=0` (the
      stale-server + unexported-env fixes held). Cruise pitch vs applied force,
      one model (airframe aero drag 1.33 N from the OFF baseline + tether force,
      pitch = `atan((1.33+F)/mg)`), four force levels:

      | arm | applied | predicted pitch | measured | err |
      |---|---|---|---|---|
      | off (enabled=0) | 0 N | 3.87° | 3.87° | (baseline) |
      | on, span 40 | 0.76 N | 6.07° | 5.98° | −0.09° |
      | span 400 | 3.15 N | 12.85° | 12.38° | −0.47° |
      | span 4000 | 18.1 N | 44.7° | 43.9° | −0.82°* |

      *saturated at the 45° `MPC_TILTMAX_AIR` limit. The model tracks the airframe
      to **≤0.5° unsaturated** across a 24× force range — far stronger evidence
      than any single A/B.
    - 🧭 **Correction: my repeated "§2 will be near-null, PX4 trims it out" was
      WRONG.** Even the 0.76 N default tether is clearly visible — **+2.1° steady
      pitch above the disabled baseline** (5.98° vs 3.87°), well over the ~0.3°
      attitude noise. Why: a multirotor produces horizontal force **only by
      tilting**, and the position controller must *hold* that tilt against a steady
      disturbance — there is no integrator that removes it. A constant force shows
      as a constant attitude offset, not a transient the loop absorbs. (One
      unmatched ULog, `18_41_25.ulg`, has no CSV/fly-log — a scrapped run; ignore.)
    - ✅ **§4.4 bring-up PASSED — `AddWorldForce` lands.** The span=4000 arm
      (`sim-logs/tetherhuge/`, header `airborne_span_m=4000 enabled=1`) logged
      ~18 N and the airframe **pitched to 44°** (theory: `atan(18/(2·9.8))` =
      42.6°), pegged near the 45° limit, and cruise dropped 6.0 → 4.9 m/s. So the
      wrench reaches the body at the right magnitude. ⚠️ **My "it will lose the
      route / mission fails" prediction was wrong** — the x500 has enough thrust
      to fly at 45° tilt and it completed 77/77. "Completed the trip" is *not*
      evidence the force is absent; the tilt/speed signature is what proves it
      lands.
    - ✅ **§1 parity EXACT** on that real log: applied force (back+drag) matches
      `tether.py` **bit-for-bit** (`droneflight tether-plugin-check` → EXACT).
    - 🐛 **…but that took fixing a false-DIVERGED in the gate itself.** The real
      log first failed parity by **3.55e-15 N** on 9/13644 samples — pure
      float noise: Python squared speed with `**2` (a `pow()` call) while the C++
      used `v*v`, and `pow(v,2) != v*v` by 1 ULP for some values. Fixed by
      squaring with a multiply in `tether.py` (drag + turn), matching the C++, so
      the applied force is bit-exact again. Separately, the **turn** term can
      *never* be bit-exact across the boundary (`math.cos` vs `std::cos` are
      different libms), so it is now held to a 1e-9 N noise floor instead of `==0`
      — it is a *logged, not injected* diagnostic, so this doesn't weaken the gate
      on the force that's actually applied. The 1-ULP tamper test still has teeth
      (it tampers tension, still `==0`). Regression:
      `test_squares_with_multiplication_not_pow_for_cpp_parity`.
    - 🧭 **Lesson: bit-exact `==` across a Python/C++ boundary is only achievable
      for pure arithmetic.** Any transcendental (`cos`, `sin`, `exp`) will differ
      by ≤1 ULP between libms; gate those to a noise floor, keep `==` for the
      arithmetic. The offline Mac test grid passed `==` only because it never hit
      the value/`cos` combinations a real flight does.
    - Two operational bugs also surfaced and are fixed — see below.
    - 🐛 **The first "OFF" control arm was not off — it recorded `enabled=1`
      (caught by the CSV header, exactly as designed).** Two causes, both fixed in
      `run_sitl.sh`: (1) it never `export`ed `DRONEFLIGHT_TETHER_ENABLED`, and
      (2) a stale Gazebo server from the previous ON arm was reused — PX4 attaches
      to a running gz server and the plugin reads its env **once in Configure**,
      so the leftover server reused ON settings. `run_sitl.sh` now exports the
      tether vars and kills stale `gz sim` servers before launch. **Re-fly both
      A/B arms.**
    - 🐛 **The load banner was invisible** — emitted with `gzmsg` (Gazebo INFO,
      verbosity ≥ 3), which PX4 runs below, so a healthy run printed nothing while
      failures (`gzerr`) printed. Now `gzwarn`. The authoritative load check is
      the CSV header's `enabled=` field, not the console (docs corrected).
    - 🆕 **`airborne_span_m` now takes an env override `DRONEFLIGHT_TETHER_SPAN`**
      so the §2 sweep (40 → 400 → 4000) is a one-shot env var, not an SDF edit
      between arms — the same edit-per-arm hazard that produced the bad OFF run.
    - What is and isn't verified:
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
    - 🔴 **Acceptance §3 (speed policy in the loop) — FLOWN 2026-07-26, and it
      RETURNED A NEGATIVE (`sim-logs/4.7/`).** The A/B is valid and the policy's
      predicted benefit **did not appear in the loop**. Synthetic turn-rich course
      `plans/slalom_test.plan` (`sitl/make_slalom_plan.py`): 32 wp, 120 m legs,
      turns 20–160°, constant 30 m — the planner can't supply one (any-angle +
      fillet keep every vertex <25°; droneplanner SHORTFALLS cross-cutting).
      Both arms **COMPLETED 32/32**, tether ON in both, parity **EXACT** in both.

      | | offline predicted | policy arm | cruise arm (`--no-speed-policy`) | Δ |
      |---|---|---|---|---|
      | **peak `turn_n`** | 0.88 vs **2.07 N** | **0.3150 N** | **0.3164 N** | **−0.4 %** |
      | mean corner `turn_n` | — | 0.1478 N | 0.1724 N | −14 % |
      | peak applied \|F\| | — | 0.7719 N (39 %) | 0.7732 N (39 %) | −0.2 % |
      | fiber snaps | 6 predicted, control arm | 0 | **0** | — |
      | cross-track max | — | **2.22 m** | 1.45 m | policy *worse* |
      | flight time | — | 1612 s | 736 s | **2.19×** |

      Logs: `s3-policy.{csv,log}` + `23_09_30.ulg`, `s3-cruise.{csv,log}` +
      `23_40_58.ulg`. The −14 % mean / −0.4 % peak is stable across corner windows
      of 10–60 m, so it is not a windowing artifact. The policy lowered the
      transient at **22/30** corners but never at the one that sets the peak.
    - ✅ **The arms are genuinely different — verified from the ULog, not trusted
      from the flag.** `position_setpoint_triplet.cruising_speed` shows the policy
      arm commanded the taper (`6.0, 5.79, 5.17, 4.33, 3.5, 2.67, 1.62, 1.0, 1.0,
      1.62 …`, min 1.0) and the control arm commanded **6.0 at all 32 waypoints**.
      Mean moving speed 2.39 vs 5.36 m/s. This is a real null, not a broken run.
    - 🔬 **Why it's null — mechanism 1: PX4 already IS a speed policy.** The
      control arm was commanded 6.0 m/s everywhere and *still* decelerated into
      every corner, scaled by bend angle — the same shape our policy implements:

      | plan bend | control-arm speed at the vertex |
      |---|---|
      | 0–35° | 4.39 m/s |
      | 35–60° | 3.07 |
      | 60–100° | 2.07 |
      | 100–140° | 1.22 |
      | 140–180° | **0.51** |

      So the offline model's premise — that a no-policy vehicle carries cruise
      through a 160° reversal — is **false in AUTO_MISSION**. There is no
      "flat cruise profile" for a multirotor mission to protect against.
    - 🔬 **Mechanism 2: slowing tightens the corner, which cancels the gain.** The
      plugin bridges waypoint-angle → continuous trajectory as
      `deflection = a_lat·L/v² = L/r`, while `turn_n ∝ v²(1−cos θ)`. At the
      transient peak the policy arm's turn radius is **5.3 m** vs the control
      arm's **13.6 m** — 2.6× tighter, so its deflection is 2.6× larger and the
      two effects fight to a draw. Slowing down does **not** monotonically reduce
      the in-loop transient, because PX4 spends the speed on a tighter turn.
    - 🔴 **The offline model is wrong in BOTH directions — and optimistic exactly
      where the peak is.** Against what was actually flown, `tether-check`'s
      waypoint model **over-predicts 7–18× at bends ≥100°** (wp19, 155°: predicted
      1.29 N, flew 0.070 N) and **under-predicts 2–5× at bends ≤35°** (wp15, a 20°
      vertex, logged a **49° per-sample deflection** at 5.06 m/s — the vehicle's
      heading excursion through a corner exceeds the geometric turn). The real
      in-loop peak lives at **45–50° bends**, where the policy barely slows and
      offline predicted only 0.24 N. Sharp reversals — the thing the policy is
      built around — are the *safest* corners in flight, not the worst.
    - ⚖️ **Verdict: §3 does not pass as written.** The corner transient did not
      fall. The fiber stayed under budget in both arms, but the control arm proves
      the policy is not what kept it there — 2.19× flight time bought ~0 peak
      reduction and slightly worse tracking. For a battery-limited *laying* drone
      that is a bad trade as currently tuned. **The plugin (§1 + §2) is done and
      sound; it is the speed policy that now has an open defect.**
    - ⚠️ **Read it narrowly.** `turn_n` is **logged, not injected**, so this is a
      quantity derived from real flown kinematics, not a force the airframe felt —
      the tether transient never fed back into the flight. And it is one
      flat-world run on a synthetic course with un-benched coefficients
      (Tech-Selection §7). What it does establish is the *flown speed and
      curvature profile*, which is real and is what the offline model gets wrong.
    - 🧭 **Consequence for the roadmap: the planner-side curvature-window trigger
      is now the WRONG fix to do first.** It would only make a miscalibrated
      policy fire more often. `tether.py` needs PX4's own corner deceleration in
      it (measure `v(bend)` from these two ULogs and use that as the corner-speed
      prior) before the trigger is worth building. Recorded in droneplanner
      SHORTFALLS cross-cutting.
    - 🧪 **A/B is one env var, not an SDF edit:** `DRONEFLIGHT_TETHER_ENABLED=0|1`
      and `DRONEFLIGHT_TETHER_CSV=<path>`. A disabled run still loads and still
      logs `enabled=0`, so a control arm is identifiable from its own artifacts —
      the lesson from the F/G filename scramble. **No `[tether]` line in the
      console = the plugin did not load; do not score that run.**
  - ✅ Failsafe-tolerance run — flown (RUNBOOK §3f).
  - Cheap offline wins: re-run the MC with forest-frac and stale-frac varied
    *separately* (the flown conflict arm showed only forest-frac doing damage),
    and settle the `DriftWalker` √distance question above.
