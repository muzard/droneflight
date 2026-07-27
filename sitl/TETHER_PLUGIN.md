# Gazebo tether wrench plugin — requirements

> ## Status (2026-07-26): **COMPILED, FLOWN, §1 + §2 PASSED**
>
> Sources are in [`tether_plugin/`](tether_plugin/) — see its
> [README](tether_plugin/README.md) for build/install/run.
>
> | | |
> |---|---|
> | Physics (`tether_math.hpp`) | ✅ bit-exact vs `tether.py` |
> | Log format (`tether_log.hpp`) | ✅ round-trips |
> | Gazebo glue (`TetherPlugin.cc`) | ✅ **compiled against gz-sim8, loaded, flew 77/77** |
> | §1 parity | ✅ EXACT on a real Gazebo log (`droneflight tether-plugin-check`) |
> | §2 disturbance | ✅ scaling law: pitch tracks force to ≤0.5° over 0 → 18 N (`sim-logs/abtest/`) |
> | §3 speed-policy in the loop | 🔴 **FLOWN 2026-07-26, NEGATIVE** — policy moved the peak turn transient −0.4 % (0.3150 vs 0.3164 N) for 2.19× flight time (`sim-logs/4.7/`, RUNBOOK §4.7) |
>
> **§3's negative is about the POLICY, not the plugin.** The plugin did its job:
> parity EXACT in both arms. What §3 showed is that PX4 already decelerates into
> corners on its own (a control arm commanded cruise at all 32 waypoints still
> flew ≥140° reversals at 0.51 m/s), and that slowing further makes it corner
> *tighter* — `deflection = a_lat·L/v² = L/r` — which cancels the `v²` reduction
> in turn tension. The offline waypoint model over-predicts 7–18× at bends ≥100°
> and under-predicts 2–5× at bends ≤35°, where the real in-loop peak sits.
>
> **The disturbance is NOT near the noise floor** (an earlier note here guessed it
> would be, and that was wrong): a multirotor holds a steady horizontal force by
> tilting, and the controller holds that tilt, so even 0.76 N is a clear +2.1°
> pitch bias above the disabled baseline. Full results + the silent-failure traps
> hit along the way (invisible banner, stale gz server, pow-vs-multiply parity):
> `droneflight/CLAUDE.md` → Slice 3, and RUNBOOK §4.

The in-the-loop half of the tether build item (RUNBOOK §4, Tech-Selection §4.2).
The offline model — `droneflight/src/droneflight/tether.py`, exercised by
`droneflight tether-check` — is the **reference this plugin must match**. The
plugin's job is to make PX4 SITL feel the payout disturbance so we can watch the
*controller's* response (attitude, altitude, cross-track) to a force the pure
kinematic study cannot show.

Scope is **v0: a drag + back-tension disturbance force**, not cable dynamics. No
catenary, no wave propagation, no slack/snatch, no snag. Those are later.

## What it must do

Each physics step, apply a **back-pull force** (a `wrench`, force only, no torque)
to the vehicle's base link:

1. Read the link's world linear velocity `v` (horizontal components).
2. Compute the tension magnitude with the **same formula and coefficients as
   `tether.py`** so the two never diverge:
   `F = back_tension_n + drag_tension_n(v)` where
   `drag_tension_n = 0.5 * rho * Cd * d * S * |v_h|^2`.
   `S` is the **airborne span** (`airborne_span_m`, ~flight altitude), **not**
   total paid-out length — the laid fiber is deposited, not towed (see the
   `tether.py` header for why this deviates from the spec's "∝ paid-out length").
3. Apply `F` directed **opposite to the horizontal velocity** (the line trails
   behind the drone). Below a small speed floor, apply only `back_tension_n` in
   the last travel direction, so a hovering drone isn't shoved by a zero-velocity
   singularity.

The **turn/reversal tension** (`turn_tension_n`) is a *validation metric*, not a
force to inject in v0 — it is the transient the speed policy is judged against
offline. The plugin may compute and log it from the link's acceleration, but the
force it applies is the steady drag back-pull. (A v1 could inject the transient.)

## Interfaces

- **Gazebo Harmonic (`gz-sim8`) C++ system plugin.** `ISystemConfigure` +
  `ISystemPreUpdate`. Register with `GZ_ADD_PLUGIN`.
- **Configure:** read the SDF `<plugin>` params (below); resolve the vehicle model
  and its base link entity; cache the `Link`.
- **PreUpdate (skip when paused):** read `components::WorldLinearVelocity`, compute
  `F`, apply via `Link::AddWorldForce()` (or an `ExternalWorldWrenchCmd`
  component). Log tension + speed at a throttled rate for post-run comparison.
- **SDF params** (names mirror `TetherConfig`, same SI units and defaults, so a
  run is reproducible against `tether-check`):
  `link_name`, `air_density_kg_m3` (1.225), `drag_coefficient` (1.2),
  `fiber_diameter_m` (0.00025), `airborne_span_m` (40.0), `back_tension_n` (0.5),
  `speed_floor_ms` (~0.3). Break/warn tensions are analysis-only; omit from the
  plugin.
- ~~**World wiring:** add the `<plugin>` block under the `x500` model in the world
  SDF (the same file `run_sitl.sh` launches).~~ **Corrected while building it:**
  the block goes in the **x500 *model* SDF**
  (`$PX4_DIR/Tools/simulation/gz/models/x500/model.sdf`), not the world SDF.
  `make px4_sitl gz_x500` spawns the vehicle into the selected world at runtime,
  so no world file contains an x500 to attach to — a `<plugin>` placed there
  loads nothing and reports nothing, which looks exactly like a working run.
  `install_tether_plugin.sh` patches the right file (idempotently, with
  `--remove`). Still airframe-agnostic: point it at any PX4 model SDF.

## Build (needs the VM — cannot compile on the Mac)

- Gazebo Harmonic + `gz-sim8` dev headers live only on the Ubuntu VM; the M4 Mac
  has no Gazebo. Build there.
- `CMakeLists.txt`: `find_package(gz-sim8)`, `find_package(gz-plugin2)`; build a
  shared lib; install to a dir on `GZ_SIM_SYSTEM_PLUGIN_PATH`.
- Add the source under `sitl/tether_plugin/`. Keep the tension math in one small
  header so it can be **unit-tested on the Mac against `tether.py`'s numbers**
  even though the Gazebo glue can't be — mirror how `ev_bridge`'s pure parts are
  tested without a sim.

## Acceptance — how we know it works

1. **Parity with the model.** At a commanded steady speed, the logged plugin
   tension equals `drag_tension_n(speed)` from `tether.py` to floating point.
   This is the gate: the in-loop force *is* the model, not a second copy.
   → **Runnable now:** `droneflight tether-plugin-check --log <csv>` re-derives
   every logged tension and exits non-zero on any difference. Already passing
   against logs the plugin's own C++ formatter writes (`tests/test_tether_plugin.py`);
   what remains is to point it at a log from a *real* Gazebo run.
2. **Visible, correct disturbance.** With the plugin on, a constant-heading cruise
   shows a small steady attitude/thrust bias to hold against the back-pull, and it
   scales with `v^2` and `airborne_span_m`. Off = no bias. Quantify it.
   → Fly `DRONEFLIGHT_TETHER_ENABLED=1` vs `=0` (same SDF, one env var) and
   compare mean roll/pitch in the ULog over a straight leg. **Expect this to be
   small or null:** 0.76 N on a ~2 kg x500 is ~2.2° of tilt at cruise, and PX4
   trims it out — so also sweep `airborne_span_m` (e.g. 40 → 400) to confirm the
   bias *scales*, which is the real evidence, rather than resting on one
   possibly-invisible difference.
3. **Speed-policy relevance in the loop.** On a route with real turns (see the
   flat-terrain caveat in `droneflight/CLAUDE.md`), compare cruise-everywhere vs
   the speed-policy speeds and show the disturbance transient at corners drops —
   the in-loop version of what `tether-check` shows offline.
   → ⚠️ **Blocked on a route, not on the plugin.** The only flown plan
   (`plans/ev_test.plan`) bends ≤16°, so the speed policy never engages and
   `tether-check` already reports the tether slack (peak 40 % of budget) there.
   This criterion needs a turn-rich plan first — the same gap the flat-world
   caveat describes. Until then §3 is untestable, and that is a planner problem.

## Do NOT

- Do not drive the force from PX4's estimate — use Gazebo truth velocity, same
  discipline as `ev_bridge`'s truth source.
- Do not scale drag by total paid-out length (the laid fiber is not in the
  airflow); use the bounded airborne span.
- Do not re-derive the coefficients — import/mirror `TetherConfig` defaults so the
  plugin and `tether.py` cannot drift apart.
