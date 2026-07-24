# Gazebo tether wrench plugin (v0)

The **in-the-loop** half of the trailing-fiber build item. Applies the payout
back-pull to the airframe inside PX4 SITL so the controller has to hold against
it — the disturbance the offline model
([`tether.py`](../../src/droneflight/tether.py)) can only predict.

Requirements and scope: [`../TETHER_PLUGIN.md`](../TETHER_PLUGIN.md).
Procedure: [`../../RUNBOOK.md`](../../RUNBOOK.md) §4.

## What it does

Every physics step, on the vehicle's base link:

```
F = back_tension_n + 0.5·ρ·Cd·d·S·|v_h|²        directed opposite horizontal travel
```

`S` is the **airborne span** (~flight altitude), not total paid-out length — the
laid fiber is deposited, not towed. Below `speed_floor_ms` the pull is pure
back-tension along the last travel direction; a vehicle that has never moved gets
**zero** force (no line out yet).

The **turn transient** (`turn_n`) is computed from the link's lateral
acceleration and logged, but **not injected**. v0 is a steady disturbance; the
transient is the metric the speed policy is judged against.

## Layout — and why it is split this way

| File | Needs Gazebo? | What |
|---|---|---|
| `tether_math.hpp` | no | all the physics: tension terms + `EvaluateStep` |
| `tether_log.hpp` | no | the CSV format (header, rows, footer) |
| `tether_probe.cc` | no | test fixture: drives both from stdin |
| `TetherPlugin.hh/.cc` | **yes** | entity lookup, `AddWorldForce`, SDF params |
| `CMakeLists.txt` | partly | builds the probe anywhere, the plugin only on the VM |
| `install_tether_plugin.sh` | no | attaches the `<plugin>` block to PX4's x500 SDF |
| `x500_tether_snippet.sdf` | — | that block, with every default spelled out |

The Mac has no Gazebo, so everything that *can* be tested there is kept out of
the plugin. What remains in `TetherPlugin.cc` is only the two things that
genuinely need a running sim. `tests/test_tether_plugin.py` compiles the probe
and diffs **every** number against `tether.py` — exactly, not within a
tolerance. Logic that migrates back into the plugin becomes untestable, so
don't move it.

## Build (Ubuntu VM)

```bash
cd ~/droneflight/sitl/tether_plugin
cmake -B build -S . && cmake --build build -j
```

On the Mac the same command builds only `tether_probe` and says so — that is
expected, and it is what the pytest suite exercises.

## Install into PX4

```bash
./install_tether_plugin.sh ~/PX4-Autopilot
```

Idempotent; keeps a pristine copy at `model.sdf.orig`; `--remove` reverts it
exactly. It patches the **x500 model SDF**, not the world SDF — `make px4_sitl
gz_x500` spawns the vehicle into the world at runtime, so a world file has no
x500 to attach to and a `<plugin>` block placed there is a silent no-op.

`run_sitl.sh` adds `build/` to `GZ_SIM_SYSTEM_PLUGIN_PATH` automatically when it
exists, and prints whether it did.

**Always confirm the plugin loaded** before scoring a run — the console must
show:

```
[tether] ENABLED on x500/base_link -- back_tension_n=0.5 airborne_span_m=40 ...
```

An unresolved plugin does not stop the sim. A run with no `[tether]` line is a
no-tether run wearing a tether label.

## Running an A/B

Both arms use the same SDF; only the environment changes, so nothing has to be
edited between runs:

```bash
DRONEFLIGHT_TETHER_ENABLED=1 DRONEFLIGHT_TETHER_CSV=/tmp/tether-on.csv  ./run_sitl.sh <plan>
DRONEFLIGHT_TETHER_ENABLED=0 DRONEFLIGHT_TETHER_CSV=/tmp/tether-off.csv ./run_sitl.sh <plan>
```

A disabled run still loads, still logs, and records `enabled=0` in its CSV
header — a control arm has to be identifiable from its own artifacts.

## Scoring a run

```bash
droneflight tether-plugin-check --log /tmp/tether-on.csv
```

Re-derives every logged tension from `tether.py` and demands an **exact** match.
Exit 0 = the in-loop force *is* the offline model. Exit 1 = they have diverged;
do not score the flight until that is resolved. It also reports the applied peak
as a fraction of the break budget, and the logged (uninjected) turn transient.

## Known simplifications (v0)

- **Force only, no torque.** The pull is applied at the centre of mass. A real
  fiber leaves the spool at an offset and would also produce a moment.
- **Horizontal only.** No vertical drag component.
- **No cable dynamics** — no catenary, no wave propagation, no slack/snatch, no
  snag. A snag that couples the *drone's* mass into the line is the fiber-break
  mechanism that actually matters, and it is a planning/avoidance problem, not a
  speed one. Nothing here models it.
- **Coefficients are un-benched** (Technology-Selection §7). Trust the
  comparisons (plugin on vs off, cruise vs corner), not the absolute newtons.
