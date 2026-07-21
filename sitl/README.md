# sitl/

Simulation assets for PX4 SITL + Gazebo Harmonic. Run instructions live in
[../RUNBOOK.md](../RUNBOOK.md).

## `run_sitl.sh` — launch at the plan's home

`./run_sitl.sh <mission.plan> [PX4-Autopilot dir]` starts `make px4_sitl gz_x500`
with `PX4_HOME_*` set to the plan's home (via `droneflight sitl-env`), so the
uploaded mission lands at the real AO instead of PX4's Zurich default — the
number-one first-flight trap. Run it **on the Ubuntu VM** (untested from macOS;
the home-injection logic it calls is unit-tested).

## Trailing-fiber tether model (required build item)

No off-the-shelf sim models a drone paying out a fiber (Technology-Selection §4.2),
so this is a build item, staged in complexity:

1. **v0 — disturbance force.** A Gazebo system plugin that applies a drag +
   back-tension wrench to the airframe, magnitude ∝ paid-out length × speed. Enough
   to study payout effects on flight and to validate `speed_policy` (slow through
   turns) in the loop.
2. **v1 — lumped-segment cable.** A chain of links/joints trailing the vehicle for
   catenary/sag behavior and ground/vegetation contact.

The plugin sources belong here once written; keep them out of the shippable
flight package (sim-only).
