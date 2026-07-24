# sitl/

Simulation assets for PX4 SITL + Gazebo Harmonic. Run instructions live in
[../RUNBOOK.md](../RUNBOOK.md).

## `run_sitl.sh` — launch at the plan's home

`./run_sitl.sh <mission.plan> [PX4-Autopilot dir]` starts `make px4_sitl gz_x500`
with `PX4_HOME_*` set to the plan's home (via `droneflight sitl-env`), so the
uploaded mission lands at the real AO instead of PX4's Zurich default — the
number-one first-flight trap. Run it **on the Ubuntu VM** (untested from macOS;
the home-injection logic it calls is unit-tested).

## `tether_plugin/` — trailing-fiber disturbance (required build item)

No off-the-shelf sim models a drone paying out a fiber (Technology-Selection §4.2),
so this is a build item, staged in complexity:

1. **v0 — disturbance force.** ✍️ **Written (2026-07-24), not yet compiled or
   flown** — needs the VM. A Gazebo Harmonic system plugin applying a
   back-tension + airborne-span-drag wrench to the airframe, opposite to travel.
   Its physics is bit-exact against [`tether.py`](../src/droneflight/tether.py)
   and tested on the Mac; the Gazebo glue is unverified. See
   [`tether_plugin/README.md`](tether_plugin/README.md) and
   [`TETHER_PLUGIN.md`](TETHER_PLUGIN.md).
   ⚠️ Note the magnitude is **not** ∝ paid-out length as the spec assumed —
   only the airborne span drags, because the laid fiber is deposited, not towed.
2. **v1 — lumped-segment cable.** A chain of links/joints trailing the vehicle for
   catenary/sag behavior and ground/vegetation contact. Not started.

Sim-only: keep it out of the shippable flight package.
