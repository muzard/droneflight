# droneflight runbook — SITL, HITL, flight

How to actually run the flight stack. The offline `validate` path runs anywhere;
**live execution needs PX4 SITL + Gazebo**, which does not run on macOS — use an
**Ubuntu 24.04 (arm64) VM** on the M4 (UTM is free; Parallels is smoother). ROS 2
Jazzy, PX4 SITL, and Gazebo Harmonic all have native arm64 builds and run without
emulation on Apple silicon. Do **not** use amd64 Docker images for Gazebo (qemu
emulation is far too slow to render).

## 0. Offline (macOS, no sim) — works today

```bash
pip install -e ".[dev]"
droneflight validate --plan /path/to/mission.plan          # policies + sensor mode
droneflight validate --plan /path/to/mission.plan --night  # sensor tradeoff
```

## 1. Ubuntu VM: install the sim stack

```bash
# ROS 2 Jazzy (LTS) — https://docs.ros.org/en/jazzy/Installation.html
# PX4 SITL
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh
# Gazebo Harmonic (gz-harmonic) — https://gazebosim.org/docs/harmonic/install
# MAVSDK server ships with the `mavsdk` pip package used here.
```

## 2. Launch PX4 SITL + Gazebo

```bash
cd PX4-Autopilot
make px4_sitl gz_x500                 # x500 quad in Gazebo Harmonic
# PX4 exposes MAVLink on udp://:14540 by default (what droneflight --url targets).
```

For **GPS-denied** flight, configure PX4 to fuse external vision instead of GPS
(EKF2): see `configs/px4/gps_denied_ekf2.params` and load it via QGroundControl
or `param load`. In SITL you can feed a simulated vision pose; on the aircraft
this comes from the companion computer (VIO/VTRN).

## 3. Fly a planned mission

```bash
# In another terminal (VM or macOS host reaching the VM's UDP):
droneflight fly --plan /path/to/mission.plan --url udp://:14540
```

The executor uploads the mission (tether-safe speeds applied), arms, starts, and
monitors progress, returning to launch on error.

## 4. Custom tether model (required build item)

No standard sim models a **trailing fiber** (Technology-Selection §4.2). Approximate
it first as a **drag + back-tension disturbance force** on the airframe
(magnitude ∝ paid-out length and speed), added as a Gazebo plugin/wrench, to
study payout effects on flight and validate the speed policy. A full catenary
cable model is a later refinement.

## 5. HITL & beyond

- **HITL:** run the real flight controller + companion computer against the sim
  to catch timing/interface issues the pure-software loop hides.
- **Perception (separate host):** VIO/VTRN and photoreal validation (Isaac Sim)
  need an NVIDIA RTX GPU on x86_64 Linux — **not** the Mac. Benchmark VIO drift on
  EuRoC/TUM/UZH before flying.

## Notes / gotchas

- Keep the sim host awake during long runs.
- GPS-denied: gate readiness on *local* position health, not global (the adapter
  already does).
- Export/compliance: thermal sensors, tactical IMUs, and the integrated autonomy
  stack may be ITAR/EAR-controlled — see Technology-Selection §8 before sharing.
