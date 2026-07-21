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

## 2. Launch PX4 SITL + Gazebo **at the plan's home**

PX4 SITL spawns at a default home (Zurich) unless told otherwise. A planned
`.plan` is at the real AO, so you **must** set home from the plan or the uploaded
mission flies hundreds of km away (or is rejected). `sitl/run_sitl.sh` does this
by injecting `PX4_HOME_*` from `droneflight sitl-env`:

```bash
./sitl/run_sitl.sh /path/to/mission.plan ~/PX4-Autopilot   # x500 quad, home = plan home
# equivalently, by hand:
#   eval "$(droneflight sitl-env --plan /path/to/mission.plan)"
#   cd PX4-Autopilot && make px4_sitl gz_x500
# PX4 exposes MAVLink on udp://:14540 (what droneflight --url targets).
```

## 3. First flight — prove the pipe (GPS-enabled)

Do the **first** run with PX4's default GPS-enabled EKF2. This isolates the
executor/MAVSDK path (upload → arm → start → monitor → RTL) from the GPS-denied
external-vision variable, so bring-up failures have one cause at a time.

```bash
# In a second shell (inside the VM, or a host that can reach the VM's UDP):
droneflight fly --plan /path/to/mission.plan --url udp://:14540
```

The executor applies tether-safe speeds, uploads, arms, starts, and monitors
progress — and now **aborts to RTL if progress stalls** for `progress_timeout_s`
(default 60 s), so a hung SITL fails safe instead of blocking. Expect to shake out
mission-item encoding, altitude framing (plan is AGL/relative-to-home), and
arming gates here first.

## 3b. Then GPS-denied (external vision)

Once the GPS-enabled flight is clean, switch to the mission's real regime: load
`configs/px4/gps_denied_ekf2.params` (QGroundControl → Parameters → Load, or
`param load` in the pxh shell, then reboot) so EKF2 fuses **external vision**
instead of GPS. SITL then needs a vision pose fed in — that injection bridge
(streaming `nav.py`/`correction.py`'s truth+error as `VISION_POSITION_ESTIMATE`)
is the **next build item**; on the aircraft this pose comes from the companion
computer (VIO/VTRN). Gate readiness on *local* position health, not global (the
adapter already does).

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
