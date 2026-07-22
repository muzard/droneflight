# droneflight runbook — SITL, HITL, flight

How to actually run the flight stack. The offline `validate` path runs anywhere;
**live execution needs PX4 SITL + Gazebo**, which does not run on macOS — use an
**Ubuntu 24.04 (arm64) VM** on the M4 (UTM is free; Parallels is smoother). PX4
SITL and Gazebo Harmonic have native arm64 builds and run without emulation on
Apple silicon. Do **not** use amd64 Docker images for Gazebo (qemu emulation is
far too slow to render). Plan on running **headless** (§1.2) and driving it over
**SSH** (§1.7); ROS 2 is **not** needed for this MVP (§1.8).

## 0. Offline (macOS, no sim) — works today

```bash
pip install -e ".[dev]"
droneflight validate --plan /path/to/mission.plan          # policies + sensor mode
droneflight validate --plan /path/to/mission.plan --night  # sensor tradeoff
```

## 1. Ubuntu VM: install the sim stack

### 1.0 What runs where (don't install everything in the VM)

The `.plan` is the integration boundary — and also the VM boundary.

| Component | Where | Why |
|---|---|---|
| `droneplanner` | **Mac only** | Pure offline. Generate the `.plan` on the Mac, hand the *file* to the VM. |
| PX4-Autopilot, Gazebo, MAVSDK | **VM only** | Cannot run on macOS. |
| `droneflight` | **both** | Mac: dev/tests/`validate`/`nav`/`simfly`. VM: `fly` + `sitl-env` against PX4. |

⚠️ **Never reuse `DroneAutomation/.venv` inside the VM** — it holds macOS-arm64
binaries. The guest gets its own venv (see §1.7).

### 1.1 Create the VM

Any arm64 hypervisor works; all run Ubuntu natively on M-silicon (no emulation).

| Option | Cost | Notes |
|---|---|---|
| **UTM** (recommended) | free | Apple Virtualization backend. Gives you a desktop if you ever want one. |
| **Multipass** | free | `brew install multipass && multipass launch 24.04 --name px4 --cpus 6 --memory 8G --disk 80G` — fastest, headless-only. |
| **Parallels / VMware Fusion** | paid / free-personal | Smoothest integration. |

Get **Ubuntu 24.04 LTS (Noble), arm64** — the *server* ISO is fine (we run
headless). Size it generously: **6+ vCPU, 8 GB RAM, 80 GB disk** — a PX4 source
build is heavy. **Tick "Install OpenSSH server"** in the installer (see §1.7 —
headless means SSH is your main interface).

> **Do not** use amd64 images or amd64 Docker: qemu emulation is far too slow to
> render.

### 1.2 ⚠️ Plan on running headless

Gazebo's **3D acceleration does not work in Apple-Silicon VMs** — UTM's GPU-GL
renders a [black world](https://github.com/utmapp/UTM/issues/3122) and Parallels
renders garbage; the fallback is software rendering (llvmpipe), which is slow.
This is fine: we validate the mission from **telemetry, not pixels**, so run
`HEADLESS=1` (the default in `sitl/run_sitl.sh`). For a visual, use
**QGroundControl on the macOS host** — its 2D map view shows the vehicle tracking
the mission and needs no 3D in the VM (requires bridged networking; see §6).

### 1.3 Install PX4 + Gazebo Harmonic

Two paths. **Source build is the one to end up on** (needed to rebuild PX4 and to
develop the tether plugin); the pre-built `.deb` is a same-day shortcut if you
just want to see a mission fly.

**(a) Source build — recommended**

```bash
sudo apt update && sudo apt install -y git
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh    # installs deps + Gazebo
# log out/in (or reboot) so group/env changes apply, then:
make px4_sitl                                        # first build: slow (10-30 min)
```

*Forgot the log-out? For **SITL it doesn't matter.** That step exists for (a) the
`dialout` group — needed only to reach **real hardware** over USB/serial, not for
UDP-based SITL, and (b) `~/.local/bin` on `PATH` for pip-installed build tools —
and if the build completed, those were found. Just confirm the binary exists
(§1.5). Only re-login before touching a physical autopilot.*

⚠️ **Building with the droneflight venv active → `Failed to import em`.** CMake
resolves Python to whatever is first on `PATH`, so an active venv shadows the
system Python that `ubuntu.sh` installed PX4's build deps into. Since
`sitl/run_sitl.sh` *needs* the venv (it calls `droneflight sitl-env`), the clean
fix is to make one environment satisfy both:

```bash
. ~/venv/bin/activate
pip install -r ~/PX4-Autopilot/Tools/setup/requirements.txt
```

Use the requirements file, **not** `pip install empy` — PX4 pins `empy==3.3.4`
because empy 4.x broke the API. (Alternative: build PX4 with the venv
deactivated, and only activate it for `droneflight` commands.)

**(b) Pre-built package — fast start.** PX4 publishes SITL `.deb`s for Noble
**arm64** on its [Releases page](https://github.com/PX4/PX4-Autopilot/releases).
Add the OSRF repo first, then install:

```bash
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install ./px4-gazebo_*.deb
PX4_SIM_MODEL=gz_x500 px4-gazebo                     # run it
```

*(With path (b) there is no `make`, so `sitl/run_sitl.sh` doesn't apply — export
the `PX4_HOME_*` vars from `droneflight sitl-env` yourself before launching.)*

### 1.4 Install droneflight (+ MAVSDK)

```bash
sudo apt install -y python3-venv python3-pip
git clone <your droneflight remote>  # or share the folder from the Mac — see §1.6/§1.7
python3 -m venv ~/venv && . ~/venv/bin/activate   # venv lives in the GUEST fs
cd /path/to/droneflight && pip install -e ".[dev]"   # pulls mavsdk (Apache-2.0)
# without cd'ing, put the extras on the path:  pip install -e "/path/to/droneflight[dev]"
```

Quote `".[dev]"` — unquoted, the shell treats `[dev]` as a glob. `[dev]` adds
ruff/mypy/pytest; plain `-e .` (pydantic + mavsdk) is enough to *fly*, but the
test suite is a useful install check (§1.5). Re-activate the venv in every new
shell/tmux pane.

If the source is a host mount, keep the venv **off** the mount (as above) — only
the source is shared, since binaries differ per-OS.

⚠️ **`error: externally-managed-environment`** — Ubuntu 24.04 ships Python 3.12
under [PEP 668](https://pythonspeed.com/articles/externally-managed-environment-pep-668/),
which blocks `pip install` into the *system* Python. The venv above **is** the
fix: activate it first (`. ~/venv/bin/activate`) and pip works normally. Do
**not** reach for `--break-system-packages` — that pollutes the OS Python that
apt manages.

MAVSDK-Python ships the `mavsdk_server` binary per-arch; on arm64 it resolves to
the `linux-arm64-musl` build. If pip picks the wrong one, force it:

```bash
MAVSDK_SERVER_ARCH=aarch64 pip install --force-reinstall --no-cache-dir mavsdk
```

### 1.5 Verify before flying

```bash
ls -l ~/PX4-Autopilot/build/px4_sitl_default/bin/px4   # the build actually produced a binary
gz sim --versions                                      # Gazebo present
python -c "import mavsdk; print('mavsdk ok')"
pytest -q                                              # 126 passed => package is sane
droneflight validate --plan /path/to/mission.plan      # offline path works
groups | grep -q dialout && echo "dialout ok" || echo "no dialout (fine for SITL)"
```

### 1.6 Getting the code and a `.plan` into the VM — **git**

> ⚠️ **The single most common failure in this project is a stale VM.** Three
> separate SITL "bugs" turned out to be the VM running an older `run_sitl.sh` or
> world file. **`git pull` on the VM after every push**, and if behaviour
> contradicts the code you are reading, verify the VM copy *first*
> (`git log --oneline -1` on both ends).

Repos: `github.com/muzard/droneplanner` and `github.com/muzard/droneflight`.

**One-time VM setup.** Clone *both*: `droneflight` is the flight stack (installed);
`droneplanner` is cloned **only to deliver sim assets** — do **not** `pip install`
it in the guest, its geospatial stack is heavy and unnecessary there.

**Auth is SSH.** GitHub dropped HTTPS password auth, so `https://` clones fail
with *"Password authentication is not supported"*. Give the VM its own key:

```bash
ssh-keygen -t ed25519 -C "px4-vm" -f ~/.ssh/id_ed25519 -N ""
cat ~/.ssh/id_ed25519.pub    # -> GitHub > Settings > SSH and GPG keys > New SSH key
ssh -o StrictHostKeyChecking=accept-new -T git@github.com   # "Hi muzard! ..." = working
```

*(An account-level key covers both repos. Tighter but fiddlier: per-repo read-only
**deploy keys** — GitHub rejects the same key on two repos, so that needs two
keypairs plus `~/.ssh/config` host aliases.)*

```bash
cd ~
git clone git@github.com:muzard/droneflight.git
git clone git@github.com:muzard/droneplanner.git    # assets only, no install
cd ~/droneflight && . ~/venv/bin/activate && pip install -e ".[dev]"
```

The Mac uses SSH remotes too (`git remote set-url origin git@github.com:muzard/<repo>.git`).

**Routine sync** — after every push from the Mac:

```bash
cd ~/droneflight  && git pull
cd ~/droneplanner && git pull      # picks up regenerated worlds/DEMs
```

**What git carries vs what it does not:**

| Artifact | Via git? |
|---|---|
| `droneflight` source, `sitl/run_sitl.sh` | ✅ |
| `demo_ao/dtm.tif`, `demo_ao/worlds/*.sdf` | ✅ (tracked in droneplanner) |
| **`.plan`** | ❌ gitignored build artifact — `scp` it |

The world's DEM `<uri>` is an **absolute path** baked in at generation time
(`--dem-path`), currently `/home/arttudrone/droneplanner/demo_ao/dtm.tif` so it
resolves inside the VM's clone. If the VM layout changes, regenerate with
`droneplanner gz-world --dem-path <new path>`.

After pulling, install the world into PX4:

```bash
cp ~/droneplanner/demo_ao/worlds/planner_terrain.sdf \
   ~/drone/PX4-Autopilot/Tools/simulation/gz/worlds/
```

**The `.plan`** is a build artifact — copy it across:

```bash
scp <mac>:/path/to/DroneAutomation/plans/ev_test.plan ~/plans/   # from the VM, or scp from the Mac
```

For a *fixed* SITL regression plan that should travel with the repo, note
droneplanner's `.gitignore` whitelists `!configs/*.plan` — drop it in
`droneplanner/configs/` and commit it deliberately.

> Repos are on GitHub now: confirm they're **private**. This program's own
> compliance notes (Technology-Selection §8, and the gotcha below) say to review
> export/ITAR posture *before sharing* the integrated autonomy stack.

### 1.7 Working over SSH (the dev loop)

Headless means **SSH is the interface** — enable it. From the Mac:

```bash
ssh-copy-id user@<vm-ip>          # key auth, not passwords; `hostname -I` in the guest
# ~/.ssh/config:  Host px4vm / HostName <vm-ip> / User <user>   ->  ssh px4vm
```

It's a local VM on a private network; just don't port-forward 22 to the internet.
(Multipass users: `multipass shell px4` already works, and it manages keys.)

Two things that save real pain:

- **Run PX4 inside `tmux`** (`sudo apt install tmux`) so a dropped SSH session
  doesn't kill the sim mid-flight. Two panes: `run_sitl.sh` in one, `fly` in the other.
- **Mount the droneflight folder from the Mac** (UTM/Parallels shared folder, or
  `multipass mount ~/Claude/Projects/DroneAutomation/droneflight px4:/home/ubuntu/droneflight`)
  and create the venv **inside the guest filesystem**, pointing at the mount:

  ```bash
  python3 -m venv ~/venv && ~/venv/bin/pip install -e /path/to/mounted/droneflight
  ```

  An editable install over a host mount means code edited on the Mac takes effect
  in the VM immediately — no copying. That is the slice-1 loop (fix
  `mavsdk_connector.py` on the Mac, re-run in the VM). Keep the **venv off the
  mount** (binaries differ per-OS) — only the source is shared.

VS Code **Remote-SSH** works well as an alternative to the mount if you'd rather
edit in the guest directly.

### 1.8 Not needed yet: ROS 2

**ROS 2 Jazzy is *not* required for this MVP** — `droneflight` speaks MAVLink to
PX4 directly via MAVSDK. ROS 2 only becomes necessary for the later perception
phase (Isaac ROS / cuVSLAM), which needs x86+RTX hardware anyway. Skip it for now
and save yourself a large install.

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

## 2b. Fly over the planned terrain (not PX4's flat world) — ⏸️ PARKED

> **Parked 2026-07-21.** The terrain world loads and flies but the vehicle tumbles
> on translation (see the open-bug section below). It is **not on the MVP critical
> path**: the altitude-frame bug it was built to catch is already fixed and
> regression-tested, and the external-vision bridge injects error rather than
> sensing terrain, so it runs fine on the flat `default.sdf`. Resume here only
> when terrain-following, tether physics, or low-altitude margins need in-the-loop
> proof.

PX4's default Gazebo world is **flat**, so a terrain-aware `.plan` flown in it
proves nothing about nap-of-the-earth flight — and actively hides altitude-frame
bugs (a relative-to-home defect that put 57/77 waypoints underground survived a
green SITL flight for exactly this reason). Generate a world from the same DTM
the route was planned over:

```bash
# on the Mac (droneplanner has the DTM + rasterio)
droneplanner demo-ao --out demo_ao/
droneplanner plan     --mission demo_ao/mission.yaml --out out_demo/
droneplanner gz-world --dtm demo_ao/dtm.tif --plan out_demo/mission.plan \
                      --base-world <PX4>/Tools/simulation/gz/worlds/default.sdf \
                      --dem-path /home/<user>/droneplanner/demo_ao/dtm.tif \
                      --out demo_ao/worlds/planner_terrain.sdf

# copy the .sdf AND the .tif it references to the VM (§1.6), then:
cp planner_terrain.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
./sitl/run_sitl.sh ~/plans/mission.plan ~/PX4-Autopilot planner_terrain   # world = arg 3
```

⚠️ **Pass the world as arg 3.** If it is omitted, PX4 silently loads its **flat**
`default.sdf` and every terrain result is meaningless — the script prints a loud
warning when no world is selected, and hard-fails if the named world is missing.
Setting `PX4_GZ_WORLD=...` as a bare shell assignment does **not** work: it is not
exported to the subshell. Confirm with the launch line
`Starting gazebo with world: .../planner_terrain.sdf` and with `gz model --list`
showing a `terrain` model. Also kill stray `gz sim`/`ruby` processes between runs
— PX4 ignores the world selection if a simulator is already running.

The world sets `<spherical_coordinates>` to the plan's home so the Gazebo origin
and `PX4_HOME_*` agree, and offsets the heightmap so home lands on the origin.
The DEM `<uri>` is an **absolute path** — keep the `.tif` at that path on the VM,
or edit the URI (alternatively put it on `GZ_SIM_RESOURCE_PATH`).

**`--base-world` is required.** The terrain is injected into PX4's own world so
it inherits `<magnetic_field>`, `<atmosphere>`, ODE @250 Hz, and — critically —
gz-sim's *default plugin set*, which loads only when a world declares **no**
`<plugin>` elements. A from-scratch world silently loses all of that: every
sensor dies ("No valid data from Accel 0", "Found 0 compass"), and with no
magnetic field the EKF yaw diverges and the vehicle tumbles. The generator also
sets `<world name>` to the output **file stem** — mismatch there makes PX4 hang
forever on *"Waiting for Gazebo world..."*.

### ⚠️ Known open bug — terrain worlds tumble on translation

The vehicle spawns correctly, climbs perfectly, then **tumbles as soon as it
translates** (`Attitude failure (roll)`, roll → 180°, land detector latches,
mission stalls). Reproducible with any heightmap present; the flat `default.sdf`
flies fine.

**Already eliminated, with ULog evidence — do not re-investigate:**

| Hypothesis | Verdict |
|---|---|
| Heightmap fails to load / needs (2^n)+1 dims | ❌ `terrain` is in `gz model --list`; 160×160 is fine |
| Vertical datum wrong | ❌ at rest `z = -0.009`, `dist_bottom = 0.010` |
| Terrain misplaced | ❌ `alt - dist_bottom` ≈ 0 near home, as planned |
| Sensors missing | ❌ fixed by inheriting PX4's world |
| Attitude/control tuning | ❌ 18 s climb at 0.2° attitude error |
| Lockstep / heightfield CPU cost | ❌ `sensor_combined` dt median 4.00 ms, p99 4.00, 0 gaps >50 ms, cpuload 0.31 |

**Next experiment (built, not yet run)** — bisect shape vs. heightmap:

```bash
droneplanner gz-world ... --flat --out demo_ao/worlds/planner_flat.sdf
# copy planner_flat.sdf AND dtm_flat.tif to the VM, then fly planner_flat
```

* **stable over flat DEM** → it is terrain *slope/normals*. Note the ODE debug
  line reports `AABB max z = 150.32` (the DEM's max elevation) rather than the
  110.32 relief passed as `<size>` — the vertical-scale semantics are worth
  pinning down there.
* **still tumbles** → it is the heightmap collision itself, shape-independent:
  try a visual-only terrain (drop `<collision>`) to confirm, then a coarser DEM.

Interim workaround: fly PX4's flat `default.sdf` for anything that is not
terrain-following (the executor, speed policy, and the EV bridge all work there).

## 3. First flight — prove the pipe (GPS-enabled)

> **"It booted and then stopped doing anything" is correct behaviour.** Once you
> see `INFO [px4] Startup script returned successfully` and a `pxh>` prompt, PX4
> is up and **idling by design** — the vehicle stays grounded and disarmed until a
> client connects and commands it. The repeating `Preflight Fail: No connection to
> the GCS` is harmless spam. Nothing more happens until you run the `fly` command
> below in a second pane. (`MAVLink only on localhost` likewise just confirms you
> should run `droneflight` *inside* the VM.)

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

## 3b. Then GPS-denied (external vision) — the EV bridge

Once the GPS-enabled flight is clean, switch to the mission's real regime. EKF2
fuses **external vision** instead of GPS, and `droneflight ev-bridge` supplies
that vision: Gazebo's ground truth **corrupted by the same drift + VTRN model**
`droneflight nav` measures offline. PX4 then flies the corrupted estimate, so the
*true* track wanders by however much the navigation error says it should. On the
aircraft this pose comes from the companion computer (VIO/VTRN); here it is
injected, which is the whole point of the MVP — real perception needs the x86+RTX
box (Track B).

Use the **flat `default.sdf`**. Terrain is parked (§2b) and irrelevant here: the
error is injected, not sensed from the ground.

### 3b.0 ⚠️ Flatten the plan first (or it flies underground)

A terrain-following plan is relative **to home**, so wherever the ground drops
away from the launch point its altitudes go *negative* — the demo AO route dips to
**−41.9 m**. That is correct over real terrain and suicidal in a flat world, where
"below home" is underground. The mission would fail on the land detector for
reasons that have nothing to do with navigation.

`flatten-plan` keeps the horizontal route — the turns and corridor the concealment
planner chose, which is what the EV experiment measures — and removes the vertical
confound:

```bash
droneflight flatten-plan --plan out_demo/mission.plan --out ~/plans/ev_test.plan --alt 30
droneflight validate     --plan ~/plans/ev_test.plan     # expect "30.0 .. 30.0 m", no warning
```

`validate` now prints the altitude range and warns when waypoints sit below home,
so this cannot silently reach SITL again. **Never fly a flattened plan over real
terrain** — terrain-following is gone from it.

### 3b.1 Two MAVLink endpoints

`droneflight fly` binds `udp://:14540`, so the bridge needs its own endpoint —
one process per UDP port. In the `pxh>` shell, after PX4 boots:

```
pxh> mavlink start -x -u 14581 -r 4000000 -f -m onboard -o 14541
```

That is a second *onboard* instance: PX4 listens on 14581 and streams to 14541,
which is where `ev-bridge` connects (its `--url` default). Check with
`mavlink status` that two instances are up.

⚠️ **Two ports, not one.** MAVSDK-Python also spawns its own `mavsdk_server` on a
**fixed gRPC port** (default 50051) and does *not* check whether it is free, so two
droneflight processes silently fight over it — the second server fails to bind and
its client attaches to the *other* process's connection, quietly ignoring one of
the `--url` values. `ev-bridge` therefore defaults to `--server-port 50052`; leave
`fly` on 50051. Each concurrent MAVSDK process needs a distinct gRPC port.

### 3b.2 Parameters

> ⚠️ **You cannot feed a `.params` file to pxh at all.** The
> `<vid> <cid> <name> <value> <type>` text in `configs/px4/` is
> **QGroundControl's** format. PX4's own `param load` / `param import` read
> **BSON**, so pointing them at these files fails with
> `tinybson: node name overflow` as they try to parse the comment header. And
> `param load` would be wrong even in BSON: it **resets every parameter to default
> first**, so loading two files in sequence silently discards the first — you would
> get `NAV_DLL_ACT=0` with EKF2 **still fusing GPS**, a "GPS-denied" run that is
> nothing of the kind.
>
> Also: `pxh>` is PX4's own prompt, not a shell — no `$USER`, no `~`, no globbing.

With QGroundControl: Tools → Load from file, both files, then reboot. **Headless**
(the normal case here), generate console commands from the same files so the two
can't drift:

```bash
droneflight px4-params --file configs/px4/gps_denied_ekf2.params \
                       --file configs/px4/sitl_bringup.params
```

Paste its output into pxh — it ends with `param save` — then `reboot`:

```
param set EKF2_EV_CTRL 11
param set EKF2_GPS_CTRL 0
param set EKF2_HGT_REF 3
param set EKF2_EV_DELAY 50.0
param set COM_ARM_WO_GPS 1
param set MIS_TAKEOFF_ALT 5.0
param set NAV_DLL_ACT 0
param set SIM_BAT_DRAIN 0.0
param save
```

`gps_denied_ekf2.params` sets `EKF2_GPS_CTRL=0` (no GPS aiding), `EKF2_EV_CTRL=11`
(EV horizontal + vertical position + yaw — the fields
`VISION_POSITION_ESTIMATE` actually carries), `EKF2_HGT_REF=3` and
`COM_ARM_WO_GPS=1`.

**After the reboot, verify — and re-add the bridge's endpoint.** `mavlink start`
does not persist across a reboot, and a silently-reset parameter is the failure
mode this whole section is guarding against:

```bash
pxh> mavlink start -x -u 14581 -r 4000000 -f -m onboard -o 14541
pxh> param show EKF2_GPS_CTRL
pxh> param show EKF2_EV_CTRL
pxh> param show EKF2_HGT_REF
pxh> param show NAV_DLL_ACT
```

Expect `0`, `11`, `3`, `0` (`param show` takes one name/pattern, so ask one at a
time). If `EKF2_GPS_CTRL` is not 0, the params did not stick and every run from
here is GPS-aided — fix it before flying, not after.

### 3b.3 Order matters: bridge first, then fly

EKF2 has no position at all until vision arrives, so `droneflight fly` would sit
at its readiness gate forever. Start the bridge first and wait for it to report a
lock.

```bash
# shell A — PX4 SITL at the plan's home, flat world
./sitl/run_sitl.sh ~/plans/ev_test.plan ~/PX4-Autopilot

# shell B — the vision stream (VTRN fixes ON)
droneflight ev-bridge --plan ~/plans/ev_test.plan --url udp://:14541 \
    --record ~/logs/ev-vtrn.jsonl

# shell C — once shell B shows est-pub settling near 0, fly the mission
droneflight fly --plan ~/plans/ev_test.plan --url udp://:14540
```

Shell B prints a line every 5 s of sim time:

```
t=   35.0  truth E,N=(    12.4,   180.6)  injected err=   3.6 m  est-pub= 0.28 m
```

* **`injected err`** — how far the pose we publish is from truth. This is the
  GPS-denied error, and it should grow with distance and shrink at fixes.
* **`est-pub`** — PX4's own estimate minus the pose we just sent. **Near zero means
  EKF2 is fusing the external vision.** Large or growing means it is rejecting the
  stream; see the troubleshooting table. This is a *readback only* — it never
  feeds the error model.

### 3b.4 ⚠️ The global origin (expect this to be the first failure)

A `.plan` is lat/lon, but with `EKF2_GPS_CTRL=0` EKF2 never gets a global origin —
it normally takes one from the GNSS fix it is no longer allowed to use. Without an
origin the local frame is purely relative, `is_home_position_ok` never goes true,
and the mission is rejected before it starts.

`ev-bridge` therefore sends `SET_GPS_GLOBAL_ORIGIN` (the plan's home) via MAVSDK's
`mavlink_direct` at startup, and prints `EKF2 global origin set to …`. This is not
a sim hack: on the aircraft it is **VTRN, not GPS, that ties the local frame to the
world**, and this is that tie.

If your `mavsdk_server` is too old to have `mavlink_direct`, the bridge fails with
a clear error. Workarounds, in order of preference: upgrade `mavsdk`
(`pip install -U mavsdk`); or run `--no-set-origin` and send the message from
QGroundControl/MAVProxy; or accept a local-only demonstration and drive the route
in Offboard mode instead of AUTO (not built).

### 3b.5 The acceptance A/B

The claim to prove is *`nav.py`'s offline finding reproduces in real flight*: safe
forest flight needs bounded navigation, not just avoidance. Fly the same plan
twice, changing only the fixes:

```bash
# arm A — VTRN fixes ON (should hold the corridor)
droneflight ev-bridge --plan ~/plans/ev_test.plan --record ~/logs/ev-vtrn.jsonl
# arm B — dead reckoning only (should wander out)
droneflight ev-bridge --plan ~/plans/ev_test.plan --no-vtrn \
    --record ~/logs/ev-novtrn.jsonl
```

Each run prints the corridor result on the **true** (Gazebo) track at the end, and
`--record` keeps the track for later:

```bash
droneflight ev-report --plan ~/plans/ev_test.plan --record ~/logs/ev-novtrn.jsonl
```

Pass = arm A `WITHIN corridor`, arm B `LEFT the corridor`. ⚠️ It only means
something on a **non-degenerate plan** — the straight-line stub from
`mission.example.yaml` has no turns and nothing to hold (see CLAUDE.md).

### 3b.6 Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `no ground-truth topic for model 'x500_0'` | model/world named differently | `gz topic -l`; pass `--model` / `--world` (the error prints what it saw) |
| `est-pub` large or growing; `ekf2 vision data stopped` | message timestamp in the wrong clock domain | try `--timestamp epoch`, then `--timestamp sim` (default `none` = PX4 stamps on receipt) |
| `fly` never gets past its readiness gate | no vision yet, or no global origin | start `ev-bridge` **first**; check it printed `EKF2 global origin set to …` |
| `injected err` stays 0.0 | the vehicle has not moved, or truth is not updating | error accrues with distance flown; check `gz topic -e -t <topic>` by hand |
| vehicle drifts off immediately in a straight line | frame error (ENU/NED swap) | `ev-bridge --dry-run` and compare published N/E against `gz topic` truth |
| bridge and `fly` fight over the port | both on 14540 | second `mavlink start` instance (§3b.1); bridge on 14541 |
| `param ... failed (2)` in pxh | `$USER`/`~` passed literally — pxh is not a shell | type the absolute path |
| `tinybson: node name overflow` on `param import` | pxh reads BSON; `.params` is QGC text | `droneflight px4-params --file ...` and paste the `param set` lines (§3b.2) |
| run looks clean but suspiciously accurate | params never applied, or `param load` wiped them; GPS still fused | re-check `param show EKF2_GPS_CTRL` = 0 (§3b.2) |
| `mavlink status` shows one instance after a reboot | `mavlink start` does not persist | re-run it after every reboot (§3b.2) |

`ev-bridge --dry-run` reads truth and computes poses **without** publishing or
connecting to PX4 — the fastest way to check topic names and frames before
involving the autopilot.

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

## 6. Networking (where to run what)

**Default: run everything inside the VM.** PX4 SITL, `droneflight fly`, and the
plan all live in the guest, so `udp://:14540` just works with zero network
configuration. This is the recommended setup for the first flight — don't add
networking as a second variable while debugging the executor.

**Optional, once it flies: QGroundControl on the macOS host** for a 2D map view
(and easy param loading) without needing 3D in the VM. This needs the VM on
**bridged** networking so it has a LAN IP the Mac can reach, and PX4's MAVLink
stream pointed at the host (PX4 SITL broadcasts to `14550` for a GCS). Exact
setup varies by hypervisor — worth doing once the basic loop is green, not before.

## 7. Logs, results, and shutting down

### What IS logged automatically — PX4 ULog

PX4 writes a full **ULog** flight log to the simulated SD card:

```
~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/<time>.ulg
```

This is the authoritative record — EKF state, position estimates, setpoints, mode
changes, arming decisions, failsafes. It persists on the VM disk across reboots.
Read it with:

- **[PX4 Flight Review](https://review.px4.io)** — drag the `.ulg` in, get plots. Easiest.
- **pyulog** — `pip install pyulog`, then `ulog_info x.ulg` / `ulog2csv x.ulg`.
- **PlotJuggler** for live/offline plotting.

⚠️ **PX4 logs *while armed* by default** — so a run that fails preflight and never
arms may leave **no `.ulg` at all**. When debugging arming/bring-up, set
`SDLOG_MODE` to one of its log-from-boot options (check the param description in
QGC or `param show SDLOG_MODE`) so you capture the pre-arm phase too.

### What is NOT logged — `droneflight` console output

`droneflight fly` logs to the **console only** (`logging.basicConfig`, stderr);
the `FlightResult` is printed once and discarded. Nothing is written to a file.
Capture it yourself:

```bash
droneflight fly --plan ~/plans/ev_test.plan --url udp://:14540 2>&1 | tee ~/logs/fly-$(date +%F-%H%M).log
```

Or log a whole tmux pane continuously:

```bash
tmux pipe-pane -o 'cat >> ~/logs/px4-console.log'      # toggle off: tmux pipe-pane
tmux capture-pane -p -S -3000 > ~/logs/scrollback.log   # dump existing scrollback
```

Do this **before** a run you might want to ask about — console scrollback is
finite and a `tmux kill-session` takes it with it.

### Getting logs off the VM (for analysis on the Mac)

The PX4 pane can't capture itself (it runs `pxh>`, not a shell) — target it from
another pane, then pull everything to the Mac:

```bash
# in the VM
tmux list-panes -a                                            # find the id, e.g. px4:0.0
tmux capture-pane -t px4:0.0 -p -S -5000 > ~/logs/px4-console.log

# on the Mac — DroneAutomation/sim-logs/ sits outside both git repos
DROP=/Users/arttu/Claude/Projects/DroneAutomation/sim-logs
scp <vm>:'~/logs/*.log' $DROP/
scp <vm>:'~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/*.ulg' $DROP/
```

Decode a `.ulg` on the Mac with `pip install pyulog` → `ulog_info x.ulg`,
`ulog_messages x.ulg`, `ulog2csv x.ulg`.

### Shutting down

Safe to power off between sessions — everything (PX4 build, venv, clone, `.ulg`
logs) lives on the VM disk and survives.

- Stop PX4 first (`shutdown` at the `pxh>` prompt, or `Ctrl-C`), then
  `sudo poweroff` — a **clean** shutdown, not a force-kill of the VM window, so
  the filesystem stays consistent.
- **tmux sessions do *not* survive a reboot.** tmux persists across *SSH
  disconnects*, not across VM restarts — you'll recreate panes next session.
- **Suspend/pause the VM instead** (hypervisor save-state) for a much faster
  resume *and* tmux sessions stay alive. Preferred if you're coming back soon.

## Notes / gotchas

- **"Preflight Fail: No connection to the ground control station" is expected
  log spam, not a failure.** A GCS is *not required* to run SITL
  ([PX4 #20478](https://github.com/PX4/PX4-Autopilot/issues/20478)); PX4 prints it
  continuously when nothing is connected. Likewise most boot-time preflight
  warnings (power/RC/GPS) simply mean *"can't arm yet"*, not *"sim is broken"*.
  The sim is healthy if you get a **`pxh>` prompt**. For the precise, current
  arming state run **`commander check`** in that shell — that is the real
  diagnostic, and several checks clear once a GCS/MAVSDK link connects.
- **Snapshot the VM once §1.5 verifies green**, before flying/tinkering. The PX4
  toolchain install is ~an hour; a snapshot makes a bad `apt` or a broken build
  a 30-second rollback instead of a redo.
- Keep the sim host awake during long runs.
- GPS-denied: gate readiness on *local* position health, not global (the adapter
  already does).
- Export/compliance: thermal sensors, tactical IMUs, and the integrated autonomy
  stack may be ITAR/EAR-controlled — see Technology-Selection §8 before sharing.
