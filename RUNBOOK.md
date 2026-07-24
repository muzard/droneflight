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
pytest -q                                              # 141 passed => package is sane
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

### 🔧 Terrain worlds tumble on translation — ROOT-CAUSED AND FIXED (2026-07-23), verify on the VM

**The vehicle was spawning buried inside the heightmap.** Gazebo uses a DEM's
**raw elevations**, not a rescale into `<size>`'s z — the `AABB max z = 150.32`
clue was the DEM's max *elevation*, not the `110.32` relief we passed. The old
model-pose offset therefore put the terrain surface under home at world
`z = min_elev` (**+40 m** real DEM, **+125 m** with `--flat`) while PX4 spawns the
vehicle at `z = 0`. Sitting in permanent collision penetration looks like a clean
hover and then flings the airframe the moment it translates.

⚠️ **The old `--flat` bisection was misleading** — the flat world buried it
*deeper*, so "flat tumbles too, therefore not slope/normals, therefore the
heightmap itself" was the same bug measured twice.

**Fixed** in `droneplanner/io/gzworld.py` by handing Gazebo a **zero-based DEM**
(`normalise_dem`), which makes raw-vs-rescaled interpretations identical. Details:
`droneflight/CLAUDE.md` → Slice 2.

**To verify here:**

```bash
# regenerate — this now writes <out>_dem.tif NEXT TO the .sdf
droneplanner gz-world --dtm demo_ao/dtm.tif --plan out_demo/mission.plan \
    --base-world $PX4_DIR/Tools/simulation/gz/worlds/default.sdf \
    --out worlds/planner_terrain.sdf --dem-path /home/<user>/worlds/planner_terrain_dem.tif
# copy BOTH files to the VM, then:
cp worlds/planner_terrain.sdf $PX4_DIR/Tools/simulation/gz/worlds/
PX4_GZ_WORLD=planner_terrain ./sitl/run_sitl.sh <plan> $PX4_DIR
```

⚠️ **Copy the `_dem.tif` too**, and make `--dem-path` match where it will live on
the VM — the SDF references it by absolute path. A missing DEM loads an empty
heightmap.

Check, in order:
1. `gz model -m terrain -p` vs a height query — the surface under home must be at
   **z ≈ 0**. **Do not** infer this from PX4's local z: PX4 zeroes its local
   origin at the spawn point, so `z = 0` at rest is tautological (that circular
   reading is what hid this bug for two sessions).
2. Vehicle rests *on* the surface, not in it — no contact jitter at idle.
3. Then **fly a translation**. Tumble gone = fixed.

*Historical (what the tumble looked like):* the vehicle spawned correctly, climbed
perfectly, then tumbled as soon as it translated (`Attitude failure (roll)`,
roll → 180°, land detector latches, mission stalls). Reproducible with any
heightmap present; the flat `default.sdf` flew fine.

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
droneflight fly --plan /path/to/mission.plan --url udpin://0.0.0.0:14540
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

> ⚠️ **Re-run this after EVERY PX4 restart.** `mavlink start` is not persistent
> (unlike the parameters, which are). Without it the bridge binds 14541 and waits
> forever for a system that never appears — and because it never reaches PX4, the
> ULog shows nothing at all and `fly` later times out looking unrelated. The
> startup banner lists the live instances: if you do not see
> `udp port 14581 remote port 14541`, it is not running. The bridge now gives up
> after `--connect-timeout` (default 30 s) and says exactly this.

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
droneflight ev-bridge --plan ~/plans/ev_test.plan --url udpin://0.0.0.0:14541 \
    --record ~/logs/ev-vtrn.jsonl

# shell C — once shell B shows est-pub settling near 0, fly the mission
droneflight fly --plan ~/plans/ev_test.plan --url udpin://0.0.0.0:14540
```

**If shell B exits immediately, that is your answer** — it fails *before* touching
PX4, so the only trace is on its own console. The two common exits are `'gz' not
on PATH` (venv/shell without Gazebo's environment) and `no ground-truth topic for
model 'x500_0'` (started before Gazebo advertised it; the bridge now polls for
`--wait-topic` seconds, default 30). A bridge that never connected looks, from
PX4's side, exactly like `fly` hanging — because `fly` then waits 30 s for a
position that never arrives.

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
| `fly` never gets past its readiness gate | no vision yet, or no global origin | start `ev-bridge` **first**; check it printed `EKF2 global origin set to …`. The timeout now names the missing health flag |
| `fly` times out ~30 s after connecting, ULog shows 0 EV samples | the bridge exited before connecting — its error is on ITS console | run the bridge alone first; `--dry-run` proves truth without PX4 |
| `injected err` stays 0.0 | the vehicle has not moved, or truth is not updating | error accrues with distance flown; check `gz topic -e -t <topic>` by hand |
| vehicle drifts off immediately in a straight line | frame error (ENU/NED swap) | `ev-bridge --dry-run` and compare published N/E against `gz topic` truth |
| bridge and `fly` fight over the port | both on 14540 | second `mavlink start` instance (§3b.1); bridge on 14541 |
| mission aborts mid-leg, `no mission progress within 60s` | fixed watchdog shorter than a long leg | fixed: the timeout is now derived from the plan (override with `fly --progress-timeout`) |
| `param ... failed (2)` in pxh | `$USER`/`~` passed literally — pxh is not a shell | type the absolute path |
| `tinybson: node name overflow` on `param import` | pxh reads BSON; `.params` is QGC text | `droneflight px4-params --file ...` and paste the `param set` lines (§3b.2) |
| run looks clean but suspiciously accurate | params never applied, or `param load` wiped them; GPS still fused | re-check `param show EKF2_GPS_CTRL` = 0 (§3b.2) |
| `mavlink status` shows one instance after a reboot | `mavlink start` does not persist | re-run it after every reboot (§3b.2) |

`ev-bridge --dry-run` reads truth and computes poses **without** publishing or
connecting to PX4 — the fastest way to check topic names and frames before
involving the autopilot.

## 3c. The GPS-denied acceptance experiment (three arms)

**Status: ✅ all three arms FLOWN on the fixed code, 2026-07-22 — and they
separated.** Measured (MISSION phase, 15 m corridor): A `vtrn` **11.8 m WITHIN**,
B `novtrn` **103.2 m LEFT**, C `conflict` **25.6 m LEFT**. ULogs
`sim-logs/{14_53_15,15_16_47,15_33_55}.ulg`, records
`sim-logs/ev-{vtrn,novtrn,conflict}.jsonl`. Full analysis, mechanism and residual
defects: CLAUDE.md → Slice 3. The procedure below is retained for re-running.

*(An earlier 77/77 flight the same day is void as evidence — degenerate basemap
plus teleported fixes, both since fixed.)*

The three arms differ **only** in the basemap handed to the VTRN corrector.
Everything else — plan, seeds, params, ramp — is held constant, so a difference
in outcome can only come from the navigation.

| Arm | Basemap | Offline prediction (seed 0) | **Measured 2026-07-22** | Verdict |
|---|---|---|---|---|
| **A `vtrn`** | 40 % forest / 10 % stale | max dev **15.0 m**, 55/107 fixes | max dev **11.8 m**, 28/62 fixes | **WITHIN** |
| **B `novtrn`** | none — dead reckoning | max dev **106.4 m**, no fixes | max dev **103.2 m**, no fixes | LEFT |
| **C `conflict`** | 70 % forest / 30 % stale | max dev **21.9 m**, 25/107 fixes | max dev **25.6 m**, 10/68 fixes | LEFT |

Deviation prediction was good to ~4 m in all three arms. Absolute *attempt* counts
differ (62–68 flown vs 107 offline) because the offline study samples fix
opportunities on its own cadence — compare availability ratios, not raw counts.

⚠️ **Arm A is predicted at 15.0 m against a 15 m tolerance** — it is a coin-flip at
this seed, and over 40 seeds it fails 4 times. Do not treat a narrow pass or fail
as signal; it is one sample. Arm B is the one that must separate cleanly.

### Running an arm

Per arm: restart SITL, re-add the endpoint, run the bridge, then fly.

```bash
# pane A — SITL (params persist across restarts; the endpoint does not)
./sitl/run_sitl.sh ~/drone/plans/ev_test.plan ~/drone/PX4-Autopilot
#   at pxh>:   mavlink start -x -u 14581 -r 4000000 -f -m onboard -o 14541
#   confirm:   param show EKF2_GPS_CTRL     -> 0

# pane B — the arm (vtrn | novtrn | conflict). Wait for `est-pub` lines.
./sitl/run_ev_arm.sh vtrn ~/drone/plans/ev_test.plan ~/logs

# pane C — fly it
droneflight fly --plan ~/drone/plans/ev_test.plan --url udpin://0.0.0.0:14540 \
    2>&1 | tee ~/logs/fly-vtrn.log
```

`run_ev_arm.sh` pins `--seed 0 --fidelity-seed 1` for every arm. **Do not vary the
seeds between arms** — a different seed is a different weather system, and the
comparison stops meaning anything.

Repeat for `novtrn` and `conflict`. Each writes `~/logs/ev-<arm>.jsonl`.

### Reading the result

```bash
droneflight ev-report --plan ~/drone/plans/ev_test.plan \
    --record ~/logs/ev-vtrn.jsonl \
    --record ~/logs/ev-novtrn.jsonl \
    --record ~/logs/ev-conflict.jsonl
```

One table, MISSION phase only (post-abort RTL excluded automatically). **The
experiment only means something if the arms disagree** — the report says so
explicitly, because all-pass or all-fail means the route decided the outcome, not
the navigation.

### What to watch during each run

* `EKF2 global origin set to …` — else `fly` will never arm.
* `est-pub` settling near 0 — EKF2 is fusing the injected vision.
* `injected err` growing with distance, and **shrinking at fixes in arms A and C**.
  In arm B it should grow monotonically to ~100 m.
* `progress watchdog: 565s` at `fly` start — a fixed 60 s aborted a healthy flight
  at 41/77 before this was derived from the plan.

## 3d. The error-axis arms (dropout, yaw) — ✅ both FLOWN 2026-07-22

**Result: neither moved the corridor** (11.8 → 12.2 m dropout, → 11.9 m yaw), and
both axes verifiably engaged. That is a finding about the *metric*: cross-track
deviation is near-blind to heading error, because a multirotor on position
setpoints does not steer with its nose. Score these arms on **EKF2 heading error
from the ULog**, MISSION phase only. Full analysis: CLAUDE.md → Slice 3.

Dropout's measured cost was **12.40° of heading error inside the gaps vs 2.07°
outside**, with position unchanged — and EKF2 only held it that well by falling
back on a magnetometer that reads perfect truth in SITL (`cs_mag_3d` 0 → 39.4 %).

⏭ **`dropout-long` (6 s) is the next run** — it crosses `EKF2_NOAID_TOUT` and
tests the position-loss failsafe, which is a different mechanism entirely.

The procedure below is retained for re-running.

### Original plan (as flown)

The three arms above varied the **basemap**. These vary the **error axis**, and
each adds exactly one on top of the `vtrn` basemap — so the already-flown
`vtrn` arm (**max dev 11.8 m**, 30 923 samples, 100 % EV uptime, heading = truth)
is their control. **No control re-flight is needed.**

Run them **one at a time**. Dropout and yaw fail through completely different
mechanisms; combining them repeats the forest/stale mistake, where two knobs
moved together and only one turned out to be doing the damage.

| Arm | Adds | Why this value | Expect |
|---|---|---|---|
| `dropout` | lock loss ~every 500 m for **3 s** | **under** EKF2's 5 s `EKF2_NOAID_TOUT` | rides through; ~6 losses, 98.2 % uptime; mission completes; corridor a little worse than 11.8 m |
| `yaw` | **1.0 °/100 m** heading drift | reaches ~7.7 ° by 5.13 km — big enough to steer, small enough not to be absurd | mission completes; true track bows off the route as the controller flies a rotated frame |
| `dropout-long` *(follow-on)* | lock loss ~every 500 m for **6 s** | **over** the 5 s timeout | EKF2 declares horizontal position invalid → **position-loss failsafe fires**. This is the genuinely untested path |

⚠️ **The 5 s number is the whole design of this experiment.** `EKF2_NOAID_TOUT`
is *not* in `configs/px4/gps_denied_ekf2.params`, so PX4's 5 s default applies.
Confirm it before flying — if this build differs, the 3 s and 6 s arms stop
meaning "under" and "over" and the pair tests nothing:

```bash
# at pxh>
param show EKF2_NOAID_TOUT
```

⚠️ **Only two failsafes are neutralised** (`NAV_DLL_ACT=0`, `SIM_BAT_DRAIN=0` in
`sitl_bringup.params`). The **position-loss failsafe is live**, which is exactly
why `dropout-long` is worth flying — and why `dropout` at 3 s should *not* trip it.

### Running them

Same three-pane flow as §3c — restart SITL per arm, re-add the endpoint:

```bash
# pane A — SITL
./sitl/run_sitl.sh ~/drone/plans/ev_test.plan ~/drone/PX4-Autopilot
#   at pxh>:   mavlink start -x -u 14581 -r 4000000 -f -m onboard -o 14541
#   confirm:   param show EKF2_GPS_CTRL      -> 0
#   confirm:   param show EKF2_NOAID_TOUT    -> 5000000 (see above)

# pane B — the arm. Wait for `est-pub` lines before starting pane C.
./sitl/run_ev_arm.sh dropout ~/drone/plans/ev_test.plan ~/logs

# pane C — fly it
droneflight fly --plan ~/drone/plans/ev_test.plan --url udpin://0.0.0.0:14540 \
    2>&1 | tee ~/logs/fly-dropout.log
```

Then repeat with `yaw`. Seeds stay pinned (`--seed 0 --fidelity-seed 1`), and the
bridge now prints them into the tee'd log, so each run records what produced it.

### What to watch — different from §3c

* **`[EV DOWN]`** in the bridge's monitor line: the stream is being withheld.
  Absent for the whole run means the dropout never fired — check the banner says
  `ev dropout: every ~500 m for 3.0 s` rather than `OFF`.
* **`est-pub` climbing during a gap, then snapping back** — EKF2 dead-reckoning on
  IMU while vision is absent. It should recover; if it does not, the 3 s arm is
  behaving like the 6 s one and the timeout is not what we think.
* **The bridge's closing summary** now reports `ev dropout: N lock losses, M
  publishes withheld (X% uptime)` and `yaw error: max D°`.
* For `yaw`: the vehicle will fly a *heading-rotated* path but still believe it is
  on route, so **`est-pub` stays near 0 while the true track bows away** — the
  error is invisible in the readback by construction. Only the corridor number
  catches it. This is the one arm where a clean-looking console means nothing.
* For `dropout-long`: watch pane C for the failsafe. Note **what PX4 actually
  does** (descend / land / hold) — that behaviour is the result, not the corridor
  number, and the run may legitimately never complete the mission.

### Reading them

Each arm is scored against the flown `vtrn` record, not against each other:

```bash
droneflight ev-report --plan ~/drone/plans/ev_test.plan \
    --record ~/logs/ev-vtrn.jsonl \
    --record ~/logs/ev-dropout.jsonl \
    --record ~/logs/ev-yaw.jsonl
```

`ev-report` reports EV uptime and max yaw error per record, so a run that
silently had its axis disabled is visible rather than being read as a null result.

⚠️ **Endpoint deviation is meaningless if `dropout-long` failsafes mid-route** —
the report flags an incomplete route, but check `finished=` in the ULog before
quoting any number from that arm.

## 3e. `dropout-long`, magnetometer denied — ✅ FLOWN 2026-07-22

**Result (`sim-logs/21_16_45.ulg`): the position-loss failsafe fired at all 6
gaps and the aircraft rode through — mission COMPLETED 77/77.** Each 6 s gap
crossed `EKF2_NOAID_TOUT`, PX4 entered DESCEND for ~2.7 s, then resumed
AUTO_MISSION when vision returned. Two findings:

* 🔴 **Failsafe action is DESCEND at 0.7 m/s — wrong direction for a NOE
  fiber-layer.** ~2.2 m given up per gap, ~13 m cumulative, toward the ground.
  Same class as the Slice-1 RTL-to-61 m defect; the action must be a deliberate
  choice (hold, not sink). Flagged, not changed.
* 🔵 **Heading was BETTER mag-denied (1.88°) than the 3 s mag-on arm (12.40°)** —
  the earlier excursion was the EV↔mag handoff, not the EV loss. **Confirmed
  single-variable by arms F and G (flown same day):** F (`22_32_36.ulg`, 6 s gaps,
  mag ON) = **11.77°**; G (`22_54_38.ulg`, no dropout, mag denied) = 2.19° = arm A.
  So mag denial is harmless with continuous EV and only helps at a dropout.
  Residual caveat: SITL gyro is low-drift, so the coast may be worse on real
  hardware.

⚠️ **F/G filename mixup (identify by ULog `EKF2_MAG_TYPE`, not by name):** F's
bridge record is a 15 s stub — score F from `22_32_36.ulg`. And **arm G
overwrote `ev-vtrn.jsonl`** — the arm-A acceptance record is gone locally (use
`14_53_15.ulg`); rename before re-running the acceptance table.

Full analysis: CLAUDE.md → Slice 3. **Only follow-up left: the failsafe-action
experiment** (position-loss action → hold, not DESCEND).

The procedure below is retained for re-runs.

### Original plan (as flown)

The honest worst case, and the run that should finally make something break.
Two things change at once versus the flown 3 s `dropout` arm:

1. **6 s gaps instead of 3 s** — crosses EKF2's 5 s `EKF2_NOAID_TOUT`, so
   horizontal position is declared invalid and the **position-loss failsafe**
   fires. That failsafe is live (only `NAV_DLL_ACT` and `SIM_BAT_DRAIN` are
   neutralised).
2. **No magnetometer** — `configs/px4/mag_denied.params`. The 3 s arm only held
   heading to 12.40° because EKF2 fell back on 3D mag fusion (`cs_mag_3d` 0.0 %
   → 39.4 %), and in SITL that magnetometer reads perfect truth. With it denied,
   **external vision is the only heading observation**, so a gap leaves EKF2 on
   gyro alone.

### ⚠️ Read the result under this rule, decided before the run

Two variables move together, so:

* **If it breaks** (failsafe fires, mission aborts, heading runs away) — that is
  a real and immediately useful worst-case result. Report it as "6 s gaps with no
  magnetometer break it", *not* as either factor alone.
* **If it does NOT break** — you have learned nothing attributable, and the two
  isolating runs below are then required. Do not report a clean combined run as
  "dropout-long is fine".

Isolating runs, only needed if the combined run survives:

| Run | Config | Isolates |
|---|---|---|
| F | `dropout-long`, mag **on** | the `NOAID_TOUT` crossing alone |
| G | `vtrn` (no dropout), mag **denied** | the new heading baseline alone |

### Parameters

Apply all three files together, then reboot:

```bash
droneflight px4-params \
    --file configs/px4/gps_denied_ekf2.params \
    --file configs/px4/sitl_bringup.params \
    --file configs/px4/mag_denied.params
```

⚠️ **`EKF2_MAG_TYPE`'s enum has changed across PX4 releases.** Verify `5 = None`
on this build before trusting it — a wrong value silently leaves the magnetometer
fusing and the whole point of the run is lost:

```bash
param show EKF2_MAG_TYPE
```

⚠️ **These params persist across SITL restarts.** Reset them afterwards or every
later arm is silently magnetometer-denied — which would invalidate comparisons
against every flight before today:

```bash
param reset EKF2_MAG_TYPE SYS_HAS_MAG EKF2_MAG_CHECK
```

### Running it

Write to a **separate log directory** — `run_ev_arm.sh` names records after the
arm, so a mag-denied `dropout-long` would otherwise overwrite a mag-enabled one:

```bash
# pane A — SITL, with all three param files applied and rebooted
./sitl/run_sitl.sh ~/drone/plans/ev_test.plan ~/drone/PX4-Autopilot
#   at pxh>:   mavlink start -x -u 14581 -r 4000000 -f -m onboard -o 14541
#   confirm:   param show EKF2_MAG_TYPE     -> 5
#   confirm:   param show EKF2_NOAID_TOUT   -> 5000000
#   confirm:   param show EKF2_GPS_CTRL     -> 0

# pane B — the arm, into its own directory
./sitl/run_ev_arm.sh dropout-long ~/drone/plans/ev_test.plan ~/logs/magdenied

# pane C — fly it
droneflight fly --plan ~/drone/plans/ev_test.plan --url udpin://0.0.0.0:14540 \
    2>&1 | tee ~/logs/magdenied/fly-dropout-long.log
```

### Expect this to be the first failure: arming

With no magnetometer and `EKF2_EV_CTRL=11`, **EV yaw is the only thing EKF2 can
align heading against**. If the bridge is not already publishing, yaw alignment
never completes and `fly` will not arm. The bridge-before-fly ordering (§3b.3) is
not just convention here — it is load-bearing. If arming fails, check
`cs_yaw_align` in the ULog before assuming the params are wrong.

### What to watch

* **The failsafe action at the first 6 s gap** — descend, land, hold, terminate?
  **That behaviour is the result of this run**, not the corridor number.
* Heading error during gaps, now with no mag fallback. The 3 s arm reached 12.40°
  *with* the magnetometer; expect materially worse, and measure it rather than
  guessing — `vehicle_attitude` vs `vehicle_attitude_groundtruth`, MISSION phase
  only (the post-mission HOLD phase adds ~11° and will corrupt a whole-log number).
* `cs_mag_3d` should now be **0 % everywhere**. If it is not, the mag is still
  fusing and the run does not test what it claims to.
* ⚠️ **If the mission aborts, the corridor and endpoint numbers are meaningless.**
  Check `finished=` in the ULog before quoting either.

## 3f. Failsafe-tolerance experiment — ✅ FLOWN 2026-07-23, it worked

**Result (`sim-logs/failsafe/15_02_03.ulg`, `EKF2_NOAID_TOUT=8 s`): the descents
are gone.** Same 6 EV gaps of 6.0 s, but **0 DESCEND episodes** (was 6),
**`xy_valid` never went false** (was ~10 % of mission samples), altitude held flat
(no ~2.2 m sawtooth), mission **COMPLETED 77/77**. Raising the no-aiding timeout
above the gap length let EKF2 coast the 6 s on IMU without ever declaring position
invalid, so the position-loss failsafe never armed.

⚠️ **The tradeoff is real and measured.** Coasting the full 6 s grew EKF2's
position error to **22.8 m in-gap** (mean 5.2 m; 11.1 m outside gaps), and the
true-track corridor was **24.5 m LEFT** — *worse* than the descent baseline's
20.3 m. So the fix removes the CFIT/altitude hazard (the point) but does not keep
the corridor: a 6 s EV dropout exceeds what bounded nav holds to 15 m whichever
failsafe response you pick. Net for a NOE fiber-layer: **not descending toward the
canopy is the right call**, and the residual horizontal excursion recovers when EV
returns. Set the timeout to just cover the expected transient — 8 s here — no more.

*The experiment as originally planned:*

The `dropout-long` arms proved the position-loss failsafe descends at 0.7 m/s
during each 6 s gap (~2.2 m lost per gap, ~13 m over 6). This run removes that,
and it is the one **program-level fix** left in the EV series rather than another
measurement.

**The fix is the no-aiding TIMEOUT, not the failsafe action.** A multirotor with
no position estimate cannot hold XY — "descend vs hold" is a false choice, because
the vehicle no longer knows where it is. The correct lever is to stop treating a
*brief* gap as position loss: `configs/px4/dropout_tolerant.params` raises
`EKF2_NOAID_TOUT` 5 s → **8 s**, just above the 6.1 s injected gap, so EKF2 coasts
on the IMU through the dropout, keeps position valid, and never trips the failsafe.

⚠️ **This is a deliberate tradeoff.** A longer timeout also delays the failsafe on
a *genuine* (permanent) vision loss — the aircraft dead-reckons blind for up to
8 s before reacting. 8 s covers our transient with margin; do not inflate it.

### Running it

Standard GPS-denied stack (mag on is fine — this is the altitude axis, independent
of the heading↔mag result), plus the tolerance file, flown as `dropout-long`:

```bash
droneflight px4-params \
    --file configs/px4/gps_denied_ekf2.params \
    --file configs/px4/sitl_bringup.params \
    --file configs/px4/dropout_tolerant.params
#   paste into pxh, reboot, then confirm:
#     param show EKF2_NOAID_TOUT     -> 8000000

./sitl/run_ev_arm.sh dropout-long ~/drone/plans/ev_test.plan ~/logs/failsafe
droneflight fly --plan ~/drone/plans/ev_test.plan --url udpin://0.0.0.0:14540 \
    2>&1 | tee ~/logs/failsafe/fly.log
```

### What to watch — this is a clean before/after

* **Zero DESCEND episodes.** `nav_state` should stay AUTO_MISSION straight
  through every gap. Against the baseline (`22_32_36.ulg` / `21_16_45.ulg`, 6
  DESCENDs each) that is the whole result.
* **Flat altitude.** Ground-truth `-z` should hold ~30 m across the gaps instead
  of sawtoothing down ~2.2 m. Cumulative altitude given up → ~0.
* **`xy_valid` stays true** through the 6 s gaps (it went false ~10 % of samples
  before). If it still goes false, the timeout did not take — re-check the param.
* ⚠️ **Confirm a gap still LONGER than 8 s does fail safe.** The point is
  tolerance to transients, not disabling the failsafe. A quick way: one
  `--dropout-duration 10` gap should still descend. If nothing ever fails safe,
  `NOAID_TOUT` is too high.

## 4. Custom tether model (required build item)

No standard sim models a **trailing fiber** (Technology-Selection §4.2), so v0
approximates it as a **drag + back-tension disturbance force** on the airframe,
applied as a Gazebo wrench. Sources: `sitl/tether_plugin/` (its README covers
layout and rationale); requirements and acceptance: `sitl/TETHER_PLUGIN.md`.

⚠️ Magnitude is **not** ∝ paid-out length as the spec assumed — only the
**airborne span** (~flight altitude) is in the airflow, because the laid fiber is
deposited, not towed. See the `tether.py` header.

### 4.1 Status before you start

**Written and Mac-tested, never compiled against Gazebo.** The physics
(`tether_math.hpp`) is bit-exact against `tether.py` and covered by 24 tests that
run on the Mac; `TetherPlugin.cc` was written from the gz-sim8 docs without a
compiler. **Budget the first VM session for build errors**, not for results.

### 4.2 Build + install (VM)

```bash
cd ~/droneflight/sitl/tether_plugin
cmake -B build -S . && cmake --build build -j     # needs gz-sim8 + gz-plugin2 dev
./install_tether_plugin.sh ~/PX4-Autopilot         # patches the x500 MODEL sdf
```

`install_tether_plugin.sh` is idempotent, saves `model.sdf.orig`, and reverts
exactly with `--remove`. It patches
`$PX4_DIR/Tools/simulation/gz/models/x500/model.sdf` — **not** the world SDF:
`make px4_sitl gz_x500` spawns the vehicle into the world at runtime, so a world
file has no x500 to attach to and a `<plugin>` block there is a silent no-op.

`run_sitl.sh` puts `tether_plugin/build` on `GZ_SIM_SYSTEM_PLUGIN_PATH` when it
exists and prints whether it did.

### 4.3 ⚠️ Confirm it loaded, every run

Gazebo does **not** fail on an unresolved system plugin, so a no-tether run is
otherwise indistinguishable from a tether run. Two checks, in order of authority:

1. **The CSV exists and has a header.** This is the reliable one: the file is
   created in `Configure`, so its presence proves the plugin loaded *and* parsed
   its parameters. Use an **absolute path** — Gazebo's cwd is PX4's build
   directory, not where you launched from, so a relative `DRONEFLIGHT_TETHER_CSV`
   lands somewhere surprising.
   ```bash
   head -2 /home/$USER/tether-on.csv     # two '#' lines = loaded, with its coefficients
   ```
2. **The console banner** — `[tether] ENABLED on x500/base_link -- back_tension_n=0.5 ...`
   ⚠️ This is a `gzmsg`, which Gazebo prints only at **verbosity ≥ 3**. If PX4
   launches the server quieter than that, the banner is absent on a perfectly
   healthy run. **Absence of the banner is not evidence of failure — absence of
   the CSV is.** If you want the banner to be dependable, either raise Gazebo's
   verbosity or promote that one line to `gzerr` in `TetherPlugin.cc`.

⚠️ **Kill stray `gz sim`/`ruby` processes between runs.** PX4 attaches to an
already-running simulator (§2b says the same thing about world selection), and
the plugin reads `DRONEFLIGHT_TETHER_*` **once, in `Configure`**. So a stale
server silently reuses the *previous* arm's settings and your A/B becomes two
copies of the same arm. The CSV header records what the plugin actually read —
**score the arm from `enabled=` in the file, never from the command you typed.**

### 4.4 Bring-up: prove the force reaches the physics

A logged tension is not an applied one. Before any real experiment, confirm the
wrench actually lands, using a magnitude too large to miss:

```bash
# absurd span -> ~27 N at 6 m/s, >1 g on a ~2 kg x500: it should visibly lose the route
DRONEFLIGHT_TETHER_CSV=$HOME/tether-huge.csv ./run_sitl.sh <plan> ~/PX4-Autopilot
#   ...with <airborne_span_m>4000</airborne_span_m> in the x500 model SDF
```

| `airborne_span_m` | drag @6 m/s | total | ≈ tilt to hold it | expect |
|---|---|---|---|---|
| 40 (default) | 0.26 N | 0.76 N | ~2.2° | near the noise floor |
| 400 | 2.65 N | 3.15 N | ~9° | clearly visible in the ULog |
| 4000 | 26.5 N | 27.0 N | can't hold | dragged off route / mission fails |

**If 4000 changes nothing, `AddWorldForce` is not landing** — the plugin is
logging a force it never applies, which is the one failure mode that looks
completely healthy from the CSV. Fix that before trusting any smaller number.
(Note the force only exists once the vehicle has *translated*: in a pure vertical
climb there is no horizontal travel direction and the pull is zero by design.)

Then walk it back to the default span and confirm a normal mission still
completes — a NaN or runaway force would show as a sim that flies worse, not as
an error message.

### 4.5 The A/B (acceptance §2)

Same SDF for both arms; only the environment changes, so nothing is edited
between runs and the arms cannot silently differ in anything else:

```bash
# shell A — tether ON
DRONEFLIGHT_TETHER_ENABLED=1 DRONEFLIGHT_TETHER_CSV=~/tether-on.csv ./run_sitl.sh <plan> ~/PX4-Autopilot
# ...then the usual `droneflight fly` in shell B (§3)

# shell A — control
DRONEFLIGHT_TETHER_ENABLED=0 DRONEFLIGHT_TETHER_CSV=~/tether-off.csv ./run_sitl.sh <plan> ~/PX4-Autopilot
```

A disabled run still loads and still logs, recording `enabled=0` in its CSV
header — a control arm must be identifiable from its own artifacts (the F/G
session in §3d is the cautionary tale).

### 4.6 Scoring

```bash
droneflight tether-plugin-check --log ~/tether-on.csv
```

Re-derives every logged tension from `tether.py`. **Exit 0 = EXACT**: the in-loop
force *is* the offline model. Exit 1 = DIVERGED — stop and reconcile
`tether_math.hpp` with `tether.py` before scoring any flight. It also prints the
applied peak as a fraction of the break budget and the logged (uninjected) turn
transient.

For the attitude bias, compare `vehicle_attitude` roll/pitch between arms over a
straight leg.

### 4.7 What to expect — and what would be a false negative

**0.76 N on a ~2 kg x500 is ~0.04 g, about 2.2° of steady tilt**, which PX4 trims
out. Acceptance §2 may measure a difference near the noise floor. That is a
legitimate finding — *the fiber does not meaningfully disturb this airframe at
cruise* — but it is **not** evidence the plugin works. Distinguish the two:

- **Parity (§4.5) is what proves the plugin works.** It is independent of whether
  the force is large enough to see.
- To show the disturbance is *real*, **sweep `airborne_span_m`** (40 → 400 in the
  SDF) and confirm the attitude bias scales with it and with `v²`. A scaling law
  is far stronger evidence than one A/B at a magnitude the controller can hide.

Acceptance §3 (speed policy in the loop) is **blocked on a route, not on the
plugin**: `plans/ev_test.plan` bends ≤16°, so the policy never engages and
`tether-check` already reports the tether slack there. It needs a turn-rich plan
— the same gap as the flat-world caveat.

A full catenary / lumped-segment cable model is a later refinement (v1).

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
droneflight fly --plan ~/plans/ev_test.plan --url udpin://0.0.0.0:14540 2>&1 | tee ~/logs/fly-$(date +%F-%H%M).log
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
