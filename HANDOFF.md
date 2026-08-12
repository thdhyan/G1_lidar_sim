# G1 sim — Handoff

Context for the next agent/session. Design rationale → [Plan.md](Plan.md).
Status board → [Tasks.md](Tasks.md). LiDAR init detail →
[docs/RTX_LIDAR_INIT.md](docs/RTX_LIDAR_INIT.md).

## Quickstart

Two environments, never mixed:

```bash
conda activate isaac                       # Python 3.12, Isaac Sim 6.0.1
source /opt/ros/jazzy/setup.bash            # ~/.bashrc does NOT source this
cd ~/Projects/thesis/G1_sim
```

**⚠️ Only one Isaac Sim fits in 8 GB VRAM.** Kill stragglers before every
run: `pkill -9 -f g1_warehouse_sim`.

### Terminal 1 — the sim (warehouse + G1 + WBC + camera + LiDAR, headless)

```bash
python scripts/g1_warehouse_sim.py --headless
```

Defaults: decoupled_wbc balance/walk policy loaded and active, D435 camera
on, RTX Mid-360 LiDAR on, 2 IRA humans + 1 Nova Carter wandering the
warehouse (cameras stripped from the Nova Carter by default — they OOM an
8 GB GPU). Publishes `/livox/mid360/points`, `/g1/camera/{rgb,depth,semantic,
camera_info}`, `/g1/camera/depth/color/points`, `/tf`, `/g1/joint_states`,
`/g1/imu`, `/clock`. Subscribes `/g1/cmd_vel`.

Useful flags: `--no-ira` (warehouse only, no dynamic actors), `--no-camera`,
`--no-locomotion` + `--freeze-robot` (upright robot, no policy, for
sensor-only testing), `--num-prims N` (fewer LiDAR prims), `--cache-scene`
(default on — reuses the baked warehouse+navmesh USD instead of a ~60-80s
rebake every run; pass `--rebake` to force a fresh IRA bake).

### Terminal 2 — robot_description + RViz (system Python 3.12, ROS Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
unset GTK_PATH GIO_MODULE_DIR LOCPATH   # see "RViz crash" gotcha below
ros2 launch launch/g1_bringup.launch.py detection:=false
```

Brings up `robot_state_publisher` (feeds `/robot_description` from the URDF,
remapped to `/g1/joint_states`) and RViz with the saved config
`rviz/g1_rtx.rviz` (RGB image + LiDAR PointCloud2 + robot model + TF, all
`use_sim_time:=true`). `detection:=false` skips the detection node — this
repo's diagnostic/detection extras are not part of the minimal
sim+camera+lidar+WBC pipeline. Pass `rviz:=false` to skip RViz itself.

## Gotchas

- **RViz crashes (and can take the launching terminal/VSCode down with it)
  if launched with VSCode's snap-polluted environment.** `GTK_PATH`,
  `GIO_MODULE_DIR`, `LOCPATH` leak in from VSCode's own snap confinement
  (point into `/snap/code/...`) when a terminal is opened inside this VSCode
  session, and collide with `rviz2`'s libpthread/glibc symbols
  (`GLIBC_PRIVATE` / `undefined symbol __libc_pthread_init`). **Always
  `unset GTK_PATH GIO_MODULE_DIR LOCPATH` before launching `rviz2`** (either
  directly or via `ros2 launch`). Do **not** touch `LD_LIBRARY_PATH` — ROS
  needs it as-is for `libOgreMain`.
- **RViz shows nothing despite real data flowing** → check `Fixed Frame` is
  `World` (root of the real TF tree, not `odom` — nothing publishes an
  `odom` frame) and that RViz is running with `use_sim_time:=true` (the
  launch file already sets this; a bare `rviz2 -d ...` invocation won't, and
  will reject all sim-stamped data as `TF_OLD_DATA`).
- **Diag/debug scripts have been deleted** (2026-08-11) — `scripts/debug_*`
  and `scripts/diag_*` are gone. Don't recreate one-off diagnostic scripts
  in `scripts/`; the production path is `g1_warehouse_sim.py` only.
- **`assets/` is git-ignored.** Vendored URDF/meshes/scan-pattern/policy
  files won't show in `git status`. If another machine needs them, copy
  `assets/robot/`, `assets/scan_patterns/`, `assets/policy/` over manually.
- **RTX LiDAR coverage is a known partial band, not a full ring** — see
  Tasks.md "Open — RTX LiDAR coverage". Don't mistake this for a launch
  misconfiguration; it's an open upstream-tracing item.
- **`record_g1_bag.sh`** targets real-hardware topics (`/utlidar/cloud`,
  `/g1/odom`) — don't use it against this sim; the sim's real topic list is
  in Tasks.md's Infra notes.
- **System RAM, not just VRAM, is tight.** The headless sim alone holds
  ~8 GB RSS; with a browser/IDE also open, `rviz2` can get silently SIGKILL'd
  (exit code -9, no error text) by the OOM killer on launch. If RViz dies
  immediately after "Subscribing to: /livox/mid360/points" with no crash
  message, check `free -h` before assuming it's a code bug — close other
  memory-heavy apps (browser tabs, extra VSCode windows) first.
