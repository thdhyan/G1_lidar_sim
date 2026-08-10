# Tasks

Status board for the G1 + Livox Mid-360 + ROS2 pipeline. Owners are whoever
picks the task up; update the status column as work lands.

Legend: ✅ done · 🔄 in progress · ⛔ blocked · ⬜ not started

---

## Task 0 — Verify warp LidarSensor on this install ✅ **GATE PASSED**

- ✅ Backed up the pip-installed IsaacLab sensor files
  → `backup_pip_isaaclab_20260729_021445/` (path also in `.backup_pip_path`)
- ✅ Installed LidarSensor modules per README Option B into the **pip package**
  (`site-packages/isaaclab/source/isaaclab/isaaclab/sensors/`), not the
  `~/Projects/IsaacLab` source checkout — the latter is IsaacLab 3.0 and is
  *not* what `import isaaclab` resolves to.
- ✅ `scripts/00_verify_warp_lidar.py` reports **PASS**

```
[GATE] sensor initialised: 20000 rays x 2 envs
[GATE] finite fraction : 1.000
[GATE] valid hits      : 2178
[GATE] hit range       : 16.046 .. 44.455 m
[GATE] closest possible: 16.046 m (phi = -7.16 deg)
[GATE] finite points   : 2178
[GATE] PASS
```

Measured closest hit matches the predicted slant range to three decimals, so
the ray-caster is geometrically correct, not merely producing output.

### Two gotchas found here — read before tuning the sensor

**`samples` selects a time window, not a subsample.** The `.npy` patterns are
time-ordered scan trajectories and the loader takes a *contiguous slice*. The
first 8,000 rows of `mid360.npy` are one upward sweep with **zero** downward
rays, so a ground plane is unhittable. Use the native 20,000, or `downsample`
(which strides) instead of shrinking `samples`.

**The Mid-360 has no straight-down ray.** Steepest tilt is −7.2°, so the blind
cone radius is ≈ 8× the mount height. Measured on the G1: sensor at **0.80 m**,
blind radius **6.40 m** — near-field sensing has to come from the camera.

## Task 1 — G1 URDF → USD ✅

- ✅ `scripts/convert_g1_urdf_to_usd.py` auto-generates `assets/g1_29dof_sensors.usd`
- ✅ `merge_fixed_joints=False` preserves both `mid360_link` and `d435_link` as USD prims
- ✅ All 29 articulated joints + all 4 required links verified present in output stage
- ✅ No hand-authoring needed; mounts already in URDF since 2023

## Task 2 — G1 config ✅

- ✅ `g1_sim/robots/unitree_g1_lidar.py` declares `G1_LIDAR_CFG` (ArticulationCfg)
  - 4 actuator groups: legs/feet/waist/arms with distinct stiffness/damping
  - Default init height 0.74 m, crouched stance
  - 29 DOF all wired
- ✅ `mid360_cfg()` factory: LidarSensorCfg with roll-π quaternion, guard vs `samples<20000`
- ✅ `camera_cfg()` factory: D435 60° FOV, pitched ~47.6° downward
- ✅ `blind_radius()` helper: 8.0 × mount height
- ✅ Offset constants mirror URDF exactly (mounts + TF agree)

## Task 3 — ROS2 Action Graph 🔄 written, **never executed**

⚠️ Node type names and attribute names are taken from the Isaac Sim 5.1 docs and
the registered-node list — they have **not** been exercised by a running graph.
Expect `og.Controller.edit` to reject at least one attribute name on first run.
`Verify.md` step 3 is what proves this.

- ✅ `g1_sim/action_graph.py` builds OmniGraph with 9+ ROS2 nodes:
  - `ROS2PublishClock` → `/clock`
  - `ROS2PublishTransformTree` → `/tf`
  - `ROS2PublishJointState` → `/g1/joint_states`
  - `ROS2CameraHelper` ×2 → `/g1/camera/{rgb,depth}`
  - `ROS2PublishCameraInfo` → `/g1/camera/camera_info`
  - `ROS2SubscribeTwist` → `/g1/cmd_vel`
  - `ROS2PublishOdometry` → `/g1/odom`
- ✅ `enable_ros2_bridge()`: loads bundled jazzy rclpy before `import rclpy`
- ✅ `read_cmd_vel()` helper: pulls latest velocity from graph

## Task 4 — In-sim LiDAR publisher 🔄 written, **never executed**

⚠️ No message has ever been published. `Verify.md` step 4 (`ros2 topic hz`) is
the proof.

- ✅ `g1_sim/lidar_publisher.py` publishes warp tensor → `sensor_msgs/PointCloud2`
  - 10 Hz rate matching sensor
  - Filters infinite/out-of-range points
  - `x,y,z,intensity` layout (matches livox_detection input)
  - Best-effort QoS (matches RViz LiDAR display config)
  - Stamps from sim time
- ✅ `LidarPointCloudPublisher` class: init + publish() + spin_once()
- ✅ Runs in-process after action graph bridge is enabled

## Task 5 — cmd_vel bridge ✅ **working, live-verified — 2026-08-10**

Solved via NVIDIA's `decoupled_wbc` (part of `GR00T-WholeBodyControl`, part of
the same walking-policy search documented below), not IsaacLab. Full trace in
`docs/decoupled_wbc_findings.md`; summary here.

**What exists now:**
- `g1_sim/wbc_bridge.py` (new) — pure numpy/onnxruntime port of
  `decoupled_wbc`'s Balance/Walk observation+action math. No Isaac Sim
  imports, verified against the same MuJoCo reference the policy ships with.
- `assets/policy/GR00T-WholeBodyControl-{Balance,Walk}.onnx` (new, 1.79 MB
  each) — the actual pretrained checkpoints, copied in from
  `third_party/GR00T-WholeBodyControl` following the `detection/pt/`
  convention.
- `g1_sim/rtx_camera.py` → `attach_cmd_vel_subscriber()` (new) — the active
  RTX pipeline (`g1_rtx_sim.py`) had no `/g1/cmd_vel` subscriber at all
  before this; the old getter in `action_graph.py` belonged to the inactive
  warp-lidar pipeline (`g1_ros2_sim.py`).
- `scripts/g1_rtx_sim.py` — wired in: reads `/g1/cmd_vel` at 50 Hz (the
  policy's trained control rate), runs the bridge, applies targets via
  `set_joint_position_targets(..., joint_names=LEG_WAIST_JOINTS)`.
  `--no-locomotion` to disable and fall back to the old passive hold.

**Live-verified** (`isaac` env, `python scripts/g1_rtx_sim.py --headless
--no-camera --no-room`): stable standing for 600 steps, then **9 s of
continuous walking driven by real `ros2 topic pub` messages on
`/g1/cmd_vel`**, pelvis height holding ~0.71–0.74 m throughout (vs. baseline
0.74–0.79 m). MuJoCo validation separately confirmed real forward
translation (7.9 m over 17 s) with the identical policy/observation code.

**Two real bugs found and fixed along the way** (not specific to this repo's
prior state — both are worth remembering):
1. `sim2mujoco/resources/robots/g1/g1_gear_wbc.yaml` in the vendored
   `GR00T-WholeBodyControl` checkout pointed at nonexistent `ft92.onnx` /
   `ft109.onnx` (stale internal training names) instead of the real
   `GR00T-WholeBodyControl-{Balance,Walk}.onnx` filenames — a bug in
   NVIDIA's public release, not this checkout. Fixed (2-line diff).
2. `convert_g1_urdf_to_usd.py` bakes a **uniform** PD drive
   (`stiffness=100, damping=10`) into every joint of the USD. The WBC policy
   was trained assuming much stiffer per-joint gains (`kp` up to 250). Using
   the USD default made the robot collapse in ~2 s when driven by the
   policy. Fixed with an explicit `Articulation.set_gains(...)` call at
   startup using the policy's real training gains — **any externally
   trained policy will hit this same trap against this USD** until
   `convert_g1_urdf_to_usd.py` or `G1_LIDAR_CFG` bakes in per-joint gains
   matching some real controller instead of a uniform placeholder.

**Still open** (see findings doc §7 for detail): forward displacement not
independently logged in the live Isaac Sim run (MuJoCo confirms it
separately); turning/strafing (`wz`/`vy`) untested live, forward-only;
50 Hz-vs-60 Hz control-rate mismatch uncorrected (policy runs ~20% faster
than trained, stable regardless); longest live run ~15 s sim-time, no
multi-minute or full-room-terrain test yet; `--no-locomotion` fallback path
implemented but not separately tested.

**Search trail for the record** (three false starts before decoupled_wbc):
`unitree_sim_isaaclab`'s `policy.onnx` is a stationary-arm manipulation
policy, not locomotion. IsaacLab's `Isaac-Velocity-Flat-G1-v0` has no shipped
checkpoint and targets the 23-DOF `G1_MINIMAL_CFG` anyway. NVIDIA's
GEAR-SONIC (`gear_sonic_deploy`) needs a full TensorRT C++ build and expects
mocap-style reference trajectories, not simple velocity commands — TensorRT
10.13 *was* installed and the build got through dependency resolution, but
the user redirected to `decoupled_wbc` before finishing it. That TensorRT
install and the `gear_sonic_deploy` checkout are left in place, untouched,
in case SONIC's fuller whole-body capability (manipulation + locomotion, not
just gait) is wanted later.

## Task 6 — Main scene 🔄 written, **never completed a run**

Two attempts, neither reached the main loop:
1. GUI mode — window closed during startup, exited before `main()`
2. `--headless` — **OOM-killed**. Nine orphaned Isaac Sim processes from the
   Task 0 diagnostics were still resident (15.3/15.6 GB RAM, swap fully
   exhausted). Those are now killed; 9.2 GB free.

⚠️ **Only one Isaac Sim process fits in 16 GB.** Kill stragglers before running:
`pkill -9 -f "python.*scripts/"`.

- ✅ `scripts/g1_ros2_sim.py` complete
  - Ground + lights
  - G1 import via `G1_LIDAR_CFG`
  - LiDAR + camera setup via factories
  - 3× pedestrian target boxes
  - Action graph + LiDAR publisher wired
  - Main loop: joint hold + render + publish
  - Stats every 200 steps
  - --steps, --headless, --no-ros2, --no-camera flags

## Task 7 — Detection ✅ (core) / ⬜ (integration)

- ✅ Weights + port complete (see prior log)
- ✅ `detection_node.py` publishes `Detection3DArray` + `MarkerArray`
- ✅ `ClusteringBackend` fallback (no torch/GPU)
- ⬜ Not yet tested against simulator's real cloud (Task 4 done; just needs to run)

## Task 8 — Launch, RViz, docs 🔄

- ✅ `rviz/g1.rviz` complete: RobotModel, TF, PointCloud2, Image, MarkerArray
  - PointCloud2 set to best-effort QoS
  - LiDAR color by Z axis (vertical)
  - Camera RGB display enabled
- ✅ `README.md`: install, environment facts, sensor mounts, detection, ROS2 topics
- ✅ `Plan.md`: goal, decisions, risks, findings (samples window trap, no nadir ray)
- ⬜ `launch/g1_bringup.launch.py`: external nodes (robot_state_publisher, detection, rviz)

---

## Environment (verified, not assumed)

| Item | Value |
|---|---|
| Conda env | `env_isaaclab`, Python 3.11.15 |
| Isaac Sim | 5.1.0 |
| IsaacLab | pip `isaaclab` 2.3.1 → `site-packages/isaaclab/source/isaaclab/isaaclab/` |
| ROS2 | jazzy, `/opt/ros/jazzy`, Python 3.12 |
| torch | 2.7.0+cu126 |
| GPU | RTX 4060 Laptop 8 GB (Ada, sm_89), nvcc 12.0 |

⚠️ `~/Projects/IsaacLab` is a **separate IsaacLab 3.0 checkout** and is *not*
the package Python imports. Its ray-caster was refactored to a backend-dispatch
design incompatible with the OmniPerception fork. Install into the pip package.

## Rollback

```bash
cp -r $(cat .backup_pip_path)/* \
  ~/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/sensors/
```

---

## RTX LiDAR pivot — Isaac Sim 6.0 (`isaacsim6-rtx-emitter` branch)

Everything above this line describes the original warp-ray-caster design on
Isaac Sim 5.1. The user directed a pivot to **RTX LiDAR with emitter state
arrays** on **Isaac Sim 6.0** (`isaac` conda env, Python 3.12), following
IsaacSim GitHub discussion #685. Status below reflects the pivot, current as
of 2026-07-29.

### RTX LiDAR + camera + robot state — ✅

- ✅ `g1_sim/rtx_lidar.py` — Mid-360 spawned as co-located RTX prims from
  generated emitter-state JSON configs (`scripts/gen_mid360_rtx_config.py`),
  3 presets (fast/light/full) trading sweep density vs GPU load
- ✅ `g1_sim/rtx_publisher.py` — reads `IsaacCreateRTXLidarScanBuffer`
  annotator directly, publishes merged `PointCloud2` via rclpy (the
  `ROS2RtxLidarHelper` OG node advertises but never emits on this setup)
- ✅ `g1_sim/rtx_camera.py` — D435 RGB/depth/semantic + `/clock`, `/tf`
  (World-rooted floating base), `/g1/joint_states`
- ✅ `scripts/g1_rtx_sim.py` — main scene, now loads Nucleus
  `simple_room.usd` by default (flat-ground fallback), pedestrian targets
- ✅ **Fixed (commit `7aa61bf`)**: azimuth-clipping bug
  (`validStartAzimuthDeg=0/360` vs emitter range `-180/180` silently
  dropped half the rays → narrow sector instead of full 360° ring) and
  orientation bug (180° roll self-occlusion → correct ~2.3° pitch per
  Unitree URDF `mid360_joint` rpy). Mount height corrected to 0.40618 m.
- ⛔ Re-verified in RViz 2026-08-10 — **still narrow, not a ring.** See below.

### Live verification — 2026-08-10

First actual run since the pivot (`Verify_RTX.md` had been sitting all-⬜).
Ran `scripts/g1_rtx_sim.py --headless --steps 0` in the `isaac` conda env
(Isaac Sim 6.0.1, pip install, no standalone bundle). No OOM this time (prior
attempt's stray processes were gone; 16 GB box, ~4-9 GB free depending on
what else is open — tight but workable headless).

**Passing:**
- ✅ Boots clean headless, no crash, no OOM
- ✅ `/livox/mid360/points`, `/tf`, `/clock`, `/g1/joint_states` all live on
  the real ROS2 graph (`ros2 topic list` / `hz` confirmed, not just code
  review)
- ✅ Publish rate ≈ 11 Hz, ~11k–20k points/message — matches the 10 Hz design
  target
- ✅ `Config 'Livox_Mid360_A' not found for OmniLidar` warning is **cosmetic**
  — present in every historical log including old "passing" runs, from a
  deprecated Python catalog-lookup path (`isaacsim.sensors.rtx.impl.commands`
  in `extsDeprecated/`) that resolves `config=` against NVIDIA's official
  Nucleus sensor catalog, not against the local JSON. It has nothing to do
  with whether the C++ RTX engine actually reads the emitter JSON (it does —
  confirmed by the point count above). Don't chase this warning again.

**New finding — azimuth still clipped, root cause identified:**

The published cloud only spans **azimuth ≈ [−90°, +90°]**, not the full
360°, regardless of `--num-prims` (reproduced with both 2 and all 4 prims —
ruling out the prim-count knob as the cause). Verified the JSON configs
themselves are correct: `Livox_Mid360_A.json`'s baked-in `azimuthDeg` values
are uniformly distributed across the full −180°…180° range (checked
numerically, ~3300 points per 30° bin across all 12 bins) — so commit
`7aa61bf`'s config-side fix is intact and not the problem.

The clipping happens downstream, at the annotator. The sim log itself says
why:

```
[Warning] [isaacsim.sensors.rtx.plugin] IsaacCreateRTXLidarScanBuffer is
deprecated and will be removed in a future release. Use the
GenericModelOutput annotator directly to access full-scan RTX Lidar data.
```

`g1_sim/rtx_publisher.py` reads `IsaacCreateRTXLidarScanBuffer` directly (see
its own docstring). That annotator is now deprecated in Isaac Sim 6.0.1 and,
per NVIDIA's own warning text, does not return full-scan data — matching the
±90° clip we measured almost exactly. **This is a regression relative to
whatever Isaac Sim build `7aa61bf` was verified on**, not a leftover of the
original bug.

⬜ **Next step**: port `rtx_publisher.py` from `IsaacCreateRTXLidarScanBuffer`
to the `GenericModelOutput` annotator NVIDIA points at. Not attempted here —
found late in this session and a swap like that needs its own verification
pass, not a blind edit.

- ⛔ Pedestrian targets are inside the blind radius. `PEDESTRIANS` in
  `scripts/g1_rtx_sim.py` are at (3, 0), (4.5, −2), (6, 2.5) m — all well
  inside the measured 9.6 m blind radius at this run's 1.21 m mount height
  (up from the earlier-documented 0.40618 m; height apparently drifted with
  some other change). None of them can ever appear in the cloud as placed.
  Either move the targets past ~10 m or lower the mount and accept a smaller
  blind cone.
- RViz: launched (`rviz2 -d rviz/g1.rviz`) but crashed instantly —
  `symbol lookup error: .../snap/core20/.../libpthread.so.0: undefined
  symbol __libc_pthread_init`. This is a snap/glibc PATH collision in this
  shell environment (likely `LD_LIBRARY_PATH` bleeding in from a snap
  package), unrelated to the sim. Try from a clean terminal outside VSCode's
  snap-polluted env before assuming it's a project bug.

### Detection — ✅ core, 🔄 live-cloud test done, 0 detections (expected)

- ✅ CenterPoint ported to torch 2.7/CUDA 12.6 (`detection/livox_centerpoint.py`)
- ✅ **Tested against the RTX sensor's actual published cloud, 2026-08-10** —
  loads real weights, runs on GPU, 0 detections. Expected: all 3 pedestrian
  targets are inside the blind radius (see above), so there is nothing for
  the model to find in this scene regardless of model correctness. Not yet
  tested with targets placed in-range.

### New: `g1_perception_ws/` — real-robot + detection workspace

Separate ROS2 Jazzy workspace (sibling of `G1_sim/`), built by 3 parallel
agents, all merged cleanly into one `setup.py`/`package.xml`:

- ✅ `lidar_bridge` — sim passthrough / real `/utlidar/cloud` re-frame onto
  canonical `/livox/mid360/points`
- ✅ `detection_bridge` — subprocess wrapper around `G1_sim/detection/detection_node.py`
- ✅ `ccvnorm_node` — LiDAR + D435 depth completion (Stereo-LiDAR-CCVNorm
  integration), pseudo-stereo mode is dependency-free and default
- ✅ `centerpoint_node` — in-process CenterPoint, graceful empty-output if
  checkpoint missing
- ✅ `pointpillar_node` + `pointpillar_model.py` — in-process PointPillar
  (Lang et al. 2019, written from scratch — no compatible pretrained
  checkpoint located), Euclidean-clustering fallback if checkpoint missing
- ✅ Clean `colcon build --symlink-install`, all 5 console scripts registered
- ✅ `vision_msgs`/`visualization_msgs` added to `package.xml` (were missing,
  added post-merge since 2 of 3 concurrent agents' nodes need them)
- ⬜ Never tested against live sim or real robot — see `Verify_RTX.md`

### Not started

- PointPillar pretrained weights: **found 2026-08-10, not yet integrated** —
  see below
- Real-robot hardware test: `/utlidar/cloud` topic/type is per Unitree docs,
  untested against an actual G1

---

## SLAM investigation — Ultra-Fusion — 2026-08-10

User pointed at `sjtuyinjie/Ultra-Fusion` (tightly-coupled LiDAR+camera+IMU
SLAM — matches our exact Mid-360 + D435 sensor pair). Full trace in
`docs/ultra_fusion_findings.md`; summary here.

- ⛔ **No public source** — repo README states source ships "after paper
  acceptance"; confirmed zero `CMakeLists.txt`/`package.xml` anywhere in the
  clone. Prebuilt `.deb` + Docker images only.
- ✅ **Runs via Docker** — the shipped images target ROS2 Humble/Ubuntu 22.04
  and ROS1 Noetic/Ubuntu 20.04, neither of which is this machine's ROS2
  Jazzy/Ubuntu 24.04, but Docker sidesteps that entirely (container brings
  its own userspace on the host kernel; `--net=host` lets its DDS layer see
  host-published topics — this is the README's own recommended pattern, not
  a workaround). Pulled `maotiandocker/ultrafusion-ros2:0.2.0` (repo's
  README names a `0.2.1` tag that doesn't exist on Docker Hub — a real, if
  minor, doc bug), hit a version-skew bug (image predates the current
  `Dockerfile.ros2`'s `ros-humble-ros2service`/`ros-humble-std-srvs` deps),
  fixed by installing those two packages inside the container to match the
  repo's own current Dockerfile spec. `uf_node` then installs clean and runs
  correctly against a real profile (parses config, subscribes topics, shuts
  down cleanly) — genuinely works, not just "installs."
- ⬜ **No sensor profile for our rig yet.** Ultra-Fusion's shipped `m3dgr_*`
  shortcuts assume a wheeled robot (`wheel_topic: /odom`); G1 is legged.
  Closest starting point is `config/m3dgr/uf_m3dgr_ros2_lio.yaml` (`wheel:
  0`), but needs our actual Mid-360→IMU and D435→IMU extrinsics plus D435
  intrinsics filled in — none of which exist in this repo yet. Not
  attempted; no calibration data available this session.
- ⬜ Not tested against live/bagged G1 sensor data, and not verified in RViz
  (needs `-e DISPLAY` + X11 mounts into the container, not attempted).
- Cost: Docker image is 4.29 GB on disk; disk dropped from 13 GB → 8.4 GB
  free over this investigation (355 GB disk, already at 97–98% used from
  unrelated causes — worth keeping an eye on before any further large pulls).

**Alternatives** (ROS2-Jazzy-native, public source, not installed — for if
the profile-authoring or live-DDS step above stalls): **FAST-LIO2** /
**Point-LIO** (LiDAR-inertial only, Livox-native, lowest risk) if visual
fusion isn't required; **FAST-LIVO2** (LiDAR+visual+inertial, closest public
match to our exact sensor triplet) if it is. LIO-SAM and DLIO also viable,
LVI-SAM has less mature ROS2 support.

---

## OpenPCDet comparison — 2026-08-10

User pointed at two local reference repos, siblings under
`~/Projects/Thesis/` (capital T — a different directory from this repo's
`~/Projects/thesis/G1_sim`, easy to typo):

- `~/Projects/Thesis/OpenPCDet` — full OpenPCDet checkout with two real
  pretrained checkpoints in `checkpoints/`: `pointpillar_7728.pth` (KITTI,
  77.28 AP) and `pv_rcnn_8369.pth`.
- `~/Projects/Thesis/livox_detection` — the actual upstream repo Plan.md's
  CenterPoint port (`detection/livox_centerpoint.py`) was reverse-engineered
  from. `livoxdetection/models/{resfpn,boolmap,centerhead}.py` match the
  filenames Plan.md cites for the five bugs found during the port. Its
  `pt/livox_model_1.pt` / `livox_model_2.pt` are the same checkpoints already
  sitting in `G1_sim/detection/pt/` — confirms the port used the right
  weights, nothing to redo there.

**`pcdet` is already built** in the `livox` conda env (torch 2.5.1+cu121,
spconv 2.3.6, `pcdet` 0.6.0 in editable/dev mode against the OpenPCDet
checkout). Smoke-tested end to end: loaded `pointpillar_7728.pth` via
`pcdet.models.build_network`, ran a forward pass on random points, got 124
candidate boxes back pre-NMS-threshold. The checkpoint and the full
voxelize → PillarVFE → BaseBEVBackbone → AnchorHeadSingle → NMS pipeline
genuinely work on this machine, today.

### PointPillar: ours vs. OpenPCDet's

| | `g1_perception_ws/.../pointpillar_model.py` (ours) | OpenPCDet `pointpillar_7728.pth` |
|---|---|---|
| Weights | **none** — written from scratch, fallback-only (per Tasks.md, confirmed still true) | **real, pretrained**, 77.28 KITTI moderate-Car AP |
| Backbone | symmetric 4×stride-2 + 4×transpose-conv (custom) | `BaseBEVBackbone`: 3 blocks `[3,5,5]` layers, 3 upsample branches concatenated to 384ch — the paper's actual architecture |
| Voxel size / grid | 0.2 m, 1120×448, range 224×89.6 m (built for Mid-360's claimed 200 m spec) | 0.16 m, range 69.12×79.36 m, KITTI front-camera FOV only |
| Classes | car, pedestrian, cyclist (livox_detection's ordering) | Car, Pedestrian, Cyclist (KITTI) |
| Pedestrian anchor | n/a (untrained) | `(0.8, 0.6, 1.73)` — close to our CenterPoint port's own synthetic-scene validation extents `(0.59, 0.58, 2.08)`, a useful independent cross-check that our human-scale assumptions aren't off |

**The state dicts are not interchangeable** — different module shapes mean
`pointpillar_7728.pth` cannot be `load_state_dict`'d into our custom
`PointPillar` class as-is. Two real paths forward, not attempted yet:

1. **Wrap real OpenPCDet as a backend** (new `openpcdet_node.py` in
   `g1_perception_ws`, calling `pcdet.models.build_network` +
   `load_params_from_file` directly, same pattern as the smoke test above).
   Gives an actually-trained detector today, at the cost of KITTI's forward-
   only point range — would need either a crop/remap of the 360° Mid-360
   cloud to a forward sector, or retraining/fine-tuning on Mid-360-shaped
   data to use it 360°.
2. **Reimplement our `Backbone2D`/`SSDHead` to match `BaseBEVBackbone`/
   `AnchorHeadSingle` exactly**, so `pointpillar_7728.pth` loads into our own
   classes and our existing 360°/224 m range config keeps working.

Option 1 is less work and gets real weights running sooner; option 2 keeps
the existing 360° range but is a rewrite with no shortcut to verify against
except re-deriving the checkpoint's tensor shapes the same way the
CenterPoint port did. Recommend 1 first, since it's the same "does this
component actually work" question Task 7 already needed answered, and this
turn's smoke test shows the answer is yes, standalone — the only unknown is
whether it's useful on a 360° pedestrian scene once wired to the sim's real
cloud.

⬜ Neither option implemented. This is a scoping note, not a completed task.

---

## Tooling added — 2026-08-10

- ✅ **Isaac Sim skill** installed via `npx skillfish add a5c-ai/babysitter
  isaac-sim` → `~/.claude/skills/isaac-sim` (also mirrored to 5 other agent
  tools' skill dirs by the same command — Copilot, Gemini CLI, Antigravity,
  OpenClaw, OpenHands).
- ✅ **Isaac Sim MCP server** (`whats2000/isaacsim-mcp-server`,
  `feat/isaac-sim-6.0.0-support` branch) cloned to
  `~/Projects/thesis/mcp-servers/isaacsim-mcp-server`. Already had a pip
  install (`isaacsim-mcp-server` 0.6.0) sitting in the `isaac` conda env from
  a prior session — registered that binary with Claude Code:
  `claude mcp add isaac-sim -s user -- /home/thakk100/miniconda3/envs/isaac/bin/isaacsim-mcp-server`
  (user scope, so it's available regardless of which project directory
  Claude is running from — the first attempt scoped it to whatever `cwd`
  happened to be at add-time, which was wrong; fixed).
  Connects over stdio to a TCP socket (`localhost:8766`) that a running Isaac
  Sim's `isaac.sim.mcp_extension` extension listens on. **Not yet exercised
  against a live sim** — needs Isaac Sim launched with
  `--ext-folder ~/Projects/thesis/mcp-servers/isaacsim-mcp-server --enable isaac.sim.mcp_extension`
  (the pip `isaacsim` Kit launcher works for this; the repo's own
  `run_isaac_sim.sh` expects a standalone app-bundle install at `$ISAACSIM_ROOT`
  which this machine doesn't have — it's a pip package instead).

  Since registered and **verified live**: extension auto-starts with zero
  extra flags on any `isaac`/`env_isaaclab` Isaac Sim launch (persistent
  `user.config.json`, not CLI args) — port 8766 listening, confirmed via a
  real sim run.

---

## Warehouse SDG pipeline — humans + Nova Carters + G1 patrol — 2026-08-10

3-process pipeline (populated warehouse sim, G1 patrol commander, rosbag
capture) per user request, using Isaac Replicator Agent (IRA) for wandering
humans/Nova Carters. Full status, exact run commands, and agent/human
verification checklists: **`Verify_Warehouse.md`**. Session recap:
**`SESSION_SUMMARY.md`**. Consolidated open items across today's whole
session: **`FUTURE_STEPS.md`**.

- ✅ **Process 1 (sim) verified**: `scripts/g1_warehouse_sim.py` — warehouse
  + 2 IRA humans + 1 Nova Carter + G1 (WBC locomotion, RTX LiDAR, camera,
  new IMU publisher, cmd_vel subscriber) ran cleanly headless for 120s.
- ✅ Two real bugs found and fixed: (1) `g1_rtx_sim.py`'s Nucleus-room
  loader passed a catalog-relative path straight to `add_reference_to_stage`
  without resolving it first — silently failed every time, always falling
  back to flat ground; fixed in new `g1_sim/warehouse.py`. (2) IRA's own
  `ensure_navmesh_ready()` polls with a **hard-coded 100-frame cap**; the
  real warehouse mesh (291 meshes) genuinely bakes fine but needs ~600-700
  frames — every attempt failed with a bogus error. Patched the poll budget
  up to 3000 frames in new `g1_sim/ira_actors.py`.
- 🔄 **Process 2** (`scripts/g1_patrol.py`, open-loop timed `/g1/cmd_vel`
  polygon patrol) written, not yet run together with process 1.
- ⬜ **Process 3** (rosbag capture + bag-to-video) — capture command
  documented in `Verify_Warehouse.md`, full 3-minute run not yet executed,
  bag→video script not yet built.
