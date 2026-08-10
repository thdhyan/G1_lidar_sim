# G1_sim — Unitree G1 with Livox Mid-360, camera, and ROS2

A reusable Isaac Sim / IsaacLab setup for the Unitree G1 humanoid carrying a
Livox Mid-360 LiDAR and a D435 camera, published over ROS2 for RViz
visualisation and 3D pedestrian detection.

> **Status: work in progress.** The LiDAR gate and the USD conversion pass. The
> ROS2 layer and scene are written but have **not yet completed a run**, and
> `cmd_vel` has no locomotion behind it. See [Verify.md](Verify.md) for what is
> proven versus assumed, [Tasks.md](Tasks.md) for the status board, and
> [Plan.md](Plan.md) for the design rationale.

---

## Environment

Verified on this machine — these are measured values, not requirements copied
from upstream docs:

| Item | Value |
|---|---|
| Conda env | `env_isaaclab`, Python 3.11.15 |
| Isaac Sim | 5.1.0 |
| IsaacLab | pip package `isaaclab` 2.3.1 |
| ROS2 | jazzy (`/opt/ros/jazzy`, Python 3.12) |
| torch | 2.7.0+cu126 |
| GPU | RTX 4060 Laptop 8 GB (Ada, sm_89), nvcc 12.0 |

### Two environment facts that drive the whole design

**1. IsaacLab lives in the pip package, not `~/Projects/IsaacLab`.**
`import isaaclab` resolves to:

```
~/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/
```

The separate `~/Projects/IsaacLab` checkout is version 3.0, whose ray-caster was
refactored into a backend-dispatch design (`FactoryBase`, warp buffers) that the
OmniPerception fork is incompatible with. **Install into the pip package.**

**2. rclpy works inside the simulator.** System rclpy is built for Python 3.12
and cannot import into the 3.11 conda env. But enabling `isaacsim.ros2.bridge`
loads a *bundled* jazzy rclpy:

```
Attempting to load system rclpy → Could not import system rclpy
Attempting to load internal rclpy for ROS Distro: jazzy → rclpy loaded
```

So the simulator can publish ROS2 messages in-process — no shared memory, no
rebuilding ROS2. **Enable the extension before any `import rclpy`.**

---

## Installation

### 1. LidarSensor into IsaacLab

Follows the OmniPerception README's manual option, retargeted at the pip package.
Back up first — this overwrites `ray_caster.py` and two `__init__.py` files:

```bash
PKG=~/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/sensors
SRC=~/Projects/thesis/OmniPerception/LidarSensor/LidarSensor/example/isaaclab/isaaclab

cp $SRC/sensors/lidar_sensor*.py                      $PKG/
cp $SRC/sensors/ray_caster/ray_caster*.py             $PKG/ray_caster/
cp $SRC/sensors/ray_caster/patterns/patterns*.py      $PKG/ray_caster/patterns/
cp $SRC/init_files/sensors_init.py                    $PKG/__init__.py
cp $SRC/init_files/patterns_init.py                   $PKG/ray_caster/patterns/__init__.py

mkdir -p $PKG/ray_caster/patterns/scan_patterns
cp ~/Projects/thesis/OmniPerception/LidarSensor/LidarSensor/sensor_pattern/sensor_lidar/scan_mode/*.npy \
   $PKG/ray_caster/patterns/scan_patterns/
```

A backup taken during setup lives in `backup_pip_isaaclab_*/`; restore with:

```bash
cp -r $(cat .backup_pip_path)/* $PKG/
```

### 2. Detection weights

Already fetched into `detection/pt/` (~15 MB each) from the upstream repo:

```bash
curl -sSL -o detection/pt/livox_model_1.pt \
  https://raw.githubusercontent.com/Livox-SDK/livox_detection/master/pt/livox_model_1.pt
```

---

## Verify the install

**→ [Verify.md](Verify.md)** is the full procedure: which terminal runs what,
what passing output looks like, and how to fix each failure. Work through it in
order — the steps are arranged to isolate faults.

The first gate on its own:

```bash
conda activate env_isaaclab
python scripts/00_verify_warp_lidar.py     # expect [GATE] PASS
```

⚠️ **Kill leftover simulators before every run.** Only one Isaac Sim fits in
16 GB; orphans cause a silent OOM kill with no traceback.

```bash
pkill -9 -f "python.*G1_sim/scripts/"
free -m | head -2      # want >6000 MB available
```

Additional diagnostics, in increasing depth:

| Script | Question it answers |
|---|---|
| `scripts/debug_prims.py` | Which USD prims exist, and which are ray-castable geometry? |
| `scripts/debug_lidar_meshes.py` | What meshes did the sensor bind, and where do rays land? |
| `scripts/debug_raycast.py` | Is the warp mesh itself hittable by a hand-rolled ray? |

---

## Detection

The upstream `livox_detection` targets Python 3.8 / torch 1.8.2 / CUDA 10.2,
which cannot run on an Ada-generation GPU. Only the pretrained checkpoints are
reused; the network is re-declared against current torch in
`detection/livox_centerpoint.py`.

Fidelity is verified rather than assumed: the checkpoint loads with **0 missing
and 0 unexpected keys** across all 275 tensors, and on a synthetic scene the
model recovers all three planted pedestrians to within ~0.1 m at ~189 ms/frame.

```bash
# CenterPoint on GPU (default)
python detection/detection_node.py --checkpoint detection/pt/livox_model_1.pt

# Geometric fallback - no torch, no GPU, no weights
python detection/detection_node.py --backend clustering
```

| Topic | Type | Notes |
|---|---|---|
| `/livox/mid360/points` | `sensor_msgs/PointCloud2` | subscribed |
| `/g1/detections` | `vision_msgs/Detection3DArray` | published |
| `/g1/detection_markers` | `visualization_msgs/MarkerArray` | published |

Classes are `car`, `pedestrian`, `cyclist`; pedestrians render yellow.

---

## Planned ROS2 interface

Not yet wired up — these are the topics Tasks 3–6 will provide.

| Topic | Type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim → |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | sim → |
| `/g1/joint_states` | `sensor_msgs/JointState` | sim → |
| `/livox/mid360/points` | `sensor_msgs/PointCloud2` | sim → |
| `/g1/camera/rgb`, `/depth` | `sensor_msgs/Image` | sim → |
| `/g1/camera/camera_info` | `sensor_msgs/CameraInfo` | sim → |
| `/g1/odom` | `nav_msgs/Odometry` | sim → |
| `/g1/cmd_vel` | `geometry_msgs/Twist` | → sim |

All external nodes must run with `use_sim_time:=true`.

---

## Sensor mounts

Both already exist in `g1_29dof.urdf` — no mount authoring was required:

| Link | Parent | xyz | rpy |
|---|---|---|---|
| `mid360_link` | `torso_link` | `0.0002835 0.00003 0.4188` | `3.14 0 0` |
| `d435_link` | `torso_link` | `0.0576235 0.01753 0.41987` | `0 0.8307767 0` |

⚠️ `mid360_link` is roll-π — the sensor is mounted inverted. An upright point
cloud in RViz is the acceptance test for getting this right.

---

## RTX LiDAR pivot (Isaac Sim 6.0)

Everything above this line is the original warp-ray-caster design on Isaac
Sim 5.1 (`env_isaaclab` conda env). The project has since pivoted to **RTX
LiDAR with emitter-state arrays** on **Isaac Sim 6.0.1** (`isaac` conda env,
Python 3.12, pip-installed — no standalone app bundle on this machine). Full
rationale in `Plan.md`, status board in `Tasks.md`. Quick facts as of the
first live run (2026-08-10):

- Real ROS2 pipeline confirmed live (`/clock`, `/tf`, `/g1/joint_states`,
  `/livox/mid360/points` at ~11 Hz).
- Point cloud is currently clipped to azimuth ±90° (not a full 360° ring) —
  traced to the `IsaacCreateRTXLidarScanBuffer` annotator being deprecated in
  this Isaac Sim build; fix is to port to `GenericModelOutput`. See
  `Tasks.md` → "Live verification — 2026-08-10" for the full trace.
- Run it: `conda activate isaac && python scripts/g1_rtx_sim.py --headless`.
- **Locomotion works now** (2026-08-10): `/g1/cmd_vel` drives the robot via
  NVIDIA's `decoupled_wbc` Balance/Walk ONNX policies — live-verified
  standing and walking. `--no-locomotion` to disable. See `Tasks.md` Task 5
  and `docs/decoupled_wbc_findings.md`.

## SLAM

`sjtuyinjie/Ultra-Fusion` (LiDAR+camera+IMU) runs on this machine via Docker
(no public source yet, but the prebuilt image works despite targeting a
different ROS distro than our host — see `docs/ultra_fusion_findings.md`).
Not wired to our sensors yet — needs a custom config profile with real G1
Mid-360/D435 extrinsics we haven't calibrated. See `Tasks.md` → "SLAM
investigation — Ultra-Fusion".

## Tooling

- **Isaac Sim skill** (`~/.claude/skills/isaac-sim`), installed via
  `npx skillfish add a5c-ai/babysitter isaac-sim`.
- **Point cloud visualization skill** (`~/.claude/skills/pointcloud-viz`):
  renders any `.pcd`/`.ply`/`.csv`/`.npy` cloud to a PNG (Open3D, falls back
  to matplotlib), with optional detection-box overlay. `python
  ~/.claude/skills/pointcloud-viz/visualize.py <input> -o <out.png>`.
- **Isaac Sim MCP server** (`whats2000/isaacsim-mcp-server`,
  `feat/isaac-sim-6.0.0-support`), cloned to
  `~/Projects/thesis/mcp-servers/isaacsim-mcp-server` and registered with
  Claude Code (`isaac-sim`, user scope). Bridges to a running Isaac Sim's
  `isaac.sim.mcp_extension` extension over a TCP socket (`localhost:8766`).
  **Auto-starts now** — no `--ext-folder`/`--enable` flags needed. Configured
  via each Kit app's persistent settings
  (`.../isaacsim/kit/data/Kit/Isaac-Sim Python/{5.1,6.0}/user.config.json`,
  keys `persistent.app.exts.userFolders` /  `.enabled`), so it comes up with
  *any* launch of Isaac Sim in either the `isaac` (6.0) or `env_isaaclab`
  (5.1) conda env, including this project's own scripts. Verified live
  2026-08-10: `isaac.sim.mcp_extension-0.6.0` auto-started at ~8s into boot,
  port 8766 listening, zero extra flags. (A stale reference to an old
  `~/Projects/isaacsim-mcp-server` checkout in a third, unrelated
  `Isaac-Sim/5.1` app config was also found and fixed to point here.)
- **RViz fix (2026-08-10)**: `rviz/g1.rviz`'s `Fixed Frame` was `odom`, which
  doesn't exist anywhere in the actual published TF tree (root is `World` —
  checked live via `ros2 topic echo /tf`). RViz silently renders nothing when
  the fixed frame is missing, which read as "no data" even though the sensor
  pipeline was fine. Fixed to `World`; also bumped `Decay Time` 0→3s since
  the Mid-360's non-repetitive scan means a single frame is a thin slice, not
  a ring — needs a few seconds of accumulation to look like anything.
  Separately, if RViz crashes with a `libpthread.so.0` / `GLIBC_PRIVATE`
  symbol error when launched from a terminal inside this VSCode session, it's
  VSCode's snap confinement leaking `GTK_PATH`/`GIO_MODULE_DIR`/`LOCPATH`
  (pointing into `/snap/core20`) into the shell — unset those before
  launching `rviz2`, don't touch `LD_LIBRARY_PATH` (ROS needs it as-is for
  `libOgreMain`).

## OpenPCDet comparison

`~/Projects/Thesis/OpenPCDet` (note: capital-T `Thesis`, a sibling directory,
not this repo) has real pretrained checkpoints — `pointpillar_7728.pth`,
77.28 KITTI AP — that our own from-scratch, weights-free
`g1_perception_ws` PointPillar doesn't have. Confirmed working end-to-end via
a live forward pass in the `livox` conda env (`pcdet` 0.6.0 already built
there). Different backbone shape means the checkpoint can't load directly
into our model class as-is; two integration paths are scoped in `Tasks.md` —
not yet implemented.

---

## Layout

```
G1_sim/
├── README.md · Plan.md · Tasks.md
├── scripts/           # verification, diagnostics, sim entrypoints
├── detection/         # ported livox_detection + ROS2 node
│   └── pt/            # pretrained checkpoints
├── g1_sim/            # config, action graph, publishers  (pending)
├── assets/            # generated USD                     (pending)
├── launch/ · rviz/    #                                   (pending)
└── backup_pip_isaaclab_*/
```
