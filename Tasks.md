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

## Task 5 — cmd_vel bridge ⛔ **not built**

Only `action_graph.read_cmd_vel()` exists — a getter. There is no
`cmd_vel_bridge.py`, and `g1_ros2_sim.py` never calls the getter, so publishing
to `/g1/cmd_vel` currently does **nothing**. The robot holds its default stance
regardless of input.

⚠️ No G1 locomotion policy checkpoint located. The ANYmal-C policy in
`simple_lidar_integration.py` will not drive a 29-DOF humanoid. Search IsaacLab
for a G1 velocity-tracking checkpoint first; if absent, fall back to a PD
stand-still hold so the cmd_vel path is still demonstrable, and track real
locomotion as follow-up.

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
