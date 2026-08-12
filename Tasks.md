# Tasks

Status board for the G1 + Livox Mid-360 + camera + WBC + warehouse pipeline.
Design rationale/history → [Plan.md](Plan.md). Run commands/gotchas →
[HANDOFF.md](HANDOFF.md).

Legend: ✅ done · 🔄 in progress · ⛔ blocked · ⬜ not started

---

## Core pipeline (`scripts/g1_warehouse_sim.py`)

- ✅ **G1 USD** — auto-converted from URDF (`scripts/convert_g1_urdf_to_usd.py`
  → `assets/g1_29dof_sensors.usd`), all 29 DOF + `mid360_link`/`d435_link`
  mounts preserved.
- ✅ **RTX Mid-360 LiDAR** — spawned on `mid360_link` (identity local
  transform — see Plan.md for the below-ground-bug fix), publishes
  `/livox/mid360/points` via in-sim rclpy (`g1_sim/rtx_publisher.py`,
  `RtxLidarPublisher`). NOT the OG `ROS2RtxLidarHelper` graph — that
  advertises the topic but never emits on this Isaac Sim build.
- ✅ **D435 camera** — RGB/depth/semantic + camera_info
  (`/g1/camera/{rgb,depth,semantic,camera_info}`), plus a synthesized
  `/g1/camera/depth/color/points` (`g1_sim/rgbd_publisher.py`).
- ✅ **decoupled_wbc locomotion** — Balance/Walk ONNX policies via
  `g1_sim/wbc_bridge.py`, driven by `/g1/cmd_vel` at 50 Hz. Runs by default
  (no flags needed); `--no-locomotion` to disable, `--freeze-robot` to hold
  the spawn pose instead (gravity disabled, for sensor-only testing).
- ✅ **IMU** — `g1_sim/rtx_camera.py` spawn/attach → `/g1/imu`.
- ✅ **Robot state** — `/tf`, `/g1/joint_states`, `/clock` via
  `attach_robot_state_publishers`.
- ✅ **Warehouse + IRA actors** — real warehouse USD (`g1_sim/warehouse.py`,
  Nucleus-path bug fixed) + wandering humans/Nova Carters
  (`g1_sim/ira_actors.py`, navmesh poll-budget bug fixed). `--cache-scene`
  loads a pre-baked scene (`assets/warehouse_ira_baked.usd`) to skip the
  ~60–80 s warehouse-download + navmesh-bake on every run — actors sit
  static at their saved pose in that path (not verified either way whether
  wander behavior resumes; use `--rebake` when actor motion matters).
  `--no-ira` falls back to a plain warehouse + static pedestrian boxes.
- ✅ **Runs cleanly headless** — verified end to end multiple times,
  including a 120 s run with 2 IRA humans + 1 Nova Carter + full sensor
  suite + WBC.
- ✅ **Diag/debug scripts removed** (2026-08-11) — `scripts/debug_prims.py`,
  `debug_ros2_bridge.py`, `debug_rtx_graph.py`, `diag_lidar_debug.py`,
  `diag_lidar_elevation.py`, `diag_lidar_points_stats.py`,
  `diag_lidar_worldz.py`, `diag_warehouse_lidar.py` all deleted. The
  production entrypoint is `scripts/g1_warehouse_sim.py` only
  (`g1_rtx_sim.py` remains as the bare-warehouse-free alternative scene per
  its own docstring).

## Open — RTX LiDAR coverage ⛔

Published cloud is a **partial band, not a full 360° ring**: azimuth
clustered ~50°–180°, elevation ~0°–40°, against real warehouse mesh geometry
(not a missing-geometry artifact — 781 real meshes present). Each individual
`emitterState` JSON genuinely spans the full `-180..180`/`-5.7..50` range, so
the drop happens between the authored USD attributes and the returned
`GenericModelOutput` hits. Leading unconfirmed hypothesis: self-occlusion
from the mount's 180°-roll orientation. Next diagnostic step: isolate on a
real `UsdGeom.Mesh` ground plane (not `Cube` — RTX likely doesn't ray-trace
implicit primitives) and try the pre-fix mount orientation (2.3° pitch, no
roll) to test the self-occlusion theory directly. → `docs/RTX_LIDAR_INIT.md`.

**New measurement, 2026-08-11 (simple-room scene, `--no-ira`, upright
balancing robot, RViz added — see below):** measured directly off the live
topics with a one-shot `rclpy` subscriber (`x,y,z` → range/elevation/azimuth
per point), not just eyeballed in RViz:

| Topic (prim) | points/window | elevation range | azimuth range |
|---|---|---|---|
| `/livox/mid360/points/a` | **0** | — | — |
| `/livox/mid360/points/b` | 1062 | **+11.2° … +87.9°** | −180° … +180° |
| `/livox/mid360/points/c` | 1029 | **+11.1° … +87.4°** | −180° … +180° |
| `/livox/mid360/points/d` | **0** | — | — |

Two distinct problems, not one: (1) prims A and D return **zero points**
every window in this scene/pose — matches the `raw_points=0` lines already
in `[WH] lidar per-prim diagnostics` output, so this isn't new, just now
confirmed at the ROS2-message level too. (2) Prims B and C, which do return
points, are **entirely upper-hemisphere** — elevation never goes below
+11°, so there are **no ground returns, no near-field returns, nothing at
or below sensor height** in either working prim. That's a stronger symptom
than the previously-logged "~0°–40°" band and is visually obvious in RViz:
two disconnected vertical streaks (ceiling/upper-wall hits), no floor plane,
matching the reporter's "point clouds are not alright." Azimuth itself is
fine here (full −180…180 on both working prims) — the defect is purely in
elevation, consistent with the self-occlusion hypothesis above (something
below the mount, physically or in sign convention, is blocking/discarding
every downward-pointing ray) but not yet isolated to a single cause.
**Not yet root-caused** — next step is still the flat-`Mesh`-ground
isolation test above, now with a concrete elevation-sign regression to
check for first (compare `gen_mid360_rtx_config.py`'s per-prim elevation
sign convention against the identity-mount fix from HANDOFF.md, since that
fix changed which transform the emitter's elevation angle composes with).

## Locomotion — untested edges

- ⬜ Turning/strafing (`vy`/`wz`) — only forward (`vx=0.5`) exercised live;
  structurally identical in the observation contract but unverified.
- ⬜ 50 Hz-trained vs. 60 Hz-physics control-rate mismatch — stable so far,
  not an exact reproduction of training conditions.
- ⬜ Lateral drift — MuJoCo validation showed ~0.68 m sideways drift over
  17 s of a pure-forward command; genuine gait asymmetry vs. harness
  artifact is unknown.
- ⬜ Longest live Isaac Sim run so far is ~15 s sim-time; no multi-minute
  soak test yet.

## Warehouse SDG pipeline (3-process design)

- ✅ **Process 1 (sim)** — `g1_warehouse_sim.py`, verified.
- 🔄 **Process 2 (`scripts/g1_patrol.py`)** — standalone open-loop
  `/g1/cmd_vel` polygon-patrol commander, written, never run together with
  process 1. Dead-reckoning only, no localization — will drift and not
  close its loop by design; real odometry/SLAM would be needed for a
  closed-loop version.
- ⬜ **Process 3 (rosbag capture)** — capture command documented, full
  3-minute run not yet executed, bag→video script not yet built
  (`cv_bridge` + OpenCV `VideoWriter` available in the ROS2 Jazzy env, just
  not wired up).

## Detection

- ✅ CenterPoint ported to torch 2.7/CUDA 12.6 (`detection/livox_centerpoint.py`),
  validated against synthetic scene (see Plan.md).
- ✅ Tested against the RTX sensor's real published cloud — runs, 0
  detections, expected (targets were inside the blind radius / outside the
  covered band at the time).
- ⬜ Not yet tried against a populated warehouse scene with IRA humans.
- ⬜ **OpenPCDet PointPillar integration** — real pretrained weights
  confirmed working standalone in a separate smoke test; not wired into
  this pipeline. See Plan.md for the two scoped integration paths.
- `g1_perception_ws/` (sibling ROS2 workspace): `lidar_bridge`,
  `detection_bridge`, `ccvnorm_node` (depth completion), `centerpoint_node`,
  `pointpillar_node` all build clean (`colcon build --symlink-install`), ⬜
  never tested against a live sim or real robot.

## SLAM — not integrated

Ultra-Fusion runs correctly via Docker on this machine but has no sensor
profile for our legged/wheel-less robot and no real Mid-360/D435 extrinsics
calibrated yet. See Plan.md for alternatives (FAST-LIO2, FAST-LIVO2, etc.).

## Infra notes

- GPU is 8 GB — only one Isaac Sim instance fits. Kill stragglers before
  every run: `pkill -9 -f g1_warehouse_sim`.
- Disk is tight machine-wide (~5 GB free / 355 GB at last check, mostly
  unrelated causes) — check `df -h /` before any large asset/Docker pull.
- GEAR-SONIC (`gear_sonic_deploy`) is parked, not finished — TensorRT 10.13
  installed, checkout untouched under
  `third_party/GR00T-WholeBodyControl/gear_sonic_deploy/`, in case its
  fuller manipulation+locomotion capability is wanted later.
- `record_g1_bag.sh` targets **real-hardware** topics (`/utlidar/cloud`,
  `/g1/odom`), neither of which this sim publishes — don't reach for it
  when testing the sim pipeline; use a raw `ros2 bag record` command
  instead (topic list: `/livox/mid360/points`, `/g1/camera/*`, `/tf`,
  `/g1/joint_states`, `/g1/imu`, `/clock`).
