# Session summary — 2026-08-10

What actually shipped today, in the order it happened. Live status lives in
[Tasks.md](Tasks.md)/[Plan.md](Plan.md); this file is a session-scoped recap.

## Tooling

- **Isaac Sim skill** installed (`npx skillfish add a5c-ai/babysitter
  isaac-sim`) → `~/.claude/skills/isaac-sim`.
- **Point cloud visualization skill** (`~/.claude/skills/pointcloud-viz`) —
  renders `.pcd`/`.ply`/`.csv`/`.npy` to PNG via Open3D (matplotlib
  fallback), optional detection-box overlay.
- **Isaac Sim MCP server** (`whats2000/isaacsim-mcp-server`,
  `feat/isaac-sim-6.0.0-support`) cloned to
  `~/Projects/thesis/mcp-servers/isaacsim-mcp-server`, registered with
  Claude Code, and configured to **auto-start with any Isaac Sim launch** in
  either the `isaac` (6.0) or `env_isaaclab` (5.1) conda env — via each Kit
  app's persistent `user.config.json`
  (`persistent.app.exts.userFolders`/`.enabled`), not launch flags. Verified
  live: extension auto-started at ~8s into boot, port 8766 listening, zero
  extra flags.

## RTX LiDAR — first live test since the pivot

First actual run of the RTX pipeline (it had been written but never
executed). Confirmed live: `/clock`, `/tf`, `/g1/joint_states`,
`/livox/mid360/points` all real over ROS2 at ~11 Hz. Found a real bug:
the published cloud was clipped to azimuth ±90° (not a full 360° ring),
traced to `IsaacCreateRTXLidarScanBuffer` being deprecated in this Isaac Sim
6.0.1 build (its own log points at `GenericModelOutput` as the fix — not yet
ported). Also fixed **RViz showing nothing**: `Fixed Frame` was `odom`,
which doesn't exist in the real TF tree (root is `World`) — RViz silently
renders nothing when the fixed frame is missing. Full trace in `Tasks.md` →
"Live verification — 2026-08-10".

## OpenPCDet comparison

Found real pretrained PointPillar weights (`pointpillar_7728.pth`, 77.28
KITTI AP) in a sibling `OpenPCDet` checkout, working end-to-end via a live
forward pass — something our from-scratch PointPillar never had. Different
backbone shape means the checkpoint can't load directly into our model;
two integration paths scoped in `Tasks.md`, not yet implemented.

## cmd_vel locomotion — decoupled_wbc — ✅ working

Task 5 (`/g1/cmd_vel` → walking) is done. After three false starts
(a manipulation-only policy mistaken for locomotion, IsaacLab's
untrained/wrong-DOF config, NVIDIA GEAR-SONIC needing a TensorRT C++ build
and mocap references rather than plain velocity commands), NVIDIA's
`decoupled_wbc` module (plain Python, ONNX+numpy) fit directly. Live-verified
in Isaac Sim: stable standing, then 9s of continuous walking driven by real
`ros2 topic pub` messages on `/g1/cmd_vel`. Two real bugs found and fixed:
a stale filename bug in NVIDIA's own vendored config, and a PD-gain mismatch
(our USD bakes a uniform 100/10 drive into every joint; the policy needs
per-joint gains up to 250) that collapsed the robot until fixed with an
explicit `Articulation.set_gains()` call — **this will bite any future
externally trained policy against this USD**, not just this one. Full trace:
`docs/decoupled_wbc_findings.md`.

## SLAM — Ultra-Fusion

Investigated per request as a LiDAR+camera+IMU SLAM option matching our
exact sensor pair. No public source yet, but the prebuilt Docker image runs
correctly on this machine despite targeting a different ROS distro (Humble/
22.04 vs. our Jazzy/24.04) — Docker's own userspace isolation sidesteps that.
Missing: a sensor profile for a legged, wheel-less robot (none of Ultra-
Fusion's shipped configs assume that) with real Mid-360/D435 extrinsics we
haven't calibrated. Full trace: `docs/ultra_fusion_findings.md`.

## Warehouse SDG pipeline — humans + Nova Carters + G1 patrol + rosbag

Built a 3-process pipeline: a populated warehouse scene (Isaac Replicator
Agent driving wandering humans and Nova Carters), a standalone G1 patrol
commander, and rosbag capture. **Process 1 (the sim) is verified working**:
warehouse + 2 humans + 1 Nova Carter + G1 (WBC locomotion, RTX LiDAR,
camera, new IMU publisher, cmd_vel subscriber) ran cleanly for 120s
headless. Two real bugs found and fixed along the way:

1. `g1_rtx_sim.py`'s Nucleus-room loader passed a catalog-relative path
   straight to `add_reference_to_stage` without resolving it against the
   asset root first — silently failed every time, falling back to flat
   ground. Fixed in the new `g1_sim/warehouse.py`.
2. Isaac Replicator Agent's own `ensure_navmesh_ready()` polls for a
   navmesh with a **hard-coded 100-frame cap**, no config knob. The real
   warehouse mesh (291 meshes) genuinely bakes fine but needs ~600-700
   frames — every attempt failed with a bogus "navmesh building failed"
   even though baking was progressing normally. Worked around in
   `g1_sim/ira_actors.py` by patching the poll budget up to 3000 frames.

**Process 2** (`scripts/g1_patrol.py`, timed open-loop `/g1/cmd_vel`
waypoint-polygon patrol) is written but not yet run live. **Process 3**
(rosbag capture + bag-to-video) is not yet finished — the topic list and
capture command still need to be nailed down and run for the full 3-minute
target. See `Verify_Warehouse.md` for exact current status and
`FUTURE_STEPS.md` for what's left.

## New/changed files (this session)

```
g1_sim/wbc_bridge.py          new — decoupled_wbc Balance/Walk ONNX bridge
g1_sim/warehouse.py            new — warehouse USD loader (Nucleus-path fix)
g1_sim/ira_actors.py           new — IRA humans/Nova Carter spawning (navmesh fix)
g1_sim/rtx_camera.py           modified — cmd_vel subscriber, IMU publisher
scripts/g1_rtx_sim.py          modified — WBC bridge wired into main loop
scripts/g1_warehouse_sim.py    new — warehouse + IRA + G1 + WBC, all sensors
scripts/g1_patrol.py           new — standalone cmd_vel patrol commander
assets/policy/                 new — Balance/Walk ONNX checkpoints
docs/decoupled_wbc_findings.md new
docs/ultra_fusion_findings.md  new
rviz/g1.rviz                   modified — Fixed Frame odom→World, Decay Time
Plan.md, Tasks.md, README.md   modified — findings merged in
```
