# Verification Plan — RTX LiDAR azimuth fix + perception workspace

Covers: commit `7aa61bf` (azimuth/orientation fix) and the new
`g1_perception_ws` (Unitree SDK bridge + CenterPoint + PointPillar).

## 1. RTX LiDAR fix (commit 7aa61bf)

**Claim**: `validStartAzimuthDeg=0/360` was clipping all negative-azimuth
emitter rays against a `[-180,180]` emitter range, producing a narrow sector.
Fixed to `[-180,180]`. Also fixed sensor orientation (was 180° roll,
self-occluding into the body; now ~2.3° pitch per Unitree URDF) and mount
height (0.4188 → 0.40618 m).

### Steps

```bash
pkill -9 -f "[p]ython.*g1_rtx_sim" 2>/dev/null
conda activate isaac
cd ~/Projects/thesis/G1_sim
python scripts/g1_rtx_sim.py --num-prims 2 --config-dir assets/lidar_configs_fast
```

```bash
# second terminal
source /opt/ros/jazzy/setup.bash
cd ~/Projects/thesis/G1_sim
ros2 launch launch/g1_bringup.launch.py detection:=false
```

### Pass criteria

Run for real 2026-08-10 (first time since the fix landed — this file had
been sitting all-unchecked). Results below; full writeup in `Tasks.md` under
"Live verification — 2026-08-10".

- [x] `ros2 topic hz /livox/mid360/points` ≈ 10 Hz — measured ≈ 11 Hz.
- [x] `ros2 topic echo /livox/mid360/points --field width --once` shows a
      point count in the thousands — measured ~11k-20k/message.
- [ ] RViz point cloud forms a **full ring** around the robot torso, not an
      arc confined to one quadrant. **FAILS.** Azimuth measured numerically
      at ±90° only (not ±180°), reproduced with both `--num-prims 2` and all
      4 prims, so it isn't the prim-count knob. The JSON configs themselves
      have correct full-360° azimuth data (checked directly). Root cause:
      `IsaacCreateRTXLidarScanBuffer` (what `rtx_publisher.py` uses) is now
      deprecated in this Isaac Sim 6.0.1 build; the sim log says outright to
      "Use the GenericModelOutput annotator directly to access full-scan RTX
      Lidar data" — i.e. the deprecated annotator no longer returns the full
      scan. Not a leftover of the original clipping bug; a new regression on
      top of the fix. Fix direction: port `rtx_publisher.py` to
      `GenericModelOutput`. Not attempted this session.
- [ ] Points originate at the robot's torso height and radiate outward — not
      independently confirmed; blocked on the same azimuth clipping above
      (can't judge radiation pattern from a ±90° slice).
- [ ] With `simple_room.usd` loaded, points terminate on walls/furniture —
      not reached; Nucleus wasn't reachable this session (`Nucleus room not
      available ... using flat ground`), so this was never using the room
      geometry in the first place. Untested either way.
- [ ] No unexpected self-occlusion — not independently checked this session.
- [x] Real end-to-end ROS2 graph confirmed live: `/clock`, `/tf`,
      `/g1/joint_states`, `/livox/mid360/points` all present in
      `ros2 topic list` against the actual running sim, not just the code.
- [x] Ran a real detection backend (`LivoxCenterPointBackend`, real weights,
      GPU) against a captured live cloud — 0 detections, but expected: the 3
      `PEDESTRIANS` targets in `g1_rtx_sim.py` are at 3-6 m, inside the
      measured 9.6 m blind radius at this run's 1.21 m mount height. Targets
      need to move past ~10 m (or the mount needs to come down) before this
      criterion is meaningfully testable.
- RViz itself crashed on launch this session — unrelated snap/glibc
  `libpthread` symbol collision in this shell's environment, not a project
  bug. Retry from a clean terminal.

### If it still fails

- Sector still narrow → check `assets/lidar_configs_fast/*.json` actually
  has the new `-180/180` values (stale copy in Isaac Sim's
  `extscache/omni.sensors.nv.common-*/data/lidar` — the mtime-conditional
  copy in `install_configs()` should catch this, but delete the cached copy
  and rerun if in doubt).
- Ring present but tilted/offset → re-check `MID360_QUAT_WXYZ` math
  independently (quaternion for rpy `(0, 0.0401, 0)`).
- Ring present but very sparse → try `--config-dir assets/lidar_configs_light`
  or `assets/lidar_configs_full` for more emitter states per prim.

## 2. `g1_perception_ws` — Unitree SDK bridge + detection nodes

Three pieces landing from parallel haiku agents: workspace skeleton +
`lidar_bridge.py` (sim/real source toggle), `centerpoint_node.py`,
`pointpillar_node.py`. Verify independently since they were built
concurrently without cross-visibility.

### Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Projects/thesis/g1_perception_ws
colcon build --symlink-install
source install/setup.bash
```

- [ ] `colcon build` succeeds with no errors (warnings about missing
      `livox_ros_driver2` are expected if not installed — real-robot path
      only).
- [ ] `ros2 pkg list | grep g1_perception` shows the package.

### lidar_bridge (sim passthrough)

```bash
ros2 launch g1_perception launch/perception.launch.py source:=sim
```

- [ ] With the Isaac Sim RTX LiDAR running (`/livox/mid360/points` already
      published directly), confirm the bridge either passes through cleanly
      or is a no-op for `source:=sim` — check for double-publishing / topic
      collisions if the bridge also tries to publish the same topic name.

### lidar_bridge (real robot path) — desk check only, no hardware here

- [ ] Read `g1_perception_ws/src/g1_perception/g1_perception/lidar_bridge.py`
      — confirm it subscribes to whatever topic/type the Unitree docs
      specify (check against
      https://support.unitree.com/home/en/G1_developer/lidar_Instructions),
      not a guessed name.
- [ ] Confirm `frame_id="mid360_link"` on republished messages, matching
      the TF tree the sim/robot_state_publisher already uses.

### centerpoint_node

```bash
ros2 run g1_perception centerpoint_node --ros-args -p checkpoint_path:=<path>
```

- [ ] Without a checkpoint present: node starts, logs a clear warning, does
      **not** crash, publishes empty `Detection3DArray`/`MarkerArray`.
- [ ] With `G1_sim/detection/pt/livox_model_1.pt` present (pending download
      agent): node loads it, publishes non-empty detections when pointed at
      the sim's `/livox/mid360/points` with pedestrian boxes in view.
- [ ] `ros2 topic hz /g1/detections/centerpoint` respects the `max_hz` cap
      even if the LiDAR publishes faster.
- [ ] RViz MarkerArray on `/g1/detection_markers/centerpoint` shows boxes
      around the 3 pedestrian cuboids in the scene, no ghost markers
      accumulating frame-to-frame (confirms DELETEALL is issued).

### pointpillar_node

```bash
ros2 run g1_perception pointpillar_node
```

- [ ] Same crash-safety check as CenterPoint: missing checkpoint →
      Euclidean-clustering fallback, not a crash.
- [ ] `ros2 topic hz /g1/detections/pointpillar` ≈ up to `max_hz` (10 Hz
      default).
- [ ] Fallback-mode boxes roughly bound the pedestrian cuboids (loose
      Euclidean clustering, not tight, but present).

### Cross-check: no topic collisions

```bash
ros2 topic list
```

- [ ] `/g1/detections/centerpoint` and `/g1/detections/pointpillar` are
      distinct topics (not both writing `/g1/detections`).
- [ ] Same for the two marker topics.
- [ ] `package.xml` / `setup.py` entry points list all three nodes
      (`lidar_bridge`, `centerpoint_node`, `pointpillar_node`) without one
      agent's edit clobbering another's — this is the main risk of three
      concurrent haiku agents editing the same `setup.py`.

## Known risk from concurrent agents

All three haiku agents write into the same
`g1_perception_ws/src/g1_perception/` package and were instructed to check
for each other's partial state before writing, but none can see the others'
live edits. **Manually diff `setup.py` and `package.xml` after all three
report done** — merge entry_points/dependencies by hand if any agent
overwrote rather than extended the file.
