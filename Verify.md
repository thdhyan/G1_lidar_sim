# Verify

Step-by-step check that the pipeline actually works, in the order that isolates
failures. Each step states what to run, what passing looks like, and what to do
when it doesn't.

**Nothing past step 1 has ever been run to completion.** Steps 3–8 are written
against the API docs, not against observed output. Expect to fix things —
`Troubleshooting` at the bottom covers what is most likely to break.

---

## Before you start

### ⚠️ Only one Isaac Sim fits in 16 GB

Every leftover simulator holds ~1–4 GB. Nine orphans from an earlier session
exhausted RAM *and* all 16 GB of swap, and the OOM killer took the sim down mid-
startup with no Python traceback — it just stops.

Kill stragglers and confirm headroom **before every sim run**:

```bash
pkill -9 -f "[p]ython.*g1_ros2_sim"
free -m | head -2        # want >6000 MB available
```

A silent death during startup is almost always this.

### Two Pythons, never mixed

| Side | Interpreter | Used for |
|---|---|---|
| simulator | conda `env_isaaclab`, Python 3.11 | `scripts/*.py` |
| ROS2 tools | system Python 3.12 + `/opt/ros/jazzy` | `ros2`, RViz, detection |

The sim reaches ROS2 through the *bundled* rclpy inside
`isaacsim.ros2.bridge`, not through `/opt/ros/jazzy`. Sourcing the ROS2 setup
in the conda shell is unnecessary and invites version conflicts.

Terminal setup used throughout:

```bash
# Terminal SIM
conda activate env_isaaclab && cd ~/Projects/thesis/G1_sim

# Terminal ROS (a separate, clean shell)
source /opt/ros/jazzy/setup.bash && cd ~/Projects/thesis/G1_sim
```

`ModuleNotFoundError: No module named 'isaaclab'` means the conda env is not
active in that shell. In a non-interactive shell (scripts, `nohup`, CI),
`conda activate` needs the hook sourced first:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
```

---

## Step 1 — LiDAR gate ✅ passes

**Terminal SIM**

```bash
python scripts/00_verify_warp_lidar.py
```

Pass:

```
[GATE] sensor initialised: 20000 rays x 2 envs
[GATE] valid hits      : 2178
[GATE] hit range       : 16.046 .. 44.455 m
[GATE] closest possible: 16.046 m (phi = -7.16 deg)
[GATE] PASS
```

Closest hit must match "closest possible" — that equality is what shows the
ray-caster is geometrically right rather than merely returning numbers.

*Zero hits?* Something lowered `samples` below 20000. See Troubleshooting.

## Step 2 — USD asset ✅ passes

**Terminal SIM**

```bash
python scripts/convert_g1_urdf_to_usd.py --force
```

Pass: `OK` for `torso_link`, `mid360_link`, `d435_link`, `pelvis`, then
`articulated joints: 29`, then `PASS`.

Unresolved-reference warnings for `d435_link/visuals`, `imu_in_torso/visuals`
and `imu_in_pelvis/visuals` are expected — those links carry no visual geometry
in the URDF. They do not affect sensor attachment.

*A link reported MISS?* `merge_fixed_joints` collapsed it into its parent; it
must stay `False`.

## Step 3 — Sim starts, no ROS2 ✅ passes

Isolates simulation from ROS2. Run this before anything involving topics.

**Terminal SIM**

```bash
pkill -9 -f "[p]ython.*g1_ros2_sim"
python scripts/g1_ros2_sim.py --headless --no-ros2 --no-camera --steps 300
```

Observed:

```
[SIM] G1 joints      : 29
[SIM] LiDAR rays     : 20000
[SIM] LiDAR height   : 0.80 m
[SIM] LiDAR blind r  : 6.40 m (no nadir ray)
[SIM] ROS2 disabled
[SIM] step    200  hits  16288  scans published 0
```

`hits` must be non-zero — the pedestrian boxes sit at 12–25 m, outside the
blind cone, so they return. `scans published 0` is correct here.

The sensor sits at **0.80 m** in the default crouched stance, not the ~1.1 m of
a fully upright G1, so the blind radius is 6.40 m.

*Exits with no `[SIM]` output at all?* OOM. Re-read "Before you start".

## Step 3.5 — ROS2 bridge smoke test ✅ passes

Enables the bridge and publishes with no robot, no LiDAR and no action graph.
Run this whenever a ROS2 problem appears — it tells you in ~40 s whether the
fault is in the ROS2 layer or in the scene.

**Terminal SIM**

```bash
python scripts/debug_ros2_bridge.py
```

Observed:

```
[BRIDGE] enabled: True
[BRIDGE] rclpy from .../isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy/rclpy/__init__.py
[BRIDGE] publisher created
[BRIDGE] published 5 messages
[BRIDGE] OK   isaacsim.ros2.bridge.ROS2Context
...  (14 node types, all OK)
[BRIDGE] PASS
```

Two things this proves:

1. The rclpy path resolves inside `isaacsim/exts/...`, so the sim is using the
   **bundled** jazzy rclpy — the whole reason in-process publishing works.
2. All 14 OmniGraph node types that `action_graph.py` references are
   registered, so the graph cannot fail on a missing node type.

## Step 4 — LiDAR reaches ROS2 🔄 unverified

**Terminal SIM**

```bash
pkill -9 -f "[p]ython.*g1_ros2_sim"
python scripts/g1_ros2_sim.py --headless --no-camera
```

Wait for `[SIM] ROS2 publishing enabled`, then:

**Terminal ROS**

```bash
ros2 topic list
ros2 topic hz /livox/mid360/points
ros2 topic echo /livox/mid360/points --field header --once
```

Pass: `/livox/mid360/points`, `/clock`, `/tf`, `/g1/joint_states` and
`/g1/odom` all listed; rate ≈ **10 Hz**; `frame_id: mid360_link`.

*Topic listed but `hz` prints nothing?* QoS mismatch — the publisher is
best-effort, so a reliable subscriber gets nothing. Add
`--qos-reliability best_effort` to the `hz` call.

## Step 5 — TF tree 🔄 unverified

**Terminal ROS**

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo torso_link mid360_link
```

Pass: `frames.pdf` contains `pelvis → torso_link → {mid360_link, d435_link}`,
and the echoed translation is ≈ `[0.0003, 0.0000, 0.4188]`.

The rotation should read **RPY ≈ [3.14, 0, 0]** — the roll-π mount. If it reads
`[0, 0, 0]`, the offset quaternion in `unitree_g1_lidar.py` is wrong and the
cloud will render upside down.

## Step 6 — RViz 🔄 unverified

Sim keeps running from step 4.

**Terminal ROS**

```bash
ros2 launch launch/g1_bringup.launch.py detection:=false
```

Pass, in RViz:

- **RobotModel** — G1 mesh renders, not a lone axis triad
- **TF** — frame triads on the robot
- **LiDAR** — a point cloud, **ground plane flat and below the robot**

That last item is the real acceptance test for the roll-π mount. An inverted
cloud means the quaternion is wrong even if every topic looks healthy.

*RobotModel empty?* `robot_state_publisher` could not read the URDF — check the
path printed at launch.

## Step 7 — Detection 🔄 unverified

Cheap backend first; it needs no GPU and isolates ROS2 plumbing from the model.

**Terminal ROS**

```bash
python3 detection/detection_node.py --backend clustering
```

Then the real model:

```bash
python3 detection/detection_node.py \
    --backend livox --checkpoint detection/pt/livox_model_1.pt
```

**Terminal ROS 2**

```bash
ros2 topic hz /g1/detection_markers
ros2 topic echo /g1/detections --once
```

Pass: markers publish, and yellow boxes appear in RViz near the three targets
at `(12, 0)`, `(18, -4)`, `(25, 6)`.

Expect ~189 ms/frame for the livox backend, so markers lag the cloud. That is
inference cost, not a fault.

*Detector runs but finds nothing?* Plausible. The model was trained on real
Livox returns; the simulated cloud has no intensity variation and the targets
are plain boxes. Confirm the clustering backend finds them — if it does, the
plumbing is fine and the gap is domain mismatch.

## Step 8 — cmd_vel ⛔ not implemented

```bash
ros2 topic pub /g1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

The topic exists and the graph subscribes, so `ros2 topic info` shows a
connection — **but the robot will not move.** `read_cmd_vel()` is never called
by the sim loop, and there is no locomotion policy behind it. Task 5.

---

## Full-stack run

Once the steps above pass:

```bash
# Terminal SIM
pkill -9 -f "[p]ython.*g1_ros2_sim"
conda activate env_isaaclab && cd ~/Projects/thesis/G1_sim
python scripts/g1_ros2_sim.py --headless

# Terminal ROS
source /opt/ros/jazzy/setup.bash && cd ~/Projects/thesis/G1_sim
ros2 launch launch/g1_bringup.launch.py
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Sim exits during startup, no traceback | OOM | `pkill -9 -f "[p]ython.*g1_ros2_sim"`; check `free -m` |
| `[GATE]` zero hits | `samples < 20000` — the pattern is sliced in time order, so a smaller window drops every downward ray | keep 20000; use `downsample` to thin |
| Point cloud upside down in RViz | roll-π mount quaternion wrong | `MID360_QUAT` must be `(0, 1, 0, 0)` wxyz |
| Nothing within ~6.4 m of the robot | not a bug — Mid-360's steepest ray is −7.2°, so the blind cone is ≈8× mount height (0.80 m → 6.40 m) | use the camera for near field |
| `ros2 topic hz` silent on a listed topic | QoS mismatch (publisher is best-effort) | `--qos-reliability best_effort` |
| `No module named 'rclpy'` in the sim | rclpy imported before the bridge was enabled | call `enable_ros2_bridge()` first |
| `No module named 'isaaclab'` | wrong interpreter | `conda activate env_isaaclab`; in a non-interactive shell source `conda.sh` first |
| `A camera was spawned without the --enable_cameras flag` | Isaac Sim needs rendering enabled before a Camera sensor initialises | `g1_ros2_sim.py` sets it automatically unless `--no-camera`; a custom script must pass `--enable_cameras` |
| Timestamps disagree / TF extrapolation errors | a node on wall-clock time | every external node needs `use_sim_time:=true` |
| `og.Controller.edit` rejects an attribute | node attribute names unverified against 5.1 | read the error, fix the name in `action_graph.py` |
| `Provided pattern list did not match any articulations`, sim stops with no traceback | the ROS2 bridge was enabled **after** the scene was built; enabling it reloads the USD stage and invalidates the articulation's physics view | call `enable_ros2_bridge()` before `SimulationContext` — `g1_ros2_sim.py` now does |

## Status

| Step | State |
|---|---|
| 1 LiDAR gate | ✅ passes |
| 2 USD asset | ✅ passes |
| 3 sim, no ROS2 | 🔄 unverified |
| 4 LiDAR → ROS2 | 🔄 unverified |
| 5 TF tree | 🔄 unverified |
| 6 RViz | 🔄 unverified |
| 7 detection | 🔄 unverified end-to-end (model itself validated on synthetic data) |
| 8 cmd_vel | ⛔ not implemented |
