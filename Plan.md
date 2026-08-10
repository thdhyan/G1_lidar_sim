# Plan — design and rationale

Why this project is shaped the way it is. Live status lives in
[Tasks.md](Tasks.md); usage lives in [README.md](README.md).

---

## Goal

One importable G1 USD carrying a Livox Mid-360 and a D435 camera, plus a ROS2
layer that lets you drive the robot, see it in RViz (robot description, TF,
joint states), and get human/pedestrian 3D boxes from LiDAR.

Before this, OmniPerception shipped a warp `LidarSensor` for IsaacLab but no G1
IsaacLab scene, no USD asset, and no ROS2 publishing — only a MuJoCo-bound
`lidar_vis_ros2.py`. Every scene had to re-declare its sensors by hand.

## Decisions

| Decision | Choice | Why |
|---|---|---|
| USD asset | auto-convert from URDF | mounts already exist in the URDF; hand-authoring buys nothing |
| LiDAR | OmniPerception warp `LidarSensor` | keeps the real `mid360.npy` scan pattern; RTX LiDAR is the fallback |
| Control | ROS2 topic `/g1/cmd_vel` | simplest real-time streaming interface |
| Detection | port `livox_detection` | its CenterPoint weights are public and Livox-specific |
| Detection process | separate from sim | inference must not stall physics; different dependency set |

## Transport architecture

Two mechanisms, split by where the data lives:

- **OmniGraph Action Graph** — anything sourced from USD prims: `/clock`, `/tf`,
  joint states, camera, odometry, `cmd_vel`. 30 ROS2 nodes are registered and
  verified present.
- **In-sim rclpy** — the LiDAR point cloud only. It lives in a torch tensor
  produced by warp, not a USD prim, so no OmniGraph node can see it.

This split exists because of a discovery that inverted the original design: the
`isaacsim.ros2.bridge` extension loads its **own bundled jazzy rclpy** into the
Python 3.11 simulator process, even though system rclpy is 3.12 and cannot
import. That removes the need for shared memory or an IPC sidecar. The
extension must be enabled *before* `import rclpy`.

**Confirmed by `scripts/debug_ros2_bridge.py`**, which enables the extension and
publishes with no robot or scene present:

```
[BRIDGE] enabled: True
[BRIDGE] rclpy from .../isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy/rclpy/__init__.py
[BRIDGE] publisher created
[BRIDGE] published 5 messages
```

The resolved path shows this is the extension's bundled rclpy, not a system one.
Keep this script — it separates "the ROS2 layer is broken" from "the scene is
broken", which are otherwise hard to tell apart.

## Detection port

Upstream `livox_detection` requires Python 3.8 / torch 1.8.2 / CUDA 10.2. CUDA
10.2 has no support for Ada (sm_89), so the original stack cannot run on the
RTX 4060 at all. Only the checkpoints are portable.

The network was therefore re-declared against torch 2.7. Architecture was first
recovered by inspecting the checkpoint's 275 tensors, then **checked against
upstream `resfpn.py` / `boolmap.py` / `centerhead.py`** — which caught five
errors that inspection alone had missed. Each produced an all-zero heatmap:

1. block 0 uses stride 2, not 1
2. the neck upsamples every level back to input resolution (`×[2,4,8,16,32]`);
   it does not downsample to a common middle stride
3. `attention_w` terminates in `softmax(dim=1)`, not `Sigmoid`
4. BatchNorm needs `eps=1e-3, momentum=0.01`
5. decoding uses `feature_map_stride=1`, and `rot` is `(cos, sin)`

**Validation.** The checkpoint loads with 0 missing and 0 unexpected keys, which
means the reconstructed topology is exactly right rather than merely plausible.
On a synthetic scene the model recovers all three planted pedestrians to within
~0.1 m with human-plausible extents (0.59 × 0.58 × 2.08 m) at ~189 ms/frame.

A `ClusteringBackend` fallback exists so the ROS2 graph, message types and RViz
config can be exercised with no torch, GPU, or weights.

## Risks

| Risk | Mitigation | Status |
|---|---|---|
| Warp sensor broken on this install | Task 0 gate before dependent work | ✅ **PASSED** |
| Installer clobbers IsaacLab files | backup taken, restore documented | ✅ handled |
| No G1 locomotion policy | search first, PD stand-still fallback | ✅ fallback active (hold stance) |
| Detection weights unavailable | they are public after all | ✅ resolved |
| 8 GB VRAM budget | `num_envs=1`, 10 Hz LiDAR, detection out-of-process | ✅ designed for it |
| Mid-360 mounted roll-π | explicit offset quaternion (0,1,0,0); upright cloud is the test | ⬜ needs sim test |

## Task 0 investigation — resolved

The gate failed with every ray reporting `max_distance`. The sensor turned out
to be fine; **the test configuration was wrong.**

### ⚠️ `samples` selects a time window, not a subsample

This is the important lesson, and it is a live trap for anyone tuning `samples`
for performance.

The `.npy` scan patterns are **time-ordered scan trajectories**, and the loader
takes a *contiguous slice*: `pattern_data[start_idx : start_idx + samples]`. So
`samples` picks a window in time, not a uniform subsample of the field of view.

For `mid360.npy` (800,000 rows spanning φ ∈ [−7.2°, +52.2°]):

| Window | φ range | fraction pointing down |
|---|---|---|
| first 8,000 | +1.43° … +52.16° | **0.0000** |
| first 20,000 (native) | −7.16° … +52.16° | 0.1485 |
| first 100,000 | −7.18° … +52.16° | 0.1497 |
| strided 8,000 | −7.19° … +52.16° | 0.1477 |

The gate used `samples=8000`, which lands inside a single upward sweep with no
downward rays whatsoever — so nothing could ever hit the ground. Using the
sensor's native 20,000 restores full coverage.

**Guidance:** to reduce ray count, prefer `downsample` (which strides the
pattern) over shrinking `samples`, or verify that the chosen window still spans
the full elevation range.

### ⚠️ The Mid-360 has no straight-down ray

A second wrong assumption, worth stating because it constrains sensor placement.

The pattern's steepest downward tilt is φ = −7.2°, so from height *h* the
nearest possible ground return is at slant range *h* / sin(7.16°) ≈ **8.0 h** —
there is no nadir ray and nothing directly beneath the sensor is ever seen.

Measured against prediction, from 2 m:

| Quantity | Value |
|---|---|
| predicted closest slant range | 16.046 m |
| measured closest hit | 16.046 m |

An exact match, which is strong evidence the ray-caster is geometrically sound.

**Consequence for the G1:** measured in the running sim, the sensor sits at
**0.80 m** in the default crouched stance, giving a **6.40 m** blind radius.
Near-field obstacles and the robot's own feet are invisible to it. If
close-range sensing is needed, that is the camera's job, not the LiDAR's.

### Steps that ruled out the alternatives

1. Meshes **are** bound — `self.meshes` contains `/World/ground`.
2. Geometry **is** discovered — `/World/ground/geometry/mesh`, type `Cube`.
3. The warp mesh is **valid** — 8 vertices, bounds ±9000 m, top surface at z≈0.
4. A hand-rolled downward ray **hits at (0, 0, 0)**, distance exactly 2.0 m.
5. The sensor's own rays had z ∈ [+0.0249, +0.7897] — all upward.

Three hypotheses were wrong and are recorded so they are not re-tried: that the
ground plane carried no ray-castable mesh; that the pattern file contained no
downward rays (it does, 15 % — but not in the sampled window); and one apparent
failure that was a bug in the diagnostic rather than the sensor.

`GroundPlaneCfg` was also swapped for an explicit cuboid, which yields a plain
`Cube` prim the mesh extraction handles directly.

## RTX LiDAR — annotator deprecation found live, 2026-08-10

First live run of the RTX pivot (see `Tasks.md` / `Verify_RTX.md` for the
full trace). The ROS2 pipeline genuinely works end to end — `/clock`, `/tf`,
`/g1/joint_states`, `/livox/mid360/points` all confirmed live via
`ros2 topic hz`/`echo` against a real running sim, not just code review. But
the published cloud is clipped to azimuth ±90°, not the full 360° ring.

This looked at first like the exact bug commit `7aa61bf` already fixed
(`validStartAzimuthDeg=0/360` vs `-180/180`). It isn't a recurrence of that
bug: the JSON configs' baked-in `azimuthDeg` values were checked directly
and are uniformly distributed across the full range. The clip reproduces
identically with 2 prims and with all 4, ruling out `--num-prims` as the
cause too. The actual cause is a level down: `g1_sim/rtx_publisher.py` reads
points via the `IsaacCreateRTXLidarScanBuffer` annotator, which this Isaac
Sim 6.0.1 build now flags as deprecated, with NVIDIA's own log message
pointing at `GenericModelOutput` instead for full-scan data. The old
annotator apparently still runs but silently returns a partial (frustum-
limited) scan rather than erroring — which is why nothing upstream caught
it. Whatever Isaac Sim build `7aa61bf` was verified against, this is a
regression relative to it, introduced by the annotator's deprecation, not a
leftover of the original config bug.

**Consequence for anyone re-testing this**: a single captured message from
this sensor is not representative of full coverage even once the annotator
fix lands, because the Mid-360's `solidState` scan is genuinely
non-repetitive — each `emitterState` (each published frame) only covers a
thin azimuth slice by design, and the full pattern only builds up after
accumulating many frames (the `stateResolutionStep=1`-per-tick cycle). Judge
coverage from an accumulated multi-second capture or RViz's own decay-time
accumulation, not a single `--once` echo.

**Separately found**: the three `PEDESTRIANS` targets in `g1_rtx_sim.py`
(3-6 m out) sit entirely inside the sensor's blind radius at this run's
measured 1.21 m mount height (9.6 m blind radius — note this is a different
mount height than the 0.40618 m documented after `7aa61bf`; something moved
it since). Detection will read zero regardless of model correctness until
either the targets move past ~10 m or the mount comes down.

## OpenPCDet comparison — 2026-08-10

Two reference repos live under `~/Projects/Thesis/` (sibling of this repo's
parent, capital-T — not the same directory as `~/Projects/thesis/G1_sim`):
`OpenPCDet` (full checkout, with real pretrained `checkpoints/pointpillar_7728.pth`
and `pv_rcnn_8369.pth`) and `livox_detection` (confirmed to be the exact
upstream this project's CenterPoint port was reverse-engineered from —
filenames and checkpoints match).

`pcdet` is already built in the `livox` conda env and was smoke-tested: real
forward pass through `pointpillar_7728.pth`, works end to end today. Our own
`g1_perception_ws` PointPillar (`pointpillar_model.py`) has no pretrained
weights and a different backbone shape (`BaseBEVBackbone`'s 3-block design
vs. our custom symmetric 4-block one), so the checkpoint can't be loaded
directly into it. Full comparison table and the two integration options
(wrap real OpenPCDet as a new backend vs. reshape our model to match) are in
`Tasks.md` — recommendation there is to wrap OpenPCDet directly rather than
reshape, since the smoke test already answers "does this component work,"
leaving only "is a KITTI-forward-range model useful on a 360° pedestrian
scene" as the open question.

## cmd_vel locomotion — decoupled_wbc — 2026-08-10

Task 5's locomotion gap is closed. After three false starts (a stationary-arm
manipulation policy mistaken for a walking one, IsaacLab's untrained/wrong-DOF
config, and NVIDIA GEAR-SONIC's TensorRT-C++-and-mocap-reference design being
more than the task needed), NVIDIA's `decoupled_wbc` module — plain Python,
ONNX+numpy, driven by planar velocity commands — turned out to fit directly.
Full technical trace (I/O contract, PD-gain bug, live results) lives in
`Tasks.md` under Task 5 and in `docs/decoupled_wbc_findings.md`; the one
finding worth repeating here because it generalizes beyond this policy:
**`convert_g1_urdf_to_usd.py` bakes uniform PD gains (100/10) into every
joint**, and any externally trained controller that assumes real per-joint
gains will silently fail (the robot collapses) against that default until the
caller overrides them explicitly. This will bite the next external policy
too, not just this one.

## SLAM — Ultra-Fusion — 2026-08-10

Investigated per user request as a LiDAR+camera+IMU SLAM option matching our
exact Mid-360+D435 pair. No public source is available yet, but the prebuilt
Docker images run correctly on this machine despite targeting a different
ROS distro (Humble/22.04) than our host (Jazzy/24.04) — Docker's own
userspace isolation makes that a non-issue. What's missing is a sensor
profile: none of Ultra-Fusion's shipped configs assume a legged, wheel-less
robot, so ours would need to be authored from scratch with real extrinsics/
intrinsics we don't have calibrated yet. Full trace and alternatives (FAST-
LIO2, FAST-LIVO2, etc.) in `Tasks.md` and `docs/ultra_fusion_findings.md`.
