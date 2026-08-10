# decoupled_wbc cmd_vel bridge - findings

Investigating Task 5 (cmd_vel bridge, ⛔ in Tasks.md) using NVIDIA's
`decoupled_wbc` (GR00T-WholeBodyControl) Balance/Walk policies. Written
2026-08-10. Not yet merged into Tasks.md/Plan.md - see repo-level note about
concurrent-agent edits.

**Summary: it works.** Standalone MuJoCo validation, a from-scratch numpy
port, and a live Isaac Sim run driven by real `/g1/cmd_vel` ROS2 messages all
confirm the bridge - both balance and forward-walk. One real bug was found
and fixed along the way (PD gain mismatch, see below).

---

## 1. What was already installed

Per the restart brief: `mujoco`, `mujoco-warp`, `mujoco-usd-converter`, `onnx`,
`onnx-ir`, `onnxscript` were already in the `isaac` conda env (dated Jun 25).
Only `onnxruntime` 1.28.0 was installed today (Aug 10, 12:44 - matches the
"small install, 19MB" the prior agent instance mentioned before being
interrupted). Nothing further needed installing. `pip check` in `isaac` shows
pre-existing minor version drift (e.g. `isaacsim-core` wants torch==2.11.0,
has 2.10.0+cu128) unrelated to this work - not touched.

Disk: 13GB free at session start, dropped to 8.4GB free over the session from
causes unrelated to this work (per the brief, "not your concern to fix"). The
2 ONNX policies + a MuJoCo/onnx smoke-test script added a few MB, negligible.

## 2. Policy files: a naming bug in the vendored repo

The task pointed at:

```
third_party/GR00T-WholeBodyControl/decoupled_wbc/sim2mujoco/resources/robots/g1/policy/GR00T-WholeBodyControl-{Balance,Walk}.onnx
```

Both exist (1.79 MB each, different content - confirmed via `md5sum`, not
duplicates). But `sim2mujoco/resources/robots/g1/g1_gear_wbc.yaml` -  the
config both `sim2mujoco/scripts/run_mujoco_gear_wbc*.py` demos load - pointed
at `policy/ft92.onnx` and `policy/ft109.onnx`, which **do not exist** in this
checkout. Cross-checked against
`decoupled_wbc/control/main/teleop/configs/configs.py` (`wbc_model_path`
default), which explicitly documents the pair as
`policy/GR00T-WholeBodyControl-Balance.onnx,policy/GR00T-WholeBodyControl-Walk.onnx`.
`ft92`/`ft109` are internal training-run names that never got updated for
the public release's renamed files - a bug in the vendored repo itself, not
something specific to this checkout.

**Fix applied** (only edit made to the third_party checkout): corrected the
two paths in `g1_gear_wbc.yaml` to the real filenames. `run_mujoco_gear_wbc.py`
(the two-policy Balance+Walk demo, matching what this task needed) now loads
correctly; `run_mujoco_gear_wbc_gait.py` (a different, single-policy variant
with gait-clock signals, not used here) was left alone.

## 3. I/O contract - verified against a running policy, not assumed

Ran `run_mujoco_gear_wbc.py`'s logic headlessly (no `mujoco.viewer` window, no
`pynput` keyboard listener - neither is installed nor needed; wrote a
standalone harness reproducing the same math driven by hardcoded commands
instead). Confirmed via `sess.get_inputs()/get_outputs()` on the loaded ONNX
sessions, and by inspecting `compute_observation()`:

| | |
|---|---|
| ONNX input | `"input"`, float32, shape `(batch, 516)` |
| ONNX output | `"output"`, float32, shape `(batch, 15)` |
| History | 6 stacked frames of an 86-dim "single obs" (516 = 6*86) |
| Control rate | policy runs every 4th physics step at `sim_dt=0.005` -> 50 Hz |
| Actuated joints | 15 (legs 6*2 + waist 3) - **not** all 29 |
| Non-actuated joints | 14 arm joints (7*2), held by a separate fixed PD (kp=100, kd=0.5) targeting 0, computed outside the policy entirely |

**Single-obs (86) layout** (`compute_observation()` in
`run_mujoco_gear_wbc.py`):

```
[0:7]    command: [vx*2.0, vy*2.0, wz*0.5, height_cmd, roll_cmd, pitch_cmd, yaw_cmd]
[7:10]   base angular velocity * 0.5   (ang_vel_scale)
[10:13]  gravity vector rotated into base frame (quat_rotate_inverse)
[13:42]  (qpos[all 29 joints] - padded_default_angles) * 1.0   (dof_pos_scale)
         padded_default_angles: 15 real values for legs+waist, zero-padded for arms
[42:71]  qvel[all 29 joints] * 0.05   (dof_vel_scale)
[71:86]  previous action (15,)
```

Note this **includes all 29 joints' positions/velocities**, not just the 15
policy-actuated ones - a detail easy to miss from the 15-dim action alone.

**Action**: `target_dof_pos[15] = onnx_output[15] * 0.25 + default_angles[15]`.
`default_angles` (legs+waist only):
`[-0.1, 0, 0, 0.3, -0.2, 0]*2 + [0, 0, 0]` (hip_pitch, hip_roll, hip_yaw, knee,
ankle_pitch, ankle_roll per leg, then waist yaw/roll/pitch).

**Policy switch**: `norm(loco_cmd_xyz) <= 0.05` -> Balance policy, else -> Walk
policy. `loco_cmd` here is the *raw* `[vx, vy, wz]` before `cmd_scale` is
applied to build the observation's `command[:3]`.

**PD gains actually used by the policy's own MuJoCo demo** (legs+waist, from
`g1_gear_wbc.yaml`):
`kp=[150,150,150,200,40,40]*2+[250,250,250]`,
`kd=[2,2,2,4,2,2]*2+[5,5,5]`. This detail turned out to matter a lot - see
§5.

**Joint order** (both the MJCF's qpos order and the observation's index
order): standard Unitree G1 29-DOF order -
`[L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee, L_ankle_pitch, L_ankle_roll,
R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee, R_ankle_pitch, R_ankle_roll,
waist_yaw, waist_roll, waist_pitch, L_shoulder_pitch, L_shoulder_roll,
L_shoulder_yaw, L_elbow, L_wrist_roll, L_wrist_pitch, L_wrist_yaw,
R_shoulder_pitch, R_shoulder_roll, R_shoulder_yaw, R_elbow, R_wrist_roll,
R_wrist_pitch, R_wrist_yaw]`. Cross-checked two independent sources: (1) the
`<joint>` order in `g1_gear_wbc.xml` itself, walked with `grep`; (2) the
`WeakMotorJointIndex` mapping in the separate, more elaborate
`control/main/teleop/configs/g1_29dof_gear_wbc.yaml` (used by the full teleop
stack, not by the simple demo) - independently lists the same 29 names in the
same order. Both agree, and it's the joint order our own G1 URDF/USD also
uses (same manufacturer joint names).

## 4. Standalone validation (MuJoCo, headless, no GUI/keyboard)

Script: `/tmp/.../scratchpad/test_wbc_headless.py` (scratchpad, not part of
this repo). Loaded both ONNX policies via onnxruntime (CPU provider - no
`CUDAExecutionProvider` registered in this onnxruntime build, "Azure,CPU"
only; irrelevant, the model is tiny and runs plenty fast on CPU), drove
`mujoco.MjModel`/`MjData` with `mj_step` directly (no viewer), switched
`loco_cmd` from `(0,0,0)` to `(0.5,0,0)` partway through.

Result: pelvis height stayed in `0.744-0.793` m throughout (never collapsed),
and under the walk command the robot travelled from `x=0` to **`x=7.9 m` over
17 s of sim time** (~0.46 m/s average forward speed) - genuine walking, not
just standing. Some lateral drift accumulated (`y` reached `-0.68` m by the
end) with a pure-forward command; not investigated further, noted as an open
item.

## 5. Isaac Sim integration

### New/changed files

- **`g1_sim/wbc_bridge.py`** (new) - pure numpy/onnxruntime port of the
  observation/action math above. No Isaac Sim imports, so it's the same code
  validated standalone (`/tmp/.../scratchpad/test_wbc_bridge_module.py`
  reran the MuJoCo check through this exact class and got the same result:
  `x=5.68 m` reached in the same number of steps as the raw reference script -
  confirms the port is numerically faithful, not just structurally similar).
  Exposes `LEG_WAIST_JOINTS` (15) and `ALL_JOINTS` (29) name lists, `KP`/`KD`
  (policy-trained gains), `WbcBridge.step(qpos_all, qvel_all, quat_wxyz,
  ang_vel_body, cmd_vx, cmd_vy, cmd_wz) -> target_dof_pos[15]`.
- **`g1_sim/rtx_camera.py`** - added `attach_cmd_vel_subscriber()`, matching
  the existing `attach_camera_publishers`/`attach_robot_state_publishers`
  pattern. `g1_rtx_sim.py`'s scene (the current active pipeline, RTX LiDAR
  pivot) had no `/g1/cmd_vel` subscriber at all - that only existed in
  `g1_sim/action_graph.py`'s `build_g1_action_graph()`, which belongs to the
  older warp-lidar pipeline (`g1_ros2_sim.py`) that isn't run any more.
  Reused `action_graph.read_cmd_vel(graph_path=...)` to read it rather than
  duplicating that logic.
- **`scripts/g1_rtx_sim.py`** - wired the bridge into `main()`:
  - `--no-locomotion` flag (default: locomotion on).
  - After the articulation is created, checks `robot_articulation.dof_names`
    contains all 29 `ALL_JOINTS` names before enabling the bridge (fails soft
    - prints and disables rather than crashing - if the USD's naming ever
    diverges).
  - Overrides the USD's baked-in PD gains with the policy-trained ones (see
    below) via `Articulation.set_gains(..., joint_names=...)`.
  - Main loop: every physics step, if `sim.current_time` has advanced past
    the next 50 Hz control tick, reads `/g1/cmd_vel` (0 if ROS2/locomotion
    disabled), reads `qpos`/`qvel` for all 29 joints and base
    quaternion/angular velocity **by name** via
    `get_joint_positions(joint_names=...)` /
    `get_world_poses()`/`get_angular_velocities()`, runs `WbcBridge.step()`,
    and applies the result via
    `set_joint_position_targets(target, joint_names=LEG_WAIST_JOINTS)`.
- **`assets/policy/`** (new) - copied the two `.onnx` files (3.6 MB total)
  and the NVIDIA Open Model License in, following the same local-checkpoint
  convention as `detection/pt/` (per Tasks.md's OpenPCDet section) rather
  than reaching into `third_party/` at runtime.

### Sidestepping the joint-order risk

This was flagged as the main risk going in. `isaacsim.core.prims.Articulation`
turns out to support `joint_names=` directly on `get_joint_positions`,
`get_joint_velocities`, `set_joint_position_targets`, and `set_gains` (checked
the installed package source,
`isaacsim/extsDeprecated/isaacsim.core.prims/.../articulation.py`) - it
converts names to whatever internal index order PhysX actually assigned via
`_convert_joint_names_to_indices()`. So `wbc_bridge.py` and `g1_rtx_sim.py`
never need to know or assume Isaac Sim's internal joint order; they just pass
the `ALL_JOINTS`/`LEG_WAIST_JOINTS` name lists every time. Confirmed working
live (see §6) - no exceptions, no `KeyError`s, and (after the gain fix) stable
standing/walking, which wouldn't happen if joints were silently
cross-wired.

### Bug found and fixed: PD gain mismatch

First live run (`--headless --no-camera --no-room --steps 200`, cmd=0)
**collapsed**: pelvis height went `0.79 m -> 0.51 m -> 0.16 m` over 100
policy updates (~2 s), despite the exact same policy holding `0.74-0.79 m`
indefinitely in the MuJoCo validation.

Root cause: `scripts/convert_g1_urdf_to_usd.py` bakes a **uniform** PD drive
(`stiffness=100, damping=10`) into the USD for every joint. The
`decoupled_wbc` policy was trained assuming the much stiffer, per-joint gains
above (`kp` up to 250, `kd` up to 5 for waist; lower for ankles). `g1_rtx_sim.py`
uses the raw `isaacsim.core.prims.Articulation` API directly against the USD
- **not** `g1_sim/robots/unitree_g1_lidar.py`'s `G1_LIDAR_CFG` (which has its
own, still-different actuator gains and is unused by this script; only
`g1_ros2_sim.py`, the older/inactive pipeline, references it). So neither the
USD default nor `G1_LIDAR_CFG` matched what the policy needs.

Fix: call `robot_articulation.set_gains(kps=KP, kds=KD,
joint_names=LEG_WAIST_JOINTS)` once at startup, using the exact
`g1_gear_wbc.yaml` values (now also in `wbc_bridge.KP`/`KD`), plus
`ARM_KP=100, ARM_KD=0.5` for the arms. Re-ran the same test after the fix:
pelvis held `0.71-0.74 m` for the full 600-step run (verified, §6).

**This is a real, generalizable finding, not specific to WBC**: any policy
imported from elsewhere that expects specific joint stiffness/damping will
silently fail against this USD's uniform 100/10 default unless the gains are
set explicitly. Worth flagging for whoever eventually revisits
`convert_g1_urdf_to_usd.py` or `G1_LIDAR_CFG`.

## 6. Live Isaac Sim results (verified, not simulated-only)

All runs: `isaac` conda env, `python scripts/g1_rtx_sim.py --headless
--no-camera --no-room --steps N`. Checked `free -h`/`df -h` before each launch
per the memory constraint; killed stragglers between runs.

- **Before the gain fix**, `cmd=(0,0,0)`, 200 steps: pelvis
  `0.79 -> 0.51 -> 0.16 m` - falls. (Confirms the bug above.)
- **After the gain fix**, `cmd=(0,0,0)`, 600 steps (~12 s sim time, 300 policy
  updates): pelvis `0.707, 0.727, 0.715, 0.725, 0.724, 0.736 m` at 50-update
  intervals - stable, no drift toward falling.
- **After the gain fix, real cmd_vel**: launched the sim, waited for
  `[RTX] cmd_vel graph : /ActionGraph/CmdVelROS2` to print (confirming the
  subscriber graph exists), then ran
  `ros2 topic pub --once /g1/cmd_vel geometry_msgs/msg/Twist "{linear:
  {x: 0.5}}"` repeatedly (12x, 1/s) from a separate shell (`source
  /opt/ros/jazzy/setup.bash` first) while the sim kept running for 900 steps
  total (~15 s sim time). Log shows the bridge picked it up and switched
  policies, and stayed stable for the entire run (`pelvis_z` at every
  50-update checkpoint): `updates=50: 0.713`, `100: 0.726`, `150: 0.718`,
  `200: 0.725`, `250: 0.736`, `300: 0.728`, `350: 0.736`, `400: 0.730`,
  `450: 0.730` - **9 seconds of continuous walking-policy control, driven by
  real ROS2 messages on `/g1/cmd_vel`, without falling** (the subscriber node
  evidently latches the last Twist, since cmd stayed `(0.50,0.00,0.00)`
  through the full run despite publishing having stopped around the 12 s
  mark). This is the actual Task 5 deliverable working end-to-end, not just
  designed.

What was **not** measured live: forward displacement in Isaac Sim (the
diagnostic print only logs `pelvis_z`, not `x`/`y`). The MuJoCo run (§4)
separately confirmed the same policy produces real forward translation, and
the live Isaac Sim run confirms the walk policy stays balanced under real
sensor/actuator conditions (contacts, the RTX LiDAR sensors' physics
overhead, etc.) rather than just accepting the command - a reasonable stand-in
for "it's walking, not just twitching in place," but strictly speaking the
odometry number itself wasn't pulled from this run. Easy follow-up: read
`robot_articulation.get_world_poses()[0][0][:2]` alongside `pelvis_z` in the
existing diagnostic print.

## 7. What's still open

- **Forward-displacement confirmation in Isaac Sim** - see above. High
  confidence given (a) MuJoCo showed real translation with the identical
  policy/observation code and (b) Isaac Sim shows a stable, non-falling
  pelvis under the same walk command, but not independently measured here.
- **Turning (`wz`) and strafing (`vy`) commands** - only forward (`vx=0.5`)
  was exercised live. The observation contract treats all three command axes
  identically (`command[:3] = [vx,vy,wz] * cmd_scale`), so there's no
  structural reason they'd behave differently, but untested.
- **Longer-duration stability** - longest continuous live run was ~12 s
  sim-time (600-1400 steps depending on the test). Not tested for minutes-long
  runs, uneven terrain, or the Nucleus `simple_room` environment (`--no-room`
  was used throughout to keep GPU/memory load down for repeated test runs on
  a memory-constrained machine - the flat-ground fallback was exercised, not
  the full room's floor collision geometry).
- **50 Hz vs 60 Hz control-rate mismatch** - the policy was trained with
  `control_decimation=4` at `sim_dt=0.005` (200 Hz physics) = 50 Hz control.
  `g1_rtx_sim.py` runs physics at 60 Hz (`SIM_RATE_HZ`) and the bridge's
  50 Hz target is achieved via a `sim.current_time` threshold check, which at
  60 Hz physics means the policy actually gets called on essentially every
  physics step (the 60 Hz step period is shorter than the 50 Hz control
  period, so the threshold trips every single step) - i.e. control effectively
  runs at 60 Hz, 20% faster than trained. Live results were stable regardless,
  but this is a known, uncorrected mismatch rather than an exact reproduction.
- **Lateral drift** - the MuJoCo run showed the pelvis drifting sideways
  (`y=-0.68 m` after 17 s of pure-forward command). Not investigated; could be
  a genuine gait asymmetry in the policy, or an artifact of the simplified
  demo harness (no yaw-correction command was ever sent). Worth watching if
  precise path-following matters later.
- **`--no-locomotion` fallback path** - implemented (falls back to whatever
  the USD's default drive does, i.e. the old passive hold) but not separately
  tested this session; only the locomotion-enabled path was exercised live.

## 8. Files touched

- `third_party/GR00T-WholeBodyControl/decoupled_wbc/sim2mujoco/resources/robots/g1/g1_gear_wbc.yaml`
  - fixed `policy_path`/`walk_policy_path` typos (2 lines).
- `G1_sim/g1_sim/wbc_bridge.py` - new.
- `G1_sim/g1_sim/rtx_camera.py` - added `attach_cmd_vel_subscriber()` +
  `TOPIC_CMD_VEL`.
- `G1_sim/scripts/g1_rtx_sim.py` - `--no-locomotion` flag, bridge
  wiring, gain override, main-loop control step.
- `G1_sim/assets/policy/` - new, 2 ONNX files + license copied from
  `third_party/`.
- Nothing under `gear_sonic_deploy/` or the TensorRT install touched, per
  instructions.
