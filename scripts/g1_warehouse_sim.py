#!/usr/bin/env python3
"""G1 in a populated warehouse: RTX LiDAR + camera + IMU + decoupled_wbc
locomotion, plus IRA-driven wandering humans and Nova Carters, on Isaac Sim 6.0.

    conda activate isaac
    python scripts/g1_warehouse_sim.py --headless --steps 200

Extends ``g1_rtx_sim.py``'s proven sensor/locomotion pipeline (kept
untouched and still the thing to run for a bare-warehouse-free scene) with:

- a real warehouse environment, via ``g1_sim.warehouse`` (fixes a Nucleus
  path-resolution bug found while building this: ``g1_rtx_sim.py``'s Nucleus
  room loader passed a catalog-relative path straight into
  ``add_reference_to_stage``, which silently failed and fell back to flat
  ground on every run - see that module's docstring for the fix)
- wandering humans and Nova Carters spawned via IRA
  (``g1_sim.ira_actors`` - see its docstring for a hard-coded-frame-budget
  bug found and worked around there)
- an IMU publisher (``g1_sim.rtx_camera.spawn_imu_sensor`` /
  ``attach_imu_publisher``) - there wasn't one before this script

Scene bootstrap order matters: IRA's ``setup_simulation()`` opens the
warehouse **as a new root stage** (a full replace, not a reference), so it
must run before anything else touches the stage. If IRA is disabled
(``--no-ira``) or fails, falls back to a plain warehouse reference (still
using the fixed loader) with no dynamic actors - the pipeline degrades
gracefully rather than aborting.
"""

import argparse
import sys
from pathlib import Path

from isaacsim import SimulationApp

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

parser = argparse.ArgumentParser(description="G1 in a populated warehouse.")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--steps", type=int, default=0, help="Stop after N steps; 0 runs forever.")
parser.add_argument("--no-ros2", action="store_true")
parser.add_argument("--no-camera", action="store_true", help="Skip the D435 camera (saves render time).")
parser.add_argument("--no-locomotion", action="store_true", help="Skip the decoupled_wbc cmd_vel bridge.")
parser.add_argument("--no-ira", action="store_true", help="Skip IRA humans/carters; warehouse + G1 only.")
parser.add_argument("--num-humans", type=int, default=2, help="IRA wandering characters. 0 disables the group.")
parser.add_argument("--num-carters", type=int, default=1, help="IRA wandering Nova Carters. 0 disables the group.")
parser.add_argument("--ira-seed", type=int, default=42)
parser.add_argument(
    "--navmesh-max-frames",
    type=int,
    default=3000,
    help="Raised navmesh-bake poll budget - see g1_sim/ira_actors.py docstring for why the stock 100 fails.",
)
parser.add_argument(
    "--config-dir",
    type=str,
    default="assets/lidar_configs_fast",
    help="Mid-360 emitter-state profiles. See g1_sim/rtx_lidar.py for the fast/light/full tradeoff.",
)
parser.add_argument("--num-prims", type=int, default=0, help="Use only the first N LiDAR sensor prims.")
args_cli = parser.parse_args()

SIM_RATE_HZ = 60.0

simulation_app = SimulationApp(
    {
        "headless": args_cli.headless,
        # Forces CPU-side buffering for LiDAR returns - see g1_rtx_sim.py for
        # why (IsaacSim discussion #685, CUDA buffer races with several RTX
        # LiDAR prims publishing over ROS2).
        "/app/sensors/nv/lidar/outputBufferOnGPU": False,
    }
)

"""Rest everything follows."""

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

from g1_sim.rtx_camera import (
    apply_semantics,
    attach_camera_publishers,
    attach_cmd_vel_subscriber,
    attach_imu_publisher,
    attach_robot_state_publishers,
    spawn_camera,
    spawn_imu_sensor,
)
from g1_sim.rtx_lidar import (
    MID360_POS,
    MID360_QUAT_WXYZ,
    attach_ros2_publishers,
    blind_radius,
    spawn_mid360,
)
from g1_sim.warehouse import WAREHOUSE_USD, build_flat_ground, load_environment

ENABLE_ROS2 = not args_cli.no_ros2
ENABLE_LOCOMOTION = not args_cli.no_locomotion
ENABLE_IRA = not args_cli.no_ira
WBC_CONTROL_HZ = 50.0  # decoupled_wbc's trained control rate

G1_USD = REPO / "assets/g1_29dof_sensors.usd"
ROBOT_PRIM = "/World/G1"
# Fallback-only static targets (used when IRA is off/unavailable, so the
# scene still has something LiDAR-visible besides the robot itself).
PEDESTRIANS = [(3.0, 0.0), (4.5, -2.0), (6.0, 2.5)]

GUI_EXTENSIONS = [
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.kit.widget.stage",
]


def enable_extensions() -> None:
    manager = omni.kit.app.get_app().get_extension_manager()

    # Must precede any rclpy import - see g1_rtx_sim.py.
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    manager.set_extension_enabled_immediate("isaacsim.sensors.rtx", True)
    # isaacsim.sensors.physics.nodes registers the IsaacReadIMU OmniGraph node.
    # Deliberately NOT enabling the deprecated isaacsim.sensors.physics
    # extension - see spawn_imu_sensor()'s docstring for the Kit-command
    # ambiguity that causes.
    manager.set_extension_enabled_immediate("isaacsim.sensors.physics.nodes", True)

    if ENABLE_IRA:
        import g1_sim.ira_actors as ira_actors

        ira_actors.enable_extension()

    if not args_cli.headless:
        for ext in GUI_EXTENSIONS:
            manager.set_extension_enabled_immediate(ext, True)

    for _ in range(20):
        omni.kit.app.get_app().update()


def build_scene_ira() -> bool:
    """Try IRA: warehouse + humans + Nova Carters, all in one stage-owning
    call. Returns True on success."""
    import g1_sim.ira_actors as ira_actors

    config_path = REPO / "assets/ira_warehouse_config.yaml"
    ira_actors.write_config(
        config_path,
        warehouse_rel=WAREHOUSE_USD.lstrip("/"),
        num_humans=args_cli.num_humans,
        num_carters=args_cli.num_carters,
        seed=args_cli.ira_seed,
        duration_s=200.0,
    )
    print(f"[WH] IRA config      : {config_path}  ({args_cli.num_humans} humans, {args_cli.num_carters} carters)")

    return ira_actors.run_setup_blocking(
        simulation_app, config_path, max_navmesh_frames=args_cli.navmesh_max_frames
    )


def build_scene_fallback(stage) -> None:
    """Warehouse with no dynamic actors - used when --no-ira is passed or
    IRA's setup failed."""
    stage.DefinePrim("/World", "Xform")
    try:
        resolved = load_environment(stage, WAREHOUSE_USD, "/World/Env")
        print(f"[WH] environment     : {resolved}")
    except Exception as e:
        print(f"[WH] warehouse load failed ({e}), using flat ground")
        build_flat_ground(stage)

    for i, (x, y) in enumerate(PEDESTRIANS):
        box = UsdGeom.Cube.Define(stage, f"/World/targets/pedestrian_{i}")
        box.CreateSizeAttr(1.0)
        box.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.875))
        box.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 1.75))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())


def spawn_g1(stage) -> None:
    if not G1_USD.exists():
        raise SystemExit(f"[WH] {G1_USD} not found - run scripts/convert_g1_urdf_to_usd.py first")
    # After IRA's setup, the stage's root layer *is* the remote warehouse USD
    # (a full stage-open, not a reference - see g1_sim/ira_actors.py) and
    # becomes the default edit target. Authoring a reference to a local
    # filesystem path there composed the arc (AddReference returns True,
    # HasAuthoredReferences() is True) but its content silently failed to
    # resolve (pelvis and every other child prim missing) - almost certainly
    # the Nucleus/HTTP-aware asset resolver mishandling a plain absolute
    # POSIX path while anchored to an https:// layer. The session layer is
    # always a local anonymous layer regardless of what the root layer is,
    # so switching the edit target there sidesteps the resolver mismatch
    # entirely; harmless in the non-IRA fallback path too, where the root
    # layer is already local anonymous.
    stage.SetEditTarget(stage.GetSessionLayer())
    robot = stage.DefinePrim(ROBOT_PRIM, "Xform")
    robot.GetReferences().AddReference(str(G1_USD))
    xform = UsdGeom.Xformable(robot)
    translate = next((op for op in xform.GetOrderedXformOps() if "translate" in op.GetOpName()), None)
    if translate is None:
        translate = xform.AddTranslateOp()
    translate.Set(Gf.Vec3d(0.0, 0.0, 0.8))


def main() -> None:
    from isaacsim.core.api import SimulationContext

    enable_extensions()

    ira_ok = False
    if ENABLE_IRA:
        ira_ok = build_scene_ira()
        if not ira_ok:
            print("[WH] IRA setup failed - falling back to warehouse with no dynamic actors")

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    if not ira_ok:
        build_scene_fallback(stage)

    # G1 always goes on top, regardless of which path built the environment.
    spawn_g1(stage)
    # A few frames of margin for the reference's composition to settle before
    # anything queries child prims (Articulation's pelvis lookup below).
    for _ in range(10):
        simulation_app.update()
    pelvis_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM}/pelvis")
    print(f"[WH] G1 prim valid   : {stage.GetPrimAtPath(ROBOT_PRIM).IsValid()}  pelvis valid: {pelvis_prim.IsValid()}")

    sim = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / SIM_RATE_HZ,
        rendering_dt=1.0 / SIM_RATE_HZ,
    )

    from isaacsim.core.prims import Articulation

    robot_articulation = Articulation(f"{ROBOT_PRIM}/pelvis", name="g1")
    sim.reset()
    print(f"[WH] articulation    : {robot_articulation.num_dof} DOF")
    print(f"[WH] IRA actors      : {'yes' if ira_ok else 'no'}")

    wbc_bridge = None
    if ENABLE_LOCOMOTION:
        from g1_sim.wbc_bridge import (
            ARM_JOINTS,
            ARM_KD,
            ARM_KP,
            ALL_JOINTS,
            KD,
            KP,
            LEG_WAIST_JOINTS,
            WbcBridge,
            quat_rotate_inverse,
        )

        dof_names = set(robot_articulation.dof_names or [])
        missing = [j for j in ALL_JOINTS if j not in dof_names]
        if missing:
            print(f"[WH] WBC bridge      : DISABLED - USD is missing joints {missing}")
        else:
            wbc_bridge = WbcBridge(
                REPO / "assets/policy/GR00T-WholeBodyControl-Balance.onnx",
                REPO / "assets/policy/GR00T-WholeBodyControl-Walk.onnx",
            )
            robot_articulation.set_gains(kps=KP[None, :], kds=KD[None, :], joint_names=LEG_WAIST_JOINTS)
            robot_articulation.set_gains(
                kps=np.full((1, len(ARM_JOINTS)), ARM_KP, dtype=np.float32),
                kds=np.full((1, len(ARM_JOINTS)), ARM_KD, dtype=np.float32),
                joint_names=ARM_JOINTS,
            )
            print("[WH] WBC bridge      : loaded (decoupled_wbc Balance/Walk, gains overridden)")

    mount = f"{ROBOT_PRIM}/torso_link"
    if not omni.usd.get_context().get_stage().GetPrimAtPath(mount).IsValid():
        raise SystemExit(f"[WH] mount prim {mount} missing from the USD")

    prim_paths = spawn_mid360(
        mount,
        config_dir=REPO / args_cli.config_dir,
        translation=MID360_POS,
        orientation=MID360_QUAT_WXYZ,
    )
    if args_cli.num_prims and args_cli.num_prims < len(prim_paths):
        prim_paths = prim_paths[: args_cli.num_prims]
    print(f"[WH] lidar prims     : {len(prim_paths)}")

    mount_height = 0.8 + MID360_POS[2]
    print(f"[WH] mount height    : {mount_height:.2f} m")
    print(f"[WH] blind radius    : {blind_radius(mount_height):.2f} m")

    publisher = None
    cmd_vel_graph_path = None
    if ENABLE_ROS2:
        graph = attach_ros2_publishers(prim_paths, sim_rate_hz=SIM_RATE_HZ, combine=True)
        print(f"[WH] lidar graph     : {graph}")

        state_graph = attach_robot_state_publishers(ROBOT_PRIM)
        print(f"[WH] state graph     : {state_graph}  (/tf, /g1/joint_states, /clock)")

        if ira_ok:
            import g1_sim.ira_actors as ira_actors

            # Re-enabled 2026-08-10: briefly disabled on the theory that this
            # graph's "[PoseTree] eInvalid" spam was uniquely responsible for
            # a stalled boot, but the same spam (just "parent .../World"
            # eInvalid, no human targets) kept firing with this disabled -
            # so it isn't the (sole) cause and disabling it bought nothing.
            # The eInvalid warning itself is still unexplained/unfixed - see
            # FUTURE_STEPS.md - but it doesn't block this from being useful.
            actor_prims = ira_actors.discover_actor_prims(stage)
            actors_graph = ira_actors.attach_actor_tf_publishers(actor_prims)
            print(f"[WH] actors tf graph : {actors_graph}  ({len(actor_prims)} actors: {actor_prims})")

            carter_prims = ira_actors.discover_prims_at(stage, "/World/Robots/carters")
            if carter_prims:
                # Publishes IRA's own existing carter IMU (read-only) -
                # see find_imu_prim()'s docstring for why we don't create one.
                carter_imu_graphs = ira_actors.attach_carter_imu_publishers(stage, carter_prims)
                print(f"[WH] carter imus     : {len(carter_imu_graphs)}/{len(carter_prims)} found -> /carter_N/imu")

        imu_prim = spawn_imu_sensor(f"{ROBOT_PRIM}/torso_link/imu_in_torso")
        imu_graph = attach_imu_publisher(imu_prim)
        print(f"[WH] imu graph       : {imu_graph}  (/g1/imu, prim={imu_prim})")

        if wbc_bridge is not None:
            cmd_vel_graph_path = attach_cmd_vel_subscriber()
            print(f"[WH] cmd_vel graph   : {cmd_vel_graph_path}  (/g1/cmd_vel)")

        if not args_cli.no_camera:
            camera_prim = spawn_camera(f"{ROBOT_PRIM}/torso_link")
            cam_graph = attach_camera_publishers(camera_prim)
            print(f"[WH] camera graph    : {cam_graph}")
            print("[WH] camera topics   : /g1/camera/{rgb,depth,semantic,camera_info}")

            labels = {ROBOT_PRIM: "robot"}
            if not ira_ok:
                labels.update({f"/World/targets/pedestrian_{i}": "pedestrian" for i in range(len(PEDESTRIANS))})
            print(f"[WH] semantics       : {apply_semantics(labels)} prims labelled")

        import rclpy

        from g1_sim.rtx_publisher import RtxLidarPublisher

        rclpy.init()
        lidar_topics = ["/livox/mid360/points", "/g1/lidar/points"]
        publisher = RtxLidarPublisher(prim_paths, topic=lidar_topics, publish_rate=10.0)
        print(f"[WH] publisher       : rclpy (annotator) -> {lidar_topics}")
    else:
        print("[WH] ROS2 disabled")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print(f"[WH] timeline        : playing={timeline.is_playing()}")
    print("[WH] running\n")

    step = 0
    scans = 0
    last_points = 0
    wbc_updates = 0
    wbc_control_period = 1.0 / WBC_CONTROL_HZ
    wbc_next_time = 0.0
    try:
        while simulation_app.is_running():
            sim.step(render=True)
            step += 1

            if wbc_bridge is not None and sim.current_time >= wbc_next_time:
                wbc_next_time = sim.current_time + wbc_control_period

                cmd_vx, cmd_vy, cmd_wz = 0.0, 0.0, 0.0
                if cmd_vel_graph_path is not None:
                    from g1_sim.action_graph import read_cmd_vel

                    cmd_vx, cmd_vy, cmd_wz = read_cmd_vel(graph_path=cmd_vel_graph_path)

                qpos_all = np.asarray(robot_articulation.get_joint_positions(joint_names=ALL_JOINTS))[0]
                qvel_all = np.asarray(robot_articulation.get_joint_velocities(joint_names=ALL_JOINTS))[0]
                _, quat_wxyz = robot_articulation.get_world_poses()
                quat_wxyz = np.asarray(quat_wxyz)[0]
                ang_vel_world = np.asarray(robot_articulation.get_angular_velocities())[0]
                ang_vel_body = quat_rotate_inverse(quat_wxyz, ang_vel_world)

                target = wbc_bridge.step(qpos_all, qvel_all, quat_wxyz, ang_vel_body, cmd_vx, cmd_vy, cmd_wz)
                robot_articulation.set_joint_position_targets(
                    target[None, :].astype(np.float32), joint_names=LEG_WAIST_JOINTS
                )
                if wbc_updates == 0:
                    # Arms held at the all-zero URDF pose (straight down at
                    # the sides) sit entirely outside the D435's downward-
                    # pitched FOV. Raise both shoulder_pitch joints (index 0
                    # left, 7 right in ARM_JOINTS) forward so the arms enter
                    # frame, per live testing request 2026-08-10. Sign/
                    # magnitude picked from the URDF's zero-pose convention
                    # (positive shoulder_pitch = forward raise for this
                    # robot family) but not yet re-verified live - adjust if
                    # the arms raise backward instead.
                    arm_pose = np.zeros((1, len(ARM_JOINTS)), dtype=np.float32)
                    arm_pose[0, 0] = 0.6   # left_shoulder_pitch_joint
                    arm_pose[0, 7] = 0.6   # right_shoulder_pitch_joint
                    robot_articulation.set_joint_position_targets(
                        arm_pose, joint_names=ARM_JOINTS
                    )
                wbc_updates += 1
                if wbc_updates % 50 == 0:
                    print(
                        f"[WH] wbc cmd=({cmd_vx:.2f},{cmd_vy:.2f},{cmd_wz:.2f})  "
                        f"updates={wbc_updates}  pelvis_z={robot_articulation.get_world_poses()[0][0][2]:.3f}"
                    )

            if publisher is not None:
                publisher.accumulate()
                sent = publisher.publish(sim.current_time)
                if sent:
                    scans += 1
                    last_points = sent
                publisher.spin_once()

            if step % 100 == 0:
                print(f"[WH] step {step:>6}  scans {scans}  points {last_points}")
            if args_cli.steps and step >= args_cli.steps:
                break
    except KeyboardInterrupt:
        print("\n[WH] interrupted")
    finally:
        if publisher is not None:
            import rclpy

            publisher.destroy()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
