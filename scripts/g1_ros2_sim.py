#!/usr/bin/env python3
"""Spawn the G1 with its Mid-360 and camera, and publish everything over ROS2.

    python scripts/g1_ros2_sim.py                 # with ROS2
    python scripts/g1_ros2_sim.py --no-ros2       # simulation only
    python scripts/g1_ros2_sim.py --headless

Published topics are listed in README.md. Run external nodes with
``use_sim_time:=true`` so they follow ``/clock``.
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

parser = argparse.ArgumentParser(description="G1 with Livox Mid-360 and ROS2.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--no-ros2", action="store_true", help="Disable all ROS2 publishing.")
parser.add_argument("--no-camera", action="store_true", help="Skip the camera (saves render time).")
parser.add_argument("--steps", type=int, default=0, help="Stop after N steps; 0 runs forever.")
parser.add_argument("--debug-vis", action="store_true", help="Draw LiDAR rays in the viewer.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Spawning a Camera without this raises at sensor-init time. --no-camera already
# says whether a camera is wanted, so derive the flag instead of making the user
# pass both.
if not args_cli.no_camera:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from g1_sim.robots.unitree_g1_lidar import (
    G1_LIDAR_CFG,
    blind_radius,
    camera_cfg,
    mid360_cfg,
)

ENABLE_ROS2 = not args_cli.no_ros2
ENABLE_CAMERA = not args_cli.no_camera

# Things for the LiDAR to see. The Mid-360's blind cone means anything closer
# than ~8x the mount height is invisible, so these sit well out from the robot.
PEDESTRIAN_POSITIONS = [(12.0, 0.0), (18.0, -4.0), (25.0, 6.0)]


@configclass
class G1SceneCfg(InteractiveSceneCfg):
    """Ground, lights, the G1 with sensors, and some pedestrian-sized targets."""

    # An explicit cuboid rather than GroundPlaneCfg: it yields a plain Cube prim
    # that the warp ray-caster's mesh extraction handles directly.
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.CuboidCfg(
            size=(120.0, 120.0, 0.2),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.25, 0.25, 0.28)),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.1)),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=3000.0),
    )

    robot = G1_LIDAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    lidar = mid360_cfg(
        "{ENV_REGEX_NS}/Robot",
        # Static world geometry only; per-environment meshes would need
        # dynamic_env_mesh_prim_paths.
        mesh_prim_paths=["/World/ground", "/World/targets"],
        debug_vis=args_cli.debug_vis,
    )

    if ENABLE_CAMERA:
        camera = camera_cfg("{ENV_REGEX_NS}/Robot")


def add_pedestrian_targets() -> None:
    """Spawn upright person-sized boxes for the detector to find."""
    for i, (x, y) in enumerate(PEDESTRIAN_POSITIONS):
        cfg = sim_utils.CuboidCfg(
            size=(0.5, 0.5, 1.75),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.6, 0.2)),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )
        cfg.func(f"/World/targets/pedestrian_{i}", cfg, translation=(x, y, 0.875))


def main() -> None:
    # Enable the ROS2 bridge BEFORE the scene is built. Enabling it later
    # reloads the USD stage, which invalidates the physics view of the
    # articulation and yields "Provided pattern list did not match any
    # articulations" with no Python traceback.
    if ENABLE_ROS2:
        from g1_sim.action_graph import enable_ros2_bridge

        enable_ros2_bridge()
        print("[SIM] ROS2 bridge enabled")

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    )
    sim.set_camera_view(eye=(8.0, 8.0, 5.0), target=(0.0, 0.0, 1.0))

    add_pedestrian_targets()
    scene = InteractiveScene(G1SceneCfg(num_envs=args_cli.num_envs, env_spacing=10.0))
    sim.reset()

    robot = scene["robot"]
    lidar = scene["lidar"]

    print(f"\n[SIM] G1 joints      : {robot.num_joints}")
    print(f"[SIM] LiDAR rays     : {lidar.num_rays}")
    mount_height = float(lidar.data.pos_w[0, 2])
    print(f"[SIM] LiDAR height   : {mount_height:.2f} m")
    print(f"[SIM] LiDAR blind r  : {blind_radius(mount_height):.2f} m (no nadir ray)")

    publisher = None
    if ENABLE_ROS2:
        from g1_sim.action_graph import build_g1_action_graph

        # Ask the articulation where it actually is rather than assuming the
        # cloner's naming. The OmniGraph nodes need the prim that carries the
        # articulation root, and a wrong path fails at runtime with only a
        # physx "did not match any articulations" log line.
        robot_prim = robot.cfg.prim_path.replace("{ENV_REGEX_NS}", "/World/envs/env_0")
        if robot.root_physx_view is not None:
            reported = robot.root_physx_view.prim_paths
            if reported:
                robot_prim = reported[0]
        print(f"[SIM] robot prim     : {robot_prim}")

        camera_prim = f"{robot_prim}/torso_link/d435_camera" if ENABLE_CAMERA else None
        build_g1_action_graph(robot_prim_path=robot_prim, camera_prim_path=camera_prim)
        print(f"[SIM] action graph   : built for {robot_prim}")

        # Import only after the bridge is enabled - it supplies rclpy.
        import rclpy

        from g1_sim.lidar_publisher import LidarPointCloudPublisher

        rclpy.init()
        publisher = LidarPointCloudPublisher(
            sensor=lidar,
            frame_id="mid360_link",
            publish_rate=lidar.cfg.update_frequency,
        )
        print("[SIM] ROS2 publishing enabled\n")
    else:
        print("[SIM] ROS2 disabled\n")

    step = 0
    scans = 0
    try:
        while simulation_app.is_running():
            # No locomotion policy yet: hold the default stance so the robot
            # stays upright while the sensor and ROS2 paths are exercised.
            robot.set_joint_position_target(robot.data.default_joint_pos)
            scene.write_data_to_sim()

            sim.step()
            scene.update(sim.get_physics_dt())

            if publisher is not None:
                if publisher.publish(sim.current_time):
                    scans += 1
                publisher.spin_once()

            step += 1
            if step % 100 == 0:
                hits = int((lidar.data.distances[0] < lidar.cfg.max_distance).sum())
                # Track the sensor pose alongside the hit count: if the height
                # never changes while the robot falls, the ray-caster is not
                # following the body it is mounted on.
                lz = float(lidar.data.pos_w[0, 2])
                bz = float(robot.data.root_pos_w[0, 2])
                print(
                    f"[SIM] step {step:>6}  base_z {bz:6.3f}  lidar_z {lz:6.3f}  "
                    f"hits {hits:>6}  scans {scans}"
                )

            if args_cli.steps and step >= args_cli.steps:
                break
    except KeyboardInterrupt:
        print("\n[SIM] interrupted")
    finally:
        if publisher is not None:
            import rclpy

            publisher.destroy()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
