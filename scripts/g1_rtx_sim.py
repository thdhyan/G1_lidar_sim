#!/usr/bin/env python3
"""G1 with an RTX LiDAR Livox Mid-360, on Isaac Sim 6.0.

Run in the ``isaac`` conda env (Isaac Sim 6.0 / Python 3.12), not
``env_isaaclab`` (5.1 / 3.11):

    conda activate isaac
    python scripts/g1_rtx_sim.py                 # GUI, action graph visible
    python scripts/g1_rtx_sim.py --headless

Unlike the warp ray-caster in ``g1_ros2_sim.py``, RTX LiDAR ray-traces the
rendered scene, so it sees the robot itself and anything spawned later, and its
returns change as the robot moves.
"""

import argparse
import sys
from pathlib import Path

from isaacsim import SimulationApp

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

parser = argparse.ArgumentParser(description="G1 with RTX LiDAR Mid-360.")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--steps", type=int, default=0, help="Stop after N steps; 0 runs forever.")
parser.add_argument("--no-ros2", action="store_true")
parser.add_argument(
    "--config-dir",
    type=str,
    default="assets/lidar_configs_light",
    help="Emitter-state profiles. The 'light' set is 4 prims; the full set is 8.",
)
parser.add_argument(
    "--separate-topics",
    action="store_true",
    help="Publish each sensor prim on its own topic instead of merging them.",
)
args_cli = parser.parse_args()

SIM_RATE_HZ = 60.0

simulation_app = SimulationApp(
    {
        "headless": args_cli.headless,
        # Forces CPU-side buffering for LiDAR returns. The GPU path hits
        # cudaMemcpyAsync races when several RTX LiDAR prims publish over ROS2
        # (IsaacSim discussion #685).
        "/app/sensors/nv/lidar/outputBufferOnGPU": False,
    }
)

"""Rest everything follows."""

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from g1_sim.rtx_lidar import (
    MID360_POS,
    MID360_QUAT_WXYZ,
    attach_ros2_publishers,
    blind_radius,
    spawn_mid360,
)

ENABLE_ROS2 = not args_cli.no_ros2

G1_USD = REPO / "assets/g1_29dof_sensors.usd"
ROBOT_PRIM = "/World/G1"
PEDESTRIANS = [(12.0, 0.0), (18.0, -4.0), (25.0, 6.0)]

# Loaded so the graph is inspectable in the GUI - Window > Visual Scripting >
# Action Graph - which is how you confirm what is publishing where.
GUI_EXTENSIONS = [
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.kit.widget.stage",
]


def enable_extensions() -> None:
    manager = omni.kit.app.get_app().get_extension_manager()

    # Must precede any rclpy import: the bridge also registers the ROS2
    # OmniGraph node types the publishers rely on.
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    manager.set_extension_enabled_immediate("isaacsim.sensors.rtx", True)

    if not args_cli.headless:
        for ext in GUI_EXTENSIONS:
            manager.set_extension_enabled_immediate(ext, True)

    for _ in range(20):
        omni.kit.app.get_app().update()


def build_scene() -> None:
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")

    ground = UsdGeom.Cube.Define(stage, "/World/ground")
    ground.CreateSizeAttr(1.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.1))
    ground.AddScaleOp().Set(Gf.Vec3f(120.0, 120.0, 0.2))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    light = UsdLux.DistantLight.Define(stage, "/World/light")
    light.CreateIntensityAttr(3000.0)

    for i, (x, y) in enumerate(PEDESTRIANS):
        box = UsdGeom.Cube.Define(stage, f"/World/targets/pedestrian_{i}")
        box.CreateSizeAttr(1.0)
        box.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.875))
        box.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 1.75))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    if not G1_USD.exists():
        raise SystemExit(
            f"[RTX] {G1_USD} not found - run scripts/convert_g1_urdf_to_usd.py first"
        )
    robot = stage.DefinePrim(ROBOT_PRIM, "Xform")
    robot.GetReferences().AddReference(str(G1_USD))
    # The referenced layer already defines transform ops, so reuse the existing
    # translate op rather than adding a second one (which USD rejects).
    xform = UsdGeom.Xformable(robot)
    translate = next(
        (op for op in xform.GetOrderedXformOps() if "translate" in op.GetOpName()), None
    )
    if translate is None:
        translate = xform.AddTranslateOp()
    translate.Set(Gf.Vec3d(0.0, 0.0, 0.8))


def main() -> None:
    from isaacsim.core.api import SimulationContext

    enable_extensions()
    build_scene()

    sim = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / SIM_RATE_HZ,
        rendering_dt=1.0 / SIM_RATE_HZ,
    )

    # The sensor mounts under torso_link, so it inherits the torso's motion.
    mount = f"{ROBOT_PRIM}/torso_link"
    if not omni.usd.get_context().get_stage().GetPrimAtPath(mount).IsValid():
        raise SystemExit(f"[RTX] mount prim {mount} missing from the USD")

    prim_paths = spawn_mid360(
        mount,
        config_dir=REPO / args_cli.config_dir,
        translation=MID360_POS,
        orientation=MID360_QUAT_WXYZ,
    )
    print(f"[RTX] sensor prims   : {len(prim_paths)}")
    for p in prim_paths:
        print(f"[RTX]   {p}")

    mount_height = 0.8 + MID360_POS[2]
    print(f"[RTX] mount height   : {mount_height:.2f} m")
    print(f"[RTX] blind radius   : {blind_radius(mount_height):.2f} m (no nadir ray)")

    if ENABLE_ROS2:
        graph = attach_ros2_publishers(
            prim_paths,
            sim_rate_hz=SIM_RATE_HZ,
            combine=not args_cli.separate_topics,
        )
        print(f"[RTX] ROS2 graph     : {graph}")
        print(f"[RTX] publishing     : /livox/mid360/points @ {SIM_RATE_HZ / 6:.0f} Hz")
    else:
        print("[RTX] ROS2 disabled")

    sim.reset()

    # OnPlaybackTick - which drives the ROS2 helpers - only fires while the
    # timeline is playing. Stepping physics alone leaves the graph dormant and
    # the topic advertised but silent.
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print(f"[RTX] timeline       : playing={timeline.is_playing()}")
    print("[RTX] running\n")

    step = 0
    try:
        while simulation_app.is_running():
            sim.step(render=True)
            step += 1

            if step % 100 == 0:
                print(f"[RTX] step {step:>6}")
            if args_cli.steps and step >= args_cli.steps:
                break
    except KeyboardInterrupt:
        print("\n[RTX] interrupted")


if __name__ == "__main__":
    main()
    simulation_app.close()
