#!/usr/bin/env python3
"""Diagnostic: why are no rays hitting anything?

Prints the meshes the sensor actually bound to, the sensor pose, and a sample of
ray hit points, for both a GroundPlaneCfg and an explicit cuboid ground.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import LidarSensorCfg
from isaaclab.sensors.ray_caster.patterns import LivoxPatternCfg
from isaaclab.utils import configclass

SENSOR_HEIGHT = 2.0


@configclass
class DebugSceneCfg(InteractiveSceneCfg):
    # A large flat cuboid instead of GroundPlaneCfg: an explicit mesh is
    # guaranteed to be ray-castable, which the ground-plane proxy may not be.
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.CuboidCfg(
            size=(60.0, 60.0, 0.2),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.1)),
    )

    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DistantLightCfg(intensity=3000.0))

    sensor_base = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/SensorBase",
        spawn=sim_utils.CuboidCfg(size=(0.1, 0.1, 0.1)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, SENSOR_HEIGHT)),
    )

    lidar = LidarSensorCfg(
        prim_path="{ENV_REGEX_NS}/SensorBase",
        pattern_cfg=LivoxPatternCfg(sensor_type="mid360", samples=8000),
        mesh_prim_paths=["/World/ground"],
        max_distance=50.0,
        min_range=0.1,
        return_pointcloud=True,
        pointcloud_in_world_frame=True,
        update_frequency=10.0,
    )


def main() -> None:
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device=args_cli.device))
    scene = InteractiveScene(DebugSceneCfg(num_envs=1, env_spacing=8.0))
    sim.reset()

    lidar = scene["lidar"]
    print(f"\n[DBG] bound meshes: {list(lidar.meshes.keys())}")
    print(f"[DBG] num_rays    : {lidar.num_rays}")

    for _ in range(10):
        sim.step()
        scene.update(sim.get_physics_dt())

    print(f"[DBG] sensor pos_w : {lidar.data.pos_w[0].tolist()}")
    print(f"[DBG] sensor quat_w: {lidar.data.quat_w[0].tolist()}")

    hits = lidar.data.ray_hits_w[0]
    finite_rows = torch.isfinite(hits).all(dim=1)
    print(f"[DBG] finite hit points: {int(finite_rows.sum())} / {hits.shape[0]}")
    print(f"[DBG] first 5 hits:\n{hits[:5]}")

    if finite_rows.any():
        good = hits[finite_rows]
        print(f"[DBG] hit z range: {good[:, 2].min():.3f} .. {good[:, 2].max():.3f}")

    distances = lidar.data.distances[0]
    print(f"[DBG] distance min/max: {distances.min():.3f} / {distances.max():.3f}")
    print(f"[DBG] distances < 50: {int((distances < 50.0).sum())}")

    # Ray directions tell us whether the pattern points anywhere useful.
    directions = lidar.ray_directions[0]
    print(f"[DBG] ray dir sample:\n{directions[:3]}")
    print(f"[DBG] rays pointing down (z<0): {int((directions[:, 2] < 0).sum())} / {directions.shape[0]}")


if __name__ == "__main__":
    main()
    simulation_app.close()
