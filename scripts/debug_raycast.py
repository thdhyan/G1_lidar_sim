#!/usr/bin/env python3
"""Diagnostic: inspect the warp mesh the sensor built and ray-cast against it manually.

If the mesh has sane bounds and a hand-rolled downward ray hits it, the fault is
in how the sensor orients or launches its rays rather than in the geometry.
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
import warp as wp

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import LidarSensorCfg
from isaaclab.sensors.ray_caster.patterns import LivoxPatternCfg
from isaaclab.utils import configclass

SENSOR_HEIGHT = 2.0


@configclass
class RaycastSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.CuboidCfg(size=(60.0, 60.0, 0.2)),
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
    scene = InteractiveScene(RaycastSceneCfg(num_envs=1, env_spacing=8.0))
    sim.reset()

    lidar = scene["lidar"]
    for _ in range(5):
        sim.step()
        scene.update(sim.get_physics_dt())

    key = list(lidar.meshes.keys())[0]
    mesh = lidar.meshes[key]
    vertices = wp.to_torch(mesh.points)
    print(f"\n[RC] mesh key      : {key}")
    print(f"[RC] vertices      : {vertices.shape[0]}")
    print(f"[RC] bounds min    : {vertices.min(dim=0).values.tolist()}")
    print(f"[RC] bounds max    : {vertices.max(dim=0).values.tolist()}")

    # Cast one unambiguous ray straight down from above the mesh centre. If this
    # misses, the mesh itself is unusable; if it hits, the sensor's own rays are
    # the problem.
    origin = wp.array([[0.0, 0.0, SENSOR_HEIGHT]], dtype=wp.vec3, device=str(lidar.device))
    direction = wp.array([[0.0, 0.0, -1.0]], dtype=wp.vec3, device=str(lidar.device))
    hits = wp.zeros((1,), dtype=wp.vec3, device=str(lidar.device))

    from isaaclab.utils.warp import raycast_mesh

    hit_points, distances, *_ = raycast_mesh(
        wp.to_torch(origin).view(1, 1, 3),
        wp.to_torch(direction).view(1, 1, 3),
        mesh=mesh,
        max_dist=100.0,
        return_distance=True,
    )
    print(f"[RC] manual down-ray hit : {hit_points.view(-1).tolist()}")
    print(f"[RC] manual down-ray dist: {None if distances is None else distances.view(-1).tolist()}")

    # Now the sensor's own rays, after whatever transform it applies.
    print(f"\n[RC] sensor pos_w  : {lidar.data.pos_w[0].tolist()}")
    directions = lidar.ray_directions[0]
    print(f"[RC] ray_directions shape: {tuple(directions.shape)}")
    print(f"[RC] rays with z<0 : {int((directions[:, 2] < 0).sum())} / {directions.shape[0]}")
    print(f"[RC] dir z min/max : {directions[:, 2].min():.4f} / {directions[:, 2].max():.4f}")

    starts = lidar.ray_starts[0]
    print(f"[RC] ray_starts z min/max: {starts[:, 2].min():.4f} / {starts[:, 2].max():.4f}")

    finite = torch.isfinite(lidar.data.ray_hits_w[0]).all(dim=1)
    print(f"[RC] sensor finite hits  : {int(finite.sum())} / {finite.shape[0]}")

    # Re-cast the sensor's own rays through the same helper that worked above.
    # If these hit, the geometry and directions are fine and the sensor is
    # mis-launching them; if they miss, the directions themselves are bad.
    down = directions[:, 2] < 0
    if int(down.sum()) > 0:
        sel = torch.nonzero(down).flatten()[:5]
        origins = lidar.data.pos_w[0].view(1, 1, 3).expand(1, sel.numel(), 3).contiguous()
        dirs = directions[sel].view(1, -1, 3).contiguous()
        hp, dist, *_ = raycast_mesh(origins, dirs, mesh=mesh, max_dist=100.0, return_distance=True)
        print(f"[RC] manual recast of sensor down-rays:")
        for i in range(sel.numel()):
            print(f"      dir={dirs[0, i].tolist()} -> hit={hp[0, i].tolist()} dist={dist[0, i].item():.3f}")
    else:
        print("[RC] sensor has NO downward rays - pattern orientation is the fault")


if __name__ == "__main__":
    main()
    simulation_app.close()
