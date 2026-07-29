#!/usr/bin/env python3
"""Task 0 gate: verify the OmniPerception warp LidarSensor works on this
Isaac Sim / IsaacLab install.

Spawns a flat ground plane, a single ray-casting Livox Mid-360 sensor on a
static prim, steps the sim, and checks that the returned hit distances are
finite and non-degenerate. Exits non-zero if the sensor is unusable.

    ./isaaclab.sh -p scripts/00_verify_warp_lidar.py
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify warp LidarSensor integration.")
parser.add_argument("--steps", type=int, default=20, help="Number of sim steps to run.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import sys

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import LidarSensorCfg
from isaaclab.sensors.ray_caster.patterns import LivoxPatternCfg
from isaaclab.utils import configclass

SENSOR_HEIGHT = 2.0

# Steepest downward tilt in the Mid-360 pattern (phi min = -7.21 deg). The
# sensor has no straight-down ray, so this sets how close a ground return can
# possibly be - see Plan.md.
MIN_DOWN_TILT_DEG = 7.16


@configclass
class VerifySceneCfg(InteractiveSceneCfg):
    """Flat ground, one light, one Mid-360."""

    # A flat cuboid rather than GroundPlaneCfg: it produces a plain Cube prim
    # that the ray-caster's mesh extraction handles directly.
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.CuboidCfg(
            size=(60.0, 60.0, 0.2),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.1)),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=3000.0),
    )

    # The sensor needs an existing prim to attach to; an Xform is enough since
    # we are not testing articulation tracking here.
    sensor_base = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/SensorBase",
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 0.1, 0.1),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, SENSOR_HEIGHT)),
    )

    lidar = LidarSensorCfg(
        prim_path="{ENV_REGEX_NS}/SensorBase",
        # The .npy pattern is a time-ordered scan trajectory and the loader takes
        # a contiguous slice of it, so the sample count selects a *window in
        # time*, not a uniform subsample. The first ~8k rows of mid360.npy are a
        # single upward sweep containing no downward rays at all; use the
        # sensor's native 20k so a full revolution is covered.
        pattern_cfg=LivoxPatternCfg(sensor_type="mid360", samples=20000),
        mesh_prim_paths=["/World/ground"],
        max_distance=50.0,
        min_range=0.1,
        return_pointcloud=True,
        pointcloud_in_world_frame=False,
        update_frequency=10.0,
        debug_vis=False,
    )


def fail(msg: str) -> None:
    print(f"\n[GATE] FAIL: {msg}")
    simulation_app.close()
    sys.exit(1)


def main() -> None:
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device=args_cli.device))
    scene = InteractiveScene(VerifySceneCfg(num_envs=2, env_spacing=8.0))
    sim.reset()

    lidar = scene["lidar"]
    print(f"[GATE] sensor initialised: {lidar.num_rays} rays x {lidar.data.pos_w.shape[0]} envs")

    for _ in range(args_cli.steps):
        sim.step()
        scene.update(sim.get_physics_dt())

    distances = lidar.data.distances
    if distances is None:
        fail("sensor produced no distance buffer")

    finite = torch.isfinite(distances)
    # Rays that miss everything legitimately report max_distance, so only the
    # rays that actually hit the ground are meaningful for the sanity check.
    hits = distances[finite & (distances > 0) & (distances < 50.0)]

    print(f"[GATE] distances shape : {tuple(distances.shape)}")
    print(f"[GATE] finite fraction : {finite.float().mean().item():.3f}")
    print(f"[GATE] valid hits      : {hits.numel()}")

    if not finite.all():
        fail("distance buffer contains NaN/Inf")
    if hits.numel() == 0:
        fail("no rays hit the ground plane - ray-casting is not working")

    print(f"[GATE] hit range       : {hits.min().item():.3f} .. {hits.max().item():.3f} m")

    # The Mid-360 tilts at most MIN_DOWN_TILT_DEG below horizontal, so the
    # closest possible ground return from SENSOR_HEIGHT is that shallow ray's
    # slant range - NOT SENSOR_HEIGHT, since the sensor has no nadir ray.
    closest_possible = SENSOR_HEIGHT / math.sin(math.radians(MIN_DOWN_TILT_DEG))
    print(f"[GATE] closest possible: {closest_possible:.3f} m (phi = -{MIN_DOWN_TILT_DEG} deg)")

    # 10% tolerance absorbs pattern quantisation and the ground slab's thickness.
    if hits.min().item() < closest_possible * 0.9:
        fail(f"closest hit {hits.min().item():.3f} m is nearer than geometry allows ({closest_possible:.3f} m)")
    if hits.min().item() > closest_possible * 1.5:
        fail(f"closest hit {hits.min().item():.3f} m is far beyond the expected {closest_possible:.3f} m")

    pointcloud = lidar.data.pointcloud
    if pointcloud is not None:
        print(f"[GATE] pointcloud shape: {tuple(pointcloud.shape)}")
        # Rays that hit nothing carry an infinite hit point by design, so only
        # the returns that actually landed are required to be finite. NaN is
        # never acceptable - it would mean a corrupted transform rather than a
        # miss - so check for it across the whole buffer.
        if torch.isnan(pointcloud).any():
            fail("pointcloud contains NaN")

        finite_points = pointcloud[torch.isfinite(pointcloud).all(dim=2)]
        print(f"[GATE] finite points   : {finite_points.shape[0]}")
        if finite_points.shape[0] == 0:
            fail("pointcloud has no finite points")

    print("\n[GATE] PASS - warp LidarSensor is functional on this install")


if __name__ == "__main__":
    main()
    simulation_app.close()
