#!/usr/bin/env python3
"""Diagnostic: dump the USD prim tree and test warp mesh extraction directly.

Answers whether `mesh_prim_paths=["/World/ground"]` can actually find geometry,
by printing every prim under /World with its type and running the sensor's own
extraction helper against the candidates.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass


@configclass
class ProbeSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.CuboidCfg(
            size=(60.0, 60.0, 0.2),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.1)),
    )
    plane = AssetBaseCfg(
        prim_path="/World/plane",
        spawn=sim_utils.GroundPlaneCfg(size=(60.0, 60.0)),
    )
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DistantLightCfg(intensity=3000.0))


def main() -> None:
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device=args_cli.device))
    InteractiveScene(ProbeSceneCfg(num_envs=1, env_spacing=8.0))
    sim.reset()

    import omni.usd
    from pxr import Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()

    print("\n[PRIMS] full tree under /World:")
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World")):
        path = str(prim.GetPath())
        type_name = prim.GetTypeName()
        marker = "  <-- GEOMETRY" if type_name in ("Mesh", "Plane", "Cube", "Sphere", "Cylinder", "Capsule", "Cone") else ""
        print(f"  {path:<70} {type_name}{marker}")

    supported = ["Mesh", "Plane", "Sphere", "Cube", "Cylinder", "Capsule", "Cone"]
    for root in ("/World/ground", "/World/plane"):
        print(f"\n[MATCH] geometry discovered under {root}:")
        found = []
        for geom_type in supported:
            found.extend(
                sim_utils.get_all_matching_child_prims(root, lambda p, gt=geom_type: p.GetTypeName() == gt)
            )
        if not found:
            exact = sim_utils.find_first_matching_prim(root)
            print(f"  no child geometry; exact prim type = {exact.GetTypeName() if exact else None}")
        for prim in found:
            print(f"  {prim.GetPath()} ({prim.GetTypeName()})")
            if prim.GetTypeName() == "Mesh":
                mesh = UsdGeom.Mesh(prim)
                points = mesh.GetPointsAttr().Get()
                counts = mesh.GetFaceVertexCountsAttr().Get()
                print(f"    points={len(points) if points else 0} faces={len(counts) if counts else 0}")


if __name__ == "__main__":
    main()
    simulation_app.close()
