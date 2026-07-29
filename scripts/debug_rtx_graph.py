#!/usr/bin/env python3
"""Inspect why the RTX LiDAR ROS2 helpers are not publishing.

Builds the same graph as ``g1_rtx_sim.py`` but then reports what the sensor
annotator actually returns, which separates "the sensor produces no points"
from "the points never reach ROS2".
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

simulation_app = SimulationApp(
    {"headless": True, "/app/sensors/nv/lidar/outputBufferOnGPU": False}
)

"""Rest everything follows."""

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
manager.set_extension_enabled_immediate("isaacsim.sensors.rtx", True)
for _ in range(20):
    omni.kit.app.get_app().update()

import omni.replicator.core as rep

from g1_sim.rtx_lidar import MID360_POS, MID360_QUAT_WXYZ, spawn_mid360

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

# A wall well outside the Mid-360's blind cone, so returns are expected.
wall = UsdGeom.Cube.Define(stage, "/World/wall")
wall.CreateSizeAttr(1.0)
wall.AddTranslateOp().Set(Gf.Vec3d(15.0, 0.0, 2.0))
wall.AddScaleOp().Set(Gf.Vec3f(0.5, 30.0, 6.0))
UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

# Bare Xform mount: this isolates the sensor from the robot USD entirely.
mount = stage.DefinePrim("/World/SensorMount", "Xform")
UsdGeom.Xformable(mount).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.5))

from isaacsim.core.api import SimulationContext

sim = SimulationContext(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)

prim_paths = spawn_mid360(
    "/World/SensorMount",
    config_dir=REPO / "assets/lidar_configs_light",
    translation=(0.0, 0.0, 0.0),
    orientation=MID360_QUAT_WXYZ,
)
print(f"[DBG] sensor prims: {len(prim_paths)}", flush=True)

# Read the sensor directly through its annotator, bypassing ROS2 entirely.
render_products = []
annotators = []
for i, path in enumerate(prim_paths):
    rp = rep.create.render_product(path, [1, 1], name=f"dbg_{i}")
    render_products.append(rp)

    # Isaac Sim 6.0 dropped the RtxSensorCpu prefix from this annotator name.
    annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
    annot.attach([rp])
    annotators.append(annot)

print(f"[DBG] annotators  : {len(annotators)}", flush=True)

from g1_sim.rtx_lidar import attach_ros2_publishers

graph = attach_ros2_publishers(prim_paths, sim_rate_hz=60.0)
print(f"[DBG] ros2 graph  : {graph}", flush=True)

sim.reset()
timeline = omni.timeline.get_timeline_interface()
timeline.play()
print(f"[DBG] playing     : {timeline.is_playing()}", flush=True)

for step in range(240):
    sim.step(render=True)

    if step % 40 == 0 and step > 0:
        counts = []
        for annot in annotators:
            data = annot.get_data()
            points = data.get("data") if isinstance(data, dict) else None
            counts.append(0 if points is None else len(points))
        print(f"[DBG] step {step:>4}  points per prim: {counts}", flush=True)

        # Report the graph's own view of itself: whether it evaluated at all,
        # and what each helper node thinks its render product is. An empty
        # renderProductPath is the usual reason a helper stays silent.
        if step == 40:
            import omni.graph.core as og

            g = og.get_graph_by_path(graph)
            print(f"[DBG] graph found : {g is not None}", flush=True)
            if g is not None:
                for node in g.get_nodes():
                    name = node.get_prim_path().rsplit("/", 1)[-1]
                    if not name.startswith("Publish_"):
                        continue
                    try:
                        rp = og.Controller.attribute("inputs:renderProductPath", node).get()
                        tn = og.Controller.attribute("inputs:topicName", node).get()
                        print(f"[DBG]   {name}: topic={tn!r} rp={rp!r}", flush=True)
                    except Exception as exc:
                        print(f"[DBG]   {name}: <{exc}>", flush=True)

total = 0
for annot in annotators:
    data = annot.get_data()
    if isinstance(data, dict):
        pts = data.get("data")
        if pts is not None:
            total += len(pts)
        if total == 0:
            print(f"[DBG] annotator keys: {list(data.keys())}", flush=True)

print(f"\n[DBG] {'PASS - sensor returns points' if total else 'FAIL - sensor returned nothing'}")
simulation_app.close()
