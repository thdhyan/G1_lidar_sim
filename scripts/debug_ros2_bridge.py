#!/usr/bin/env python3
"""Isolate the ROS2 bridge from the rest of the scene.

Enables isaacsim.ros2.bridge, imports the bundled rclpy, and publishes one
PointCloud2 - with no robot, no LiDAR and no action graph. If this passes, a
failure in g1_ros2_sim.py is in the scene, not in the ROS2 layer.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np

print("[BRIDGE] enabling isaacsim.ros2.bridge ...", flush=True)
import omni.kit.app

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
for _ in range(20):
    omni.kit.app.get_app().update()
print(f"[BRIDGE] enabled: {manager.is_extension_enabled('isaacsim.ros2.bridge')}", flush=True)

print("[BRIDGE] importing rclpy ...", flush=True)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

print(f"[BRIDGE] rclpy from {rclpy.__file__}", flush=True)

rclpy.init()
node = Node("bridge_smoke_test")
pub = node.create_publisher(PointCloud2, "/bridge_test/points", 1)
print("[BRIDGE] publisher created", flush=True)

msg = PointCloud2()
msg.header.frame_id = "test_frame"
msg.height = 1
msg.width = 3
msg.fields = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
msg.point_step = 12
msg.row_step = 36
msg.is_dense = True
msg.data = np.zeros((3, 3), dtype=np.float32).tobytes()

for i in range(5):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.0)
print("[BRIDGE] published 5 messages", flush=True)

# Verify the OmniGraph ROS2 node types the action graph depends on exist.
import omni.graph.core as og

registry = og.get_registered_nodes()
registered = {n if isinstance(n, str) else n.get_node_type_name() for n in registry}
required = [
    "isaacsim.ros2.bridge.ROS2Context",
    "isaacsim.ros2.bridge.ROS2PublishClock",
    "isaacsim.ros2.bridge.ROS2PublishTransformTree",
    "isaacsim.ros2.bridge.ROS2PublishJointState",
    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
    "isaacsim.ros2.bridge.ROS2SubscribeTwist",
    "isaacsim.ros2.bridge.ROS2PublishOdometry",
    "isaacsim.ros2.bridge.ROS2CameraHelper",
    "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
    "isaacsim.core.nodes.IsaacReadSimulationTime",
    "isaacsim.core.nodes.IsaacArticulationController",
    "isaacsim.core.nodes.IsaacComputeOdometry",
    "isaacsim.core.nodes.IsaacCreateRenderProduct",
    "omni.graph.action.OnPlaybackTick",
]
missing = [name for name in required if name not in registered]
for name in required:
    print(f"[BRIDGE] {'OK  ' if name in registered else 'MISS'} {name}", flush=True)

node.destroy_node()
rclpy.shutdown()

print(f"\n[BRIDGE] {'FAIL: missing node types' if missing else 'PASS'}", flush=True)
simulation_app.close()
