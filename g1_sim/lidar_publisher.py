"""Publish the warp LiDAR point cloud to ROS2 from inside the simulator.

The Mid-360's returns live in a torch tensor produced by warp, not in a USD
prim, so the OmniGraph ROS2 nodes cannot reach them. This module runs a small
rclpy publisher in-process instead, using the jazzy rclpy bundled with
``isaacsim.ros2.bridge``.

:func:`g1_sim.action_graph.enable_ros2_bridge` must have run first - importing
rclpy before that fails, because the system rclpy is built for Python 3.12.
"""

from __future__ import annotations

import numpy as np
import torch


class LidarPointCloudPublisher:
    """Publishes ``sensor_msgs/PointCloud2`` from a :class:`LidarSensor`.

    Args:
        sensor: the IsaacLab LiDAR sensor to read.
        topic: topic to publish on.
        frame_id: frame the points are expressed in. Must match the TF frame of
            the sensor mount, or RViz will place the cloud incorrectly.
        env_id: which environment to publish; only one makes sense on a topic.
        publish_rate: target rate in Hz. Should match the sensor's own update
            frequency - publishing faster only repeats identical scans.
        max_range: returns beyond this are dropped as misses.
    """

    def __init__(
        self,
        sensor,
        topic: str = "/livox/mid360/points",
        frame_id: str = "mid360_link",
        env_id: int = 0,
        publish_rate: float = 10.0,
        max_range: float = 40.0,
    ):
        from rclpy.node import Node
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField

        self._PointCloud2 = PointCloud2
        self._PointField = PointField

        self.sensor = sensor
        self.frame_id = frame_id
        self.env_id = env_id
        self.max_range = max_range
        self.publish_period = 1.0 / publish_rate
        self._last_publish = -float("inf")

        self.node = Node("g1_lidar_publisher")
        # Best-effort matches how sensor streams are normally consumed: a
        # subscriber that falls behind should drop scans, not accumulate them.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.node.create_publisher(PointCloud2, topic, qos)

        # x, y, z, intensity as float32 - the layout livox_detection expects.
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.node.get_logger().info(f"publishing {topic} in frame {frame_id}")

    def publish(self, sim_time: float) -> bool:
        """Publish one scan if enough simulated time has passed.

        Returns True when a message was sent.
        """
        if sim_time - self._last_publish < self.publish_period:
            return False
        self._last_publish = sim_time

        points = self._gather_points()
        self.publisher.publish(self._to_message(points, sim_time))
        return True

    def _gather_points(self) -> np.ndarray:
        """Extract finite, in-range returns as an ``(N, 4)`` float32 array."""
        data = self.sensor.data
        cloud = data.pointcloud
        if cloud is None:
            # pointcloud is optional; fall back to world-frame hit points.
            cloud = data.ray_hits_w

        points = cloud[self.env_id]

        # Rays that hit nothing carry infinite coordinates by design.
        valid = torch.isfinite(points).all(dim=1)
        if data.distances is not None:
            distances = data.distances[self.env_id]
            valid &= distances < self.max_range
        points = points[valid]

        if points.numel() == 0:
            return np.zeros((0, 4), dtype=np.float32)

        xyz = points.cpu().numpy().astype(np.float32)
        # The simulated sensor returns no reflectance, so intensity is a
        # constant placeholder rather than a fabricated value.
        intensity = np.full((xyz.shape[0], 1), 100.0, dtype=np.float32)
        return np.hstack([xyz, intensity])

    def _to_message(self, points: np.ndarray, sim_time: float):
        msg = self._PointCloud2()
        msg.header.frame_id = self.frame_id
        msg.header.stamp.sec = int(sim_time)
        msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)

        msg.height = 1  # unorganised cloud
        msg.width = points.shape[0]
        msg.fields = self._fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 float32 values
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True  # non-finite returns were already removed
        msg.data = points.tobytes()
        return msg

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """Service pending rclpy callbacks without blocking the sim loop."""
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def destroy(self) -> None:
        self.node.destroy_node()
