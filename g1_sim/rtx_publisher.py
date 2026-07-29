"""Publish RTX LiDAR returns to ROS2 by reading the sensor annotator directly.

``ROS2RtxLidarHelper`` advertises its topic but never emits on this setup, so
this reads the same data the helper would - via the
``IsaacCreateRTXLidarScanBuffer`` annotator, which is verified to return
~175,000 points per prim per frame - and publishes it with rclpy instead.

The several co-located sensor prims that make up one Mid-360 are merged into a
single cloud, so subscribers see one sensor.

In the ``isaac`` env (Python 3.12) the system ROS2 jazzy rclpy imports
directly, so no bundled-rclpy workaround is needed.
"""

from __future__ import annotations

import numpy as np


class RtxLidarPublisher:
    """Merges several RTX LiDAR prims into one ``sensor_msgs/PointCloud2``.

    Args:
        prim_paths: sensor prims from :func:`g1_sim.rtx_lidar.spawn_mid360`.
        topic: topic to publish on.
        frame_id: TF frame the points are expressed in.
        publish_rate: target rate in Hz; should match the sensor's scan rate.
        max_points: cap on points per message. The full Mid-360 returns ~700k
            per frame across all prims, which is far more than the real
            sensor's 20k and enough to stall RViz, so the cloud is subsampled.
    """

    def __init__(
        self,
        prim_paths: list[str],
        topic: str = "/livox/mid360/points",
        frame_id: str = "mid360_link",
        publish_rate: float = 10.0,
        max_points: int = 20000,
    ):
        import omni.replicator.core as rep
        from rclpy.node import Node
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField

        self._PointCloud2 = PointCloud2
        self.frame_id = frame_id
        self.max_points = max_points
        self.publish_period = 1.0 / publish_rate
        self._last_publish = -float("inf")

        # One annotator per prim. Isaac Sim 6.0 dropped the RtxSensorCpu prefix
        # this annotator carried in earlier releases.
        self.annotators = []
        for i, prim_path in enumerate(prim_paths):
            rp = rep.create.render_product(prim_path, [1, 1], name=f"mid360_pub_{i}")
            annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
            annot.attach([rp])
            self.annotators.append(annot)

        self.node = Node("g1_rtx_lidar_publisher")
        # Best-effort matches how sensor streams are normally consumed: a
        # subscriber that falls behind should drop scans, not queue them.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.node.create_publisher(PointCloud2, topic, qos)

        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.node.get_logger().info(
            f"publishing {topic} in frame {frame_id} from {len(prim_paths)} prims"
        )

    def publish(self, sim_time: float) -> int:
        """Publish one merged scan if due. Returns the number of points sent."""
        if sim_time - self._last_publish < self.publish_period:
            return 0
        self._last_publish = sim_time

        points, intensities = self._gather()
        if points is None:
            return 0

        self.publisher.publish(self._to_message(points, intensities, sim_time))
        return len(points)

    def _gather(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Collect and merge the returns from every prim."""
        chunks = []
        intensity_chunks = []

        for annot in self.annotators:
            data = annot.get_data()
            if not isinstance(data, dict):
                continue
            xyz = data.get("data")
            if xyz is None or len(xyz) == 0:
                continue

            xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
            chunks.append(xyz)

            # The annotator may or may not supply intensity depending on the
            # profile; fall back to a constant rather than inventing values.
            intensity = data.get("intensity")
            if intensity is not None and len(intensity) == len(xyz):
                intensity_chunks.append(np.asarray(intensity, dtype=np.float32))
            else:
                intensity_chunks.append(np.full(len(xyz), 100.0, dtype=np.float32))

        if not chunks:
            return None, None

        points = np.vstack(chunks)
        intensities = np.concatenate(intensity_chunks)

        # Drop non-finite returns and the exact (0,0,0) origin points that
        # stand for rays which hit nothing. Testing the norm rather than each
        # component keeps legitimate returns that happen to lie on an axis.
        finite = np.isfinite(points).all(axis=1)
        nonzero = np.linalg.norm(points, axis=1) > 1e-6
        keep = finite & nonzero
        points, intensities = points[keep], intensities[keep]
        if len(points) == 0:
            return None, None

        if len(points) > self.max_points:
            # Stride rather than slice: a contiguous slice would take one part
            # of the sweep, whereas striding preserves the pattern's shape.
            idx = np.linspace(0, len(points) - 1, self.max_points).astype(np.int64)
            points, intensities = points[idx], intensities[idx]

        return points, intensities

    def _to_message(self, points: np.ndarray, intensities: np.ndarray, sim_time: float):
        msg = self._PointCloud2()
        msg.header.frame_id = self.frame_id
        msg.header.stamp.sec = int(sim_time)
        msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)

        msg.height = 1  # unorganised cloud
        msg.width = len(points)
        msg.fields = self._fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.hstack([points, intensities[:, None]]).astype(np.float32).tobytes()
        return msg

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """Service pending rclpy callbacks without blocking the sim loop."""
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def destroy(self) -> None:
        self.node.destroy_node()
