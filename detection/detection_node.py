#!/usr/bin/env python3
"""ROS2 node: LiDAR point cloud -> 3D detections + RViz markers.

Runs as its own process against system Python and ``/opt/ros/jazzy`` - it is
deliberately not part of the simulator process, so heavy inference never stalls
the physics loop and the detector can use its own dependency set.

    ros2 run ... detection_node.py --checkpoint pt/livox_model_1.pt
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from backend import CLASS_COLORS, CLASS_NAMES, ClusteringBackend, DetectionBackend


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Yaw-only rotation as ``(x, y, z, w)``; boxes are never rolled or pitched."""
    return (0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0)))


class DetectionNode(Node):
    def __init__(self, backend: DetectionBackend, input_topic: str, frame_override: str | None):
        super().__init__("g1_lidar_detection")
        self.backend = backend
        self.frame_override = frame_override

        # Sensor data is best-effort: dropping a stale cloud beats queueing it.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.detection_pub = self.create_publisher(Detection3DArray, "/g1/detections", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/g1/detection_markers", 10)
        self.create_subscription(PointCloud2, input_topic, self.on_cloud, sensor_qos)

        self._busy = False
        self.get_logger().info(f"backend={backend.name} listening on {input_topic}")

    def on_cloud(self, msg: PointCloud2) -> None:
        # Inference is slower than the sensor rate; skip rather than fall behind.
        if self._busy:
            return
        self._busy = True
        try:
            points = self.cloud_to_array(msg)
            boxes, scores, labels = self.backend.infer(points)
            frame = self.frame_override or msg.header.frame_id
            self.detection_pub.publish(self.to_detection_array(boxes, scores, labels, msg.header.stamp, frame))
            self.marker_pub.publish(self.to_markers(boxes, labels, msg.header.stamp, frame))
        except Exception as exc:  # keep the node alive across a bad frame
            self.get_logger().error(f"detection failed: {exc}")
        finally:
            self._busy = False

    @staticmethod
    def cloud_to_array(msg: PointCloud2) -> np.ndarray:
        available = {f.name for f in msg.fields}
        fields = ["x", "y", "z"] + (["intensity"] if "intensity" in available else [])
        raw = point_cloud2.read_points(msg, field_names=fields, skip_nans=True)
        if raw.shape[0] == 0:
            return np.zeros((0, 4), dtype=np.float32)

        points = np.stack([raw[name] for name in fields], axis=-1).astype(np.float32)
        if points.shape[1] == 3:  # pad a zero intensity column when absent
            points = np.hstack([points, np.zeros((points.shape[0], 1), dtype=np.float32)])
        return points

    def to_detection_array(self, boxes, scores, labels, stamp, frame: str) -> Detection3DArray:
        array = Detection3DArray()
        array.header.stamp = stamp
        array.header.frame_id = frame

        for box, score, label in zip(boxes, scores, labels):
            detection = Detection3D()
            detection.header = array.header

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = CLASS_NAMES[int(label)]
            hypothesis.hypothesis.score = float(score)
            detection.results.append(hypothesis)

            bbox = BoundingBox3D()
            bbox.center.position.x = float(box[0])
            bbox.center.position.y = float(box[1])
            bbox.center.position.z = float(box[2])
            qx, qy, qz, qw = yaw_to_quaternion(float(box[6]))
            bbox.center.orientation.x = qx
            bbox.center.orientation.y = qy
            bbox.center.orientation.z = qz
            bbox.center.orientation.w = qw
            bbox.size.x = float(box[3])
            bbox.size.y = float(box[4])
            bbox.size.z = float(box[5])
            detection.bbox = bbox

            array.detections.append(detection)
        return array

    def to_markers(self, boxes, labels, stamp, frame: str) -> MarkerArray:
        markers = MarkerArray()

        # Clear previous frame's boxes first, otherwise stale markers linger in
        # RViz whenever the detection count drops between frames.
        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (box, label) in enumerate(zip(boxes, labels)):
            name = CLASS_NAMES[int(label)]
            red, green, blue = CLASS_COLORS[name]

            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = stamp
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            qx, qy, qz, qw = yaw_to_quaternion(float(box[6]))
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = float(box[3])
            marker.scale.y = float(box[4])
            marker.scale.z = float(box[5])
            marker.color.r, marker.color.g, marker.color.b = red, green, blue
            marker.color.a = 0.4  # translucent so the point cloud stays visible
            markers.markers.append(marker)

        return markers


def build_backend(args: argparse.Namespace) -> DetectionBackend:
    if args.backend == "clustering":
        return ClusteringBackend()

    # Imported lazily so the clustering fallback needs no torch installed.
    from livox_centerpoint import LivoxCenterPointBackend

    return LivoxCenterPointBackend(
        checkpoint=args.checkpoint,
        device=args.device,
        score_threshold=args.score_threshold,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 LiDAR 3D detection node.")
    parser.add_argument("--backend", choices=("livox", "clustering"), default="livox")
    parser.add_argument("--checkpoint", default="pt/livox_model_1.pt")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--score-threshold", type=float, default=0.4)
    parser.add_argument("--input-topic", default="/livox/mid360/points")
    parser.add_argument(
        "--frame",
        default=None,
        help="Override the cloud's frame_id when publishing detections.",
    )
    args = parser.parse_args()

    backend = build_backend(args)
    try:
        backend.load()
    except Exception as exc:
        # A wrong checkpoint should be loud and fatal, not silently degraded.
        print(f"[detection] failed to load {args.backend} backend: {exc}", file=sys.stderr)
        raise SystemExit(1)

    rclpy.init()
    node = DetectionNode(backend, args.input_topic, args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
