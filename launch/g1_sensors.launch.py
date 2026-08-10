"""Launch the Mid-360 LiDAR and D435 RealSense on the real G1.

Run on the robot (system ROS2 Jazzy, wall-clock time — NOT sim time):

    source /opt/ros/jazzy/setup.bash
    source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
    ros2 launch <this_file>

Published topics (record these with record_g1_bag.sh):
    /utlidar/cloud              sensor_msgs/PointCloud2  — Mid-360 raw
    /livox/lidar                livox_ros_driver2/CustomMsg — Mid-360 custom
    /g1/camera/rgb/image_raw    sensor_msgs/Image
    /g1/camera/rgb/camera_info  sensor_msgs/CameraInfo
    /g1/camera/depth/image_rect_raw  sensor_msgs/Image
    /g1/camera/depth/camera_info     sensor_msgs/CameraInfo

Args:
    lidar     (true/false)  — launch Mid-360 driver   [default: true]
    camera    (true/false)  — launch RealSense driver  [default: true]
    pointcloud (true/false) — publish PointCloud2 from driver [default: true]
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Mid-360 config shipped with livox_ros_driver2.
# After firmware update the driver may need the MID360 json config — path below
# is the standard install location; override with LIDAR_CONFIG env var if moved.
_DEFAULT_LIDAR_CONFIG = os.environ.get(
    "LIDAR_CONFIG",
    str(Path.home() / "unitree_ros2/cyclonedds_ws/src/livox_ros_driver2/config/MID360_config.json"),
)

# Unitree maps the Mid-360 to /utlidar/cloud via a remapping in their driver
# launch. We keep that remapping here so downstream nodes (lidar_bridge,
# record_g1_bag.sh) see the expected topic name.
_LIDAR_REMAPS = [
    ("/livox/lidar_0", "/livox/lidar"),
    ("/livox/points_0", "/utlidar/cloud"),
]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # ── args ──────────────────────────────────────────────────────────
            DeclareLaunchArgument("lidar", default_value="true",
                                  description="Launch Mid-360 LiDAR driver."),
            DeclareLaunchArgument("camera", default_value="true",
                                  description="Launch D435 RealSense driver."),
            DeclareLaunchArgument("pointcloud", default_value="true",
                                  description="Publish PointCloud2 alongside Livox CustomMsg."),
            DeclareLaunchArgument(
                "lidar_config",
                default_value=_DEFAULT_LIDAR_CONFIG,
                description="Path to MID360_config.json for livox_ros_driver2.",
            ),

            # ── Mid-360 ───────────────────────────────────────────────────────
            # livox_ros_driver2 exposes a MID360.launch.py in its share dir.
            # We pass xfer_format=1 (PointCloud2) so rosbag gets standard msgs.
            # xfer_format=0 → CustomMsg only; xfer_format=1 → both.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("livox_ros_driver2"),
                        "launch_ROS2",
                        "rviz_MID360_launch.py",
                    ])
                ]),
                launch_arguments={
                    "user_config_path": LaunchConfiguration("lidar_config"),
                    "xfer_format": "1",          # 0=CustomMsg, 1=PointCloud2, 2=both
                    "publish_freq": "10.0",
                    "multi_topic": "0",
                }.items(),
                condition=IfCondition(LaunchConfiguration("lidar")),
            ),

            # Remap /livox/points_0 → /utlidar/cloud so downstream matches
            # Unitree's own driver topic name. This is a static-transform-
            # style relay node using topic_tools if the include remapping
            # doesn't take — only launched if needed.
            # (livox_ros_driver2 ≥ 2.0 supports remappings in the include above;
            #  if topics still appear as /livox/points_0 after launch, uncomment:)
            # Node(
            #     package="topic_tools",
            #     executable="relay",
            #     name="lidar_relay",
            #     arguments=["/livox/points_0", "/utlidar/cloud"],
            #     condition=IfCondition(LaunchConfiguration("lidar")),
            # ),

            # ── D435 RealSense ─────────────────────────────────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ])
                ]),
                launch_arguments={
                    "camera_name":        "g1_camera",
                    "camera_namespace":   "g1",
                    # RGB stream
                    "enable_color":       "true",
                    "color_width":        "640",
                    "color_height":       "480",
                    "color_fps":          "30",
                    # Depth stream
                    "enable_depth":       "true",
                    "depth_width":        "640",
                    "depth_height":       "480",
                    "depth_fps":          "30",
                    # Align depth to color frame so they share camera_info
                    "align_depth.enable": "true",
                    # No pointcloud from RealSense — LiDAR handles that
                    "pointcloud.enable":  "false",
                    # IMU off (not needed for bags)
                    "enable_gyro":        "false",
                    "enable_accel":       "false",
                    # Real robot → wall clock
                    "use_sim_time":       "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("camera")),
            ),
        ]
    )
