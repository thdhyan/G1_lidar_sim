"""Bring up the external ROS2 nodes that sit alongside the simulator.

The simulator is NOT launched here - it runs in the Python 3.11 conda env and
this launch file runs under system Python 3.12 with /opt/ros/jazzy. Start the
sim first, then:

    ros2 launch launch/g1_bringup.launch.py

Every node runs with use_sim_time so it follows the sim's /clock.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = Path(__file__).resolve().parent.parent
URDF = REPO.parent / "OmniPerception/LidarSensor/LidarSensor/resources/robots/g1_29/g1_29dof.urdf"
RVIZ_CONFIG = REPO / "rviz/g1.rviz"

# Sim time is non-negotiable: the sim publishes /clock and every timestamp here
# comes from it, so a node on wall-clock time would mis-order every transform.
USE_SIM_TIME = {"use_sim_time": True}


def generate_launch_description() -> LaunchDescription:
    backend = LaunchConfiguration("backend")
    checkpoint = LaunchConfiguration("checkpoint")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="livox",
                description="Detection backend: 'livox' (CenterPoint) or 'clustering'.",
            ),
            DeclareLaunchArgument(
                "checkpoint",
                default_value=str(REPO / "detection/pt/livox_model_1.pt"),
                description="Checkpoint for the livox backend.",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Launch RViz."
            ),
            DeclareLaunchArgument(
                "detection", default_value="true", description="Launch the detector."
            ),
            # Feeds /robot_description so RViz can draw the robot mesh. The sim
            # publishes joint states; this turns them into the model's pose.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    USE_SIM_TIME,
                    {"robot_description": URDF.read_text() if URDF.exists() else ""},
                ],
                remappings=[("/joint_states", "/g1/joint_states")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(RVIZ_CONFIG)],
                parameters=[USE_SIM_TIME],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # Not a ROS2 package - run the script directly on its own path.
            Node(
                package="python3",
                executable=str(REPO / "detection/detection_node.py"),
                name="g1_detection",
                output="screen",
                arguments=["--backend", backend, "--checkpoint", checkpoint],
                parameters=[USE_SIM_TIME],
                condition=IfCondition(LaunchConfiguration("detection")),
            ),
        ]
    )
