"""ROS2 OmniGraph Action Graphs for the G1.

Everything sourced from USD prims - clock, TF, joint states, camera, cmd_vel -
is published by native OmniGraph nodes running in C++. The LiDAR point cloud is
the exception: it lives in a torch tensor produced by warp rather than a USD
prim, so no OmniGraph node can see it and it is published separately by
:mod:`g1_sim.lidar_publisher`.

The ``isaacsim.ros2.bridge`` extension must be enabled before this module is
used; :func:`enable_ros2_bridge` does that and also makes the bundled jazzy
rclpy importable.
"""

from __future__ import annotations

import omni.kit.app

# Topic names, collected here so the publishers, launch file and RViz config
# can all agree on one source of truth.
TOPIC_CLOCK = "/clock"
TOPIC_JOINT_STATES = "/g1/joint_states"
TOPIC_JOINT_COMMAND = "/g1/joint_command"
TOPIC_ODOM = "/g1/odom"
TOPIC_CMD_VEL = "/g1/cmd_vel"
TOPIC_CAMERA_RGB = "/g1/camera/rgb"
TOPIC_CAMERA_DEPTH = "/g1/camera/depth"
TOPIC_CAMERA_INFO = "/g1/camera/camera_info"
TOPIC_LIDAR_POINTS = "/livox/mid360/points"

GRAPH_PATH = "/ActionGraph/G1ROS2"


def enable_ros2_bridge() -> None:
    """Enable the ROS2 bridge extension.

    Must run before ``import rclpy``. The system rclpy is built for Python 3.12
    and cannot be imported into this 3.11 process; enabling the extension puts
    its own bundled jazzy rclpy on the path instead.
    """
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled("isaacsim.ros2.bridge"):
        manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        # The extension registers its OmniGraph node types asynchronously.
        for _ in range(20):
            omni.kit.app.get_app().update()


def build_g1_action_graph(
    robot_prim_path: str,
    camera_prim_path: str | None = None,
    graph_path: str = GRAPH_PATH,
    publish_odom: bool = True,
) -> str:
    """Create the G1's ROS2 Action Graph.

    Args:
        robot_prim_path: absolute path to the robot articulation, e.g.
            ``/World/envs/env_0/Robot``.
        camera_prim_path: absolute path to the camera prim. Camera publishing is
            skipped when this is ``None``.
        graph_path: where to create the graph prim.
        publish_odom: also publish base odometry.

    Returns:
        The graph prim path.
    """
    enable_ros2_bridge()

    import omni.graph.core as og

    nodes = [
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("SubscribeJointCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ]

    connections = [
        # Clock: every publisher stamps from the same simulation time so
        # downstream nodes running with use_sim_time stay in lockstep.
        ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("Context.outputs:context", "PublishClock.inputs:context"),
        ("SimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        # TF - gives RViz the robot's frame tree, including the sensor mounts.
        ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
        ("Context.outputs:context", "PublishTF.inputs:context"),
        ("SimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        # Joint states out.
        ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("Context.outputs:context", "PublishJointState.inputs:context"),
        ("SimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        # Joint commands in, straight into the articulation controller.
        ("OnTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
        ("Context.outputs:context", "SubscribeJointCommand.inputs:context"),
        ("SubscribeJointCommand.outputs:execOut", "ArticulationController.inputs:execIn"),
        ("SubscribeJointCommand.outputs:jointNames", "ArticulationController.inputs:jointNames"),
        (
            "SubscribeJointCommand.outputs:positionCommand",
            "ArticulationController.inputs:positionCommand",
        ),
        (
            "SubscribeJointCommand.outputs:velocityCommand",
            "ArticulationController.inputs:velocityCommand",
        ),
        ("SubscribeJointCommand.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        # Velocity commands in. The graph only receives them; turning a Twist
        # into joint targets is the locomotion policy's job, so cmd_vel_bridge
        # reads this node's output rather than wiring it to the controller.
        ("OnTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
        ("Context.outputs:context", "SubscribeTwist.inputs:context"),
    ]

    values = [
        ("PublishClock.inputs:topicName", TOPIC_CLOCK),
        ("PublishJointState.inputs:topicName", TOPIC_JOINT_STATES),
        ("PublishJointState.inputs:targetPrim", [robot_prim_path]),
        ("SubscribeJointCommand.inputs:topicName", TOPIC_JOINT_COMMAND),
        ("ArticulationController.inputs:targetPrim", [robot_prim_path]),
        ("SubscribeTwist.inputs:topicName", TOPIC_CMD_VEL),
        ("PublishTF.inputs:targetPrims", [robot_prim_path]),
        ("PublishTF.inputs:topicName", "/tf"),
    ]

    if publish_odom:
        nodes += [
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ]
        connections += [
            ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
            ("Context.outputs:context", "PublishOdom.inputs:context"),
            ("SimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
        ]
        values += [
            ("ComputeOdom.inputs:chassisPrim", [robot_prim_path]),
            ("PublishOdom.inputs:topicName", TOPIC_ODOM),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_link"),
        ]

    if camera_prim_path is not None:
        nodes += [
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CameraRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]
        connections += [
            ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "CameraRGB.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "CameraDepth.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraRGB.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraDepth.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
            ("Context.outputs:context", "CameraRGB.inputs:context"),
            ("Context.outputs:context", "CameraDepth.inputs:context"),
            ("Context.outputs:context", "CameraInfo.inputs:context"),
        ]
        values += [
            ("CreateRenderProduct.inputs:cameraPrim", [camera_prim_path]),
            ("CameraRGB.inputs:topicName", TOPIC_CAMERA_RGB),
            ("CameraRGB.inputs:type", "rgb"),
            ("CameraRGB.inputs:frameId", "d435_link"),
            ("CameraDepth.inputs:topicName", TOPIC_CAMERA_DEPTH),
            ("CameraDepth.inputs:type", "depth"),
            ("CameraDepth.inputs:frameId", "d435_link"),
            ("CameraInfo.inputs:topicName", TOPIC_CAMERA_INFO),
            ("CameraInfo.inputs:frameId", "d435_link"),
        ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.CONNECT: connections,
            og.Controller.Keys.SET_VALUES: values,
        },
    )

    return graph_path


def read_cmd_vel(graph_path: str = GRAPH_PATH) -> tuple[float, float, float]:
    """Read the latest Twist off the graph's subscriber.

    Returns ``(linear_x, linear_y, angular_z)`` - the three components a planar
    locomotion policy consumes. Returns zeros if nothing has been received.
    """
    import omni.graph.core as og

    try:
        node = og.Controller.node(f"{graph_path}/SubscribeTwist")
        linear = og.Controller.attribute("outputs:linearVelocity", node).get()
        angular = og.Controller.attribute("outputs:angularVelocity", node).get()
        return float(linear[0]), float(linear[1]), float(angular[2])
    except Exception:
        # Before the first message arrives the attributes may not resolve; a
        # zero command is the safe interpretation.
        return 0.0, 0.0, 0.0
