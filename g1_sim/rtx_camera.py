"""Camera and robot-state ROS2 publishing for the RTX G1 scene.

Publishes what RViz needs to draw the robot and its sensors:

===============================  =============================  ==============
Topic                            Type                           Source
===============================  =============================  ==============
``/g1/camera/rgb``               ``sensor_msgs/Image``          D435 camera
``/g1/camera/depth``             ``sensor_msgs/Image``          D435 camera
``/g1/camera/semantic``          ``sensor_msgs/Image``          D435 camera
``/g1/camera/camera_info``       ``sensor_msgs/CameraInfo``     D435 camera
``/tf``                          ``tf2_msgs/TFMessage``         articulation
``/g1/joint_states``             ``sensor_msgs/JointState``     articulation
``/clock``                       ``rosgraph_msgs/Clock``        simulation
===============================  =============================  ==============

Semantic segmentation only labels prims carrying semantics; see
:func:`apply_semantics`.
"""

from __future__ import annotations

# Mirrors g1_29dof.urdf: d435_link is a fixed child of torso_link, pitched
# ~47.6 deg down so the camera covers the near ground the LiDAR cannot see.
D435_POS = (0.0576235, 0.01753, 0.41987)
D435_PITCH_RAD = 0.8307767239493009

TOPIC_RGB = "/g1/camera/rgb"
TOPIC_DEPTH = "/g1/camera/depth"
TOPIC_SEMANTIC = "/g1/camera/semantic"
TOPIC_CAMERA_INFO = "/g1/camera/camera_info"
TOPIC_JOINT_STATES = "/g1/joint_states"
TOPIC_CLOCK = "/clock"

CAMERA_FRAME = "d435_link"


def spawn_camera(
    parent_prim_path: str,
    name: str = "d435_camera",
    width: int = 640,
    height: int = 480,
) -> str:
    """Create a D435-style camera prim under ``parent_prim_path``.

    Resolution defaults below the real D435's 1280x720: rendering three
    annotators at full resolution dominates the frame budget on an 8 GB GPU.

    Returns the camera prim path.
    """
    import math

    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    path = f"{parent_prim_path}/{name}"

    camera = UsdGeom.Camera.Define(stage, path)
    # ~69 deg horizontal FOV, matching the D435.
    camera.CreateFocalLengthAttr(1.93)
    camera.CreateHorizontalApertureAttr(2.682)
    camera.CreateVerticalApertureAttr(1.509)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 30.0))

    xform = UsdGeom.Xformable(camera)
    xform.AddTranslateOp().Set(Gf.Vec3d(*D435_POS))
    # USD cameras look down -Z while the URDF frame looks down +X, so the
    # mount pitch is applied on top of a -90 deg Y rotation.
    xform.AddRotateXYZOp().Set(
        Gf.Vec3f(0.0, -90.0 + math.degrees(D435_PITCH_RAD), 0.0)
    )

    return path


def apply_semantics(prim_paths: dict[str, str]) -> int:
    """Label prims so semantic segmentation has classes to report.

    Args:
        prim_paths: ``{prim_path: class_name}``. Unlabelled geometry renders as
            background, so the pedestrian targets in particular need labelling
            for the segmentation image to be useful.

    Returns:
        How many prims were labelled.
    """
    import omni.usd
    from pxr import Semantics

    stage = omni.usd.get_context().get_stage()
    labelled = 0

    for prim_path, class_name in prim_paths.items():
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            continue
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr("class")
        sem.CreateSemanticDataAttr(class_name)
        labelled += 1

    return labelled


def attach_camera_publishers(
    camera_prim_path: str,
    graph_path: str = "/ActionGraph/CameraROS2",
    width: int = 640,
    height: int = 480,
    frame_id: str = CAMERA_FRAME,
) -> str:
    """Publish RGB, depth, semantic segmentation and camera_info.

    Each data type needs its own ROS2CameraHelper - a helper handles exactly
    one type and cannot be switched after activation - but all of them share a
    single render product.
    """
    import omni.graph.core as og

    nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("RunOneFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ]
    connections = [
        ("OnTick.outputs:tick", "RunOneFrame.inputs:execIn"),
        ("RunOneFrame.outputs:step", "RenderProduct.inputs:execIn"),
        ("RenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
        ("RenderProduct.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
        ("Context.outputs:context", "CameraInfo.inputs:context"),
    ]
    values = [
        ("RenderProduct.inputs:cameraPrim", [camera_prim_path]),
        ("RenderProduct.inputs:width", width),
        ("RenderProduct.inputs:height", height),
        ("CameraInfo.inputs:topicName", TOPIC_CAMERA_INFO),
        ("CameraInfo.inputs:frameId", frame_id),
    ]

    for label, data_type, topic in (
        ("RGB", "rgb", TOPIC_RGB),
        ("Depth", "depth", TOPIC_DEPTH),
        ("Semantic", "semantic_segmentation", TOPIC_SEMANTIC),
    ):
        node = f"Camera{label}"
        nodes.append((node, "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connections += [
            ("RenderProduct.outputs:execOut", f"{node}.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", f"{node}.inputs:renderProductPath"),
            ("Context.outputs:context", f"{node}.inputs:context"),
        ]
        values += [
            (f"{node}.inputs:type", data_type),
            (f"{node}.inputs:topicName", topic),
            (f"{node}.inputs:frameId", frame_id),
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


def attach_robot_state_publishers(
    robot_prim_path: str,
    graph_path: str = "/ActionGraph/RobotStateROS2",
    root_frame: str = "pelvis",
) -> str:
    """Publish ``/clock``, ``/tf`` and joint states.

    RViz needs all three to animate the robot: the description supplies the
    meshes, TF places the links, and joint states drive the articulation.
    Every timestamp comes from simulation time, so external nodes must run
    with ``use_sim_time:=true``.
    """
    import omni.graph.core as og

    nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]
    connections = [
        ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("Context.outputs:context", "PublishClock.inputs:context"),
        ("SimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
        ("Context.outputs:context", "PublishTF.inputs:context"),
        ("SimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("Context.outputs:context", "PublishJointState.inputs:context"),
        ("SimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
    ]
    # Both nodes need the prim carrying ArticulationRootAPI, which for this USD
    # is the pelvis - not the reference root. Pointing them at the reference
    # root instead yields "did not match any articulations" and silence.
    articulation_prim = f"{robot_prim_path}/{root_frame}"

    values = [
        ("PublishClock.inputs:topicName", TOPIC_CLOCK),
        ("PublishTF.inputs:topicName", "/tf"),
        # Root the tree at /World so the pelvis is a floating base whose pose
        # moves relative to it - that is what makes the robot visibly fall in
        # RViz. Rooting at the pelvis instead pins it to the origin and only
        # the joints appear to move.
        ("PublishTF.inputs:parentPrim", ["/World"]),
        # Target the articulation itself so every link - and so every joint
        # frame - is emitted, rather than just the reference root's transform.
        ("PublishTF.inputs:targetPrims", [articulation_prim]),
        ("PublishJointState.inputs:topicName", TOPIC_JOINT_STATES),
        ("PublishJointState.inputs:targetPrim", [articulation_prim]),
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
