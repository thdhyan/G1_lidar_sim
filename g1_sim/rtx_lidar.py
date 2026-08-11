"""RTX LiDAR Livox Mid-360 for the G1, using emitter state arrays.

The warp ray-caster only tests rays against a fixed list of static meshes, so
it cannot see the robot or anything spawned later. RTX LiDAR ray-traces the
real rendered scene instead, which means self-occlusion and dynamic objects
come for free.

The Mid-360's sweep is non-repetitive, so it is expressed as a sequence of
``emitterStates`` - one per frame - generated from the real scan pattern by
``scripts/gen_mid360_rtx_config.py``. Because the Hydra API caps one prim at
~5 MB of emitter data, the pattern is split across several prims sharing a
transform; their union is the full pattern.

Requires **Isaac Sim 6.0+**. On 5.1 every RTX sensor renders at the simulation
frame rate regardless of ``tickRate``, so a 10 Hz sensor in a 60 Hz sim fires
6x too often - which both corrupts the point rate and triggers CUDA buffer
races. 6.0 enables multi-tick rendering by default and honours ``tickRate``.

Cost. Every prim is a separate ray-tracing pass each render, so the load
scales with prim count and emitters per state, and RTX LiDAR is expensive on
a small GPU. Three presets trade sweep completeness against startup and frame
time:

===========  ======  ==========  =======  =====================
Preset       Prims   States      Size     Cycle
===========  ======  ==========  =======  =====================
``fast``     4       2 each      5.9 MB   0.2 s  (default)
``light``    4       5 each      15 MB    0.5 s
(full)       8       5 each      30 MB    0.5 s, denser sweep
===========  ======  ==========  =======  =====================

If the GUI stutters, pass ``--num-prims 2`` or add ``--no-camera``; the camera
renders three annotators per frame and is usually the larger cost of the two.
"""

from __future__ import annotations

import shutil
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CONFIG_DIR = REPO / "assets/lidar_configs"

# Matches the real device and the generated configs.
SCAN_RATE_HZ = 10.0

# Mount on torso_link per the URDF actually used by
# convert_g1_urdf_to_usd.py (OmniPerception/LidarSensor/.../g1_29dof.urdf):
# xyz=(0.0002835, 0.00003, 0.4188), rpy=(3.14, 0, 0) — a 180 deg roll, no
# pitch. NVIDIA's GR00T-WholeBodyControl repo ships a *different* g1_29dof.urdf
# with a different mid360_joint (xyz z=0.40618, rpy=(0, 0.0401, 0), no roll) -
# this file previously copied that one by mistake. Since the USD's torso_link
# frame comes from the OmniPerception URDF, not GR00T's, the mismatch pointed
# the sensor's local Z into the ceiling instead of the ground. Getting this
# wrong inverts the cloud - verify by checking the live cloud in RViz covers
# the ground/room, not open sky, before touching this again.
# Briefly lowered 0.2 m (to 0.2188) 2026-08-10 to test whether the real
# mount's ~9.7 m blind cone explained a narrow-arc-not-ring cloud. Reverted
# to the real URDF mount height per user request - didn't resolve the arc
# issue and the sensor's still-unexplained non-publishing that session took
# priority. If revisiting the blind-cone theory, the math was: blind_radius
# = 8x mount height, so lowering height shrinks the cone and lets closer
# geometry register.
MID360_POS = (0.0002835, 0.00003, 0.4188)
# rpy=(3.14, 0, 0) -> wxyz quaternion for a 180 deg roll about X-axis.
_MID360_ROLL = __import__("math").pi
MID360_QUAT_WXYZ = (
    __import__("math").cos(_MID360_ROLL / 2),  # w
    __import__("math").sin(_MID360_ROLL / 2),  # x (roll)
    0.0,                                        # y (pitch=0)
    0.0,                                        # z (yaw=0)
)


def install_configs(config_dir: Path | str = CONFIG_DIR) -> list[str]:
    """Copy the generated profiles where Isaac Sim's sensor loader looks.

    ``IsaacSensorCreateRtxLidar`` resolves ``config=`` against the RTX sensor
    extension's own ``data/lidar`` directory, not against an arbitrary path, so
    the JSON files have to be placed there.

    Returns the config names (file stems) that were installed.
    """
    import isaacsim

    config_dir = Path(config_dir)
    if not config_dir.is_dir():
        raise FileNotFoundError(
            f"{config_dir} not found - run scripts/gen_mid360_rtx_config.py first"
        )

    profiles = sorted(config_dir.glob("Livox_Mid360_*.json"))
    if not profiles:
        raise FileNotFoundError(f"no Livox_Mid360_*.json in {config_dir}")

    pkg_root = Path(isaacsim.__file__).parent
    targets = sorted(pkg_root.glob("extscache/omni.sensors.nv.common-*/data/lidar"))
    if not targets:
        raise RuntimeError("could not locate the RTX sensor config directory")

    # These files are ~4 MB each and Isaac Sim re-parses them at load, so only
    # copy when the destination is missing or stale - an unconditional copy of
    # 15 MB visibly slows every launch.
    copied = 0
    for target in targets:
        for profile in profiles:
            dest = target / profile.name
            if dest.exists() and dest.stat().st_mtime >= profile.stat().st_mtime:
                continue
            shutil.copy2(profile, dest)
            copied += 1

    if copied:
        print(f"[RTX] installed {copied} lidar profile(s)")

    return [p.stem for p in profiles]


def spawn_mid360(
    parent_prim_path: str,
    config_dir: Path | str = CONFIG_DIR,
    translation: tuple[float, float, float] = MID360_POS,
    orientation: tuple[float, float, float, float] = MID360_QUAT_WXYZ,
) -> list[str]:
    """Spawn the Mid-360 as several co-located RTX LiDAR prims.

    Args:
        parent_prim_path: prim to mount under, e.g.
            ``/World/envs/env_0/Robot/torso_link``.
        config_dir: directory holding the generated profiles.
        translation: sensor offset from the parent, in metres.
        orientation: sensor rotation as a wxyz quaternion. The default carries
            the URDF's 180 deg roll; getting it wrong inverts the cloud.

    Returns:
        The spawned prim paths, one per profile.
    """
    import omni.kit.commands
    from pxr import Gf, Sdf

    names = install_configs(config_dir)

    prim_paths = []
    for name in names:
        path = f"{parent_prim_path}/{name}"
        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=name,
            parent=parent_prim_path,
            config=name,
            translation=Gf.Vec3d(*translation),
            orientation=Gf.Quatd(*orientation),
        )
        if prim is None:
            raise RuntimeError(f"failed to create RTX LiDAR from config '{name}'")

        # Isaac Sim 6.0 enables multi-tick rendering, so tickRate genuinely
        # limits how often the sensor renders. This is what keeps the point
        # rate at the real 200k/s instead of the simulation frame rate - on
        # 5.1 the attribute was ignored and the sensor fired every frame.
        tick_attr = prim.CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float)
        tick_attr.Set(SCAN_RATE_HZ)

        prim_paths.append(path)

    return prim_paths


def attach_ros2_publishers(
    prim_paths: list[str],
    topic: str = "/livox/mid360/points",
    frame_id: str = "mid360_link",
    sim_rate_hz: float = 60.0,
    combine: bool = True,
) -> str:
    """Publish the RTX LiDAR returns as ``sensor_msgs/PointCloud2``.

    Args:
        prim_paths: the sensor prims from :func:`spawn_mid360`.
        topic: topic to publish on. With ``combine``, every prim publishes here
            so subscribers see one merged cloud; otherwise each prim gets its
            own ``topic/a``, ``topic/b``, ... which is useful for debugging
            which part of the sweep a point came from.
        frame_id: TF frame the points are expressed in.
        sim_rate_hz: simulation frame rate, used to derive the publish divisor.
        combine: publish all prims to one topic.

    Returns:
        The graph path holding the publishers.
    """
    import omni.graph.core as og

    graph_path = "/ActionGraph/RtxLidarROS2"

    # Mirrors the graph Isaac Sim's own "ROS 2 OmniGraphs > RTX Lidar" menu
    # builds. Two pieces are easy to miss and leave the topic advertised but
    # silent: a ROS2Context feeding every helper, and render products created
    # *inside* the graph by IsaacCreateRenderProduct rather than beforehand by
    # replicator - the helper reads the graph's own render product path.
    nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        # Renders one frame so the render products are valid before the helpers
        # first read them.
        ("RunOneFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
    ]
    connections = [("OnTick.outputs:tick", "RunOneFrame.inputs:execIn")]
    values = []

    for i, prim_path in enumerate(prim_paths):
        suffix = chr(ord("A") + i)
        rp_node = f"RenderProduct_{suffix}"
        helper = f"Publish_{suffix}"

        nodes += [
            (rp_node, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            (helper, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ]
        connections += [
            ("RunOneFrame.outputs:step", f"{rp_node}.inputs:execIn"),
            (f"{rp_node}.outputs:execOut", f"{helper}.inputs:execIn"),
            (f"{rp_node}.outputs:renderProductPath", f"{helper}.inputs:renderProductPath"),
            ("Context.outputs:context", f"{helper}.inputs:context"),
        ]
        values += [
            (f"{rp_node}.inputs:cameraPrim", [prim_path]),
            # RTX LiDAR needs only a 1x1 texture: the returns come from the
            # sensor, not from the rendered image.
            (f"{rp_node}.inputs:width", 1),
            (f"{rp_node}.inputs:height", 1),
            (f"{helper}.inputs:type", "point_cloud"),
            (
                f"{helper}.inputs:topicName",
                topic if combine else f"{topic}/{chr(ord('a') + i)}",
            ),
            (f"{helper}.inputs:frameId", frame_id),
            (f"{helper}.inputs:queueSize", 1),
            # frameSkipCount is deprecated on 6.0 - tickRate (set in
            # spawn_mid360) governs the sensor's render rate instead.
            (f"{helper}.inputs:frameSkipCount", 0),
            # fullScan=False emits each frame's returns as they fire, which is
            # what makes the non-repetitive sweep visible over time.
            (f"{helper}.inputs:fullScan", False),
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


def blind_radius(mount_height: float, min_tilt_deg: float = 7.16) -> float:
    """Radius of the Mid-360's blind cone, in metres.

    The steepest downward ray is about -7.2 deg and there is no nadir ray, so
    nothing within roughly 8x the mount height is ever seen.
    """
    import math

    return mount_height / math.tan(math.radians(min_tilt_deg))
