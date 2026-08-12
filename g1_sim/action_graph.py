"""Shared OmniGraph helpers for the G1 RTX pipeline.

Clock, TF, joint states, camera and cmd_vel are published by native OmniGraph
nodes wired up in :mod:`g1_sim.rtx_camera`; the RTX LiDAR cloud is published by
:mod:`g1_sim.rtx_publisher`. This module only holds the small piece both RTX
sims share: reading the latest Twist back off a cmd_vel subscriber graph.
"""

from __future__ import annotations

GRAPH_PATH = "/ActionGraph/G1ROS2"


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
