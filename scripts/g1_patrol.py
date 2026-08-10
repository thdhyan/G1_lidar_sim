#!/usr/bin/env python3
"""Standalone G1 patrol commander - process 2 of the 3-process pipeline.

A plain rclpy node (no Isaac Sim imports) that repeatedly publishes
``/g1/cmd_vel`` Twist messages to walk the G1 in an open loop around the
warehouse: no localization/SLAM feedback exists yet (see Plan.md), so this
is deliberately timed dead-reckoning, not a closed-loop navigator - matches
the user-confirmed scope for this pass.

Run as a genuinely separate OS process from the sim, system Python with
ROS2 sourced (same pattern as other standalone rclpy scripts this session,
e.g. ``debug_ros2_bridge.py`` inside the sim process, or the
``g1_perception_ws`` nodes outside it):

    source /opt/ros/jazzy/setup.bash
    python3 scripts/g1_patrol.py
    python3 scripts/g1_patrol.py --forward-speed 0.35 --loop-legs 6
    python3 scripts/g1_patrol.py --once   # publish one full loop then exit (testing)

Patrol pattern: a closed N-gon (default a square, 4 legs) - walk forward for
``--leg-duration`` seconds, then turn in place through ``2*pi/legs`` radians,
repeating forever. Real displacement per leg depends on the WBC policy's
actual walking speed (measured ~0.46 m/s forward in MuJoCo per
docs/decoupled_wbc_findings.md), not the commanded ``--forward-speed``
directly, and open-loop dead reckoning will drift - the loop will not close
exactly. That is expected and out of scope to fix here (would need real
localization).
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

TOPIC_CMD_VEL = "/g1/cmd_vel"
PUBLISH_HZ = 10.0


class PatrolCommander(Node):
    def __init__(
        self,
        forward_speed: float,
        turn_speed: float,
        leg_duration: float,
        legs: int,
        once: bool,
    ) -> None:
        super().__init__("g1_patrol_commander")
        self._pub = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)
        self._forward_speed = forward_speed
        self._turn_speed = turn_speed
        self._leg_duration = leg_duration
        self._legs = legs
        self._once = once

        turn_angle = 2.0 * math.pi / legs
        self._turn_duration = turn_angle / turn_speed

        self.get_logger().info(
            f"patrol: {legs}-leg loop, forward {forward_speed:.2f} m/s x {leg_duration:.1f}s "
            f"({forward_speed*leg_duration:.2f} m/leg), turn {math.degrees(turn_angle):.0f} deg "
            f"@ {turn_speed:.2f} rad/s ({self._turn_duration:.1f}s), "
            f"{'once' if once else 'forever'}"
        )

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._pub.publish(msg)

    def _hold(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
        """Publish (vx, vy, wz) at PUBLISH_HZ for duration_s, spinning the
        node so any subscriptions/timers still process."""
        period = 1.0 / PUBLISH_HZ
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            self._publish(vx, vy, wz)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def run(self) -> None:
        try:
            loops = 0
            while rclpy.ok():
                for leg in range(self._legs):
                    self.get_logger().info(f"loop {loops} leg {leg}: forward")
                    self._hold(self._forward_speed, 0.0, 0.0, self._leg_duration)
                    self.get_logger().info(f"loop {loops} leg {leg}: turn")
                    self._hold(0.0, 0.0, self._turn_speed, self._turn_duration)
                loops += 1
                if self._once:
                    break
        finally:
            # Leave the robot commanded to stop rather than mid-turn.
            self._hold(0.0, 0.0, 0.0, 0.2)


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 patrol commander - publishes /g1/cmd_vel in a loop.")
    parser.add_argument("--forward-speed", type=float, default=0.35, help="m/s commanded during each forward leg.")
    parser.add_argument("--turn-speed", type=float, default=0.35, help="rad/s commanded during each turn.")
    parser.add_argument("--leg-duration", type=float, default=6.0, help="Seconds walked forward per leg.")
    parser.add_argument("--loop-legs", type=int, default=4, help="Number of straight legs in the patrol polygon.")
    parser.add_argument("--once", action="store_true", help="Run one full loop then exit (for short tests).")
    args = parser.parse_args()

    rclpy.init()
    node = PatrolCommander(
        forward_speed=args.forward_speed,
        turn_speed=args.turn_speed,
        leg_duration=args.leg_duration,
        legs=args.loop_legs,
        once=args.once,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
