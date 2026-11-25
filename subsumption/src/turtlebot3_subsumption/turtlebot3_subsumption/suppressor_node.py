#!/usr/bin/env python3
"""Reusable ROS 2 Suppressor Template Node for the Subsumption Model.

Implements the dual-callback timestamped Suppressor mechanism from:
Minglong Li et al., "Subsumption Model Implemented on ROS for Mobile Robots"
(IEEE, 2016), Section II.D.

When a Twist arrives on `upper_topic`, the node updates `t_upper` and
unconditionally publishes the upper-layer Twist to `output_topic`.
When a Twist arrives on `lower_topic`, the node computes:
    dt = t_lower - t_upper
If 0 <= dt < timespan, the lower-layer Twist is suppressed (dropped).
Otherwise, the lower-layer Twist is forwarded to `output_topic`.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SuppressorNode(Node):
    """Parameterizable Subsumption Suppressor node."""

    def __init__(self) -> None:
        super().__init__('suppressor_node')

        self.declare_parameter('lower_topic', '/lower/cmd_vel')
        self.declare_parameter('upper_topic', '/upper/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('timespan', 0.5)

        self.lower_topic = str(self.get_parameter('lower_topic').value)
        self.upper_topic = str(self.get_parameter('upper_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.timespan = float(self.get_parameter('timespan').value)

        self.t_upper = None
        self.is_suppressing = False

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.lower_sub = self.create_subscription(
            Twist, self.lower_topic, self.lower_callback, 10
        )
        self.upper_sub = self.create_subscription(
            Twist, self.upper_topic, self.upper_callback, 10
        )

        self.get_logger().info(
            f'Suppressor ready: lower="{self.lower_topic}", '
            f'upper="{self.upper_topic}", output="{self.output_topic}", '
            f'timespan={self.timespan:.2f}s'
        )

    def upper_callback(self, msg: Twist) -> None:
        """Record timestamp and immediately forward upper-layer replacement."""
        self.t_upper = self.get_clock().now()
        if not self.is_suppressing:
            self.is_suppressing = True
            self.get_logger().info(
                f'Suppressing "{self.lower_topic}" with "{self.upper_topic}" '
                f'(timespan={self.timespan:.2f}s)'
            )
        self.pub.publish(msg)

    def lower_callback(self, msg: Twist) -> None:
        """Forward lower-layer Twist only when not suppressed by upper-layer."""
        t_lower = self.get_clock().now()

        if self.t_upper is not None:
            dt = (t_lower - self.t_upper).nanoseconds * 1e-9
            if dt < 0.0:
                self.t_upper = None
            elif dt < self.timespan:
                return

        if self.is_suppressing:
            self.is_suppressing = False
            self.get_logger().info(
                f'Released suppression; resuming "{self.lower_topic}"'
            )

        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SuppressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
