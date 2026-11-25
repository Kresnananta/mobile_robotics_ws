#!/usr/bin/env python3
"""Reusable ROS 2 Inhibitor Template Node for the Subsumption Model.

Implements the dual-callback timestamped Inhibitor mechanism from:
Minglong Li et al., "Subsumption Model Implemented on ROS for Mobile Robots"
(IEEE, 2016), Section II.D.

When a signal arrives on `upper_topic`, the node records its timestamp `t_upper`.
When a Twist message arrives on `lower_topic`, the node computes:
    dt = t_lower - t_upper
If 0 <= dt < timespan, the lower-layer message is inhibited (dropped).
Otherwise, it is forwarded unchanged to `output_topic`.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class InhibitorNode(Node):
    """Parameterizable Subsumption Inhibitor node."""

    def __init__(self) -> None:
        super().__init__('inhibitor_node')

        self.declare_parameter('lower_topic', '/wander/cmd_vel')
        self.declare_parameter('upper_topic', '/explore/inhibit')
        self.declare_parameter('output_topic', '/wander/inhibited_cmd_vel')
        self.declare_parameter('timespan', 1.5)
        self.declare_parameter('upper_msg_type', 'bool')

        self.lower_topic = str(self.get_parameter('lower_topic').value)
        self.upper_topic = str(self.get_parameter('upper_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.timespan = float(self.get_parameter('timespan').value)
        self.upper_msg_type = str(self.get_parameter('upper_msg_type').value).lower()

        self.t_upper = None
        self.is_inhibiting = False

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.lower_sub = self.create_subscription(
            Twist, self.lower_topic, self.lower_callback, 10
        )

        if self.upper_msg_type == 'twist':
            self.upper_sub = self.create_subscription(
                Twist, self.upper_topic, self.upper_twist_callback, 10
            )
        else:
            self.upper_sub = self.create_subscription(
                Bool, self.upper_topic, self.upper_bool_callback, 10
            )

        self.get_logger().info(
            f'Inhibitor ready: lower="{self.lower_topic}", '
            f'upper="{self.upper_topic}" ({self.upper_msg_type}), '
            f'output="{self.output_topic}", timespan={self.timespan:.2f}s'
        )

    def upper_bool_callback(self, msg: Bool) -> None:
        """Record timestamp whenever an active inhibiting trigger arrives."""
        if msg.data:
            self.t_upper = self.get_clock().now()
            if not self.is_inhibiting:
                self.is_inhibiting = True
                self.get_logger().info(
                    f'Inhibiting "{self.lower_topic}" for {self.timespan:.2f}s'
                )

    def upper_twist_callback(self, _msg: Twist) -> None:
        """Record timestamp whenever any Twist arrives on the upper wire."""
        self.t_upper = self.get_clock().now()
        if not self.is_inhibiting:
            self.is_inhibiting = True
            self.get_logger().info(
                f'Inhibiting "{self.lower_topic}" for {self.timespan:.2f}s'
            )

    def lower_callback(self, msg: Twist) -> None:
        """Forward lower-layer Twist unless within the inhibiting timespan."""
        t_lower = self.get_clock().now()

        if self.t_upper is not None:
            dt = (t_lower - self.t_upper).nanoseconds * 1e-9
            if dt < 0.0:
                # Simulation clock jumped backward / reset
                self.t_upper = None
            elif dt < self.timespan:
                return

        if self.is_inhibiting:
            self.is_inhibiting = False
            self.get_logger().info(
                f'Released inhibition; forwarding "{self.lower_topic}"'
            )

        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InhibitorNode()
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
