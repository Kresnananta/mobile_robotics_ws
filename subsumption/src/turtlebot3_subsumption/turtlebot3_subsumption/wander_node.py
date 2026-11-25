#!/usr/bin/env python3
"""Layer 1 Behavior Node: Wander (Aimless Stochastic Locomotion).

Implements Brooks' Layer 1 wandering competence as a ROS 2 Finite State
Machine (Paper Section II.B):
  - SAMPLE_HEADING: Samples a new random angular rate, turn duration, and
    forward cruise duration.
  - TURN_RANDOM: Executes a gentle curved turn for `turn_duration` seconds.
  - CRUISE_FORWARD: Drives forward with slight stochastic drift for
    `cruise_duration` seconds, then transitions back to SAMPLE_HEADING.

Obstacle avoidance is deliberately delegated to Layer 0 (AvoidNode) via the
Subsumption safety suppressor, preserving strict modular separation.
"""

from enum import Enum
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WanderState(Enum):
    SAMPLE_HEADING = 'SAMPLE_HEADING'
    TURN_RANDOM = 'TURN_RANDOM'
    CRUISE_FORWARD = 'CRUISE_FORWARD'


class WanderNode(Node):
    """Layer 1 stochastic wandering FSM node."""

    def __init__(self) -> None:
        super().__init__('wander_node')

        self.declare_parameter('output_topic', '/wander/cmd_vel')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('turn_linear_speed', 0.04)
        self.declare_parameter('min_angular_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('min_turn_duration', 1.0)
        self.declare_parameter('max_turn_duration', 2.2)
        self.declare_parameter('min_cruise_duration', 2.5)
        self.declare_parameter('max_cruise_duration', 4.5)
        self.declare_parameter('random_seed', -1)

        self.output_topic = str(self.get_parameter('output_topic').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_linear_speed = float(self.get_parameter('turn_linear_speed').value)
        self.min_ang = float(self.get_parameter('min_angular_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        self.min_turn_dur = float(self.get_parameter('min_turn_duration').value)
        self.max_turn_dur = float(self.get_parameter('max_turn_duration').value)
        self.min_cruise_dur = float(self.get_parameter('min_cruise_duration').value)
        self.max_cruise_dur = float(self.get_parameter('max_cruise_duration').value)
        seed = int(self.get_parameter('random_seed').value)

        self.rng = random.Random(seed if seed >= 0 else None)

        self.state = WanderState.SAMPLE_HEADING
        self.phase_start = None
        self.turn_omega = 0.0
        self.cruise_drift = 0.0
        self.turn_duration = 1.5
        self.cruise_duration = 3.0

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self.timer_callback)

        self.get_logger().info(
            f'Layer 1 WanderNode started: out="{self.output_topic}", '
            f'v={self.linear_speed:.2f}m/s, w=[{self.min_ang:.2f}, {self.max_ang:.2f}]rad/s'
        )

    def _sample_new_segment(self, now) -> None:
        direction = self.rng.choice([-1.0, 1.0])
        self.turn_omega = direction * self.rng.uniform(self.min_ang, self.max_ang)
        self.cruise_drift = self.rng.uniform(-0.08, 0.08)
        self.turn_duration = self.rng.uniform(self.min_turn_dur, self.max_turn_dur)
        self.cruise_duration = self.rng.uniform(self.min_cruise_dur, self.max_cruise_dur)
        self.phase_start = now
        self.state = WanderState.TURN_RANDOM
        self.get_logger().info(
            f'Wander FSM -> TURN_RANDOM (w={self.turn_omega:+.2f}rad/s for '
            f'{self.turn_duration:.1f}s, then cruise {self.cruise_duration:.1f}s)'
        )

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            # Wait for Gazebo /clock when use_sim_time is active
            return

        if self.state == WanderState.SAMPLE_HEADING or self.phase_start is None:
            self._sample_new_segment(now)

        elapsed = (now - self.phase_start).nanoseconds * 1e-9
        if elapsed < 0.0:
            # Clock reset
            self._sample_new_segment(now)
            elapsed = 0.0

        cmd = Twist()
        if self.state == WanderState.TURN_RANDOM:
            if elapsed >= self.turn_duration:
                self.state = WanderState.CRUISE_FORWARD
                self.phase_start = now
                self.get_logger().info(
                    f'Wander FSM -> CRUISE_FORWARD (v={self.linear_speed:.2f}m/s)'
                )
            else:
                cmd.linear.x = self.turn_linear_speed
                cmd.angular.z = self.turn_omega
                self.pub.publish(cmd)
                return

        if self.state == WanderState.CRUISE_FORWARD:
            if elapsed >= self.cruise_duration:
                self._sample_new_segment(now)
                cmd.linear.x = self.turn_linear_speed
                cmd.angular.z = self.turn_omega
            else:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = self.cruise_drift

        self.pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WanderNode()
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
