#!/usr/bin/env python3
"""Layer 0 Behavior Node: Avoid Contact (Survival Reflex).

Implements Brooks' Layer 0 survival competence ("feelforce" + "collide" +
"runaway") as a ROS 2 Finite State Machine (Paper Section II.B):
  - IDLE_SAFE: No obstacle within danger threshold. Emits a single Twist(0, 0)
    upon transitioning from an active avoidance state (to halt Gazebo diff_drive
    latching in standalone Level 0), then remains silent so lower-priority
    layers can operate unimpeded in Levels 1 and 2.
  - RUNAWAY: Obstacle within `danger_distance` (hysteresis exit `clear_distance`).
    Computes an inverse-square virtual repulsive force from 360-degree /scan
    data and moves/turns away from the obstacle vector.
  - EMERGENCY_COLLIDE: Obstacle critically close in front (`< critical_distance`).
    Halts forward velocity, backs away if rear is clear, and rotates firmly
    toward the more open side.
"""

from enum import Enum
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AvoidState(Enum):
    IDLE_SAFE = 'IDLE_SAFE'
    RUNAWAY = 'RUNAWAY'
    EMERGENCY_COLLIDE = 'EMERGENCY_COLLIDE'


class AvoidNode(Node):
    """Layer 0 reactive obstacle avoidance FSM node."""

    def __init__(self) -> None:
        super().__init__('avoid_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/avoid/cmd_vel')
        self.declare_parameter('critical_distance', 0.26)
        self.declare_parameter('danger_distance', 0.38)
        self.declare_parameter('clear_distance', 0.45)
        self.declare_parameter('influence_distance', 0.58)
        self.declare_parameter('front_arc_deg', 55.0)
        self.declare_parameter('escape_linear_speed', 0.10)
        self.declare_parameter('reverse_linear_speed', -0.08)
        self.declare_parameter('escape_angular_speed', 0.95)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.critical_dist = float(self.get_parameter('critical_distance').value)
        self.danger_dist = float(self.get_parameter('danger_distance').value)
        self.clear_dist = float(self.get_parameter('clear_distance').value)
        self.influence_dist = float(self.get_parameter('influence_distance').value)
        self.front_arc_rad = math.radians(
            float(self.get_parameter('front_arc_deg').value)
        )
        self.escape_lin = float(self.get_parameter('escape_linear_speed').value)
        self.reverse_lin = float(self.get_parameter('reverse_linear_speed').value)
        self.escape_ang = float(self.get_parameter('escape_angular_speed').value)

        self.state = AvoidState.IDLE_SAFE
        self.sticky_turn_dir = 1.0  # +1 left, -1 right

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Layer 0 AvoidNode started: scan="{self.scan_topic}", '
            f'out="{self.output_topic}", critical={self.critical_dist:.2f}m, '
            f'danger={self.danger_dist:.2f}m, clear={self.clear_dist:.2f}m'
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _set_state(self, new_state: AvoidState, d_min: float, d_front: float) -> None:
        if new_state != self.state:
            self.get_logger().info(
                f'Avoid FSM: {self.state.value} -> {new_state.value} '
                f'(d_min={d_min:.2f}m, d_front={d_front:.2f}m)'
            )
            self.state = new_state

    def scan_callback(self, msg: LaserScan) -> None:
        range_min = max(msg.range_min, 0.05)
        range_max = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0.5 else 3.5

        d_min = range_max
        d_front = range_max
        d_rear = range_max
        left_sum = 0.0
        left_cnt = 0
        right_sum = 0.0
        right_cnt = 0

        fx = 0.0
        fy = 0.0

        for idx, raw_r in enumerate(msg.ranges):
            if math.isnan(raw_r):
                continue
            if math.isinf(raw_r):
                r = range_max
            elif raw_r < range_min:
                continue
            else:
                r = min(raw_r, range_max)

            theta = self._normalize_angle(msg.angle_min + idx * msg.angle_increment)

            if r < d_min:
                d_min = r
            if abs(theta) <= self.front_arc_rad and r < d_front:
                d_front = r
            if abs(theta) >= (math.pi - self.front_arc_rad) and r < d_rear:
                d_rear = r

            if 0.35 <= theta <= 1.85:
                left_sum += r
                left_cnt += 1
            elif -1.85 <= theta <= -0.35:
                right_sum += r
                right_cnt += 1

            # Virtual repulsive force ("feelforce" module)
            if r < self.influence_dist:
                r_clamped = max(r, 0.08)
                mag = (1.0 / r_clamped - 1.0 / self.influence_dist) / (r_clamped * r_clamped)
                fx -= mag * math.cos(theta)
                fy -= mag * math.sin(theta)

        left_clear = (left_sum / left_cnt) if left_cnt > 0 else range_max
        right_clear = (right_sum / right_cnt) if right_cnt > 0 else range_max

        prev_state = self.state

        # --- FSM State Transition Logic (with hysteresis) ---
        if d_front < self.critical_dist:
            if prev_state != AvoidState.EMERGENCY_COLLIDE:
                # Choose turn direction toward side with greater clearance
                if abs(left_clear - right_clear) > 0.08:
                    self.sticky_turn_dir = 1.0 if left_clear >= right_clear else -1.0
                elif abs(fy) > 1e-3:
                    self.sticky_turn_dir = 1.0 if fy >= 0.0 else -1.0
            self._set_state(AvoidState.EMERGENCY_COLLIDE, d_min, d_front)

        elif prev_state == AvoidState.IDLE_SAFE:
            if d_min < self.danger_dist:
                self._set_state(AvoidState.RUNAWAY, d_min, d_front)

        elif prev_state == AvoidState.EMERGENCY_COLLIDE:
            if d_min >= self.clear_dist:
                self._set_state(AvoidState.IDLE_SAFE, d_min, d_front)
            elif d_front >= self.danger_dist:
                self._set_state(AvoidState.RUNAWAY, d_min, d_front)

        elif prev_state == AvoidState.RUNAWAY:
            if d_min >= self.clear_dist:
                self._set_state(AvoidState.IDLE_SAFE, d_min, d_front)

        # --- FSM Output Dispatch ---
        if self.state == AvoidState.IDLE_SAFE:
            if prev_state != AvoidState.IDLE_SAFE:
                # Emit one stop command on exit to halt latched Gazebo diff_drive
                self.pub.publish(Twist())
            return

        cmd = Twist()
        if self.state == AvoidState.EMERGENCY_COLLIDE:
            cmd.linear.x = self.reverse_lin if d_rear > 0.30 else 0.0
            cmd.angular.z = self.sticky_turn_dir * self.escape_ang
            self.pub.publish(cmd)
            return

        # State == AvoidState.RUNAWAY
        if math.hypot(fx, fy) > 1e-3:
            escape_angle = math.atan2(fy, fx)
        else:
            escape_angle = (math.pi / 2.0) * (1.0 if left_clear >= right_clear else -1.0)

        # Update sticky turn preference
        if abs(escape_angle) > 0.15:
            self.sticky_turn_dir = 1.0 if escape_angle >= 0.0 else -1.0

        if abs(escape_angle) < 1.15 and d_front > self.danger_dist:
            # Obstacle is behind or to rear-side: move forward away from it
            cmd.linear.x = self.escape_lin * max(0.35, math.cos(escape_angle))
            cmd.angular.z = max(-self.escape_ang, min(self.escape_ang, 1.1 * escape_angle))
        else:
            # Obstacle is in front or close to side: turn briskly toward escape direction
            raw_w = 1.25 * escape_angle
            if abs(raw_w) < 0.45:
                raw_w = 0.45 * self.sticky_turn_dir
            cmd.angular.z = max(-self.escape_ang, min(self.escape_ang, raw_w))

            if d_front < self.danger_dist:
                cmd.linear.x = self.reverse_lin * 0.5 if d_rear > self.clear_dist else 0.0
            else:
                # Side obstacle while front is clear: creep forward while steering away
                cmd.linear.x = self.escape_lin * 0.6

        self.pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidNode()
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
