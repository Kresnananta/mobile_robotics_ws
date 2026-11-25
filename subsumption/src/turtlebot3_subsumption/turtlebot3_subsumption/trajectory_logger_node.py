#!/usr/bin/env python3
"""Optional Trajectory & Active-Layer Telemetry Logger.

Records robot pose (/odom), minimum obstacle clearance (/scan), final actuator
command (/cmd_vel), and which subsumption layer currently owns control into a
CSV file for post-run trajectory visualization and quantitative analysis.
Disabled by default in launch files so `rqt_graph` remains purely architectural.
"""

import csv
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TrajectoryLoggerNode(Node):
    """Logs 2D trajectory and active subsumption layer to CSV."""

    def __init__(self) -> None:
        super().__init__('trajectory_logger_node')

        self.declare_parameter('level', 2)
        self.declare_parameter('output_csv', '/mobile_robotics_ws/subsumption_traj.csv')
        self.declare_parameter('sample_rate_hz', 5.0)
        self.declare_parameter('avoid_timespan', 0.5)
        self.declare_parameter('explore_timespan', 1.0)

        self.level = int(self.get_parameter('level').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.avoid_timespan = float(self.get_parameter('avoid_timespan').value)
        self.explore_timespan = float(self.get_parameter('explore_timespan').value)

        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        self.csv_file = open(self.output_csv, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time_s', 'x', 'y', 'yaw', 'linear_x', 'angular_z',
            'min_scan', 'active_layer'
        ])

        self.start_time = None
        self.pose = None
        self.min_scan = float('nan')
        self.last_cmd = Twist()

        self.t_avoid = None
        self.t_explore = None
        self.t_wander = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Twist, '/avoid/cmd_vel', self.avoid_cb, 10)
        self.create_subscription(Twist, '/wander/cmd_vel', self.wander_cb, 10)
        self.create_subscription(Twist, '/explore/cmd_vel', self.explore_cb, 10)

        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self.sample_cb)
        self.get_logger().info(
            f'TrajectoryLogger recording Level {self.level} -> "{self.output_csv}"'
        )

    def odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (pos.x, pos.y, yaw)

    def scan_cb(self, msg: LaserScan) -> None:
        r_min = max(msg.range_min, 0.05)
        r_max = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0.5 else 3.5
        best = r_max
        for r in msg.ranges:
            if math.isnan(r) or r < r_min:
                continue
            val = r_max if math.isinf(r) else min(r, r_max)
            if val < best:
                best = val
        self.min_scan = best

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def avoid_cb(self, _msg: Twist) -> None:
        self.t_avoid = self.get_clock().now()

    def wander_cb(self, _msg: Twist) -> None:
        self.t_wander = self.get_clock().now()

    def explore_cb(self, _msg: Twist) -> None:
        self.t_explore = self.get_clock().now()

    def _infer_active_layer(self, now) -> str:
        if self.level == 0:
            moving = abs(self.last_cmd.linear.x) > 0.005 or abs(self.last_cmd.angular.z) > 0.01
            return 'L0_AVOID' if moving else 'IDLE'

        if self.t_avoid is not None:
            dt = (now - self.t_avoid).nanoseconds * 1e-9
            if 0.0 <= dt < self.avoid_timespan:
                return 'L0_AVOID'
        if self.level >= 2 and self.t_explore is not None:
            dt = (now - self.t_explore).nanoseconds * 1e-9
            if 0.0 <= dt < self.explore_timespan:
                return 'L2_EXPLORE'
        if self.t_wander is not None:
            dt = (now - self.t_wander).nanoseconds * 1e-9
            if 0.0 <= dt < 0.6:
                return 'L1_WANDER'
        return 'IDLE'

    def sample_cb(self) -> None:
        now = self.get_clock().now()
        if now.nanoseconds == 0 or self.pose is None:
            return
        if self.start_time is None:
            self.start_time = now

        t_rel = (now - self.start_time).nanoseconds * 1e-9
        if t_rel < 0.0:
            self.start_time = now
            t_rel = 0.0

        x, y, yaw = self.pose
        layer = self._infer_active_layer(now)
        self.writer.writerow([
            f'{t_rel:.3f}', f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}',
            f'{self.last_cmd.linear.x:.4f}', f'{self.last_cmd.angular.z:.4f}',
            f'{self.min_scan:.4f}', layer
        ])
        self.csv_file.flush()

    def destroy_node(self) -> None:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryLoggerNode()
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
