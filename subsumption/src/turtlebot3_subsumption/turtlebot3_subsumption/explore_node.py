#!/usr/bin/env python3
"""Layer 2 Behavior Node: Explore (Open-Space & Corridor Seeker).

Implements Brooks' Layer 2 exploration competence ("whenlook" + "look" +
"pathplan", adapted to 360-degree 2D LiDAR per Minglong Li et al. Section IV.A.1)
as a ROS 2 Finite State Machine:
  - MONITOR_IDLE: Observes LiDAR sectors; remains quiet when no clear open
    corridor stands out, allowing WanderNode + AvoidNode to break symmetry.
  - LOOK_SCAN: Pulses `Bool(True)` on `/explore/inhibit` (driving
    `inhibitor_wander`) while evaluating overlapping angular LiDAR sectors.
  - PLAN_PATH: Selects the highest-scoring open corridor using both sector mean
    range and lower-percentile clearance (penalizing narrow pillars) with mild
    heading continuity to avoid oscillation.
  - HEAD_OPEN_SPACE: Publishes proportional steering + forward velocity on
    `/explore/cmd_vel` (suppressing WanderNode via `suppressor_l1_l2`) while
    keeping `/explore/inhibit` active.
"""

from enum import Enum
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ExploreState(Enum):
    MONITOR_IDLE = 'MONITOR_IDLE'
    LOOK_SCAN = 'LOOK_SCAN'
    PLAN_PATH = 'PLAN_PATH'
    HEAD_OPEN_SPACE = 'HEAD_OPEN_SPACE'


class ExploreNode(Node):
    """Layer 2 open-space / corridor exploration FSM node."""

    def __init__(self) -> None:
        super().__init__('explore_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/explore/cmd_vel')
        self.declare_parameter('inhibit_topic', '/explore/inhibit')
        self.declare_parameter('fov_deg', 240.0)
        self.declare_parameter('sector_width_deg', 32.0)
        self.declare_parameter('sector_step_deg', 10.0)
        self.declare_parameter('explore_min_clearance', 1.25)
        self.declare_parameter('replan_front_threshold', 0.75)
        self.declare_parameter('min_plan_hold', 1.5)
        self.declare_parameter('look_interval', 4.5)
        self.declare_parameter('look_duration', 0.30)
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('max_angular_speed', 0.85)
        self.declare_parameter('angular_kp', 1.15)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.inhibit_topic = str(self.get_parameter('inhibit_topic').value)
        self.fov_rad = math.radians(float(self.get_parameter('fov_deg').value))
        self.sector_half_rad = math.radians(
            float(self.get_parameter('sector_width_deg').value) * 0.5
        )
        self.sector_step_rad = math.radians(
            float(self.get_parameter('sector_step_deg').value)
        )
        self.explore_min_clearance = float(
            self.get_parameter('explore_min_clearance').value
        )
        self.replan_front_threshold = float(
            self.get_parameter('replan_front_threshold').value
        )
        self.min_plan_hold = float(self.get_parameter('min_plan_hold').value)
        self.look_interval = float(self.get_parameter('look_interval').value)
        self.look_duration = float(self.get_parameter('look_duration').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)

        self.state = ExploreState.MONITOR_IDLE
        self.target_angle = 0.0
        self.look_start_time = None
        self.last_plan_time = None
        self.best_candidate_angle = 0.0
        self.best_candidate_clearance = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.inhibit_pub = self.create_publisher(Bool, self.inhibit_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Layer 2 ExploreNode started: scan="{self.scan_topic}", '
            f'cmd="{self.output_topic}", inhibit="{self.inhibit_topic}", '
            f'min_clearance={self.explore_min_clearance:.2f}m'
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _set_state(self, new_state: ExploreState, detail: str = '') -> None:
        if new_state != self.state:
            suffix = f' ({detail})' if detail else ''
            self.get_logger().info(
                f'Explore FSM: {self.state.value} -> {new_state.value}{suffix}'
            )
            self.state = new_state

    def _evaluate_sectors(self, msg: LaserScan):
        """Return (best_angle, best_raw_clearance, front_clearance)."""
        range_min = max(msg.range_min, 0.05)
        range_max = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0.5 else 3.5

        rays = []
        front_rays = []
        half_fov = self.fov_rad * 0.5

        for idx, raw_r in enumerate(msg.ranges):
            if math.isnan(raw_r):
                continue
            if math.isinf(raw_r):
                # Open corridor beyond max sensor range scores full range_max
                r = range_max
            elif raw_r < range_min:
                continue
            else:
                r = min(raw_r, range_max)

            theta = self._normalize_angle(msg.angle_min + idx * msg.angle_increment)
            if abs(theta) <= half_fov + self.sector_half_rad:
                rays.append((theta, r))
            if abs(theta) <= math.radians(20.0):
                front_rays.append(r)

        front_clearance = min(front_rays) if front_rays else 0.0
        if not rays:
            return 0.0, 0.0, front_clearance

        best_score = -1.0
        best_angle = 0.0
        best_raw_clearance = 0.0

        num_steps = int(round(self.fov_rad / self.sector_step_rad)) + 1
        for k in range(num_steps):
            center = -half_fov + k * self.sector_step_rad
            sector_vals = [
                r for (theta, r) in rays
                if abs(self._normalize_angle(theta - center)) <= self.sector_half_rad
            ]
            if len(sector_vals) < 3:
                continue

            sector_vals.sort()
            mean_r = sum(sector_vals) / len(sector_vals)
            p15_idx = max(0, int(0.15 * (len(sector_vals) - 1)))
            p15_r = sector_vals[p15_idx]

            # Combine average depth with lower-percentile safety to avoid thin pillars
            raw_clearance = 0.60 * mean_r + 0.40 * p15_r

            # Mild continuity & forward preference to prevent symmetric flip-flopping
            continuity = 0.85 + 0.15 * math.cos(self._normalize_angle(center - self.target_angle))
            forward_bias = 0.90 + 0.10 * math.cos(center)
            score = raw_clearance * continuity * forward_bias

            if score > best_score:
                best_score = score
                best_angle = center
                best_raw_clearance = raw_clearance

        return best_angle, best_raw_clearance, front_clearance

    def scan_callback(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        best_angle, best_clearance, front_clearance = self._evaluate_sectors(msg)

        # --- State: MONITOR_IDLE ---
        if self.state == ExploreState.MONITOR_IDLE:
            if best_clearance >= self.explore_min_clearance:
                self.look_start_time = now
                self.best_candidate_angle = best_angle
                self.best_candidate_clearance = best_clearance
                self._set_state(
                    ExploreState.LOOK_SCAN,
                    f'open space detected: {best_clearance:.2f}m @ {math.degrees(best_angle):+.0f}deg'
                )
                self.inhibit_pub.publish(Bool(data=True))
            return

        # --- State: LOOK_SCAN (Brooks' whenlook + look + Inhibitor trigger) ---
        if self.state == ExploreState.LOOK_SCAN:
            # Keep wander inhibited while scanning & planning
            self.inhibit_pub.publish(Bool(data=True))

            # Smooth candidate estimate during look window
            self.best_candidate_angle = best_angle
            self.best_candidate_clearance = best_clearance

            dt_look = (now - self.look_start_time).nanoseconds * 1e-9
            if dt_look < 0.0:
                self.look_start_time = now
                return
            if dt_look < self.look_duration:
                return

            self._set_state(ExploreState.PLAN_PATH)

        # --- State: PLAN_PATH (Brooks' pathplan) ---
        if self.state == ExploreState.PLAN_PATH:
            self.inhibit_pub.publish(Bool(data=True))
            if self.best_candidate_clearance >= self.explore_min_clearance:
                self.target_angle = self.best_candidate_angle
                self.last_plan_time = now
                self._set_state(
                    ExploreState.HEAD_OPEN_SPACE,
                    f'heading={math.degrees(self.target_angle):+.0f}deg, '
                    f'clearance={self.best_candidate_clearance:.2f}m'
                )
            else:
                self._set_state(
                    ExploreState.MONITOR_IDLE,
                    f'no wide corridor ({self.best_candidate_clearance:.2f}m)'
                )
                return

        # --- State: HEAD_OPEN_SPACE ---
        if self.state == ExploreState.HEAD_OPEN_SPACE:
            # Yield back to lower layers if surrounded closely in all sectors
            if best_clearance < self.explore_min_clearance * 0.80:
                self._set_state(
                    ExploreState.MONITOR_IDLE,
                    f'enclosed area ({best_clearance:.2f}m), yielding to wander'
                )
                return

            dt_plan = (
                (now - self.last_plan_time).nanoseconds * 1e-9
                if self.last_plan_time is not None
                else self.look_interval
            )
            if dt_plan < 0.0:
                self.last_plan_time = now
                dt_plan = 0.0

            # Periodically re-enter LOOK_SCAN (whenlook) or if path ahead closes in
            # after allowing at least min_plan_hold seconds to steer into current plan
            need_front_replan = (
                dt_plan >= self.min_plan_hold
                and front_clearance < self.replan_front_threshold
                and abs(self.target_angle) < 0.40
            )
            if dt_plan >= self.look_interval or need_front_replan:
                self.look_start_time = now
                self.best_candidate_angle = best_angle
                self.best_candidate_clearance = best_clearance
                self._set_state(
                    ExploreState.LOOK_SCAN,
                    f'refreshing open corridor (front={front_clearance:.2f}m)'
                )
                self.inhibit_pub.publish(Bool(data=True))
                return

            # Smoothly track current open corridor heading
            err = self._normalize_angle(best_angle - self.target_angle)
            self.target_angle = self._normalize_angle(self.target_angle + 0.35 * err)

            # Keep Wander inhibited and publish Explore velocity to suppressor_l1_l2
            self.inhibit_pub.publish(Bool(data=True))

            cmd = Twist()
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, self.angular_kp * self.target_angle)
            )
            if abs(self.target_angle) < 1.05:
                cmd.linear.x = self.linear_speed * max(0.20, math.cos(self.target_angle))
            else:
                # Sharp turn into side corridor before accelerating
                cmd.linear.x = 0.03

            self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExploreNode()
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
