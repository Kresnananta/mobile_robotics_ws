#!/usr/bin/env python3
"""
bug2.py -- Assignment 2. THIS IS THE ONLY FILE YOU EDIT.

TurtleBot3 Burger only. Everything is hardcoded below: no ROS parameters, no
command-line arguments. To change something, change the constant and rebuild.

    cd ~/mobile_robotics_ws && colcon build --packages-select a2_worlds
    source install/setup.bash
    ros2 launch a2_worlds bug_world.launch.py world:=bug_1_circles y_pose:=0.10
    ros2 run a2_worlds bug2                       # another terminal

There are two driving functions and the main loop calls exactly one of them:

    go_and_stop()   GIVEN, complete. Drives at the goal and stops at the first
                    obstacle, printing the hit point it implies. Run this
                    FIRST: it proves the world, the robot, the laser and the
                    odometry all work, and it shows you the H arithmetic.

    bug2()          YOURS. Four TODO blocks inside one function.

Switch between them with MODE below. Nothing else changes.

Of the lines in this file you need to read exactly two things: the five
constants at the top, and the function `bug2()`. Everything else -- the
subscriptions, the 20 Hz loop, the scan filtering, the m-line arithmetic, the
CSV and the RViz markers -- is plumbing, already written, and you can ignore
it until something breaks.
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# ======================= SET THESE FIVE, THEN REBUILD =======================
Y0 = -0.20            # YOUR start offset: 0.10 * (NRP % 5) - 0.20
MODE = 'bug2'        # 'test' = go_and_stop (given) | 'bug2' = yours
RUN_NAME = 'w1'      # names the CSV for this run: w1, w2, w3, w1_naive, ...
HAND = +1            # contour following: +1 = wall on your left, -1 = right
DELTA = 0.25         # progress margin, metres. Set to -1e6 for the naive run
# ============================================================================

# TurtleBot3 Burger. Do not change these two: they are the robot.
R_ROBOT = 0.1126         # centre to furthest perimeter point
SCAN_OFFSET_X = -0.032   # base_scan sits this far behind the robot centre

GOAL_AHEAD = 10.0        # the goal is 10 m along +x from the world origin
SPEED = 0.15             # cruise, m/s
K_HEADING = 1.5          # proportional gain on heading error
HIT_RANGE = 0.45         # closer than this straight ahead counts as a hit
WALL_DIST = 0.40         # standoff held while following a contour
EPS = 0.08               # "on the m-line" tolerance
GOAL_TOL = 0.25          # "arrived"
CONE = math.radians(20)  # half-width of the front / side cones
TIMEOUT = 240.0          # seconds of sim time before giving up
OUT_DIR = os.path.expanduser('~/a2_results')


def wrap(a):
    """Wrap an angle to (-pi, pi]. Use this on EVERY angle difference."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    """Yaw from a quaternion; roll and pitch are zero for a planar robot."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Bug2(Node):

    def __init__(self):
        super().__init__('bug2')
        self.pose = None          # (x, y, yaw)
        self.start = None         # pose at the first odometry message
        self.goal = None          # set with it, see on_odom
        self.ux = self.uy = 0.0   # unit vector along the m-line
        self.state = 'GOAL_SEEK'
        self.hit = None           # (x, y) of the current hit point
        self.d_hit = None         # d(H, goal) at that hit point
        self.front = self.left = self.right = self.near = float('inf')
        self.rows, self.trail, self.marks = [], [], {}
        self.path = 0.0
        self.changes = 0
        self.clear = float('inf')
        self.t0 = None
        self.done = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mk = self.create_publisher(MarkerArray, 'a2_markers', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)        # 20 Hz
        self.get_logger().info(f'MODE={MODE}  Y0={Y0:+.2f}  run={RUN_NAME}')

    # ------------------------------------------------------------- the m-line
    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.start is not None:
            return

        # Fixed ONCE, here, and never touched again. A recomputed m-line
        # always passes through the robot, so d_mline is always 0 and the
        # leave condition is always true -- the robot then loops forever.
        self.start = self.pose
        sx, sy, _ = self.start

        # The goal is at (10, 0) in WORLD coordinates and you start at
        # (0, Y0), so relative to your start it is (10, -Y0). Writing it this
        # way works whether /odom reports world coordinates or starts at zero
        # on your spawn pose -- you do not have to know which.
        self.goal = (sx + GOAL_AHEAD, sy - Y0)

        dx, dy = self.goal[0] - sx, self.goal[1] - sy
        n = math.hypot(dx, dy)
        self.ux, self.uy = dx / n, dy / n
        self.t0 = self.now()
        self.get_logger().info(
            f'start ({sx:+.3f}, {sy:+.3f})  goal ({self.goal[0]:+.3f}, '
            f'{self.goal[1]:+.3f})  m-line {n:.4f} m, '
            f'tilt {math.degrees(math.atan2(dy, dx)):+.3f} deg')

    def d_mline(self, x, y):
        """Signed distance to the m-line: + right of start->goal, - left."""
        return (x - self.start[0]) * self.uy - (y - self.start[1]) * self.ux

    def d_goal(self, x, y):
        return math.hypot(self.goal[0] - x, self.goal[1] - y)

    def goal_bearing(self, x, y, yaw):
        """Bearing of the goal in the robot frame: 0 ahead, + to the left."""
        return wrap(math.atan2(self.goal[1] - y, self.goal[0] - x) - yaw)

    # ------------------------------------------------------------- the laser
    def on_scan(self, msg):
        """Nearest valid range ahead, left, right, and over the whole scan.

        Beam i points at angle_min + i*angle_increment -- index 0 is NOT
        ahead. REP-117: inf/nan mean "nothing in range"; the TB3 driver also
        reports a no-return as 0.0, so isfinite() alone is not enough.
        """
        f = l = r = near = float('inf')
        for i, d in enumerate(msg.ranges):
            if not math.isfinite(d) or d <= 0.0:
                continue
            if d < msg.range_min or d > msg.range_max:
                continue
            th = wrap(msg.angle_min + i * msg.angle_increment)
            near = min(near, d)
            if abs(th) <= CONE:
                f = min(f, d)
            if abs(wrap(th - math.pi / 2)) <= CONE:
                l = min(l, d)
            if abs(wrap(th + math.pi / 2)) <= CONE:
                r = min(r, d)
        self.front, self.left, self.right, self.near = f, l, r, near

    def side(self):
        """Range to the wall on the side HAND selects."""
        return self.left if HAND > 0 else self.right

    # --------------------------------------------------------- the main loop
    def tick(self):
        if self.pose is None or self.start is None or self.done:
            return
        x, y, _ = self.pose

        if self.d_goal(x, y) < GOAL_TOL:
            return self.finish('GOAL REACHED')
        if self.now() - self.t0 > TIMEOUT:
            return self.finish('TIMEOUT')

        v, w = self.go_and_stop() if MODE == 'test' else self.bug2()
        if self.done:               # the drive function stopped us
            return

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, max(-1.0, min(1.0, w))
        self.pub.publish(cmd)
        self.log()

    # ===================================== GIVEN: the primitive test function
    def go_and_stop(self):
        """Drive at the goal, stop at the first obstacle, print the hit point.

        Not Bug 2: no contour following, no m-line logic. Returns (v, w).
        """
        x, y, yaw = self.pose
        if self.front < HIT_RANGE:
            self.report_hit()
            self.finish('STOPPED at the first obstacle')
            return 0.0, 0.0
        err = self.goal_bearing(x, y, yaw)
        return (SPEED if abs(err) < 0.30 else 0.03), K_HEADING * err

    def report_hit(self):
        """Where the obstacle surface is, and the hit point H it implies."""
        x, y, yaw = self.pose
        ux, uy = math.cos(yaw), math.sin(yaw)
        sx, sy = x + SCAN_OFFSET_X * ux, y + SCAN_OFFSET_X * uy   # laser origin
        fx, fy = sx + self.front * ux, sy + self.front * uy       # surface
        hx, hy = fx - R_ROBOT * ux, fy - R_ROBOT * uy             # robot body
        log = self.get_logger()
        log.info('-' * 58)
        log.info(f'  pose            ({x:+.3f}, {y:+.3f})  '
                 f'yaw {math.degrees(yaw):+.1f} deg')
        log.info(f'  front range      {self.front:.3f} m')
        log.info(f'  surface at      ({fx:+.3f}, {fy:+.3f})')
        log.info(f'  => hit point H  ({hx:+.3f}, {hy:+.3f})')
        log.info(f'  => d(H, goal)    {self.d_goal(hx, hy):.3f} m')
        log.info('  reference, Y0 = 0:  W1 and W2  H = (2.39, 0) d = 7.61  '
                 '|  W3  H = (7.29, 0) d = 2.71')
        log.info('-' * 58)
        return hx, hy

    # ============================================ YOURS: four TODOs, one place
    def bug2(self):
        """Bug 2. Return (v, w): forward speed and turn rate.

        Available to you, all already computed:
            self.pose                 (x, y, yaw)
            self.front/left/right     nearest valid range in each cone
            self.side()               the wall on the side HAND selects
            self.state                'GOAL_SEEK' or 'WALL_FOLLOW'
            self.hit, self.d_hit      the latched hit point and d(H, goal)
            self.d_mline(x, y)        signed distance to the m-line
            self.d_goal(x, y)         distance to the goal
            self.goal_bearing(...)    heading error toward the goal
            self.set_state('...')     changes state and counts the change
            self.mark('hit'|'leave', x, y)   draws H or L in RViz
            self.report_hit()         returns (hx, hy), the correct H

        Constants: SPEED, K_HEADING, HIT_RANGE, WALL_DIST, EPS, DELTA, HAND.
        """
        x, y, yaw = self.pose

        if self.state == 'GOAL_SEEK':
            # TODO 2 -- when self.front < HIT_RANGE you have hit something.
            if self.front < HIT_RANGE:
                hx, hy = self.report_hit()
                # Safeguard: declare goal unreachable if H was already visited
                # for (kind, _), m in self.marks.items():
                    # if kind == 'hit' and math.hypot(
                            # hx - m.pose.position.x,
                            # hy - m.pose.position.y) < 0.15:
                        # self.finish('GOAL UNREACHABLE')
                        # return 0.0, 0.0

                self.hit = (hx, hy)
                self.d_hit = self.d_goal(hx, hy)
                self.mark('hit', hx, hy)
                self.set_state('WALL_FOLLOW')
                return 0.0, 0.0

            # TODO 1 -- drive along the m-line toward the goal.
            err = self.goal_bearing(x, y, yaw)
            v = SPEED if abs(err) < 0.30 else 0.03
            w = K_HEADING * err
            return v, w

        # ---- WALL_FOLLOW ----
        # TODO 4 -- the leave condition (both tests required).
        if abs(self.d_mline(x, y)) < EPS and self.d_goal(x, y) < self.d_hit - DELTA:
            self.mark('leave', x, y)
            self.hit = None
            self.d_hit = None
            self.set_state('GOAL_SEEK')
            return 0.0, 0.0
        #
        # TODO 3 -- otherwise follow the contour at WALL_DIST, three cases:
        err = self.side() - WALL_DIST
        if self.front < HIT_RANGE:
            # Case 1: something close ahead -> turn away, do not drive forward
            return 0.0, -HAND * 0.8
        elif self.side() > 2.0 * WALL_DIST and self.near < WALL_DIST + 0.10:
            # Blind-spot guard: obstacle is at front-side (20-70 deg), drive straight until side cone catches it
            return SPEED, 0.0
        elif err > 0.0:
            # Case 2: the wall is too far -> curve toward it (radius ~0.33 m on outer corners)
            w = HAND * min(1.5 * err, 0.45)
            return SPEED, w
        else:
            # Case 3: the wall is too close -> curve away from it
            v = SPEED if err > -0.15 else 0.08
            w = HAND * max(2.0 * err, -0.75)
            return v, w

    def set_state(self, s):
        if s != self.state:
            self.changes += 1
            self.get_logger().info(f'{self.state} -> {s}')
        self.state = s

    # ---------------------------------------------- RViz, the CSV, the finish
    def mark(self, kind, x, y):
        """A coloured sphere in RViz. kind is 'hit' (red) or 'leave' (blue)."""
        rgb = (0.9, 0.1, 0.1) if kind == 'hit' else (0.2, 0.4, 1.0)
        m = _marker(kind, len(self.marks), Marker.SPHERE, rgb, 0.14)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.02
        self.marks[(kind, m.id)] = m

    def publish_markers(self):
        line = _marker('mline', 0, Marker.LINE_STRIP, (0.1, 0.8, 0.2), 0.03)
        line.points = [_pt(*self.start[:2]), _pt(*self.goal)]
        trail = _marker('trail', 0, Marker.LINE_STRIP, (1.0, 1.0, 1.0), 0.02)
        trail.points = [_pt(px, py) for px, py in self.trail[::5]]
        arr = MarkerArray()
        for m in [line, trail] + list(self.marks.values()):
            m.header.stamp = self.get_clock().now().to_msg()
            arr.markers.append(m)
        self.mk.publish(arr)

    def log(self):
        x, y, yaw = self.pose
        if self.trail:
            self.path += math.hypot(x - self.trail[-1][0], y - self.trail[-1][1])
        self.trail.append((x, y))
        if math.isfinite(self.near):
            self.clear = min(self.clear, self.near - R_ROBOT)
        self.rows.append([round(self.now() - self.t0, 2), round(x, 4),
                          round(y, 4), round(math.degrees(yaw), 1),
                          _n(self.front), _n(self.near),
                          round(self.d_goal(x, y), 4),
                          round(self.d_mline(x, y), 4), self.state])
        if len(self.trail) % 5 == 0:
            self.publish_markers()
        self.get_logger().info(
            f'{self.state:11s} x={x:+.2f} y={y:+.2f} front={_n(self.front)} '
            f'side={_n(self.side())} d_mline={self.d_mline(x, y):+.3f} '
            f'd_goal={self.d_goal(x, y):5.2f}', throttle_duration_sec=1.0)

    def finish(self, why):
        """Stop, write the two CSVs, print the summary. Called exactly once."""
        self.done = True
        for _ in range(5):
            self.pub.publish(Twist())

        x, y, _ = self.pose
        reached = self.d_goal(x, y) < GOAL_TOL
        straight = math.hypot(self.goal[0] - self.start[0],
                              self.goal[1] - self.start[1])
        ratio = self.path / straight
        clear = round(self.clear, 4) if math.isfinite(self.clear) else ''

        _write(RUN_NAME + '.csv', 'w',
               ['t', 'x', 'y', 'yaw_deg', 'front', 'near', 'd_goal',
                'd_mline', 'state'], self.rows)
        _write('summary.csv', 'a',
               ['run', 'mode', 'reached', 'time_s', 'path_m', 'ratio',
                'min_clearance_m', 'state_changes', 'final_d_goal_m',
                'Y0', 'HAND', 'DELTA'],
               [[RUN_NAME, MODE, reached, round(self.now() - self.t0, 1),
                 round(self.path, 3), round(ratio, 3), clear, self.changes,
                 round(self.d_goal(x, y), 3), Y0, HAND, DELTA]])

        log = self.get_logger()
        log.info(f'{why}   reached={reached}')
        log.info(f'  path {self.path:.3f} m / straight {straight:.3f} m '
                 f'= {ratio:.2f}x')
        log.info(f'  min clearance {self.clear:+.4f} m   '
                 f'state changes {self.changes}')
        log.info(f'  wrote {RUN_NAME}.csv, appended summary.csv, in {OUT_DIR}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def _marker(ns, mid, mtype, rgb, scale):
    m = Marker()
    m.header.frame_id = 'odom'
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
    m.scale.x = m.scale.y = m.scale.z = scale
    return m


def _write(name, mode, header, rows):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    need_header = mode == 'w' or not os.path.isfile(path)
    with open(path, mode, newline='') as f:
        w = csv.writer(f)
        if need_header:
            w.writerow(header)
        w.writerows(rows)


def _pt(x, y, z=0.02):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _n(v):
    return round(v, 3) if math.isfinite(v) else 'inf'


def main():
    rclpy.init()
    node = Bug2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.start is not None and not node.done:
            node.finish('INTERRUPTED')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
