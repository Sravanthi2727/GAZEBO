#!/usr/bin/env python3
"""
Task 5 — Search-and-rescue in a maze.
Bug-2 style: drive toward the goal using sensors; when blocked, follow the
right-hand wall until the start-goal line (m-line) is reached closer to the goal
with a clear heading. No pre-programmed waypoint list.

Run AFTER Gazebo and the ROS-Gazebo bridge are up.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def valid_range(r, fallback):
    if r is None or math.isnan(r) or math.isinf(r) or r <= 0.0:
        return fallback
    return r


def sector_min(msg, a0, a1):
    fallback = msg.range_max if msg.range_max > 0.0 else 8.0
    if not msg.ranges or msg.angle_increment == 0.0:
        return fallback
    best = fallback
    angle = msg.angle_min
    for r in msg.ranges:
        if a0 <= angle <= a1:
            v = valid_range(r, fallback)
            if v < best:
                best = v
        angle += msg.angle_increment
    return best


def range_toward(msg, robot_yaw, world_heading):
    """Range along a world-frame heading, using the current scan."""
    fallback = msg.range_max if msg.range_max > 0.0 else 8.0
    rel = math.atan2(math.sin(world_heading - robot_yaw), math.cos(world_heading - robot_yaw))
    return sector_min(msg, rel - 0.22, rel + 0.22)


class BugAlgorithmNode(Node):
    def __init__(self):
        super().__init__('bug_algorithm_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.create_subscription(Odometry, '/robot/odometry', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/robot/scan', self.scan_cb, qos_profile_sensor_data)

        self.spawn_x = 1.5
        self.spawn_y = 1.5
        self.start_x = 1.5
        self.start_y = 1.5
        self.goal_x = 10.5
        self.goal_y = 1.5
        self.odom0_x = None
        self.odom0_y = None

        self.x = self.start_x
        self.y = self.start_y
        self.yaw = 0.0
        self.have_odom = False
        self.have_scan = False
        self.arrived = False
        self.last_scan = None

        self.front = 8.0
        self.right = 8.0
        self.front_right = 8.0
        self.left = 8.0

        self.state = 'SEEK_GOAL'
        self.hit_goal_dist = None
        self.follow_steps = 0
        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info('Task 5: Bug-2 maze search. Stop on the green rescue pad.')

    def odom_cb(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        if self.odom0_x is None:
            self.odom0_x = ox
            self.odom0_y = oy
        self.x = self.spawn_x + (ox - self.odom0_x)
        self.y = self.spawn_y + (oy - self.odom0_y)
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    def publish_stop(self):
        self.publisher.publish(Twist())

    def scan_cb(self, msg):
        self.last_scan = msg
        self.front = sector_min(msg, -0.40, 0.40)
        self.front_right = sector_min(msg, -0.95, -0.40)
        self.right = sector_min(msg, -1.55, -0.85)
        self.left = sector_min(msg, 0.70, 1.45)
        self.have_scan = True

    def goal_dist(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def mline_distance(self):
        x0, y0 = self.start_x, self.start_y
        x1, y1 = self.goal_x, self.goal_y
        den = math.hypot(x1 - x0, y1 - y0)
        if den < 1e-6:
            return 0.0
        num = abs((y1 - y0) * self.x - (x1 - x0) * self.y + x1 * y0 - y1 * x0)
        return num / den

    def heading_to_goal(self):
        return math.atan2(self.goal_y - self.y, self.goal_x - self.x)

    def seek_goal(self, cmd):
        heading = self.heading_to_goal()
        err = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))
        if abs(err) > 0.35:
            cmd.linear.x = 0.08
            cmd.angular.z = 1.3 * err
        else:
            cmd.linear.x = 0.50
            cmd.angular.z = 1.1 * err

    def follow_right_wall(self, cmd):
        # Right-hand rule: keep a wall on the right, turn left when the front is blocked.
        if self.front < 0.85:
            cmd.linear.x = 0.05
            cmd.angular.z = 1.15
        elif self.front_right < 0.70:
            cmd.linear.x = 0.22
            cmd.angular.z = 0.70
        elif self.right < 0.55:
            cmd.linear.x = 0.28
            cmd.angular.z = 0.55
        elif self.right > 1.15:
            cmd.linear.x = 0.28
            cmd.angular.z = -0.55
        else:
            cmd.linear.x = 0.42
            cmd.angular.z = 0.0

    def can_leave_wall(self):
        if self.follow_steps < 25:
            return False
        if self.mline_distance() > 0.40:
            return False
        d = self.goal_dist()
        if self.hit_goal_dist is None or d > self.hit_goal_dist - 0.55:
            return False
        if self.last_scan is None:
            return False
        toward = range_toward(self.last_scan, self.yaw, self.heading_to_goal())
        return toward > 1.40 and self.front > 1.10

    def navigate(self):
        cmd = Twist()
        if self.arrived:
            self.publish_stop()
            return
        if not self.have_odom or not self.have_scan:
            self.get_logger().info('Waiting for odom + LiDAR (is the bridge running?)', throttle_duration_sec=2.0)
            self.publisher.publish(cmd)
            return

        d = self.goal_dist()
        if d < 0.75:
            self.arrived = True
            self.get_logger().info('Reached the green rescue pad. Holding stop.')
            self.publish_stop()
            return
        if d < 1.5:
            self.seek_goal(cmd)
            cmd.linear.x = min(cmd.linear.x, 0.22)
            self.publisher.publish(cmd)
            return

        if self.state == 'SEEK_GOAL':
            if self.front < 1.05:
                self.state = 'FOLLOW_WALL'
                self.hit_goal_dist = d
                self.follow_steps = 0
                self.get_logger().info(f'Path blocked at ({self.x:.2f},{self.y:.2f}). Right-wall follow.')
            else:
                self.get_logger().info(f'Seeking goal. d={d:.2f} m', throttle_duration_sec=1.2)
                self.seek_goal(cmd)
        else:
            self.follow_steps += 1
            if self.can_leave_wall():
                self.state = 'SEEK_GOAL'
                self.get_logger().info('Back on m-line, closer to goal, heading clear. Seeking again.')
                self.seek_goal(cmd)
            else:
                self.get_logger().info(
                    f'Wall follow. front={self.front:.2f} right={self.right:.2f} mline={self.mline_distance():.2f}',
                    throttle_duration_sec=1.0,
                )
                self.follow_right_wall(cmd)

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BugAlgorithmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
