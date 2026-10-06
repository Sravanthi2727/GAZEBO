#!/usr/bin/env python3
"""
Task 3 — Shopping-mall service robot.
Goes to a destination while steering around people and furniture using LiDAR.

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
    """Minimum valid range for rays whose angle is in [a0, a1] (robot frame)."""
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


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.create_subscription(Odometry, '/robot/odometry', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/robot/scan', self.scan_cb, qos_profile_sensor_data)

        # Green pad in shopping_mall.sdf (world). Odom is converted using spawn pose.
        self.spawn_x = 0.4
        self.spawn_y = 0.0
        self.goal_x = 11.2
        self.goal_y = 0.0
        self.odom0_x = None
        self.odom0_y = None
        self.x = self.spawn_x
        self.y = self.spawn_y
        self.yaw = 0.0
        self.have_odom = False
        self.have_scan = False
        self.arrived = False

        self.front = 8.0
        self.left = 8.0
        self.right = 8.0
        self.front_left = 8.0
        self.front_right = 8.0

        # Keep turning the same way until the front is clearly clear (stops flicker).
        self.avoid_dir = 0.0
        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info('Task 3: mall obstacle avoidance. Stop on the green exit pad.')

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
        self.front = sector_min(msg, -0.50, 0.50)
        self.front_left = sector_min(msg, 0.35, 0.90)
        self.front_right = sector_min(msg, -0.90, -0.35)
        self.left = sector_min(msg, 0.70, 1.40)
        self.right = sector_min(msg, -1.40, -0.70)
        self.have_scan = True

    def navigate(self):
        cmd = Twist()
        if self.arrived:
            self.publish_stop()
            return
        if not self.have_odom or not self.have_scan:
            self.get_logger().info('Waiting for odom + LiDAR (is the bridge running?)', throttle_duration_sec=2.0)
            self.publisher.publish(cmd)
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)
        # Pad is 1.4 m; hold still so we do not roll into the far wall.
        if dist < 0.75:
            self.arrived = True
            self.get_logger().info('Reached the green shop-exit pad. Holding stop.')
            self.publish_stop()
            return

        heading = math.atan2(dy, dx)
        err = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))

        # Near the pad, ignore nearby walls and just creep onto the mark.
        if dist < 1.5:
            cmd.linear.x = 0.20
            cmd.angular.z = max(-0.8, min(0.8, 1.4 * err))
            self.publisher.publish(cmd)
            return

        blocked = self.front < 1.05 or self.front_left < 0.75 or self.front_right < 0.75
        if blocked:
            if abs(self.avoid_dir) < 0.05:
                # Turn toward the side with more free space.
                self.avoid_dir = 1.0 if self.left > self.right else -1.0
            cmd.linear.x = 0.12
            cmd.angular.z = 1.1 * self.avoid_dir
            self.get_logger().info(
                f'Obstacle: front={self.front:.2f} L={self.left:.2f} R={self.right:.2f} -> turn {self.avoid_dir:+.0f}',
                throttle_duration_sec=0.6,
            )
        else:
            self.avoid_dir = 0.0
            # Slight side bias so we slide past nearby objects instead of clipping them.
            side_bias = 0.0
            if self.front_right < 1.1:
                side_bias += 0.6
            if self.front_left < 1.1:
                side_bias -= 0.6
            cmd.linear.x = 0.55 if abs(err) < 0.6 else 0.25
            cmd.angular.z = max(-1.0, min(1.0, 1.4 * err + side_bias))
            self.get_logger().info(f'Clear. dist_to_goal={dist:.2f} m', throttle_duration_sec=1.5)

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
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
