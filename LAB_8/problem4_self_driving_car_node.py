#!/usr/bin/env python3
"""
Problem 4: Self-Driving Car in a Simulated Road
-----------------------------------------------
Autonomous vehicle that follows road lane markings using camera computer vision
and avoids static/dynamic obstacles using LiDAR.

Subscribes:
  /car/camera/image_raw  (sensor_msgs/msg/Image)
  /car/scan              (sensor_msgs/msg/LaserScan)
  /car/odometry          (nav_msgs/msg/Odometry)

Publishes:
  /car/cmd_vel           (geometry_msgs/msg/Twist)
"""

import math
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


class Problem4SelfDrivingCarNode(Node):
    def __init__(self):
        super().__init__('problem4_self_driving_car_node')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/car/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image, '/car/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, '/car/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/car/odometry', self.odom_callback, 10)

        # Vehicle State
        self.x = 0.0
        self.y = 0.0
        self.has_image = False
        self.has_scan = False

        # Vision Pipeline State
        self.lane_error = 0.0
        self.prev_lane_error = 0.0
        self.lane_detected = False

        # Obstacle State
        self.min_front_obs = 15.0
        self.min_left_obs = 15.0
        self.min_right_obs = 15.0

        # Steering PID Gains
        self.kp = 1.35
        self.kd = 0.45

        # Speed Control Settings
        self.cruise_speed = 0.42
        self.caution_speed = 0.20
        self.stop_distance = 1.40
        self.caution_distance = 3.20

        # Periodic Control Timer (15 Hz)
        self.last_step_time = time.time()
        self.last_log_time = time.time()
        self.timer = self.create_timer(0.066, self.control_step)
        self.get_logger().info("Problem 4 Self-Driving Car Node Initialized.")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return
        self.has_scan = True

        fronts = []
        lefts = []
        rights = []

        angle = msg.angle_min
        for r in msg.ranges:
            if not (math.isnan(r) or math.isinf(r) or r <= 0.1):
                if -0.35 <= angle <= 0.35:
                    fronts.append(r)
                elif 0.35 < angle <= 1.2:
                    lefts.append(r)
                elif -1.2 <= angle < -0.35:
                    rights.append(r)
            angle += msg.angle_increment

        self.min_front_obs = min(fronts) if fronts else 15.0
        self.min_left_obs = min(lefts) if lefts else 15.0
        self.min_right_obs = min(rights) if rights else 15.0

    def image_callback(self, msg: Image):
        """Processes camera feed to detect lane center and compute lateral error"""
        self.has_image = True
        try:
            # Decode image buffer directly to numpy array
            height = msg.height
            width = msg.width

            if msg.encoding in ['rgb8', 'bgr8']:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                if msg.encoding == 'rgb8':
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                else:
                    bgr = img
            else:
                # Fallback grayscale or raw
                bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, -1))
                if bgr.shape[2] == 1:
                    bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

            # Region of Interest: Look at lower half where road surface is visible
            roi_y_start = int(height * 0.55)
            roi = bgr[roi_y_start:height, 0:width]

            # Convert to HSV for robust color segmentation
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # 1. Yellow Center Dash Line Mask
            lower_yellow = np.array([15, 70, 70])
            upper_yellow = np.array([40, 255, 255])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # 2. White Outer Boundary Line Mask
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([180, 50, 255])
            white_mask = cv2.inRange(hsv, lower_white, upper_white)

            # Combine lane masks
            combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

            # Find Moments & Centroid of Lane Markers
            M = cv2.moments(combined_mask)
            if M["m00"] > 500:
                cx = int(M["m10"] / M["m00"])
                # Desired tracking target is the middle of the right-hand lane
                # Vehicle camera center is width / 2
                lane_target_x = width * 0.5
                self.lane_error = (cx - lane_target_x) / (width * 0.5)
                self.lane_detected = True
            else:
                # No distinct lane detected: keep previous error with slight decay
                self.lane_error *= 0.8
                self.lane_detected = False

        except Exception as e:
            self.get_logger().warn(f"Image processing error: {e}")

    def control_step(self):
        now = time.time()
        dt = max(0.01, now - self.last_step_time)
        self.last_step_time = now

        cmd = Twist()

        # 1. Longitudinal Speed Regulation based on LiDAR Obstacles
        if self.min_front_obs < self.stop_distance:
            # Emergency Obstacle Stop or Evasive Lane Change
            self.get_logger().warn(
                f"[COLLISION HAZARD] Obstacle at {self.min_front_obs:.2f}m! Executing obstacle avoidance stop/swerve.")
            cmd.linear.x = 0.05
            # Swerve into open adjacent side
            if self.min_left_obs > self.min_right_obs:
                cmd.angular.z = 0.8  # Swerve left
            else:
                cmd.angular.z = -0.8 # Swerve right
        elif self.min_front_obs < self.caution_distance:
            # Caution Zone: Decelerate speed
            speed_factor = (self.min_front_obs - self.stop_distance) / (self.caution_distance - self.stop_distance)
            cmd.linear.x = self.caution_speed + (self.cruise_speed - self.caution_speed) * speed_factor

            # Lateral Steering via Camera Lane Tracking
            d_error = (self.lane_error - self.prev_lane_error) / dt
            self.prev_lane_error = self.lane_error
            cmd.angular.z = - (self.kp * self.lane_error + self.kd * d_error)
        else:
            # Clear Road: Nominal Cruise Speed
            cmd.linear.x = self.cruise_speed

            # Lateral Steering via Camera Lane Tracking PID
            d_error = (self.lane_error - self.prev_lane_error) / dt
            self.prev_lane_error = self.lane_error
            cmd.angular.z = - (self.kp * self.lane_error + self.kd * d_error)

        # Enforce angular speed limits
        cmd.angular.z = max(-1.2, min(1.2, cmd.angular.z))

        self.cmd_pub.publish(cmd)

        # Periodic Log
        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            status = "CRUISE" if self.min_front_obs >= self.caution_distance else ("CAUTION" if self.min_front_obs >= self.stop_distance else "STOP/AVOID")
            print(f"[Self-Driving Car] Mode: {status:10s} | Speed: {cmd.linear.x:4.2f}m/s | Steer: {cmd.angular.z:5.2f}rad/s | Lane Err: {self.lane_error:5.2f} | Front Obs: {self.min_front_obs:4.2f}m")


def main():
    rclpy.init()
    node = Problem4SelfDrivingCarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            stop_cmd = Twist()
            node.cmd_pub.publish(stop_cmd)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
