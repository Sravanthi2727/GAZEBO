#!/usr/bin/env python3
"""
Problem 1: Battery-Aware Autonomous Robot Agent
------------------------------------------------
Performs a sequence of tasks while continuously monitoring its battery level.
Autonomously decides whether to continue its task, return to a charging
station upon reaching a low battery threshold, and resume the task after charging.

Subscribes:
  /problem1/odometry  (nav_msgs/msg/Odometry)
  /problem1/scan      (sensor_msgs/msg/LaserScan)

Publishes:
  /problem1/cmd_vel        (geometry_msgs/msg/Twist)
  /problem1/battery_state  (sensor_msgs/msg/BatteryState)
  /problem1/task_status    (std_msgs/msg/String)
"""

import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from std_msgs.msg import String


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Problem1BatteryAwareRobotNode(Node):
    # FSM State Constants
    STATE_IDLE = "IDLE"
    STATE_NAVIGATING_TO_TASK = "NAVIGATING_TO_TASK"
    STATE_EXECUTING_TASK = "EXECUTING_TASK"
    STATE_RETURNING_TO_CHARGER = "RETURNING_TO_CHARGER"
    STATE_CHARGING = "CHARGING"
    STATE_RESUMING_TASK = "RESUMING_TASK"
    STATE_MISSION_COMPLETED = "MISSION_COMPLETED"

    def __init__(self):
        super().__init__('problem1_battery_aware_robot_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/problem1/cmd_vel', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/problem1/battery_state', 10)
        self.status_pub = self.create_publisher(String, '/problem1/task_status', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/problem1/odometry', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/problem1/scan', self.scan_callback, qos_profile_sensor_data)

        # Robot Pose & Sensor State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_odom = False
        self.has_scan = False

        # Obstacle sectors (ranges)
        self.min_front = 10.0
        self.min_left = 10.0
        self.min_right = 10.0

        # Charging Dock & Tasks
        self.charger_pose = (0.0, 0.0)
        self.task_list = [
            {"id": 1, "name": "Task 1 (Inspection A)", "x": 4.5, "y": 3.5, "duration": 3.0},
            {"id": 2, "name": "Task 2 (Pickup B)", "x": 5.0, "y": -4.0, "duration": 3.0},
            {"id": 3, "name": "Task 3 (Assembly C)", "x": -4.5, "y": -3.5, "duration": 3.0},
            {"id": 4, "name": "Task 4 (Quality D)", "x": -4.0, "y": 4.0, "duration": 3.0},
        ]
        self.current_task_idx = 0
        self.interrupted_task_idx = 0
        self.task_start_time = None

        # Battery Simulation Parameters
        self.battery_level = 100.0  # Percentage [0.0 - 100.0]
        self.low_battery_threshold = 28.0  # Threshold to trigger return to dock
        self.critical_battery_threshold = 12.0
        self.full_battery_threshold = 95.0
        self.drain_rate_idle = 0.15  # % per second
        self.drain_rate_motion = 1.35  # % per second at max speed
        self.charge_rate = 7.5  # % per second at charging dock

        # State Machine
        self.state = self.STATE_IDLE
        self.last_update_time = time.time()
        self.last_log_time = time.time()

        # Motion control limits
        self.max_lin_speed = 0.45
        self.max_ang_speed = 0.75
        self.arrival_tolerance = 0.35

        # Main Control Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Problem 1 Battery-Aware Robot Node Initialized.")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.has_odom = True

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return
        self.has_scan = True
        
        # Divide into sectors: Right [-1.5 to -0.3], Front [-0.3 to 0.3], Left [0.3 to 1.5]
        front_dists = []
        left_dists = []
        right_dists = []

        angle = msg.angle_min
        for r in msg.ranges:
            if not (math.isnan(r) or math.isinf(r) or r <= 0.05):
                if -0.35 <= angle <= 0.35:
                    front_dists.append(r)
                elif 0.35 < angle <= 1.4:
                    left_dists.append(r)
                elif -1.4 <= angle < -0.35:
                    right_dists.append(r)
            angle += msg.angle_increment

        self.min_front = min(front_dists) if front_dists else msg.range_max
        self.min_left = min(left_dists) if left_dists else msg.range_max
        self.min_right = min(right_dists) if right_dists else msg.range_max

    def update_battery(self, dt, is_moving):
        if self.state == self.STATE_CHARGING:
            # Recharging
            self.battery_level = min(100.0, self.battery_level + self.charge_rate * dt)
        else:
            # Discharging
            drain = self.drain_rate_idle * dt
            if is_moving:
                drain += self.drain_rate_motion * dt
            self.battery_level = max(0.0, self.battery_level - drain)

        # Publish BatteryState
        bat_msg = BatteryState()
        bat_msg.percentage = float(self.battery_level / 100.0)
        bat_msg.voltage = 12.0 * (self.battery_level / 100.0)
        if self.state == self.STATE_CHARGING:
            bat_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self.battery_level >= self.full_battery_threshold:
            bat_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            bat_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(bat_msg)

    def navigate_towards(self, target_x, target_y):
        """Navigate to target with obstacle avoidance"""
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        goal_yaw = math.atan2(dy, dx)
        heading_err = (goal_yaw - self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        cmd = Twist()

        # Check for immediate obstacles
        obs_dist_threshold = 0.65
        front_blocked = self.min_front < obs_dist_threshold
        left_close = self.min_left < obs_dist_threshold
        right_close = self.min_right < obs_dist_threshold

        if front_blocked:
            # Repulsive turn away from nearest obstacle
            cmd.linear.x = 0.05
            if self.min_left > self.min_right:
                cmd.angular.z = self.max_ang_speed  # Turn left
            else:
                cmd.angular.z = -self.max_ang_speed  # Turn right
        elif left_close and cmd.angular.z > 0:
            cmd.angular.z = -0.3
        elif right_close and cmd.angular.z < 0:
            cmd.angular.z = 0.3
        else:
            # Normal goal seeking
            if abs(heading_err) > 0.45:
                # Turn in place toward goal
                cmd.linear.x = 0.05
                cmd.angular.z = math.copysign(self.max_ang_speed, heading_err)
            else:
                # Drive forward with proportional steering
                speed_scale = max(0.2, min(1.0, dist / 2.0))
                cmd.linear.x = self.max_lin_speed * speed_scale
                cmd.angular.z = 1.2 * heading_err

        return cmd, dist

    def control_loop(self):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        if not self.has_odom:
            if now - self.last_log_time > 2.0:
                self.get_logger().info("Waiting for /problem1/odometry from Gazebo bridge...")
                self.last_log_time = now
            return

        cmd = Twist()
        is_moving = False

        # State Machine Transitions & Behaviors
        if self.state == self.STATE_IDLE:
            self.state = self.STATE_NAVIGATING_TO_TASK
            self.get_logger().info(f"Starting Mission -> Moving to {self.task_list[self.current_task_idx]['name']}")

        elif self.state == self.STATE_NAVIGATING_TO_TASK:
            # Autonomous decision: Check battery level vs safety threshold
            dist_to_dock = math.hypot(self.charger_pose[0] - self.x, self.charger_pose[1] - self.y)
            estimated_energy_to_dock = (dist_to_dock / self.max_lin_speed) * self.drain_rate_motion + 5.0

            if self.battery_level <= max(self.low_battery_threshold, estimated_energy_to_dock):
                self.get_logger().warn(
                    f"LOW BATTERY DETECTED: {self.battery_level:.1f}%! Aborting current task. Returning to Charging Station.")
                self.interrupted_task_idx = self.current_task_idx
                self.state = self.STATE_RETURNING_TO_CHARGER
            else:
                target = self.task_list[self.current_task_idx]
                cmd, dist = self.navigate_towards(target['x'], target['y'])
                is_moving = True

                if dist < self.arrival_tolerance:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.task_start_time = time.time()
                    self.state = self.STATE_EXECUTING_TASK
                    self.get_logger().info(f"Arrived at {target['name']}. Beginning task execution...")

        elif self.state == self.STATE_EXECUTING_TASK:
            # Robot stops at workstation to perform task
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            is_moving = False
            target = self.task_list[self.current_task_idx]

            # Still check battery during task execution
            if self.battery_level <= self.low_battery_threshold:
                self.get_logger().warn(
                    f"Low battery during execution ({self.battery_level:.1f}%). Suspending task to recharge.")
                self.interrupted_task_idx = self.current_task_idx
                self.state = self.STATE_RETURNING_TO_CHARGER
            else:
                elapsed = time.time() - self.task_start_time
                if elapsed >= target['duration']:
                    self.get_logger().info(f"Completed {target['name']} successfully.")
                    self.current_task_idx += 1
                    if self.current_task_idx >= len(self.task_list):
                        self.state = self.STATE_MISSION_COMPLETED
                        self.get_logger().info("ALL TASKS COMPLETED SUCCESSFULLY! Mission Complete.")
                    else:
                        self.state = self.STATE_NAVIGATING_TO_TASK
                        self.get_logger().info(f"Advancing to {self.task_list[self.current_task_idx]['name']}")

        elif self.state == self.STATE_RETURNING_TO_CHARGER:
            cmd, dist = self.navigate_towards(self.charger_pose[0], self.charger_pose[1])
            is_moving = True

            if dist < self.arrival_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.state = self.STATE_CHARGING
                self.get_logger().info("Docked at Charging Station. Commencing Rapid Battery Recharge.")

        elif self.state == self.STATE_CHARGING:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            is_moving = False

            if self.battery_level >= self.full_battery_threshold:
                self.get_logger().info(
                    f"Battery replenished to {self.battery_level:.1f}%. Resuming interrupted task #{self.interrupted_task_idx + 1}!")
                self.current_task_idx = self.interrupted_task_idx
                self.state = self.STATE_RESUMING_TASK

        elif self.state == self.STATE_RESUMING_TASK:
            self.state = self.STATE_NAVIGATING_TO_TASK
            self.get_logger().info(f"Resuming mission at {self.task_list[self.current_task_idx]['name']}")

        elif self.state == self.STATE_MISSION_COMPLETED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            is_moving = False

        # Apply battery update
        self.update_battery(dt, is_moving)

        # Publish velocity
        self.cmd_pub.publish(cmd)

        # Publish Task Status
        status_msg = String()
        status_msg.data = f"State: {self.state} | Bat: {self.battery_level:.1f}% | Task: {self.current_task_idx + 1}/{len(self.task_list)}"
        self.status_pub.publish(status_msg)

        # Periodic Log Output (every 1 second)
        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            print(f"[Problem 1 Agent] State: {self.state:22s} | Battery: {self.battery_level:5.1f}% | Pose: ({self.x:5.2f}, {self.y:5.2f}) | Front Obs: {self.min_front:4.2f}m")


def main(args=None):
    rclpy.init(args=args)
    node = Problem1BatteryAwareRobotNode()
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
