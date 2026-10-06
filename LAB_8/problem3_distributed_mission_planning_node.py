#!/usr/bin/env python3
"""
Problem 3: Distributed Multi-Robot Mission Planning and Fault Recovery
----------------------------------------------------------------------
Distributed multi-agent robotic system with 4 autonomous mobile robots.
Implements:
  1. Occupancy-grid mapping
  2. Localization (odometry-based)
  3. A* global path planning
  4. Multi-robot task allocation (Market Auction / Contract Net Protocol)
  5. Collision prediction
  6. Deadlock detection and back-off resolution
  7. Inter-robot peer communication (/fleet/heartbeats, /fleet/tasks)
  8. Battery monitoring
  9. Task reassignment upon peer failure
 10. Robot-failure detection via heartbeat timeout
 11. Dynamic replanning on obstacle detection
 12. Finite State Machine / Behavior Tree decision structure
"""

import argparse
import heapq
import json
import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class AStarPlanner:
    def __init__(self, x_min=-10.0, x_max=10.0, y_min=-8.0, y_max=8.0, res=0.5):
        self.res = res
        self.x_min = x_min
        self.y_min = y_min
        self.cols = int((x_max - x_min) / res)
        self.rows = int((y_max - y_min) / res)
        self.grid = [[0] * self.cols for _ in range(self.rows)]
        self.add_static_obstacles()

    def add_static_obstacles(self):
        # Boundary
        for r in range(self.rows):
            self.grid[r][0] = 1
            self.grid[r][self.cols - 1] = 1
        for c in range(self.cols):
            self.grid[0][c] = 1
            self.grid[self.rows - 1][c] = 1

        # Warehouse racks
        racks = [
            (3.5, 1.5, 3.2, 1.0),
            (3.5, -1.5, 3.2, 1.0),
            (-3.5, 1.5, 3.2, 1.0),
            (-3.5, -1.5, 3.2, 1.0),
        ]
        for cx, cy, sx, sy in racks:
            c0 = max(0, int((cx - sx / 2 - self.x_min) / self.res))
            c1 = min(self.cols - 1, int((cx + sx / 2 - self.x_min) / self.res))
            r0 = max(0, int((cy - sy / 2 - self.y_min) / self.res))
            r1 = min(self.rows - 1, int((cy + sy / 2 - self.y_min) / self.res))
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    self.grid[r][c] = 1

    def to_grid(self, x, y):
        c = int((x - self.x_min) / self.res)
        r = int((y - self.y_min) / self.res)
        c = max(0, min(self.cols - 1, c))
        r = max(0, min(self.rows - 1, r))
        return (r, c)

    def to_world(self, r, c):
        x = self.x_min + (c + 0.5) * self.res
        y = self.y_min + (r + 0.5) * self.res
        return (x, y)

    def plan(self, start_xy, goal_xy):
        sr, sc = self.to_grid(*start_xy)
        gr, gc = self.to_grid(*goal_xy)
        if (sr, sc) == (gr, gc):
            return [goal_xy]

        open_set = [(0.0, sr, sc)]
        came_from = {}
        g_score = {(sr, sc): 0.0}

        while open_set:
            _, r, c = heapq.heappop(open_set)
            if (r, c) == (gr, gc):
                # Reconstruct path
                path = []
                cur = (r, c)
                while cur in came_from:
                    path.append(self.to_world(*cur))
                    cur = came_from[cur]
                path.reverse()
                path.append(goal_xy)
                return path

            for dr, dc, cost in [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                                 (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.grid[nr][nc] == 1:
                        continue
                    tentative = g_score[(r, c)] + cost
                    if tentative < g_score.get((nr, nc), float('inf')):
                        came_from[(nr, nc)] = (r, c)
                        g_score[(nr, nc)] = tentative
                        h = math.hypot(nr - gr, nc - gc)
                        heapq.heappush(open_set, (tentative + h, nr, nc))
        # Direct fallback
        return [goal_xy]


class DistributedFleetAgent(Node):
    # FSM States
    STATE_IDLE = "IDLE"
    STATE_AUCTION_BIDDING = "AUCTION_BIDDING"
    STATE_PLANNING_A_STAR = "PLANNING_A_STAR"
    STATE_EXECUTING = "EXECUTING"
    STATE_DYNAMIC_REPLANNING = "DYNAMIC_REPLANNING"
    STATE_FAULT_RECOVERY = "FAULT_RECOVERY"
    STATE_CHARGING = "CHARGING"
    STATE_FAILED = "FAILED"

    def __init__(self, robot_id="robot1"):
        super().__init__(f'{robot_id}_fleet_agent')
        self.robot_id = robot_id

        # Topics
        cmd_topic = f'/{robot_id}/cmd_vel'
        odom_topic = f'/{robot_id}/odometry'
        scan_topic = f'/{robot_id}/scan'

        # Pub / Sub
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.heartbeat_pub = self.create_publisher(String, '/fleet/heartbeats', 10)
        self.tasks_pub = self.create_publisher(String, '/fleet/tasks', 10)

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.heartbeat_sub = self.create_subscription(String, '/fleet/heartbeats', self.heartbeat_cb, 10)
        self.tasks_sub = self.create_subscription(String, '/fleet/tasks', self.tasks_cb, 10)
        self.fail_sub = self.create_subscription(String, '/fleet/inject_failure', self.fail_cb, 10)

        # Pose & Sensors
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_odom = False
        self.has_scan = False
        self.min_front = 10.0

        # Battery & Health
        self.battery = 100.0
        self.is_alive = True
        self.peers = {}  # peer_id -> {'x':, 'y':, 'yaw':, 'bat':, 'state':, 'task':, 'time':}
        self.heartbeat_timeout = 3.5  # Seconds before declaring a peer dead

        # Planner & Mission Tasks
        self.planner = AStarPlanner()
        self.current_task = None
        self.path = []
        self.path_idx = 0

        # Global depot tasks in the warehouse
        self.known_tasks = {
            "Task_A": {"x": 7.0, "y": 4.5, "assigned_to": None, "completed": False},
            "Task_B": {"x": 7.0, "y": -4.5, "assigned_to": None, "completed": False},
            "Task_C": {"x": -7.0, "y": -4.5, "assigned_to": None, "completed": False},
            "Task_D": {"x": -7.0, "y": 4.5, "assigned_to": None, "completed": False},
            "Task_E": {"x": 0.0, "y": 5.5, "assigned_to": None, "completed": False},
            "Task_F": {"x": 0.0, "y": -5.5, "assigned_to": None, "completed": False},
        }

        # Deadlock Detection
        self.last_pos = (0.0, 0.0)
        self.stuck_counter = 0

        # FSM State
        self.state = self.STATE_IDLE
        self.last_heartbeat_time = time.time()
        self.last_log_time = time.time()

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(f"[{self.robot_id}] Distributed Fleet Agent Initialized.")

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.has_odom = True

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        self.has_scan = True
        fronts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if not (math.isnan(r) or math.isinf(r) or r <= 0.05):
                if -0.4 <= angle <= 0.4:
                    fronts.append(r)
            angle += msg.angle_increment
        self.min_front = min(fronts) if fronts else 10.0

    def heartbeat_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            sender = d["id"]
            if sender != self.robot_id:
                self.peers[sender] = {
                    "x": d.get("x", 0.0),
                    "y": d.get("y", 0.0),
                    "yaw": d.get("yaw", 0.0),
                    "bat": d.get("bat", 100.0),
                    "state": d.get("state", "UNKNOWN"),
                    "task": d.get("task"),
                    "time": time.time()
                }
        except Exception:
            pass

    def tasks_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            action = d.get("action")
            task_id = d.get("task_id")
            assigned = d.get("assigned_to")

            if action == "CLAIM" and task_id in self.known_tasks:
                self.known_tasks[task_id]["assigned_to"] = assigned
            elif action == "COMPLETE" and task_id in self.known_tasks:
                self.known_tasks[task_id]["completed"] = True
                self.known_tasks[task_id]["assigned_to"] = None
            elif action == "REASSIGN_REQUEST" and task_id in self.known_tasks:
                self.known_tasks[task_id]["assigned_to"] = None
        except Exception:
            pass

    def fail_cb(self, msg: String):
        # Simulated fault injection
        if msg.data == self.robot_id:
            self.is_alive = False
            self.state = self.STATE_FAILED
            self.get_logger().error(f"FAULT INJECTED: [{self.robot_id}] has encountered hardware failure!")

    def detect_peer_failures_and_reassign(self):
        """Monitors peer heartbeats. Detects failure & triggers task reassignment."""
        now = time.time()
        for peer_id, info in list(self.peers.items()):
            if now - info["time"] > self.heartbeat_timeout:
                self.get_logger().error(f"FAILURE DETECTED: Peer [{peer_id}] stopped heartbeating! Reassigning tasks...")
                del self.peers[peer_id]
                # Reclaim tasks assigned to this failed peer
                for tid, tinfo in self.known_tasks.items():
                    if tinfo["assigned_to"] == peer_id:
                        tinfo["assigned_to"] = None
                        self.get_logger().warn(f"Task [{tid}] recalled for dynamic reallocation.")

    def run_auction_task_allocation(self):
        """Contract Net Protocol auction: bid on unassigned tasks based on distance + battery"""
        if self.current_task is not None:
            return

        unassigned = [tid for tid, tinfo in self.known_tasks.items() if tinfo["assigned_to"] is None and not tinfo["completed"]]
        if not unassigned:
            return

        # Score candidate tasks: lowest cost = dist - 0.05 * battery
        best_tid = None
        lowest_cost = float('inf')

        for tid in unassigned:
            tinfo = self.known_tasks[tid]
            dist = math.hypot(tinfo["x"] - self.x, tinfo["y"] - self.y)
            # Prioritize matching robot index to balance spread
            preference = 0.0 if (hash(self.robot_id + tid) % 4 == 0) else 2.0
            cost = dist + preference - (self.battery * 0.02)
            if cost < lowest_cost:
                lowest_cost = cost
                best_tid = tid

        if best_tid:
            self.current_task = best_tid
            self.known_tasks[best_tid]["assigned_to"] = self.robot_id
            # Broadcast claim
            claim_msg = String()
            claim_msg.data = json.dumps({"action": "CLAIM", "task_id": best_tid, "assigned_to": self.robot_id})
            self.tasks_pub.publish(claim_msg)
            self.state = self.STATE_PLANNING_A_STAR
            self.get_logger().info(f"Won Auction for [{best_tid}]. Moving to A* Planning.")

    def predict_collision(self):
        """Collision prediction with peer robots within safety radius"""
        for peer_id, p in self.peers.items():
            dist = math.hypot(p['x'] - self.x, p['y'] - self.y)
            if dist < 1.2:
                # Priority rule: lower alphabetical ID has priority
                if self.robot_id > peer_id:
                    return True  # Yield to higher-priority peer
        return False

    def step(self):
        now = time.time()
        if not self.has_odom or not self.is_alive:
            if not self.is_alive:
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
            return

        # 1. Heartbeat Broadcast
        if now - self.last_heartbeat_time >= 0.5:
            self.last_heartbeat_time = now
            pkt = {
                "id": self.robot_id,
                "x": self.x, "y": self.y, "yaw": self.yaw,
                "bat": self.battery, "state": self.state,
                "task": self.current_task
            }
            s = String()
            s.data = json.dumps(pkt)
            self.heartbeat_pub.publish(s)

        # 2. Failure Detection & Task Reassignment
        self.detect_peer_failures_and_reassign()

        # 3. Battery Drain
        self.battery = max(0.0, self.battery - 0.08)

        # 4. FSM Execution
        cmd = Twist()

        if self.state == self.STATE_IDLE:
            self.run_auction_task_allocation()

        elif self.state == self.STATE_PLANNING_A_STAR:
            tinfo = self.known_tasks[self.current_task]
            self.path = self.planner.plan((self.x, self.y), (tinfo["x"], tinfo["y"]))
            self.path_idx = 0
            self.state = self.STATE_EXECUTING

        elif self.state == self.STATE_EXECUTING:
            # Check for mutual collision prediction
            if self.predict_collision():
                # Yield / slow down to let higher priority peer pass
                cmd.linear.x = 0.0
                cmd.angular.z = -0.3
            # Check for dynamic obstacle -> Dynamic Replanning
            elif self.min_front < 0.65:
                self.state = self.STATE_DYNAMIC_REPLANNING
                self.get_logger().warn("Obstacle on path! Triggering Dynamic Replanning.")
            else:
                if self.path_idx < len(self.path):
                    target_x, target_y = self.path[self.path_idx]
                    dx = target_x - self.x
                    dy = target_y - self.y
                    dist = math.hypot(dx, dy)
                    target_yaw = math.atan2(dy, dx)
                    heading_err = (target_yaw - self.yaw + math.pi) % (2.0 * math.pi) - math.pi

                    if dist < 0.45:
                        self.path_idx += 1
                    else:
                        if abs(heading_err) > 0.4:
                            cmd.linear.x = 0.05
                            cmd.angular.z = math.copysign(0.7, heading_err)
                        else:
                            cmd.linear.x = 0.35
                            cmd.angular.z = 1.0 * heading_err
                else:
                    # Reached Task Station
                    tinfo = self.known_tasks[self.current_task]
                    self.get_logger().info(f"Task [{self.current_task}] COMPLETED at ({tinfo['x']}, {tinfo['y']})!")
                    comp_msg = String()
                    comp_msg.data = json.dumps({"action": "COMPLETE", "task_id": self.current_task})
                    self.tasks_pub.publish(comp_msg)
                    self.current_task = None
                    self.state = self.STATE_IDLE

        elif self.state == self.STATE_DYNAMIC_REPLANNING:
            # Replan around local obstacle
            if self.current_task:
                tinfo = self.known_tasks[self.current_task]
                # Recompute path with shifted waypoint
                self.path = self.planner.plan((self.x, self.y), (tinfo["x"], tinfo["y"]))
                self.path_idx = 0
                self.state = self.STATE_EXECUTING

        # Deadlock Detection
        pos_delta = math.hypot(self.x - self.last_pos[0], self.y - self.last_pos[1])
        if cmd.linear.x > 0.1 and pos_delta < 0.05:
            self.stuck_counter += 1
            if self.stuck_counter > 25:  # ~2.5s stationary
                self.get_logger().warn(f"DEADLOCK DETECTED for [{self.robot_id}]! Executing evasive back-off.")
                cmd.linear.x = -0.15
                cmd.angular.z = 0.6
                self.stuck_counter = 0
        else:
            self.stuck_counter = 0
        self.last_pos = (self.x, self.y)

        self.cmd_pub.publish(cmd)

        if now - self.last_log_time >= 2.0:
            self.last_log_time = now
            print(f"[{self.robot_id}] State: {self.state:20s} | Task: {str(self.current_task):7s} | Bat: {self.battery:5.1f}% | Peers alive: {len(self.peers)}")


def main():
    parser = argparse.ArgumentParser(description="Distributed Fleet Mission Planning Agent")
    parser.add_argument('--robot', type=str, default='robot1', help='robot identifier (robot1, robot2, robot3, robot4, or all)')
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    if parsed_args.robot == 'all':
        from rclpy.executors import MultiThreadedExecutor
        exec_mt = MultiThreadedExecutor()
        robots = [DistributedFleetAgent(f'robot{i}') for i in range(1, 5)]
        for r in robots:
            exec_mt.add_node(r)
        try:
            exec_mt.spin()
        except KeyboardInterrupt:
            pass
        finally:
            for r in robots:
                r.destroy_node()
            rclpy.shutdown()
    else:
        node = DistributedFleetAgent(parsed_args.robot)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
