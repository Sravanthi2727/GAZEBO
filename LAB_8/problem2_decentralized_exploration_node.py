#!/usr/bin/env python3
"""
Problem 2: Decentralized Multi-Robot Exploration
------------------------------------------------
Independent, decentralized exploration node without a central controller.
Multiple robots explore unknown partitioned environments using local sensors.
Features:
  - Local occupancy grid mapping from LiDAR
  - Frontier extraction & cost-utility selection
  - P2P information-sharing via /exploration/peer_broadcast
  - Anti-redundancy frontier discounting
  - Graceful communication loss & degraded local fallback mode
"""

import argparse
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


class DecentralizedExplorerAgent(Node):
    def __init__(self, robot_id="robot1"):
        super().__init__(f'{robot_id}_decentralized_explorer')
        self.robot_id = robot_id

        # Topics
        cmd_topic = f'/{robot_id}/cmd_vel'
        odom_topic = f'/{robot_id}/odometry'
        scan_topic = f'/{robot_id}/scan'

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.peer_pub = self.create_publisher(String, '/exploration/peer_broadcast', 10)

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.peer_sub = self.create_subscription(String, '/exploration/peer_broadcast', self.peer_cb, 10)

        # Robot Pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_odom = False
        self.has_scan = False

        # Local Occupancy Grid (2D centered around local environment)
        self.res = 0.35  # meters per cell
        self.grid_size = 46  # 46x46 cells (~16m x 16m)
        self.origin = -8.0
        # -1 = unknown, 0 = free, 100 = occupied
        self.grid = [[-1] * self.grid_size for _ in range(self.grid_size)]

        # Peer tracking for redundancy prevention & comm loss handling
        self.peer_data = {}  # {peer_id: {'x': x, 'y': y, 'goal': (gx, gy), 'time': t}}
        self.comms_range_cutoff = 7.0  # Max radio range in meters
        self.comms_timeout = 4.0      # Seconds before declaring comm loss

        # Current exploration target
        self.target_goal = None
        self.min_front = 10.0
        self.min_left = 10.0
        self.min_right = 10.0

        # Metrics
        self.explored_cells_count = 0
        self.last_broadcast_time = time.time()
        self.last_log_time = time.time()

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(f"[{self.robot_id}] Decentralized Agent Started.")

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.has_odom = True

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges or not self.has_odom:
            return
        self.has_scan = True

        angle = msg.angle_min
        fronts, lefts, rights = [], [], []

        for r in msg.ranges:
            if not (math.isnan(r) or math.isinf(r) or r <= 0.05):
                # Sector ranges for collision avoidance
                if -0.35 <= angle <= 0.35:
                    fronts.append(r)
                elif 0.35 < angle <= 1.4:
                    lefts.append(r)
                elif -1.4 <= angle < -0.35:
                    rights.append(r)

                # Raytrace into local occupancy grid
                beam_len = min(r, 6.0)
                end_x = self.x + beam_len * math.cos(self.yaw + angle)
                end_y = self.y + beam_len * math.sin(self.yaw + angle)
                self.raytrace(self.x, self.y, end_x, end_y, is_hit=(r < 6.0))
            angle += msg.angle_increment

        self.min_front = min(fronts) if fronts else 10.0
        self.min_left = min(lefts) if lefts else 10.0
        self.min_right = min(rights) if rights else 10.0

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin) / self.res)
        gy = int((wy - self.origin) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.origin + (gx + 0.5) * self.res
        wy = self.origin + (gy + 0.5) * self.res
        return wx, wy

    def raytrace(self, x0, y0, x1, y1, is_hit):
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)

        # Bresenham's raytrace
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        x, y = gx0, gy0

        while True:
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                if x == gx1 and y == gy1:
                    if is_hit:
                        self.grid[y][x] = 100  # Occupied
                    else:
                        self.grid[y][x] = 0    # Free
                    break
                else:
                    self.grid[y][x] = 0        # Free
            if x == gx1 and y == gy1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def peer_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            sender = payload.get("id")
            if sender == self.robot_id:
                return

            px = payload.get("x", 0.0)
            py = payload.get("y", 0.0)
            dist_to_peer = math.hypot(px - self.x, py - self.y)

            # Check communication loss condition: Out of radio range
            if dist_to_peer > self.comms_range_cutoff:
                # Comms dropped due to distance / walls
                return

            self.peer_data[sender] = {
                "x": px,
                "y": py,
                "goal": payload.get("goal"),
                "time": time.time()
            }
        except Exception:
            pass

    def extract_frontiers(self):
        """Find boundary cells between known free cells and unknown cells"""
        frontiers = []
        for gy in range(1, self.grid_size - 1):
            for gx in range(1, self.grid_size - 1):
                if self.grid[gy][gx] == 0:  # Free cell
                    # Check 8-neighborhood for unknown (-1)
                    has_unknown = False
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            if self.grid[gy + dy][gx + dx] == -1:
                                has_unknown = True
                                break
                        if has_unknown:
                            break
                    if has_unknown:
                        wx, wy = self.grid_to_world(gx, gy)
                        frontiers.append((wx, wy))
        return frontiers

    def select_best_frontier(self, frontiers):
        """Scores frontiers balancing distance, utility, and peer repulsion"""
        if not frontiers:
            return None

        # Sample or cluster candidate frontiers to keep processing fast
        step = max(1, len(frontiers) // 25)
        candidates = frontiers[::step]

        best_score = -float('inf')
        best_f = None
        now = time.time()

        for fx, fy in candidates:
            dist_to_robot = math.hypot(fx - self.x, fy - self.y)
            if dist_to_robot < 0.8:
                continue

            # Base utility: higher score for reachable frontiers
            score = 15.0 - (1.2 * dist_to_robot)

            # Prevent redundant exploration: penalize frontiers near peers' locations or claimed goals
            for peer_id, pinfo in self.peer_data.items():
                # Ignore stale peers (communication loss handling)
                if now - pinfo["time"] > self.comms_timeout:
                    continue

                # Peer distance penalty
                p_dist = math.hypot(fx - pinfo["x"], fy - pinfo["y"])
                if p_dist < 4.0:
                    score -= (4.0 - p_dist) * 4.0

                # Peer claimed goal penalty
                p_goal = pinfo.get("goal")
                if p_goal:
                    pgx, pgy = p_goal
                    g_dist = math.hypot(fx - pgx, fy - pgy)
                    if g_dist < 3.5:
                        score -= 20.0  # Strongly avoid peer's target

            if score > best_score:
                best_score = score
                best_f = (fx, fy)

        return best_f

    def step(self):
        now = time.time()
        if not self.has_odom or not self.has_scan:
            return

        # Periodically select or update exploration target
        frontiers = self.extract_frontiers()
        if self.target_goal is None or math.hypot(self.target_goal[0] - self.x, self.target_goal[1] - self.y) < 1.0:
            self.target_goal = self.select_best_frontier(frontiers)

        # Broadcast state to peers (every 0.5 sec)
        if now - self.last_broadcast_time > 0.5:
            self.last_broadcast_time = now
            pkt = {
                "id": self.robot_id,
                "x": self.x,
                "y": self.y,
                "goal": self.target_goal,
                "time": now
            }
            s = String()
            s.data = json.dumps(pkt)
            self.peer_pub.publish(s)

        # Motion Control with Obstacle Avoidance
        cmd = Twist()
        if self.target_goal is not None:
            gx, gy = self.target_goal
            dx = gx - self.x
            dy = gy - self.y
            dist = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            heading_err = (target_yaw - self.yaw + math.pi) % (2.0 * math.pi) - math.pi

            # Reactive collision avoidance
            if self.min_front < 0.40:
                # Too close to wall/corner: reverse and pivot away
                cmd.linear.x = -0.15
                cmd.angular.z = 0.8 if self.min_left > self.min_right else -0.8
            elif self.min_front < 0.65:
                # Close to obstacle: rotate in place away from obstacle (do not push forward)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.75 if self.min_left > self.min_right else -0.75
            elif abs(heading_err) > 0.5:
                cmd.linear.x = 0.08
                cmd.angular.z = math.copysign(0.7, heading_err)
            else:
                cmd.linear.x = min(0.40, max(0.15, dist * 0.3))
                cmd.angular.z = 1.0 * heading_err
        else:
            # Fallback if no frontiers found: wander safely
            if self.min_front < 0.55:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.7
            else:
                cmd.linear.x = 0.3

        # Stuck detection and auto-recovery
        pos_dist_change = math.hypot(self.x - getattr(self, 'prev_x', self.x), self.y - getattr(self, 'prev_y', self.y))
        if getattr(self, 'stuck_timer', 0) > 0:
            self.stuck_timer -= 1
            cmd.linear.x = -0.18
            cmd.angular.z = 0.75
        elif cmd.linear.x > 0.05 and pos_dist_change < 0.03:
            self.stuck_counter = getattr(self, 'stuck_counter', 0) + 1
            if self.stuck_counter > 20:  # ~2.0 seconds stalled
                self.stuck_timer = 15    # back up and pivot for 1.5s
                self.target_goal = None  # drop blocked target and re-plan
                self.stuck_counter = 0
        else:
            self.stuck_counter = 0

        self.prev_x = self.x
        self.prev_y = self.y

        self.cmd_pub.publish(cmd)

        # Periodic Status Logging
        if now - self.last_log_time >= 1.5:
            self.last_log_time = now
            active_peers = [p for p, d in self.peer_data.items() if now - d['time'] <= self.comms_timeout]
            print(f"[{self.robot_id}] Pose: ({self.x:5.2f}, {self.y:5.2f}) | Active Peers: {active_peers} | Frontiers: {len(frontiers)} | Target: {self.target_goal}")


def main():
    parser = argparse.ArgumentParser(description="Decentralized Multi-Robot Exploration Agent")
    parser.add_argument('--robot', type=str, default='robot1', help='robot identifier (robot1, robot2, robot3, or all)')
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    if parsed_args.robot == 'all':
        from rclpy.executors import MultiThreadedExecutor
        exec_mt = MultiThreadedExecutor()
        n1 = DecentralizedExplorerAgent('robot1')
        n2 = DecentralizedExplorerAgent('robot2')
        n3 = DecentralizedExplorerAgent('robot3')
        exec_mt.add_node(n1)
        exec_mt.add_node(n2)
        exec_mt.add_node(n3)
        try:
            exec_mt.spin()
        except KeyboardInterrupt:
            pass
        finally:
            n1.destroy_node()
            n2.destroy_node()
            n3.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
        node = DecentralizedExplorerAgent(parsed_args.robot)
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
