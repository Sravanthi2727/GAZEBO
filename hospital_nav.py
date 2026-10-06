#!/usr/bin/env python3
"""
Task 6 — Hospital delivery with a known map.
Uses:
  * occupancy grid (static hospital map)
  * odometry as localization
  * A* global path planning
  * LiDAR to react to people/objects that are not on the map, then replan

Run AFTER Gazebo and the ROS-Gazebo bridge are up.
"""
import heapq
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


class OccupancyMap:
    """Matches hospital_nav.sdf walls. 0 = free, 1 = occupied."""

    def __init__(self):
        self.origin_x = -1.0
        self.origin_y = -3.0
        self.res = 0.25
        self.width = 56
        self.height = 40
        self.static = [[0] * self.width for _ in range(self.height)]
        self.dynamic = [[0] * self.width for _ in range(self.height)]
        walls = [
            (6.0, -1.7, 12.2, 0.20),
            (-0.1, 0.0, 0.20, 3.6),
            (12.1, 0.0, 0.20, 3.6),
            (3.6, 1.7, 7.4, 0.20),
            (11.05, 1.7, 2.1, 0.20),
            (7.3, 3.7, 0.20, 4.0),
            (10.0, 3.7, 0.20, 4.0),
            (8.65, 5.7, 2.9, 0.20),
        ]
        for cx, cy, sx, sy in walls:
            self._stamp_rect(self.static, cx, cy, sx, sy)
        self._inflate(self.static, cells=2)

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.res)
        gy = int((y - self.origin_y) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.origin_x + (gx + 0.5) * self.res
        y = self.origin_y + (gy + 0.5) * self.res
        return x, y

    def _stamp_rect(self, grid, cx, cy, sx, sy):
        x0, x1 = cx - sx / 2.0, cx + sx / 2.0
        y0, y1 = cy - sy / 2.0, cy + sy / 2.0
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)
        for gy in range(min(gy0, gy1), max(gy0, gy1) + 1):
            for gx in range(min(gx0, gx1), max(gx0, gx1) + 1):
                if self._in_bounds(gx, gy):
                    grid[gy][gx] = 1

    def _inflate(self, grid, cells):
        occ = [(x, y) for y in range(self.height) for x in range(self.width) if grid[y][x] == 1]
        for gx, gy in occ:
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    nx, ny = gx + dx, gy + dy
                    if self._in_bounds(nx, ny):
                        grid[ny][nx] = 1

    def blocked(self, gx, gy):
        if not self._in_bounds(gx, gy):
            return True
        return self.static[gy][gx] == 1 or self.dynamic[gy][gx] == 1

    def mark_world_obstacle(self, x, y, cells=2):
        gx, gy = self.world_to_grid(x, y)
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                nx, ny = gx + dx, gy + dy
                if self._in_bounds(nx, ny) and self.static[ny][nx] == 0:
                    self.dynamic[ny][nx] = 1

    def astar(self, start_xy, goal_xy):
        sx, sy = self.world_to_grid(*start_xy)
        gx, gy = self.world_to_grid(*goal_xy)
        if self.blocked(sx, sy):
            for r in range(1, 4):
                found = False
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if not self.blocked(sx + dx, sy + dy):
                            sx, sy = sx + dx, sy + dy
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
        if self.blocked(gx, gy) or self.blocked(sx, sy):
            return []

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        start = (sx, sy)
        goal = (gx, gy)
        openh = [(h(start, goal), 0.0, start)]
        came = {start: None}
        cost = {start: 0.0}
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while openh:
            _, g, cur = heapq.heappop(openh)
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(self.grid_to_world(*cur))
                    cur = came[cur]
                path.reverse()
                return self._simplify(path)
            if g > cost.get(cur, 1e9):
                continue
            cx, cy = cur
            for dx, dy in moves:
                nxt = (cx + dx, cy + dy)
                if self.blocked(*nxt):
                    continue
                step = math.hypot(dx, dy) * self.res
                ng = g + step
                if ng < cost.get(nxt, 1e9):
                    cost[nxt] = ng
                    came[nxt] = cur
                    heapq.heappush(openh, (ng + h(nxt, goal), ng, nxt))
        return []

    def _simplify(self, path):
        if len(path) < 3:
            return path
        out = [path[0]]
        for i in range(1, len(path) - 1):
            ax, ay = out[-1]
            bx, by = path[i]
            cx, cy = path[i + 1]
            # Keep the point if the direction changes.
            if abs((bx - ax) * (cy - by) - (by - ay) * (cx - bx)) > 1e-6:
                out.append(path[i])
        out.append(path[-1])
        return out


class HospitalNavNode(Node):
    def __init__(self):
        super().__init__('hospital_nav_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.create_subscription(Odometry, '/robot/odometry', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/robot/scan', self.scan_cb, qos_profile_sensor_data)

        self.grid = OccupancyMap()
        self.spawn_x = 1.4
        self.spawn_y = 0.0
        self.goal_x = 8.65
        self.goal_y = 4.60
        self.odom0_x = None
        self.odom0_y = None
        self.x = self.spawn_x
        self.y = self.spawn_y
        self.yaw = 0.0
        self.have_odom = False
        self.have_scan = False
        self.arrived = False
        self.last_scan = None
        self.front = 8.0
        self.left = 8.0
        self.right = 8.0

        self.path = []
        self.wp = 0
        self.planned = False
        self.replan_cool = 0
        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info('Task 6: map + A* + LiDAR. Stop on the green patient-room pad.')

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
        self.left = sector_min(msg, 0.60, 1.40)
        self.right = sector_min(msg, -1.40, -0.60)
        self.have_scan = True

    def plan(self, reason):
        self.path = self.grid.astar((self.x, self.y), (self.goal_x, self.goal_y))
        self.wp = 0
        self.planned = True
        self.replan_cool = 15
        n = len(self.path)
        self.get_logger().info(f'{reason}: A* returned {n} waypoints.')
        if n == 0:
            self.get_logger().error('A* failed. Check that start/goal are in free space.')

    def ingest_lidar_obstacles(self):
        if self.last_scan is None:
            return False
        added = False
        fallback = self.last_scan.range_max if self.last_scan.range_max > 0.0 else 8.0
        angle = self.last_scan.angle_min
        for r in self.last_scan.ranges:
            v = valid_range(r, fallback)
            if 0.35 < v < 1.50 and abs(angle) < 0.70:
                wx = self.x + v * math.cos(self.yaw + angle)
                wy = self.y + v * math.sin(self.yaw + angle)
                gx, gy = self.grid.world_to_grid(wx, wy)
                if self.grid._in_bounds(gx, gy) and self.grid.static[gy][gx] == 0:
                    if self.grid.dynamic[gy][gx] == 0:
                        added = True
                    self.grid.mark_world_obstacle(wx, wy, cells=2)
            angle += self.last_scan.angle_increment
        return added

    def current_target(self):
        if not self.path:
            return self.goal_x, self.goal_y
        while self.wp < len(self.path) - 1:
            tx, ty = self.path[self.wp]
            if math.hypot(tx - self.x, ty - self.y) < 0.45:
                self.wp += 1
            else:
                break
        return self.path[min(self.wp, len(self.path) - 1)]

    def navigate(self):
        cmd = Twist()
        if self.arrived:
            self.publish_stop()
            return
        if not self.have_odom or not self.have_scan:
            self.get_logger().info('Waiting for odom + LiDAR (is the bridge running?)', throttle_duration_sec=2.0)
            self.publisher.publish(cmd)
            return

        if self.replan_cool > 0:
            self.replan_cool -= 1

        if not self.planned:
            self.plan('Initial plan from pharmacy')

        dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if dist_goal < 0.75:
            self.arrived = True
            self.get_logger().info('Reached the green patient-room pad. Holding stop.')
            self.publish_stop()
            return
        # Room walls are close to the pad — do not treat them as people to dodge.
        if dist_goal < 1.4:
            heading = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            err = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))
            cmd.linear.x = 0.18
            cmd.angular.z = max(-0.8, min(0.8, 1.4 * err))
            self.publisher.publish(cmd)
            return

        if self.front < 1.00 and self.replan_cool == 0:
            added = self.ingest_lidar_obstacles()
            if added:
                self.plan('LiDAR saw an unmapped obstacle; replanning')
            cmd.linear.x = 0.08
            cmd.angular.z = 0.9 if self.left > self.right else -0.9
            self.get_logger().info(
                f'Local avoid. front={self.front:.2f} pose=({self.x:.2f},{self.y:.2f})',
                throttle_duration_sec=0.6,
            )
            self.publisher.publish(cmd)
            return

        tx, ty = self.current_target()
        heading = math.atan2(ty - self.y, tx - self.x)
        err = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))
        if abs(err) > 0.40:
            cmd.linear.x = 0.10
            cmd.angular.z = 1.3 * err
        else:
            cmd.linear.x = 0.50
            cmd.angular.z = 1.1 * err
        self.get_logger().info(
            f'Following map path. goal_d={dist_goal:.2f} wp={self.wp}/{max(len(self.path)-1,0)}',
            throttle_duration_sec=1.5,
        )
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HospitalNavNode()
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
