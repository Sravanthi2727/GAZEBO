import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/robot/odometry', self.odom_callback, 10)
        
        # PREDEFINED PATH
        self.waypoints = [
            (3.0, -2.5),  # Waypoint 1: Drive safely below Rack 1
            (6.5, 3.0),   # Waypoint 2: Drive safely above Rack 2
            (8.0, 3.0)    # Waypoint 3: The Unloading Zone
        ]
        self.current_wp_index = 0
        self.arrived = False
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.timer = self.create_timer(0.1, self.navigate)

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def navigate(self):
        msg = Twist()
        
        if self.arrived:
            self.publisher.publish(msg)
            return

        if self.current_wp_index >= len(self.waypoints):
            self.arrived = True
            self.get_logger().info('Reached Unloading Zone. Holding stop.')
            self.publisher.publish(msg)
            return 

        goal_x, goal_y = self.waypoints[self.current_wp_index]
        distance = math.sqrt((goal_x - self.current_x)**2 + (goal_y - self.current_y)**2)
        
        # Increased radius to 0.5 to prevent circling
        if distance < 0.5:
            self.get_logger().info(f'📍 Reached Waypoint {self.current_wp_index + 1}')
            self.current_wp_index += 1
            return 
            
        target_angle = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
        angle_error = target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error)) 
        
        # TURN THEN DRIVE CONTROLLER
        if abs(angle_error) > 0.15:
            # If the robot is not facing the waypoint, STOP and turn in place
            msg.linear.x = 0.0  
            msg.angular.z = 1.2 * angle_error
        else:
            # Once facing the right way, drive straight
            msg.linear.x = 0.8  
            msg.angular.z = 0.5 * angle_error # Tiny corrections to stay straight
            
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publisher.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()