import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/robot/odometry', self.odom_callback, 10)
        
        # Hospital Ward Destination
        self.goal_x = 8.0
        self.goal_y = 0.0
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

        distance = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        if distance < 0.35:
            self.arrived = True
            self.get_logger().info('Arrived at Hospital Ward. Holding stop.')
            self.publisher.publish(msg)
            return 
        # Calculate angle to goal
        target_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # PROPORTIONAL CONTROLLER (Fixes the pendulum wobble)
        if abs(angle_error) > 0.05:
            msg.linear.x = 0.5  # Move a bit slower while turning
            msg.angular.z = 1.2 * angle_error # Smooth steering
        else:
            msg.linear.x = 1.0  # Drive fast when facing straight
            msg.angular.z = 0.0
            
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publisher.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()