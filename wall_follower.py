import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/robot/scan', self.scan_callback, 10)
        
        self.target_wall_distance = 1.0  
        
        self.cmd = Twist()
        self.timer = self.create_timer(0.1, self.navigate)
        
        self.distance_front = 10.0
        self.distance_right = 10.0
        self.previous_error = 0.0

    def scan_callback(self, msg):
        # FAILSAFE: If the bridge sends empty or incomplete data during startup, ignore it!
        if not msg.ranges or len(msg.ranges) < 200:
            return
            
        ranges = [r if not math.isinf(r) else 10.0 for r in msg.ranges]
        
        self.distance_front = min(ranges[160:200])
        self.distance_right = min(ranges[70:110])

    def navigate(self):
        if self.distance_front < 1.2:
            self.get_logger().info('⚠️️ Corner detected! Turning left...', throttle_duration_sec=0.5)
            self.cmd.linear.x = 0.2   
            self.cmd.angular.z = 1.2  
            
        else:
            self.cmd.linear.x = 0.6  
            
            error = self.target_wall_distance - self.distance_right
            error_derivative = error - self.previous_error
            
            Kp = 1.2  
            Kd = 6.0  
            
            self.cmd.angular.z = (Kp * error) + (Kd * error_derivative)
            self.previous_error = error
            
            self.cmd.angular.z = max(min(self.cmd.angular.z, 0.8), -0.8)
            
            self.get_logger().info(f'Tracking wall smoothly... Distance: {self.distance_right:.2f}m', throttle_duration_sec=1.0)
            
        self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()