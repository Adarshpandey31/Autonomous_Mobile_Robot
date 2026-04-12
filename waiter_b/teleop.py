import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)

        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.last_cmd = Twist()
        self.timer = self.create_timer(0.1, self.publish_last)

        self.loop()
        
    def publish_last(self):
        self.cmd_vel_pub.publish(self.last_cmd)
    
    def loop(self):
        self.get_logger().info("Teleop Node Started. Use WASD keys to move, QE to rotate, X to stop, Ctrl+C to exit.")
        try:
            while rclpy.ok():
                key = input("Enter command: ").lower()
                twist = Twist()

                if key == 'w':
                    twist.linear.x += self.linear_speed
                elif key == 's':
                    twist.linear.x += -self.linear_speed
                elif key == 'a':
                    twist.linear.y += self.linear_speed
                elif key == 'd':
                    twist.linear.y += -self.linear_speed
                elif key == 'q':
                    twist.angular.z += self.angular_speed
                elif key == 'e':
                    twist.angular.z += -self.angular_speed
                elif key == 'x':
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                else:
                    self.get_logger().info("Invalid key. Use WASD for movement, QE for rotation, X to stop.")
                    continue

                self.cmd_vel_pub.publish(twist)
                self.last_cmd = twist
                self.get_logger().info(f"Published: {twist}")
        except KeyboardInterrupt:
            self.get_logger().info("Teleop Node Stopped.")
            
def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()