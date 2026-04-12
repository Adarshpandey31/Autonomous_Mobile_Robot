import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import serial
import threading
import time

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Serial setup — match Arduino baud
        self.ser = serial.Serial(port, baudrate, timeout=0.01)

        # ROS pubs/subs
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Background reader
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    # ------------------ SEND VELOCITY ------------------
    def cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        line = f"{vx:.3f} {wz:.3f}\n"
        try:
            self.ser.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ------------------ READ IMU ------------------
    def read_loop(self):
        self.get_logger().info("Starting read loop...")
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                # self.get_logger().info(f"Received line: {line}")
            except Exception:
                continue
            if not line:
                continue
            # Expected: t,<ms>,quat,<w,x,y,z>,gyro,<gx,gy,gz>,acc,<ax,ay,az>
            try:
                tokens = line.split(',')
                if len(tokens) < 15:
                    continue

                t_idx = tokens.index('t')
                q_idx = tokens.index('quat')
                g_idx = tokens.index('gyro')
                a_idx = tokens.index('acc')

                # Extract floats
                qw, qx, qy, qz = map(float, tokens[q_idx+1:q_idx+5])
                gx, gy, gz = map(float, tokens[g_idx+1:g_idx+4])
                ax, ay, az = map(float, tokens[a_idx+1:a_idx+4])

                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.orientation.w = qw
                msg.orientation.x = qx
                msg.orientation.y = qy
                msg.orientation.z = qz
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az

                self.imu_pub.publish(msg)
                # self.get_logger().info(f"Published IMU: {msg}")
            except Exception as e:
                self.get_logger().warn(f"Bad IMU line: {line} ({e})")

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
