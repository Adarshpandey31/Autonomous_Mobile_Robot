#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SimpleScanFilter(Node):
    def __init__(self):
        super().__init__('simple_scan_filter')

        # Parameters
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('lower_threshold', 0.25)
        self.declare_parameter('upper_threshold', 8.0)

        # Angle limits for front 180°
        self.declare_parameter('front_min_angle_deg', -90.0)
        self.declare_parameter('front_max_angle_deg',  90.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.lower_thresh = self.get_parameter('lower_threshold').value
        self.upper_thresh = self.get_parameter('upper_threshold').value

        self.front_min = math.radians(
            self.get_parameter('front_min_angle_deg').value
        )
        self.front_max = math.radians(
            self.get_parameter('front_max_angle_deg').value
        )

        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, 10)
        self.publisher = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(
            f"Filtering {input_topic} → {output_topic}, "
            f"range [{self.lower_thresh},{self.upper_thresh}] m, "
            f"angles [{self.front_min},{self.front_max}] rad"
        )

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header = msg.header

        # Copy same metadata
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        filtered_ranges = []

        angle = msg.angle_min
        for r in msg.ranges:

            # FRONT ARC FILTERING (180 degrees)
            if self.front_min <= angle <= self.front_max:
                filtered_ranges.append(float('inf'))
            else:
                # RANGE FILTERING
                if self.lower_thresh <= r <= self.upper_thresh:
                    filtered_ranges.append(r)
                else:
                    filtered_ranges.append(float('inf'))

            angle += msg.angle_increment

        filtered.ranges = filtered_ranges
        filtered.intensities = msg.intensities

        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
