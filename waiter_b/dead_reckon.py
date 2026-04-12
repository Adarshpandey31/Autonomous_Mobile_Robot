#!/usr/bin/env python3
#
# Calibrated odometry generator for a robot with no wheel encoders.
# Integrates /cmd_vel using measured real-world velocity scaling.
#
# Publishes:
#   /odom   (nav_msgs/Odometry)
# Broadcasts:
#   odom -> base_link  (tf2)
#

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

import tf_transformations
import tf2_ros

import math


class CalibratedDeadReckonOdom(Node):
    def __init__(self):
        super().__init__("calibrated_dead_reckon")

        # ===== Parameters =====
        self.declare_parameter("linear_scale", 0.67141)
        self.declare_parameter("angular_scale", 0.53047)
        self.declare_parameter("linear_bias", 0.0)
        self.declare_parameter("angular_bias", 0.0)
        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.linear_bias = float(self.get_parameter("linear_bias").value)
        self.angular_bias = float(self.get_parameter("angular_bias").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # ===== Internal state =====
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Last commanded velocities
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0  # for holonomic later
        self.cmd_wz = 0.0

        # ===== Publishers / Subscribers =====
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ===== Timer for integration =====
        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self.update)

        self.get_logger().info(
            f"Calibrated odom running: lin_scale={self.linear_scale:.5f}, "
            f"ang_scale={self.angular_scale:.5f}, rate={self.publish_rate}Hz"
        )

    # ========== CALLBACK: Receive command velocities ==========
    def cmd_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y   # not integrated unless needed
        self.cmd_wz = msg.angular.z

    # ========== MAIN UPDATE LOOP ==========
    def update(self):
        dt = 1.0 / self.publish_rate

        # Apply calibrations
        vx = self.linear_scale * self.cmd_vx + self.linear_bias
        wz = self.angular_scale * self.cmd_wz + self.angular_bias

        # Integrate odometry
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt
        self.theta += wz * dt

        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Quaternion for yaw
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)

        # ===== Publish TF =====
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # self.tf_broadcaster.sendTransform(t)

        # ===== Publish Odometry message =====
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.angular.z = float(wz)

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CalibratedDeadReckonOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
