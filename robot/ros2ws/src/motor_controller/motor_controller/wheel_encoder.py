#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gpiozero import (
    Button,
)
from std_msgs.msg import String, Bool
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class WheelEncoder(Node):
    def __init__(self):
        super().__init__('wheel_encoder')
        self.prev_time = self.get_clock().now()
        self.left_encoder = Button(5, bounce_time=0.005)
        self.right_encoder = Button(6, bounce_time=0.005)

        self.motor_left_backward_flag = False
        self.motor_right_backward_flag = False

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.ticks_per_rev = 20
        self.wheel_radius = 0.03
        self.wheel_separation = 0.16

        self.left_encoder.when_pressed = self.count_left
        self.right_encoder.when_pressed = self.count_right

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # self.tick_pub = self.create_publisher(String, 'encoder_ticks', 10)

        self.subscribe_motor_left_backward_flag = self.create_subscription(Bool, 'motor_left_backward_flag', self.motor_left_backward_callback, 10)
        self.subscribe_motor_right_backward_flag = self.create_subscription(Bool, 'motor_right_backward_flag', self.motor_right_backward_callback, 10)

        self.timer = self.create_timer(0.01, self.publish_odometry)  # 0.01 seconds = 10 ms

    def motor_left_backward_callback(self, msg):
        self.motor_left_backward_flag = msg.data

    def motor_right_backward_callback(self, msg):
        self.motor_right_backward_flag = msg.data

    def count_left(self):
        self.left_ticks += 1
        # self.publish_odometry()
        # self.publish_ticks()

    def count_right(self):
        self.right_ticks += 1
        # self.publish_odometry()
        # self.publish_ticks()

    def publish_odometry(self):
        # Calculate distance per tick
        distance_per_tick = 2 * math.pi * self.wheel_radius / self.ticks_per_rev

        if(self.motor_left_backward_flag):
            leftSign=-1
        else:
            leftSign=1

        if(self.motor_right_backward_flag):
            rightSign=-1
        else:
            rightSign=1
        

        # Calculate wheel displacements
        delta_left = (self.left_ticks - self.prev_left_ticks) * distance_per_tick * leftSign
        delta_right = (self.right_ticks - self.prev_right_ticks) * distance_per_tick * rightSign

        # Save current ticks for next update
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # Calculate robot movement
        delta_s = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_separation

        # Time since last update
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # seconds
        self.prev_time = current_time

        # Avoid division by zero
        if dt == 0:
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity = delta_s / dt
            angular_velocity = delta_theta / dt

        # Update pose
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)

        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Fill Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set velocities
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        # Also publish the TF transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.odom_pub.publish(odom_msg)
        self.tf_broadcaster.sendTransform(t)

    def publish_ticks(self):
        ticks_msg = String()
        ticks_msg.data = f"Left: {self.left_ticks}, Right: {self.right_ticks}"
        self.tick_pub.publish(ticks_msg)

    def destroy_node(self):
        self.left_encoder.close()
        self.right_encoder.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()