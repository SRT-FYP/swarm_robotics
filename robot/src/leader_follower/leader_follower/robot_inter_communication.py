#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Order(Node):

    def __init__(self):
        super().__init__("order")
        self.cmd_vel_pub_=self.create_publisher(Twist, "/follower/cmd_vel", 10) #selecting message type to be published, the topic to publish to, and the message capacity
        self.timer =self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("draw circle node has started")

    def send_velocity_command(self):
        msg= Twist()
        msg.linear.x=2.0
        msg.angular.z=1.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Order()
    rclpy.spin(node)
    rclpy.shutdown()