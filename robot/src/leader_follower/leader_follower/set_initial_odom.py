#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion


class OdomPub(Node):

    def __init__(self):
        super().__init__("odom_init")
        # self.start_time=self.get_clock().now().to_msg()
        self.odom_pub_=self.create_publisher(Odometry, "/follower/odom", 1) #selecting message type to be published, the topic to publish to, and the message capacity
        self.send_odom_command()
        self.get_logger().info("initiating follower odom frame")

    def send_odom_command(self):
        msg= Odometry()
        msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg.pose.pose.position.x=1.0
        msg.header.frame_id='l_odom'
        msg.child_frame_id='f_base_footprint'
        self.odom_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPub()
    rclpy.spin_once(node)
    rclpy.shutdown()

# class InitialOdomSetter(Node):
#     def __init__(self):
#         super().__init__('initial_odom_setter')

#         # Subscriber to /odom
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/follower/odom',
#             self.odom_callback,
#             10
#         )

#         # Publisher to /odom_corrected
#         self.odom_pub = self.create_publisher(Odometry, '/follower/odom', 10)

#     def odom_callback(self, msg):
#         self.get_logger().info('Setting initial odometry pose.')
#         msg.pose.pose.position.x=1.0
#         msg.header.frame_id='l_odom'
#         msg.child_frame_id='f_base_footprint'

#         self.odom_sub.destroy()
#         self.odom_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = InitialOdomSetter()
#     rclpy.spin_once(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()