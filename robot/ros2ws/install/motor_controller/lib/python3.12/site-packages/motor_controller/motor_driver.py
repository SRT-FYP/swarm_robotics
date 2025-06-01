#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gpiozero import (
    PWMOutputDevice,
    DigitalOutputDevice,
)

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        self.motor_pwm_left = PWMOutputDevice(12)   # GPIO 12 (PWM)
        self.motor_pwm_right = PWMOutputDevice(13)  # GPIO 13 (PWM)

        self.motor_left_forward = DigitalOutputDevice(22)
        self.motor_left_backward = DigitalOutputDevice(23)
        self.motor_right_forward = DigitalOutputDevice(24)
        self.motor_right_backward = DigitalOutputDevice(25)


        self.wheel_separation = 0.16
        self.wheel_radius = 0.03  # Adjust based on your robot's wheel radius
        self.max_wheel_speed = 15.0
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_left_backward_flag = self.create_publisher(Bool, 'motor_left_backward_flag', 10)
        self.publisher_right_backward_flag = self.create_publisher(Bool, 'motor_right_backward_flag', 10)

        self.get_logger().info('Motor driver node initialized using GPIOZero')

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Simple differential drive calculations
        # Adjust these calculations based on your robot's configuration
        # left_speed = linear_x - angular_z
        # right_speed = linear_x + angular_z
        left_speed = (linear_x - (angular_z * self.wheel_separation*2.3 / 2)) / self.wheel_radius
        right_speed = (linear_x + (angular_z * self.wheel_separation*2.3 / 2)) / self.wheel_radius

        # Clamp speeds to [0,1] range (gpiozero expects values in this range)
        

        if left_speed > 0:
            self.motor_left_forward.on()
            self.motor_left_backward.off()
            self.publisher_left_backward_flag.publish(Bool(data=False))
        elif left_speed < 0:
            self.motor_left_forward.off()
            self.motor_left_backward.on()
            self.publisher_left_backward_flag.publish(Bool(data=True))
        else:
            self.motor_left_forward.off()
            self.motor_left_backward.off()
            self.publisher_left_backward_flag.publish(Bool(data=False))

        if right_speed > 0:
            self.motor_right_forward.on()
            self.motor_right_backward.off()
            self.publisher_right_backward_flag.publish(Bool(data=False))
        elif right_speed < 0:
            self.motor_right_forward.off()
            self.motor_right_backward.on()
            self.publisher_right_backward_flag.publish(Bool(data=True))
        else:
            self.motor_right_forward.off()
            self.motor_right_backward.off()
            self.publisher_right_backward_flag.publish(Bool(data=False))


        left_speed = abs(left_speed) / self.max_wheel_speed
        right_speed = abs(right_speed) / self.max_wheel_speed
        left_speed = max(min(left_speed, 1), 0)
        right_speed = max(min(right_speed, 1), 0)

        if left_speed == right_speed and left_speed != 0 and left_speed != 1:
            # If both speeds are equal and not at max, set them to a default speed
            left_speed = right_speed = 0.5

        self.motor_pwm_left.value = left_speed
        self.motor_pwm_right.value = right_speed

        self.get_logger().debug(f'Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}')

    def destroy_node(self):
        # Stop motors and clean up
        self.motor_left_forward.off()
        self.motor_left_backward.off()
        self.motor_right_forward.off()
        self.motor_right_backward.off()
        self.motor_pwm_left.close()
        self.motor_pwm_right.close()
        self.publisher_left_backward_flag.publish(Bool(data=False))
        self.publisher_right_backward_flag.publish(Bool(data=False))

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriver()
    try:
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()