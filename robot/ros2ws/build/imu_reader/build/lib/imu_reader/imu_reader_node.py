import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
import math
import numpy as np
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

class Mpu9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_node')
        self.sensor = MPU9250(
            address_mpu_master=MPU9050_ADDRESS_68,  # In case the MPU9250 is connected to another I2C device
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_1000,
            afs=AFS_8G,
            )
        self.sensor.configureMPU6500(self.sensor.gfs,self.sensor.afs)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        # self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50 Hz

    def publish_imu_data(self):
        imu_msg = Imu()
        # mag_msg = MagneticField()

        accel = self.sensor.readAccelerometerMaster()
        gyro = self.sensor.readGyroscopeMaster()
        # mag = self.sensor.readMagnet()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        imu_msg.angular_velocity.x = math.radians(gyro[0])
        imu_msg.angular_velocity.y = math.radians(gyro[1])
        imu_msg.angular_velocity.z = math.radians(gyro[2])

        # Orientation is unknown
        imu_msg.orientation_covariance[0] = -1.0
        # # Magnetometer data
        # mag_msg.header = imu_msg.header
        # mag_msg.magnetic_field.x = mag['x'] * 1e-6  # uT to Tesla
        # mag_msg.magnetic_field.y = mag['y'] * 1e-6
        # mag_msg.magnetic_field.z = mag['z'] * 1e-6
        print(accel)
        print(gyro)
        self.imu_pub.publish(imu_msg)
        # self.mag_pub.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Mpu9250Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()