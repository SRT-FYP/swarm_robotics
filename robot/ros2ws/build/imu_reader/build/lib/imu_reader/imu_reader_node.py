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
            address_mpu_master=MPU9050_ADDRESS_68,
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_1000,
            afs=AFS_8G,
            )
        self.sensor.configureMPU6500(self.sensor.gfs,self.sensor.afs) #using this instead of .configure() since my mpu9250 is not giving magnetometer values, so its basically functioning as a MPU6500

        # self.gx_offset, self.gy_offset, self.gz_offset = self.calibrate_gyro()

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
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

        imu_msg.linear_acceleration.x = float(accel[0])
        imu_msg.linear_acceleration.y = float(accel[1])
        imu_msg.linear_acceleration.z = float(accel[2])

        imu_msg.angular_velocity.x = float(math.radians(gyro[0])) #- self.gx_offset
        imu_msg.angular_velocity.y = float(math.radians(gyro[1])) #- self.gy_offset
        imu_msg.angular_velocity.z = float(math.radians(gyro[2])) #- self.gz_offset

        # Orientation is unknown
        imu_msg.orientation_covariance[0] = -1.0
        # # Magnetometer data
        # mag_msg.header = imu_msg.header
        # mag_msg.magnetic_field.x = mag['x'] * 1e-6  # uT to Tesla
        # mag_msg.magnetic_field.y = mag['y'] * 1e-6
        # mag_msg.magnetic_field.z = mag['z'] * 1e-6
        
        self.imu_pub.publish(imu_msg)
        # self.mag_pub.publish(mag_msg)

    def calibrate_gyro(self, num_samples=1000):
        print("Calibrating gyroscope...")
        print("Please keep the sensor stationary during calibration.")
        
        gyro_data = []
        for _ in range(num_samples):
            gyro_data.append(self.sensor.readGyroscopeMaster())
            time.sleep(0.01)
        
        gyro_data = np.array(gyro_data)
        gx_offset, gy_offset, gz_offset = np.mean(gyro_data, axis=0)
        
        print("Calibration complete.")
        print("Gyroscope offsets: gx_offset={}, gy_offset={}, gz_offset={}".format(gx_offset, gy_offset, gz_offset))
        
        return gx_offset, gy_offset, gz_offset

    def calibrate_accel(self, num_samples=1000):
        print("Calibrating accelerometer...")
        print("Please follow the instructions for each axis.")
        
        accel_offsets = [0, 0, 0]
        axis_labels = ['x', 'y', 'z']
        
        for axis in range(3):
            print("Orient the sensor so that the {} axis is pointed against gravity.".format(axis_labels[axis]))
            input("Press Enter when ready...")
            
            accel_data = []
            for _ in range(num_samples):
                accel_data.append(self.sensor.readAccelerometerMaster()[axis])
                time.sleep(0.01)
            
            accel_offset = np.mean(accel_data)
            accel_offsets[axis] = accel_offset - 1
            
            print("{} axis offset: {}".format(axis_labels[axis], accel_offsets[axis]))
        
        print("Calibration complete.")
        print("Accelerometer offsets: ax_offset={}, ay_offset={}, az_offset={}".format(*accel_offsets))
        
        return accel_offsets


def main(args=None):
    rclpy.init(args=args)
    node = Mpu9250Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()