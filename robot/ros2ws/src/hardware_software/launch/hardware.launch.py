import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    yd_lidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    
    motor_control=Node(
        package='motor_controller',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    wheel_encoder=Node(
        package='motor_controller',
        # executable='wheel_encoder_for_fusion',
        executable='wheel_encoder',
        name='wheel_encoder',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    ydlidar=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(yd_lidar_pkg, 'launch', 'ydlidar_launch.py')),
            )

    # rplidar=Node(
    #         package='rplidar_ros',
    #         executable='rplidar_composition',
    #         output='screen',
    #         parameters=[{
    #             'serial_port': '/dev/ttyUSB0',
    #             'frame_id': 'base_scan',
    #             'angle_compensate': True,
    #             'scan_mode': 'Standard'
    #         }]
    #     )

    imu = Node(
        package='imu_reader',
        executable='imu_reader_node',
        name='imu_reader',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()
    ld.add_action(motor_control)
    ld.add_action(wheel_encoder)
    ld.add_action(ydlidar)
    # ld.add_action(rplidar)
    # ld.add_action(imu)
    return ld

