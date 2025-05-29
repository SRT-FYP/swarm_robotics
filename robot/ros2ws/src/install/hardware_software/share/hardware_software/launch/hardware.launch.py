import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    motor_control=Node(
        package='motor_controller',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    wheel_encoder=Node(
        package='motor_controller',
        executable='wheel_encoder',
        name='wheel_encoder',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    rplidar=Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'base_scan',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )

    ld=LaunchDescription()
    ld.add_action(motor_control)
    ld.add_action(wheel_encoder)
    ld.add_action(rplidar)
    return ld

