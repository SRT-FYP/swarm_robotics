import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')
    robot_localization_file_path = os.path.join(hardware_software_pkg, 'params', 'ekf_filter.yaml')

    tf_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'tf.launch.py')),
            )

    hardware_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'hardware.launch.py')),
            )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': False}],
        remappings=[
            ('odometry/filtered', 'odom'),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(hardware_launch)
    ld.add_action(tf_launch)
    # ld.add_action(robot_localization)
    
    return ld