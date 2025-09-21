import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')  
    rviz_config_file = os.path.join(hardware_software_pkg, 'rviz', 'basic.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    ld = LaunchDescription()
   
    ld.add_action(rviz)
    return ld