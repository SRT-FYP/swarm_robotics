import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    leader_follower_pkg = get_package_share_directory('leader_follower')  
    rviz_config_file = os.path.join(leader_follower_pkg, 'config', 'nav2_default_view.rviz')

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