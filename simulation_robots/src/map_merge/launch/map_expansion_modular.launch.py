import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    
    use_sim_time = LaunchConfiguration("use_sim_time")
   
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

   
    node = Node(
        package="map_merge",
        name="map_expand",
        executable="map_expansion_modular",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(node)
    return ld
