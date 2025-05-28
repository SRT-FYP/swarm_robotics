import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    nav2_tb3=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),'launch','tb3_simulation_launch.py')),
        launch_arguments={'slam': 'True'}.items()
    )

    exploration=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('explore_lite'),'launch','explore.launch.py')),
    )

    return LaunchDescription(
        [
            nav2_tb3,
            exploration,
        ]
    )