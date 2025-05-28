from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    explore_config = os.path.join(
        get_package_share_directory('hardware_software'),
        'params',
        'explore_params.yaml'
    )

    hardware_software_pkg = get_package_share_directory('hardware_software')
    nav2_params= os.path.join(hardware_software_pkg, 'params', 'nav2_params.yaml')

    min_frontier_size = 0.2

    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    slam_params_file = os.path.join(hardware_software_pkg, 'params', 'slam.yaml')
    
    slam=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': slam_params_file,
                }.items(),
            )

    nav2_launch=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # os.path.join(nav2_bringup_dir, 'launch',"navigation_launch.py")
                    os.path.join(hardware_software_pkg, 'launch',"navigation_launch.py")
                ),
                launch_arguments={
                    'use_sim_time': 'False',
                    'autostart': 'True',
                    'params_file': nav2_params,
                    'use_composition': 'False',
                    'use_respawn': 'True',
                }.items(),
            )
    
    hardware_setup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'hardware_setup.launch.py')),
            )

    explore_node = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[explore_config, {"use_sim_time": False}, {"min_frontier_size": min_frontier_size}],
        output="screen",
    )

    ld=LaunchDescription()
    ld.add_action(hardware_setup)
    ld.add_action(slam)
    ld.add_action(nav2_launch)
    ld.add_action(explore_node)
    return ld