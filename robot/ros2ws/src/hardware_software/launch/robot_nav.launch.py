from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')
    nav2_params= os.path.join(hardware_software_pkg, 'params', 'nav2_params.yaml')

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
    
    slam_hardware_setup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'slam.launch.py')),
            )

    ld=LaunchDescription()
    ld.add_action(slam_hardware_setup)
    ld.add_action(nav2_launch)
    return ld