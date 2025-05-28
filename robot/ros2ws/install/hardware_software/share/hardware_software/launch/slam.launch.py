import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    slam_params_file = os.path.join(hardware_software_pkg, 'params', 'slam.yaml')
    

    slam=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': slam_params_file,
                }.items(),
            )
    hardware_setup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'hardware_setup.launch.py')),
            )

    ld = LaunchDescription()
    ld.add_action(hardware_setup)
    ld.add_action(slam)
    return ld