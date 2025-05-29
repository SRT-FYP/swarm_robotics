import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')
    
    tf_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'tf.launch.py')),
            )

    hardware_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'hardware.launch.py')),
            )

    ld = LaunchDescription()
    ld.add_action(hardware_launch)
    ld.add_action(tf_launch)
    
    return ld