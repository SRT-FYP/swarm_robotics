import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    slam_params_file = os.path.join(hardware_software_pkg, 'params', 'slam.yaml')
    
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for robot",
    )
    declare_lidar_type_argument = DeclareLaunchArgument(
        "lidar_type",
        default_value="rplidar",
        description="Type of LIDAR to use (ydlidar or rplidar)",
    )

    namespace = LaunchConfiguration("namespace")
    lidar_type = LaunchConfiguration("lidar_type")

    slam = GroupAction(
        [
            PushRosNamespace(namespace=namespace),

            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/slam_toolbox/scan_visualization', dst='slam_toolbox/scan_visualization'),
            SetRemap(src='/slam_toolbox/graph_visualization', dst='slam_toolbox/graph_visualization'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': slam_params_file,
                }.items(),
            )
        ]
     )
    hardware_setup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'hardware_setup.launch.py')),
                launch_arguments={'namespace': namespace, 'lidar_type': lidar_type}.items()
            )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_lidar_type_argument)
    ld.add_action(hardware_setup)
    ld.add_action(slam)
    return ld