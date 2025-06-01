import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')
    nav2_params= os.path.join(hardware_software_pkg, 'params', 'nav2_params.yaml')

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

    nav2_launch=GroupAction(
        [
            PushRosNamespace(namespace=namespace),

            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/odom', dst='odom'),
            SetRemap(src='/cmd_vel', dst='cmd_vel'),
            SetRemap(src='/global_plan', dst='global_plan'),
            SetRemap(src='/local_plan', dst='local_plan'),
            SetRemap(src='/trajectories', dst='trajectories'),
            SetRemap(src='/local_costmap/published_footprint', dst='local_costmap/published_footprint'),
            SetRemap(src='/diagnostics', dst='diagnostics'),

            IncludeLaunchDescription(
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
                    'namespace': namespace,
                }.items(),
            ),
        ]
    )
    
    slam_hardware_setup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'slam.launch.py')),
                launch_arguments={'namespace': namespace, 'lidar_type': lidar_type}.items()
            )

    ld=LaunchDescription()
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_lidar_type_argument)
    ld.add_action(slam_hardware_setup)
    ld.add_action(nav2_launch)
    return ld