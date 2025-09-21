import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    explore_config = os.path.join(
        get_package_share_directory('hardware_software'),
        'params',
        'explore_params.yaml'
    )

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

    nav2_params= os.path.join(hardware_software_pkg, 'params', 'temp_nav2_params_namespaced.yaml')    

    if(lidar_type == "rplidar"):
        # For RPLIDAR, we set a smaller minimum frontier size, it has a smaller range
        min_frontier_size = 0.01
    else:
        # For YDLIDAR, we set a larger minimum frontier size, it has a larger range
        min_frontier_size = 0.1

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
    
    hardware_setup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'hardware_setup.launch.py')),
                launch_arguments={'namespace': namespace, 'lidar_type': lidar_type}.items()
            )

    explore_node = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[explore_config, {"use_sim_time": False}, {"min_frontier_size": min_frontier_size}],
        output="screen",
        remappings = [("/tf", "tf"), ("/tf_static", "tf_static")],
        namespace=namespace,
    )

    ld=LaunchDescription()
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_lidar_type_argument)
    ld.add_action(hardware_setup)
    ld.add_action(slam)
    ld.add_action(nav2_launch)
    ld.add_action(explore_node)
    return ld