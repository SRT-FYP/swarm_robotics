import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    hardware_software_pkg = get_package_share_directory('hardware_software')
    robot_localization_file_path = os.path.join(hardware_software_pkg, 'params', 'ekf_filter.yaml')

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

    tf_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'tf.launch.py')),
                launch_arguments={'namespace': namespace}.items()
            )

    hardware_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'hardware.launch.py')),
                launch_arguments={'namespace': namespace, 'lidar_type': lidar_type}.items()
            )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': False}],
        remappings=[
            ('odometry/filtered', 'odom'),('/tf', 'tf'), ('/tf_static', 'tf_static')
        ],
        namespace=namespace,

    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_lidar_type_argument)
    ld.add_action(hardware_launch)
    ld.add_action(tf_launch)
    # ld.add_action(robot_localization)
    
    return ld