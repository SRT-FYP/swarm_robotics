import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    map_merge_pkg = get_package_share_directory('map_merge')
    hardware_software_pkg = get_package_share_directory('hardware_software')

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

    declare_map_merger_flag= DeclareLaunchArgument(
        "map_merger_robot",
        default_value="false",
        description="Assign the corresponding the robot the role of merging the individual maps into a single whole map",
    )


    namespace = LaunchConfiguration("namespace")
    lidar_type = LaunchConfiguration("lidar_type")
    map_merger_robot = LaunchConfiguration("map_merger_robot")

    launch_map_expansion = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_merge_pkg, 'launch', 'map_expansion.launch.py')),
        )
    
    launch_map_merger = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_merge_pkg, 'launch', 'map_merge.launch.py')),
        )

    map_merger_group = GroupAction(
        actions=[
            launch_map_expansion,
            launch_map_merger,
        ],
        condition=IfCondition(map_merger_robot)
    )

    robot_exploration = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(hardware_software_pkg, 'launch', 'robot_explore.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'lidar_type': lidar_type,
            }.items()
        )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_lidar_type_argument)
    ld.add_action(declare_map_merger_flag)
    ld.add_action(robot_exploration)
    ld.add_action(map_merger_group)
    return ld
