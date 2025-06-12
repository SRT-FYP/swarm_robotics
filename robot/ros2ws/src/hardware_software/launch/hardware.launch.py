import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import Node, SetRemap, PushRosNamespace


def generate_launch_description():

    yd_lidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    
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

    motor_control=Node(
        package='motor_controller',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[{'use_sim_time': False}],
        namespace=namespace,
    )
    wheel_encoder=Node(
        package='motor_controller',
        # executable='wheel_encoder_for_fusion',
        executable='wheel_encoder',
        name='wheel_encoder',
        output='screen',
        parameters=[{'use_sim_time': False}],
        namespace=namespace,
        remappings=[('/tf', 'tf')]
    )

    ydlidar=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(yd_lidar_pkg, 'launch', 'ydlidar_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': 'false',
                }.items(),
                condition=IfCondition(PythonExpression(['"', lidar_type, '" == "ydlidar"'])),
            )

    rplidar = GroupAction(
        actions=[
            Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                name='rplidar_composition',
                output='screen',
                parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'frame_id': 'base_scan',
                    'angle_compensate': True,
                    'scan_mode': 'Standard'
                }],
                namespace=namespace,
            )
        ],
        condition=IfCondition(PythonExpression(['"', lidar_type, '" == "rplidar"']))
    )

    imu = Node(
        package='imu_reader',
        executable='imu_reader_node',
        name='imu_reader',
        output='screen',
        parameters=[{'use_sim_time': False}],
        namespace=namespace,
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_lidar_type_argument)
    ld.add_action(motor_control)
    ld.add_action(wheel_encoder)
    ld.add_action(ydlidar)
    ld.add_action(rplidar)
    # ld.add_action(imu)
    return ld

