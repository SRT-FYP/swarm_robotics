from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

    package_dir = get_package_share_directory('hardware_software')
    robot_description_config = xacro.process_file(
        os.path.join(package_dir, 'urdf', 'turtlebot3_burger.urdf'),)

    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': False, 'robot_description': robot_description_config.toxml()}
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld