import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    robot_controllers_param=os.path.join(
        get_package_share_directory('my_robot_control'),
        'config',
        'diffbot_controller.yaml'
    )
    
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [get_package_share_directory('my_robot_control'), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

    diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diffbot_controller",
                "--param-file",
                robot_controllers_param,
                "--controller-ros-args",
                "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",],
        )

    # Delete the Gazebo stuff and replace it with this
    controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_controllers_param],
            output="both",
        )
    

    ld= LaunchDescription()
    ld.add_action(robot_state_pub_node)
    ld.add_action(diff_drive_spawner)
    ld.add_action(controller_manager)

    return ld