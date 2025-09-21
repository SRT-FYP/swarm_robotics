import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable

def generate_launch_description():
    world = os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'turtlebot3_world.world'
            ) 
    
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    declare_robots_arg = DeclareLaunchArgument(
        'robots',
        default_value=TextSubstitution(
            text="robot1={x: 1.5, y: 1.0, yaw: 1.5707}; robot2={x: 0.8, y: 1.0, yaw: 1.5707}"
        ),
        description='Multi-robot pose configurations'
    )
    
    multi_nav2=IncludeLaunchDescription(os.path.join(get_package_share_directory('nav2_bringup'),'launch','unique_multi_tb3_simulation_launch.py'))
    cloned_nav2=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),'launch','cloned_multi_tb3_simulation_launch.py'))
    cloned_nav2_launch=IncludeLaunchDescription(
        cloned_nav2,
        launch_arguments={'robots': LaunchConfiguration('robots')}.items()
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={'namespace': 'leader',
                          'use_sim_time': 'True',
                          'robot_name': 'waffle_leader',
                          'robot_sdf': os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'),
                          'x_pose': '-1.0',
                          'y_pose': '-0.5',
                          'z_pose': '0.01',
                          'roll': '0.0',
                          'pitch': '0.0',
                          'yaw': '0.0'}.items()
                          )

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':['-r -v -v4 ', world],'on_exit_shutdown':'true'}.items())

    ld =LaunchDescription()

    ld.add_action(multi_nav2)
    # ld.add_action(declare_robots_arg)
    # ld.add_action(cloned_nav2_launch)
    # ld.add_action(gz_robot)
    ld.add_action(gazeboLaunch)

    return ld

