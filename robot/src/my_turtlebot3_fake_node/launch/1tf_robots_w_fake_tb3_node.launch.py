# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Ryan Shim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
from launch.actions import AppendEnvironmentVariable
from pathlib import Path
import xacro



def generate_launch_description():
    # ---------------------------setting up variables------------------------------------------
    package_dir=get_package_share_directory('leader_follower')
    node_dir=get_package_share_directory('my_turtlebot3_fake_node')
    bridge_params=os.path.join(node_dir,'params','turtlebot3_waffle_bridge_absolute_tf.yaml')

    param_dir = os.path.join(
            node_dir,
            'params',
            'waffle.yaml')

    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_fake_node'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'turtlebot3_waffle.urdf'
    
    robot_description_config = xacro.process_file(
        os.path.join(node_dir, 'urdf', urdf_file_name),
    )
    robot_model = os.path.join(node_dir, 'urdf', 'gz_waffle.sdf.xacro')
    
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

    world = os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'turtlebot3_world.world'
            ) 
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('my_turtlebot3_fake_node'),
        'rviz',
        'default.rviz'
    )

    # ----------------------------------------launch-------------------------------------------------------
    log_msg=LogInfo(msg=['Execute Turtlebot3 Fake Node!!'])

    rviz_launch=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    gazebo_world_launch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':['-r -v -v4 ',world],'on_exit_shutdown':'true'}.items())

    set_env_vars_resources1=AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(package_dir, 'models'))
    
    set_env_vars_resources2=AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(package_dir)).parent.resolve()))

    leader_tb3_node=Node(
        package='my_turtlebot3_fake_node',
        executable='turtlebot3_fake_node',
        parameters=[param_dir,{'robot_odom_pose': [0.0,0.0,0.0],'frame_prefix':'l/','odom_frame_prefix':'l/'}],
        namespace='leader',
        output='screen')


    follower_tb3_node=Node(
        package='my_turtlebot3_fake_node',
        executable='turtlebot3_fake_node',
        namespace='follower',
        parameters=[param_dir,{'robot_odom_pose': [1.0,0.0,0.0],'frame_prefix':'f/','odom_frame_prefix':'l/'}],
        output='screen')

    leader_robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='leader',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_config.toxml(),'frame_prefix':'l/'}],
    )

    follower_robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_config.toxml(),'frame_prefix':'f/'}],
    )

    leader_tb3_gazebo_spawn=Node(
        package='ros_gz_sim',
        executable='create',
        namespace='leader',
        arguments=[
            '-name', 'waffle_leader',
            '-string', Command([
                FindExecutable(name='xacro'),' ','namespace:=','leader',' ','frame_prefix:=','l/',' ','odom_frame_prefix:=','l/',' ', robot_model]),
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.01',
            '-R', '0.0', 
            '-P', '0.0', 
            '-Y', '0.0'
        ],
        output='screen',
        )
    
    follower_tb3_gazebo_spawn=Node(
        package='ros_gz_sim',
        executable='create',
        namespace='follower',
        arguments=[
            '-name', 'waffle_follower',
            '-string', Command([
                FindExecutable(name='xacro'),' ','namespace:=','follower',' ','frame_prefix:=','f/',' ','odom_frame_prefix:=','l/',' ', robot_model]),
            '-x', '-1.0',
            '-y', '-0.5',
            '-z', '0.01',
            '-R', '0.0', 
            '-P', '0.0', 
            '-Y', '0.0'
        ],
        output='screen',
        )

    leader_ros_gz_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='leader',
        parameters=[
            {
                'config_file':bridge_params,
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )


    follower_ros_gz_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='follower',
        parameters=[
            {
                'config_file':bridge_params,
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    leader_ros_gz_image_bridge=Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace='leader',
        arguments=['camera/image_raw'],
        output='screen',
    )

    follower_ros_gz_image_bridge=Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace='follower',
        arguments=['camera/image_raw'],
        output='screen',
    )

    
    
    ld=LaunchDescription()

    ld.add_action(log_msg)
    ld.add_action(rviz_launch)
    ld.add_action(gazebo_world_launch)
    ld.add_action(set_env_vars_resources1)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(leader_tb3_node)
    ld.add_action(follower_tb3_node)
    ld.add_action(leader_robot_state_publisher)
    ld.add_action(follower_robot_state_publisher)
    ld.add_action(leader_tb3_gazebo_spawn)
    ld.add_action(follower_tb3_gazebo_spawn)
    ld.add_action(leader_ros_gz_bridge)
    ld.add_action(follower_ros_gz_bridge)
    ld.add_action(leader_ros_gz_image_bridge)
    ld.add_action(follower_ros_gz_image_bridge)
    
    return ld

