from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
from launch.substitutions import LaunchConfiguration
import xacro
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    online_async_dir = os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
    map_merge_pkg = get_package_share_directory('map_merge')  

    leader_rviz_config= os.path.join(
        get_package_share_directory('leader_follower'),
        'config',
        'leader_rviz.rviz'
    )
    follower_rviz_config= os.path.join(
        get_package_share_directory('leader_follower'),
        'config',
        'follower_rviz.rviz'
    )
    merged_map_rviz_config= os.path.join(
        get_package_share_directory('leader_follower'),
        'config',
        'merged_map.rviz'
    )
        
    slam_params_file=(os.path.join
            (get_package_share_directory('my_turtlebot3_fake_node'), 'config', 'slam.yaml')
        )
    
    remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/scan', 'scan'),
                    ('/odom', 'odom'),
                    ('/map', 'map'),
                    ('/map_metadata', 'map_metadata'),
                    ('/slam_toolbox/scan_visualization', 'slam_toolbox/scan_visualization'),
                    ('/slam_toolbox/graph_visualization', 'slam_toolbox/graph_visualization'),
                ]

    # launch_follower_slam_async=IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(online_async_dir),
    #         launch_arguments={
    #             'use_sim_time': 'true',
    #         }.items()
    #     )

    
    launch_follower_slam_toolbox = GroupAction(
        [
            PushRosNamespace(namespace='follower'),

            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/slam_toolbox/scan_visualization', dst='slam_toolbox/scan_visualization'),
            SetRemap(src='/slam_toolbox/graph_visualization', dst='slam_toolbox/graph_visualization'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(online_async_dir),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_params_file,
                }.items(),
            )
        ]
     )
    
    launch_leader_slam_toolbox = GroupAction(
        [
            PushRosNamespace(namespace='leader'),

            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/slam_toolbox/scan_visualization', dst='slam_toolbox/scan_visualization'),
            SetRemap(src='/slam_toolbox/graph_visualization', dst='slam_toolbox/graph_visualization'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(online_async_dir),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_params_file,
                }.items(),
            )
        ]
     )
    
    launch_multi_robots=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('leader_follower'), 'launch', 'robots_in_world.launch.py')),
        )
    
    launch_leader_rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', leader_rviz_config],
        remappings=[
                ('/tf', '/leader/tf'),
                ('/tf_static', '/leader/tf_static')
            ]
    )
    launch_follower_rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', follower_rviz_config],
        remappings=[
                ('/tf', '/follower/tf'),
                ('/tf_static', '/follower/tf_static')
            ]
    )
    launch_merged_map_rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', merged_map_rviz_config],
        remappings=[
                ('/tf', '/follower/tf'),
                ('/tf_static', '/follower/tf_static')
            ]
    )

    launch_map_expansion = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_merge_pkg, 'launch', 'map_expansion.launch.py')),
        )
    
    launch_map_merger = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_merge_pkg, 'launch', 'map_merge.launch.py')),
        )
    
    ld=LaunchDescription()
    
    ld.add_action(launch_multi_robots)
    ld.add_action(launch_leader_slam_toolbox)
    ld.add_action(launch_follower_slam_toolbox)
    ld.add_action(launch_leader_rviz)
    ld.add_action(launch_follower_rviz)
    ld.add_action(launch_merged_map_rviz)
    ld.add_action(launch_map_expansion)
    ld.add_action(launch_map_merger)

    return ld
