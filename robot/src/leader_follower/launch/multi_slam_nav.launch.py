import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, GroupAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
import xacro

def generate_launch_description():
    package_dir = get_package_share_directory('leader_follower')
    nav2_bringup_dir=get_package_share_directory('nav2_bringup')
    map_merge_pkg = get_package_share_directory('map_merge')

    params_file = os.path.join(
        get_package_share_directory("leader_follower"),
        "params",
        "nav2_multirobot_params_all.yaml",
    )

    tb3_world = os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'turtlebot3_world.world'
            )
    tb3_house = os.path.join(
                package_dir,
                'worlds',
                'turtlebot3_house_closed.world'
            )
    world = tb3_house

    
    robot_model = os.path.join(package_dir, 'urdf', 'gz_waffle_multi_tf.sdf.xacro')

    robot_description_config = xacro.process_file(
        os.path.join(package_dir, 'urdf', 'turtlebot3_waffle.urdf'),)
    
    bridge_params = os.path.join(
        package_dir,
        'params',
        'turtlebot3_waffle_bridge.yaml'
    )
    slam_params_file=(os.path.join
            (get_package_share_directory('my_turtlebot3_fake_node'), 'config', 'slam.yaml')
        )
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    
    online_async_dir = os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
   
    namespaced_nav2_rviz=os.path.join(
            get_package_share_directory('leader_follower'),
            'config',
            'nav2_default_view.rviz'
        )

    merged_map_rviz_config= os.path.join(
        get_package_share_directory('leader_follower'),
        'config',
        'merged_map.rviz'
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    turtlebot3_world_gazebo=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':['-r -v -v4 ',world],'on_exit_shutdown':'true'}.items())

    with open(params_file, 'r') as f:
        yaml_content = f.read()

    leader_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='leader',
        arguments=[
            '-name', 'waffle_leader',
            '-string', Command([
                FindExecutable(name='xacro'), ' ', 'namespace:=',
                'leader', ' ',  robot_model]),
            '-x', '-1.0',
            '-y', '2.5', #for tb3_house
            # '-y', '-0.5', #for tb3_world
            '-z', '0.01',
            '-R', '0.0', 
            '-P', '0.0', 
            '-Y', '0.0'
        ],
        output='screen',
    )

    follower_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='follower',
        arguments=[
            '-name', 'waffle_follower',
            '-string', Command([
                FindExecutable(name='xacro'), ' ', 'namespace:=',
                'follower', ' ',  robot_model]),
            '-x', '-1.5',
            '-y', '1.5', #for tb3_house
            # '-y', '-0.5', #for tb3_world
            '-z', '0.01',
            '-R', '0.0', 
            '-P', '0.0', 
            '-Y', '0.0'
        ],
        output='screen',
    )

    leader_robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='leader',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description_config.toxml()}
        ],
        remappings=remappings,
    )

    follower_robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='follower',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description_config.toxml()}
        ],
        remappings=remappings,
    )
   
    leader_start_gazebo_ros_bridge = Node(
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

    # leader_gazebo_ros_image_bridge_cmd = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     namespace='leader',
    #     arguments=['camera/image_raw'],
    #     output='screen',
    # )

    follower_start_gazebo_ros_bridge = Node(
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

    # follower_gazebo_ros_image_bridge_cmd = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     namespace='follower',
    #     arguments=['camera/image_raw'],
    #     output='screen',
    # )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(package_dir, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(package_dir)).parent.resolve()))

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
    
    launch_leader_rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', namespaced_nav2_rviz],
        namespace='leader',
        remappings=[
                ('/tf', '/leader/tf'),
                ('/tf_static', '/leader/tf_static')
            ],
        parameters=[
                {'use_sim_time': use_sim_time}
            ],
    )

    launch_follower_rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', namespaced_nav2_rviz],
        namespace='follower',
        remappings=[
                ('/tf', '/follower/tf'),
                ('/tf_static', '/follower/tf_static')
            ],
        parameters=[
                {'use_sim_time': use_sim_time}
            ],
    )

    leader_yaml_final = yaml_content.replace('<robot_namespace>', 'leader')
    
    leader_rewritten_yaml_path = os.path.join(package_dir,'/tmp/nav2_params_rewritten_leader.yaml')
    with open(leader_rewritten_yaml_path, 'w') as f:
        f.write(leader_yaml_final)

    launch_leader_nav2=GroupAction(
        [
            PushRosNamespace(namespace='leader'),

            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/odom', dst='odom'),
            SetRemap(src='/cmd_vel', dst='cmd_vel'),
            SetRemap(src='/global_plan', dst='global_plan'),
            SetRemap(src='/local_plan', dst='local_plan'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # os.path.join(nav2_bringup_dir, 'launch',"navigation_launch.py")
                    os.path.join(package_dir, 'launch',"navigation_launch.py")
                ),
                launch_arguments={
                    'namespace': 'leader',
                    'use_sim_time': use_sim_time,
                    'autostart': 'True',
                    'params_file': leader_rewritten_yaml_path,
                    'use_composition': 'False',
                    'use_respawn': 'True',
                }.items(),
            ),
        ]
    )

    follower_yaml_final = yaml_content.replace('<robot_namespace>', 'follower')
    
    follower_rewritten_yaml_path = os.path.join(package_dir,'/tmp/nav2_params_rewritten_follower.yaml')
    with open(follower_rewritten_yaml_path, 'w') as f:
        f.write(follower_yaml_final)

    launch_follower_nav2=GroupAction(
        [
            PushRosNamespace(namespace='follower'),

            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/odom', dst='odom'),
            SetRemap(src='/cmd_vel', dst='cmd_vel'),
            SetRemap(src='/global_plan', dst='global_plan'),
            SetRemap(src='/local_plan', dst='local_plan'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # os.path.join(nav2_bringup_dir, 'launch',"navigation_launch.py")
                    os.path.join(package_dir, 'launch',"navigation_launch.py")
                ),
                launch_arguments={
                    'namespace': 'follower',
                    'use_sim_time': use_sim_time,
                    'autostart': 'True',
                    'params_file': follower_rewritten_yaml_path,
                    'use_composition': 'False',
                    'use_respawn': 'True',
                }.items(),
            ),
        ]
    )

    launch_merged_map_rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', merged_map_rviz_config],
        remappings=[
                ('/tf', '/leader/tf'),
                ('/tf_static', '/leader/tf_static')
            ],
        parameters=[
                {'use_sim_time': use_sim_time}
            ],
    )

    launch_map_expansion = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_merge_pkg, 'launch', 'map_expansion.launch.py')),
        )
    
    launch_map_merger = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_merge_pkg, 'launch', 'map_merge.launch.py')),
        )

    leader_exploration=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('explore_lite'),'launch','explore.launch.py')),
        launch_arguments={
            'namespace': 'leader',
            'use_sim_time': use_sim_time,
        }.items()
    )
    follower_exploration=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('explore_lite'),'launch','explore.launch.py')),
        launch_arguments={
            'namespace': 'follower',
            'use_sim_time': use_sim_time,
        }.items()
    )

    ld=LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(turtlebot3_world_gazebo)
    ld.add_action(leader_robot_state_publisher)
    ld.add_action(follower_robot_state_publisher)
    ld.add_action(leader_node)
    ld.add_action(follower_node)
    ld.add_action(leader_start_gazebo_ros_bridge)
    # ld.add_action(leader_gazebo_ros_image_bridge_cmd)
    ld.add_action(follower_start_gazebo_ros_bridge)
    # ld.add_action(follower_gazebo_ros_image_bridge_cmd)
    ld.add_action(launch_leader_slam_toolbox)
    ld.add_action(launch_follower_slam_toolbox)
    ld.add_action(launch_leader_rviz)
    ld.add_action(launch_follower_rviz)
    ld.add_action(launch_leader_nav2)
    ld.add_action(launch_follower_nav2)
    ld.add_action(launch_merged_map_rviz)
    ld.add_action(launch_map_expansion)
    ld.add_action(launch_map_merger)
    ld.add_action(leader_exploration)
    ld.add_action(follower_exploration)

    return ld