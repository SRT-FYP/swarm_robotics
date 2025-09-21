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
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    package_dir = get_package_share_directory('leader_follower')
    nav2_bringup_dir=get_package_share_directory('nav2_bringup')

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
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'turtlebot3_house.world'
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
   
    leader_rviz_config= os.path.join(
            get_package_share_directory('leader_follower'),
            'config',
            'leader_rviz.rviz'
        )
    namespaced_nav2_rviz=os.path.join(
            get_package_share_directory('leader_follower'),
            'config',
            'nav2_default_view.rviz'
        )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    turtlebot3_world_gazebo=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':['-r -v -v4 ',world],'on_exit_shutdown':'true'}.items())

    with open(params_file, 'r') as f:
        yaml_content = f.read()

    robot_namespace = 'leader'
    yaml_content = yaml_content.replace('<robot_namespace>', robot_namespace)
    
    rewritten_yaml_path = os.path.join(package_dir,'/tmp/nav2_params_rewritten.yaml')
    with open(rewritten_yaml_path, 'w') as f:
        f.write(yaml_content)

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

    leader_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace='leader',
        arguments=['camera/image_raw'],
        output='screen',
    )

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
            ]
    )

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
                    os.path.join(nav2_bringup_dir, 'launch',"navigation_launch.py")
                ),
                launch_arguments={
                    'namespace': 'leader',
                    'use_sim_time': use_sim_time,
                    'autostart': 'True',
                    'params_file': rewritten_yaml_path,
                    'use_composition': 'False',
                    'use_respawn': 'True',
                }.items(),
            ),
        ]
    )


    ld=LaunchDescription()
    ld.add_action(turtlebot3_world_gazebo)
    ld.add_action(leader_robot_state_publisher)
    ld.add_action(leader_node)
    ld.add_action(leader_start_gazebo_ros_bridge)
    ld.add_action(leader_gazebo_ros_image_bridge_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(launch_leader_slam_toolbox)
    ld.add_action(launch_leader_rviz)
    ld.add_action(launch_leader_nav2)

    return ld