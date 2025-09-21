import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
from launch.actions import AppendEnvironmentVariable
import xacro



def generate_launch_description():
    package_dir = get_package_share_directory('leader_follower')
    package_models =os.path.join(package_dir,'turtlebot3_models')
    world_models = os.path.join(package_dir,'worlds')
    world = os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'turtlebot3_world.world'
            ) 


    # world = os.path.join(
    #             world_models,
    #             'waypoint.world'
    #         )
    
    urdf = os.path.join(package_dir, 'urdf', 'turtlebot3_waffle.urdf') #originally like that
    robot_model = os.path.join(package_dir, 'urdf', 'gz_waffle_1tf.sdf.xacro')
    sdf_path= os.path.join(
        package_models,
        'turtlebot3_waffle',
        'model.sdf'
    )
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    

    leader_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='leader',
        arguments=[
            '-name', 'waffle_leader',
            '-string', Command([
                FindExecutable(name='xacro'),' ','namespace:=','leader',' ','frame_prefix:=','l_',' ','odom_frame_prefix:=','l_',' ', robot_model]),
            '-x', '-2.0',
            '-y', '-0.5',
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
                FindExecutable(name='xacro'),' ','namespace:=','follower',' ','frame_prefix:=','f_',' ','odom_frame_prefix:=','l_',' ', robot_model]),
            '-x', '-1.0',
            '-y', '-0.5',
            '-z', '0.01',
            '-R', '0.0' ,
            '-P', '0.0', 
            '-Y', '0.0'
        ],
        output='screen',
    )

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # empty_world_gazebo=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':['-r -v -v4 empty.sdf'],'on_exit_shutdown':'true'}.items())
    turtlebot3_world_gazebo=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':['-r -v -v4 ',world],'on_exit_shutdown':'true'}.items())

    robot_description_config = xacro.process_file(
        os.path.join(package_dir, 'urdf', 'turtlebot3_waffle.urdf'),
        # mappings={
        #     'frame_prefix':'f_'
        # },
    )
    
    leader_robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='leader',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description_config.toxml(), 'frame_prefix':'l_'}
        ],
        # remappings=remappings,
    )
    follower_robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='follower',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description_config.toxml(), 'frame_prefix':'f_'}
        ],
        # remappings=remappings,
    )

    bridge_params = os.path.join(
        package_dir,
        'params',
        'turtlebot3_waffle_bridge_absolute_tf.yaml'
    )
    

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

    follower_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace='follower',
        arguments=['camera/image_raw'],
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


    ld=LaunchDescription()
    # ld.add_action(empty_world_gazebo)
    ld.add_action(turtlebot3_world_gazebo)
    ld.add_action(leader_robot_state_publisher)
    ld.add_action(follower_robot_state_publisher)
    ld.add_action(leader_node)
    ld.add_action(follower_node)
    ld.add_action(leader_start_gazebo_ros_bridge)
    ld.add_action(follower_start_gazebo_ros_bridge)
    ld.add_action(follower_gazebo_ros_image_bridge_cmd)
    ld.add_action(leader_gazebo_ros_image_bridge_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    return ld