import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    leader_follower_pkg = get_package_share_directory('leader_follower')  

    namespaced_nav2_rviz=os.path.join(
            get_package_share_directory('leader_follower'),
            'config',
            'nav2_default_view.rviz'
        )
    
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for robot",
    )
    
    namespace = LaunchConfiguration("namespace")

    rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', namespaced_nav2_rviz],
        namespace=namespace,
        remappings=[
            ('/tf', [TextSubstitution(text='/'), namespace, TextSubstitution(text='/tf')]),
            ('/tf_static', [TextSubstitution(text='/'), namespace, TextSubstitution(text='/tf_static')])
        ],
        parameters=[
                {'use_sim_time': False}
            ],
    )


    ld = LaunchDescription()
   
    ld.add_action(rviz)
    return ld