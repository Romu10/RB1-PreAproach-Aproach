import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='approach_node',
            output='screen',
            name='service_approach',
        ),
        
        Node(
            package='attach_shelf',
            executable='rotate_node',
            output='screen',
            name='service_rotate',
        ),

        DeclareLaunchArgument('obstacle', default_value='0.0'),
        DeclareLaunchArgument('degrees', default_value='0'),
        DeclareLaunchArgument('final_approach', default_value="false"),
        DeclareLaunchArgument('rviz_config_file', default_value='/home/user/ros2_ws/src/attach_shelf/rviz/config_approach.rviz'),

        # Ahora LogInfo está dentro de la lista de acciones.
        LogInfo(msg=LaunchConfiguration('obstacle')),
        LogInfo(msg=LaunchConfiguration('degrees')),
        LogInfo(msg=LaunchConfiguration('final_approach')),

        Node(
            package='attach_shelf',
            executable='param_vel_node_v2',
            output='screen',
            emulate_tty=True,
            arguments=[
                LaunchConfiguration('obstacle'),
                LaunchConfiguration('degrees'),
                LaunchConfiguration('final_approach'),
            ],
            parameters=[
                {'obstacle': LaunchConfiguration('obstacle')},
                {'degrees': LaunchConfiguration('degrees')},
                {'final_approach': LaunchConfiguration('final_approach')},
            ],
        ),

        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    output='screen',
        #    arguments=['-d', LaunchConfiguration('rviz_config_file')],
        #),
    ])
