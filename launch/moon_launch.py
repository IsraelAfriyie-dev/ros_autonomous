#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    model = LaunchConfiguration('model')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')

    # Package directories
    turtlebot3_description_pkg = get_package_share_directory('turtlebot3_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    ros_autonomous_pkg = get_package_share_directory('ros_autonomous')

    world_path = os.path.join(ros_autonomous_pkg, 'worlds', 'moon.world')
    
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=TextSubstitution(text='waffle_pi')),
        DeclareLaunchArgument('x_pos', default_value=TextSubstitution(text='-2.0')),
        DeclareLaunchArgument('y_pos', default_value=TextSubstitution(text='0.5')),
        DeclareLaunchArgument('z_pos', default_value=TextSubstitution(text='0.0')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([
                        turtlebot3_description_pkg, 'urdf',
                        TextSubstitution(text='turtlebot3_'), model,
                        TextSubstitution(text='.urdf.xacro')
                    ])
                ])
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', TextSubstitution(text='turtlebot3_'), model,
                '-topic', 'robot_description',
                '-x', x_pos,
                '-y', y_pos,
                '-z', z_pos
            ],
            output='screen'
        )
    ])
