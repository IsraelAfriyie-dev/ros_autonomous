#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    model = LaunchConfiguration('model', default=EnvironmentVariable('TURTLEBOT3_MODEL'))
    x_pos = LaunchConfiguration('x_pos', default='-2.0')
    y_pos = LaunchConfiguration('y_pos', default='0.5')
    z_pos = LaunchConfiguration('z_pos', default='0.0')

    world_path = os.path.join(get_package_share_directory('ros_autonomous_slam'), 'worlds', 'moon.world')
    robot_desc_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', f'turtlebot3_{model.perform({})}.urdf.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=EnvironmentVariable('TURTLEBOT3_MODEL')),
        DeclareLaunchArgument('x_pos', default_value='-2.0'),
        DeclareLaunchArgument('y_pos', default_value='0.5'),
        DeclareLaunchArgument('z_pos', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )),
            launch_arguments={'world': world_path}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': os.popen(f'xacro {robot_desc_path}').read()}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'turtlebot3_{model.perform({})}',
                '-x', x_pos,
                '-y', y_pos,
                '-z', z_pos,
                '-topic', 'robot_description'
            ],
            output='screen'
        ),
    ])
