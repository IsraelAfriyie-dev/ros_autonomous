#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    model = LaunchConfiguration('model')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')

    # Get package paths
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    turtlebot3_description_pkg = get_package_share_directory('turtlebot3_description')
    
    # Try to get ros_autonomous package world
    try:
        ros_autonomous_pkg = get_package_share_directory('ros_autonomous')
        world_path = os.path.join(ros_autonomous_pkg, 'worlds', 'moon.world')
    except:
        world_path = None

    # Fallback to default world
    if world_path and os.path.exists(world_path):
        world_file = world_path
    else:
        world_file = os.path.join(gazebo_ros_pkg, 'worlds', 'empty.world')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('model', default_value='burger', description='TurtleBot3 model'),
        DeclareLaunchArgument('x_pos', default_value='-2.0', description='X position'),
        DeclareLaunchArgument('y_pos', default_value='0.5', description='Y position'),
        DeclareLaunchArgument('z_pos', default_value='0.0', description='Z position'),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Publish robot_state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('turtlebot3_description'),
                        'urdf',
                        TextSubstitution(text='turtlebot3_'),
                        model,
                        TextSubstitution(text='.urdf.xacro')
                    ])
                ])
            }]
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot3',
            arguments=[
                '-entity', 'turtlebot3',
                '-topic', 'robot_description',
                '-x', x_pos,
                '-y', y_pos,
                '-z', z_pos
            ],
            output='screen'
        )
    ])
