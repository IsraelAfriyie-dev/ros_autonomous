#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
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
    
    # Package directories
    turtlebot3_description_pkg = get_package_share_directory('turtlebot3_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    # Try to get ros_autonomous package, fallback if it doesn't exist
    try:
        ros_autonomous_pkg = get_package_share_directory('ros_autonomous')
        world_path = os.path.join(ros_autonomous_pkg, 'worlds', 'moon.world')
    except:
        world_path = None
    
    # Use custom world if it exists, otherwise use empty world
    if world_path and os.path.exists(world_path):
        world_file = world_path
    else:
        world_file = os.path.join(gazebo_ros_pkg, 'worlds', 'empty.world')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'model', 
            default_value='burger',  # Change to burger if waffle_pi isn't available
            description='TurtleBot3 model type'
        ),
        DeclareLaunchArgument(
            'x_pos', 
            default_value='-2.0',
            description='Initial X position of the robot'
        ),
        DeclareLaunchArgument(
            'y_pos', 
            default_value='0.5',
            description='Initial Y position of the robot'
        ),
        DeclareLaunchArgument(
            'z_pos', 
            default_value='0.0',
            description='Initial Z position of the robot'
        ),
        
        # Launch Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'true'
            }.items()
        ),
        
        # Publish robot state using xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', 
                    PathJoinSubstitution([
                        FindPackageShare('turtlebot3_description'),
                        'urdf',
                        [model, '.urdf.xacro']
                    ])
                ])
            }]
        ),
        
        # Spawn TurtleBot3 in Gazebo
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
