#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    model = LaunchConfiguration('model')
    map_file = LaunchConfiguration('map_file')
    open_rviz = LaunchConfiguration('open_rviz')
    move_forward_only = LaunchConfiguration('move_forward_only')

    turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_navigation = get_package_share_directory('turtlebot3_navigation')
    ros_autonomous = get_package_share_directory('ros_autonomous')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='waffle_pi'),
        DeclareLaunchArgument('map_file', default_value=os.path.join(ros_autonomous_slam, 'maps', 'my_map.yaml')),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument('move_forward_only', default_value='false'),

        # Bringup Turtlebot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot3_bringup, 'launch', 'turtlebot3_remote.launch.py')),
            launch_arguments={'model': model}.items()
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),

        # AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot3_navigation, 'launch', 'amcl.launch.py'))
        ),

        # Move Base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot3_navigation, 'launch', 'move_base.launch.py')),
            launch_arguments={
                'model': model,
                'move_forward_only': move_forward_only
            }.items()
        ),

        # RViz (conditional)
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', os.path.join(ros_autonomous, 'rviz', 'turtlebot3_navigation.rviz')],
                output='screen'
            )
        ], condition=IfCondition(open_rviz))
    ])

