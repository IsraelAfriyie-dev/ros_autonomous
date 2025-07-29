#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():
    model = LaunchConfiguration('model')
    slam_methods = LaunchConfiguration('slam_methods')
    config_basename = LaunchConfiguration('configuration_basename')
    open_rviz = LaunchConfiguration('open_rviz')

    turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_slam = get_package_share_directory('turtlebot3_slam')
    ros_autonomous_slam = get_package_share_directory('ros_autonomous_slam')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=TextSubstitution(text='waffle_pi')),
        DeclareLaunchArgument('slam_methods', default_value=TextSubstitution(text='gmapping')),
        DeclareLaunchArgument('configuration_basename', default_value=TextSubstitution(text='turtlebot3_lds_2d.lua')),
        DeclareLaunchArgument('open_rviz', default_value=TextSubstitution(text='true')),

        # TurtleBot3 Remote Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_bringup, 'launch', 'turtlebot3_remote_launch.py')
            ),
            launch_arguments={'model': model}.items()
        ),

        # SLAM Method Launch (e.g., gmapping, cartographer, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_slam, 'launch', f'turtlebot3_{slam_methods.perform(None)}_launch.py')
            ),
            launch_arguments={
                'model': model,
                'configuration_basename': config_basename
            }.items()
        ),

        # RViz Visualization
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(ros_autonomous_slam, 'rviz', f'turtlebot3_{slam_methods.perform(None)}.rviz')],
                output='screen',
                condition=IfCondition(open_rviz)
            )
        ])
    ])
