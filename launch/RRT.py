#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    eta = LaunchConfiguration('eta')
    Geta = LaunchConfiguration('Geta')

    return LaunchDescription([
        DeclareLaunchArgument('eta', default_value='1.0'),
        DeclareLaunchArgument('Geta', default_value='15.0'),

        Node(
            package='ros_autonomous',
            executable='global_rrt_detector',
            name='global_detector',
            output='screen',
            parameters=[{
                'eta': Geta,
                'map_topic': '/map'
            }]
        ),

        Node(
            package='ros_autonomous',
            executable='local_rrt_detector',
            name='local_detector',
            output='screen',
            parameters=[{
                'eta': eta,
                'map_topic': '/map',
                'robot_frame': '/base_link'
            }]
        ),

        Node(
            package='ros_autonomous',
            executable='filter.py',
            name='filter',
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'info_radius': 1,
                'costmap_clearing_threshold': 70,
                'goals_topic': '/detected_points',
                'namespace': '',
                'n_robots': 1,
                'rate': 100
            }]
        ),

        Node(
            package='ros_autonomous',
            executable='assigner.py',
            name='assigner',
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'global_frame': '/map',
                'info_radius': 1,
                'info_multiplier': 3.0,
                'hysteresis_radius': 3.0,
                'hysteresis_gain': 2.0,
                'frontiers_topic': '/filtered_points',
                'n_robots': 1,
                'namespace': '',
                'delay_after_assignement': 0.5,
                'rate': 100
            }]
        )
    ])
