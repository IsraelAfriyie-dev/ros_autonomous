#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_autonomous_slam',
            executable='wall_follow.py',
            name='local_detector',
            output='screen'
        )
    ])
