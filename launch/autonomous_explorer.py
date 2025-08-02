#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch configurations
    model = LaunchConfiguration('model')
    slam_methods = LaunchConfiguration('slam_methods')
    configuration_basename = LaunchConfiguration('configuration_basename')
    open_rviz = LaunchConfiguration('open_rviz')
    
    # Package directories
    turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_cartographer = get_package_share_directory('turtlebot3_cartographer')
    ros_autonomous = get_package_share_directory('ros_autonomous')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'model', 
            default_value='waffle_pi',
            description='TurtleBot3 model type'
        ),
        DeclareLaunchArgument(
            'slam_methods', 
            default_value='cartographer',  # Changed from 'gmapping' to match cartographer package
            description='SLAM method to use'
        ),
        DeclareLaunchArgument(
            'configuration_basename', 
            default_value='turtlebot3_lds_2d.lua',
            description='Configuration file for cartographer'
        ),
        DeclareLaunchArgument(
            'open_rviz', 
            default_value='true',
            description='Whether to open RViz'
        ),
        
        # Include TurtleBot3 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_bringup, 'launch', 'turtlebot3_remote.launch.py')
            ),
            launch_arguments={'model': model}.items()
        ),
        
        # Include Cartographer SLAM launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_cartographer, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={
                'configuration_basename': configuration_basename,
                'use_sim_time': 'false'  # Add this for real robot
            }.items()
        ),
        
        # RViz (conditional)
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d', os.path.join(ros_autonomous, 'rviz', 'turtlebot3_cartographer.rviz')
                ]
            )
        ], condition=IfCondition(open_rviz))
    ])
