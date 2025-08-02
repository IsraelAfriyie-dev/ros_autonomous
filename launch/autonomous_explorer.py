#!/usr/bin/env python3

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
    model = LaunchConfiguration('model')
    slam_methods = LaunchConfiguration('slam_methods')
    configuration_basename = LaunchConfiguration('configuration_basename')
    open_rviz = LaunchConfiguration('open_rviz')

    turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
   # turtlebot3_slam = get_package_share_directory('turtlebot3_slam')
    turtlebot3_slam = get_package_share_directory('turtlebot3_cartographer')
    ros_autonomous = get_package_share_directory('ros_autonomous')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('model', default_value='waffle_pi'),
        DeclareLaunchArgument('slam_methods', default_value='gmapping'),
        DeclareLaunchArgument('configuration_basename', default_value='turtlebot3_lds_2d.lua'),
        DeclareLaunchArgument('open_rviz', default_value='true'),

        # Include TurtleBot3 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_bringup, 'launch', 'turtlebot3_remote.launch.py')
            ),
            launch_arguments={'model': model}.items()
        ),

        # Include the SLAM method launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    turtlebot3_slam, 'launch', 'turtlebot3_' + slam_methods.perform(None) + '.launch.py'
                )
            ),
            launch_arguments={
                'model': model,
                'configuration_basename': configuration_basename
            }.items()
        ),

        # RViz (optional)
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d', os.path.join(ros_autonomous, 'rviz', 'turtlebot3_' + slam_methods.perform(None) + '.rviz')
                ]
            )
        ], condition=IfCondition(open_rviz))
    ])
