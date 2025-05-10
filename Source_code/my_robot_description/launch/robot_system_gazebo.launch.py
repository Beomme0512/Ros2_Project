from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'launch',
                'robot_gazebo.launch.py'
            ])
        )
    )

    return LaunchDescription([
        robot_gazebo_launch,
        Node(package='time_scheduler', executable='scheduler_node', output='screen'),
        Node(package='rx_process', executable='direction_node', output='screen'),
        Node(package='motioncontroller', executable='motion_ctrl_node', output='screen'),
    ])
