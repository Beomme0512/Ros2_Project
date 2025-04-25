from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'launch',
                'robot_gazebo.launch.py'
            ])
        )
    )

    return LaunchDescription([
        gazebo_launch,
        Node(package='Time_Scheuler', executable='scheduler_node', output='screen'),
        Node(package='Rx_Process', executable='direction_node', output='screen'),
        Node(package='MotionController', executable='motion_ctrl_node', output='screen'),
    ])
