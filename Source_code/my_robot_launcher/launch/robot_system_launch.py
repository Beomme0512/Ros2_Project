from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='time_scheuler',
            executable='scheduler_node',
            name='scheduler_node',
            output='screen'
        ),
        Node(
            package='rx_process',
            executable='direction_node',
            name='direction_node',
            output='screen'
        ),
        Node(
            package='motioncontroller',
            executable='motion_ctrl_node',
            name='motion_ctrl_node',
            output='screen'
        )
    ])