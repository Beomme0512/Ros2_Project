from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Time_Scheuler',
            executable='scheduler_node',
            name='scheduler_node',
            output='screen'
        ),
        Node(
            package='Rx_Process',
            executable='direction_node',
            name='direction_node',
            output='screen'
        ),
        Node(
            package='MotionController',
            executable='motion_ctrl_node',
            name='motion_ctrl_node',
            output='screen'
        )
    ])