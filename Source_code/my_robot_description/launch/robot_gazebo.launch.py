from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'worlds',
        'simple_room.world'
    ])

    xacro_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                TextSubstitution(text='xacro '),
                xacro_path
            ]),
            value_type=str
        )
    }

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'
        # ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_bot',
                '-topic', 'robot_description',
                '-robot_namespace', '/'
            ],
            output='screen'
        )
    ])