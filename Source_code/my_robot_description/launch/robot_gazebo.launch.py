from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():
    # Xacro 파일 경로
    xacro_file = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'my_robot_description', 'urdf', 'my_robot.urdf.xacro'
    )
    
    # Xacro 파일 파싱
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 디버깅 출력
    print("[DEBUG] Robot Description:")
    print(robot_description)

    return LaunchDescription([
        # Gazebo 실행
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # robot_state_publisher 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        # 로봇 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_bot', '-topic', 'robot_description','-robot_namespace', '/'],
            output='screen'
        )
    ])
