#!/bin/bash
set -ex  # ← x 옵션 추가: 실행되는 모든 명령 출력됨 (디버그용)

#!/bin/bash
set -e  # 에러 발생 시 즉시 종료

# === 설정 ===
SESSION_NAME="ros2_robot"
WORKSPACE_DIR=~/ros2_ws
LAUNCH_PACKAGE=my_robot_description
LAUNCH_FILE=robot_system_gazebo.launch.py
KEYBOARD_NODE_PKG=rx_process
KEYBOARD_NODE_EXEC=direction_node


# === ROS2 환경 설정 ===
source /opt/ros/humble/setup.bash
cd "$WORKSPACE_DIR"

echo "🧼 이전 빌드 삭제..."
rm -rf build/ install/ log/

echo "🔧 robot_interface 패키지 먼저 빌드 중..."
colcon build --packages-select robot_interface
source install/setup.bash

echo "🔧 전체 패키지 빌드 중..."
colcon build
source install/setup.bash

