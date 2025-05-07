#!/bin/bash
set -ex  # 디버깅 및 에러 발생 시 종료

# === 설정 ===
SESSION_NAME="ros2_robot"
WORKSPACE_DIR=~/ros2_ws
LAUNCH_PACKAGE=my_robot_description
LAUNCH_FILE=robot_system_gazebo.launch.py
KEYBOARD_NODE_PKG=rx_process
KEYBOARD_NODE_EXEC=direction_node

# === tmux 설치 확인 ===
if ! command -v tmux &> /dev/null; then
  echo "❌ tmux가 설치되어 있지 않습니다. 설치 후 다시 시도하세요:"
  echo "   sudo apt install tmux"
  exit 1
fi

# === ROS2 환경 설정 ===
source /opt/ros/humble/setup.bash
cd "$WORKSPACE_DIR"

# === robot_interface 먼저 빌드 ===
echo "🔧 robot_interface 패키지 먼저 빌드 중..."
colcon build --packages-select robot_interface
source install/setup.bash

# === 전체 빌드 ===
echo "🔧 전체 패키지 빌드 중..."
colcon build
source install/setup.bash
# === Gazebo 프로세스 종료 ===