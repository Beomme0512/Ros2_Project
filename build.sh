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

# === tmux 설치 확인 ===
if ! command -v tmux &> /dev/null; then
  echo "❌ tmux가 설치되어 있지 않습니다. 다음 명령어로 설치하세요:"
  echo "   sudo apt install tmux"
  exit 1
fi

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

# === tmux 세션 정리 ===
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "⚠️ 기존 tmux 세션 '$SESSION_NAME' 종료 중..."
  tmux kill-session -t $SESSION_NAME
fi
