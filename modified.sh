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

# === Gazebo 종료 ===
echo "🛑 실행 중인 Gazebo 종료 중..."
killall -q -9 gzserver gzclient || echo "✅ 실행 중인 Gazebo 없음"

# === tmux 세션 정리 ===
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "⚠️ 기존 tmux 세션 '$SESSION_NAME' 종료..."
  tmux kill-session -t $SESSION_NAME
fi

# === tmux 세션 시작 ===
echo "🚀 tmux 세션 시작: $SESSION_NAME"
tmux new-session -d -s $SESSION_NAME

# ▶ 첫 번째 패널: Gazebo Launch
tmux send-keys -t $SESSION_NAME:0 \
  "source /opt/ros/humble/setup.bash && cd $WORKSPACE_DIR && source install/setup.bash && ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE" C-m

# ▶ 두 번째 패널: 방향키 노드 실행
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.1 \
  "source /opt/ros/humble/setup.bash && cd $WORKSPACE_DIR && source install/setup.bash && ros2 run $KEYBOARD_NODE_PKG $KEYBOARD_NODE_EXEC" C-m

# === 세션 전환 ===
tmux attach-session -t $SESSION_NAME
