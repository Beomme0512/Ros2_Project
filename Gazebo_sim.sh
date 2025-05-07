
#!/bin/bash

# 사용자 정의 변수들
SESSION_NAME=my_sim_session
WORKSPACE_DIR=~/ros2_ws
LAUNCH_PACKAGE=my_robot_launcher
LAUNCH_FILE=robot_sim.launch.py
KEYBOARD_NODE_PKG=keyboard_input
KEYBOARD_NODE_EXEC=direction_node

# === Ctrl+C 시 처리 함수 등록 ===
cleanup() {
  echo "🧹 Ctrl+C 감지됨. tmux 세션 '$SESSION_NAME' 종료 중..."
  tmux kill-session -t $SESSION_NAME 2>/dev/null
  exit 0
}
trap cleanup SIGINT

# === Kill any running Gazebo processes ===
echo "🛑 Checking and killing any existing Gazebo processes..."
killall -q -9 gzserver gzclient || echo "✅ No existing Gazebo process found."


# === tmux 세션 정리 ===
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "⚠️ 기존 tmux 세션 '$SESSION_NAME' 종료..."
  tmux kill-session -t $SESSION_NAME
fi

source install/setup.bash

# === tmux 세션 시작 ===
echo "🚀 tmux 세션 시작: $SESSION_NAME"
tmux new-session -d -s $SESSION_NAME

# 첫 번째 패널: Gazebo 시뮬레이션 실행
tmux send-keys -t $SESSION_NAME:0 \
  "source /opt/ros/humble/setup.bash && cd $WORKSPACE_DIR && source install/setup.bash && ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE" C-m

# 패널 수직 분할
tmux split-window -v -t $SESSION_NAME

# 두 번째 패널: direction_node 실행
tmux send-keys -t $SESSION_NAME:0.1 \
  "source /opt/ros/humble/setup.bash && cd $WORKSPACE_DIR && source install/setup.bash && ros2 run $KEYBOARD_NODE_PKG $KEYBOARD_NODE_EXEC" C-m

# tmux 세션으로 전환
tmux attach-session -t $SESSION_NAME
# === 세션 종료 시 처리 ===