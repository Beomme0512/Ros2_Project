#!/bin/bash
set -e  # 에러 발생 시 즉시 종료

# === 설정 ===
SESSION_NAME="ros2_robot"
WORKSPACE_DIR=~/ros2_ws
LAUNCH_PACKAGE=my_robot_description
LAUNCH_FILE=robot_system_gazebo.launch.py
KEYBOARD_NODE_PKG=rx_process
KEYBOARD_NODE_EXEC=direction_node

# === 가상환경 체크 및 비활성화 ===
if [[ "$VIRTUAL_ENV" != "" ]]; then
  echo "⚠️ 가상환경이 활성화되어 있습니다: $VIRTUAL_ENV"
  echo "⛔ deactivate 처리 후 시스템 Python으로 복원합니다"
  deactivate || echo "(⚠️ 자동 비활성화 실패: 수동으로 처리하세요)"
  unset PYTHONPATH
  unset _PYTHON_SYSCONFIGDATA_NAME
fi

# === Python 경로 강제 설정 ===
echo "📌 시스템 Python 환경 강제 적용 중..."
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
hash -r  # 경로 캐시 리셋
which python3  # 디버깅용

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