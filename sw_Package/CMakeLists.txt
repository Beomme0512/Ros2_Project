cmake_minimum_required(VERSION 3.5)
project(control)

# 메시지 빌드 도구
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 의존성 메시지 타입
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 메시지 파일 추가
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Python 빌드 설정 (ROS2 Python 패키지)
ament_python_install_package(${PROJECT_NAME})

# 설치 경로 설정
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 메시지 타입 export
ament_export_dependencies(rosidl_default_runtime)

ament_package()