cmake_minimum_required(VERSION 3.8)
project(time_scheduler)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interface REQUIRED)

# 실행파일 추가
add_executable(scheduler_node src/scheduler_node.cpp)
ament_target_dependencies(scheduler_node rclcpp robot_interface)

# 설치 대상 지정
install(TARGETS
  scheduler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()