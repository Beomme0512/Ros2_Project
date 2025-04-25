cmake_minimum_required(VERSION 3.8)
project(motioncontroller)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(motion_ctrl_node src/motion_ctrl_node.cpp)
ament_target_dependencies(motion_ctrl_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  motion_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()