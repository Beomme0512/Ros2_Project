cmake_minimum_required(VERSION 3.8)
project(rx_process)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(direction_node src/direction_node.cpp)
ament_target_dependencies(direction_node rclcpp std_msgs)

install(TARGETS
  direction_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()