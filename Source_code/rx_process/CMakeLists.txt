cmake_minimum_required(VERSION 3.8)
project(rx_process)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interface REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(direction_node src/direction_node.cpp)
ament_target_dependencies(direction_node rclcpp std_msgs robot_interface)
add_executable(uss_rx_node src/uss_rx_node.cpp)
ament_target_dependencies(uss_rx_node rclcpp std_msgs sensor_msgs robot_interface)

install(TARGETS
  direction_node
  uss_rx_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()