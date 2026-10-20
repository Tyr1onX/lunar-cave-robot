cmake_minimum_required(VERSION 3.8)
project(chassis_serial_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(chassis_serial_bridge_node
  src/chassis_serial_bridge_node.cpp
)
target_compile_features(chassis_serial_bridge_node PUBLIC cxx_std_17)
target_compile_options(chassis_serial_bridge_node PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(chassis_serial_bridge_node rclcpp geometry_msgs std_msgs)

install(TARGETS chassis_serial_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES src/chassis_serial_protocol.hpp
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
