cmake_minimum_required(VERSION 3.5)
project(pcd_processor)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(pcd_to_grid_map 
  src/pcd_to_grid_map.cpp
)

ament_target_dependencies(pcd_to_grid_map 
  rclcpp sensor_msgs nav_msgs pcl_msgs PCL pcl_conversions
)

target_link_libraries(pcd_to_grid_map 
  ${PCL_LIBRARIES}
)

add_executable(pcd_voxel_grid_filter 
  src/pcd_voxel_grid_filter.cpp
)

ament_target_dependencies(pcd_voxel_grid_filter 
  rclcpp sensor_msgs nav_msgs pcl_msgs PCL pcl_conversions
)

target_link_libraries(pcd_voxel_grid_filter 
  ${PCL_LIBRARIES}
)

install(TARGETS 
  pcd_to_grid_map
  pcd_voxel_grid_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

