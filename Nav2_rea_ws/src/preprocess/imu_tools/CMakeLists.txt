cmake_minimum_required(VERSION 3.8)
project(imu_tools)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TRANSFORM_NODE "imu_transform")
set(FILTER_NODE "imu_complementary_filter")


# ******************** imu_transform ********************
ament_auto_add_library(${TRANSFORM_NODE} SHARED
    imu_transform/imu_transform.cpp
)

rclcpp_components_register_node(${TRANSFORM_NODE}
    PLUGIN "tars_imu_tools::ImuTransform"
    EXECUTABLE ${TRANSFORM_NODE}_node
)


# ******************** imu_complementary_filter ********************
ament_auto_add_library(${FILTER_NODE} SHARED
    imu_complementary_filter/complementary_filter_ros.cpp
    imu_complementary_filter/complementary_filter.cpp
)

rclcpp_components_register_node(${FILTER_NODE}
    PLUGIN "tars_imu_tools::ComplementaryFilterROS"
    EXECUTABLE ${FILTER_NODE}_node
)


ament_auto_package(
    INSTALL_TO_SHARE
    config launch
)
