cmake_minimum_required(VERSION 3.8)
project(pointcloud_tools)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

set(FUSION_NODE "pointcloud_fusion")
set(LASERSCAN_NODE "pointcloud_to_laserscan")


# ******************** PointCloud Fusion ********************
ament_auto_add_library(${FUSION_NODE} SHARED
    pointcloud_fusion/pointcloud_fusion.cpp
)

target_link_libraries(${FUSION_NODE}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

rclcpp_components_register_node(${FUSION_NODE}
    PLUGIN "tars_pointcloud_tools::PointCloudFusion"
    EXECUTABLE ${FUSION_NODE}_node
)


# ******************** PointCloud To LaserScan ********************
ament_auto_add_library(${LASERSCAN_NODE} SHARED
    pointcloud_to_laserscan/pointcloud_to_laserscan.cpp
)

rclcpp_components_register_node(${LASERSCAN_NODE}
    PLUGIN "tars_pointcloud_tools::PointCloudToLaserScan"
    EXECUTABLE ${LASERSCAN_NODE}_node
)


ament_auto_package(
    INSTALL_TO_SHARE
    config launch
)
