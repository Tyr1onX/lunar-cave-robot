cmake_minimum_required(VERSION 3.8)
project(alignment)
set(CMAKE_BUILD_TYPE "RELEASE") # gicp 配准需要release编译

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()
find_package(Eigen3 REQUIRED)

# ******************** Init Pose Alignment ********************
set(ALIGNMENT_NODE "pose_alignment")
file(GLOB_RECURSE ALIGNMENT_NODE_SRCS "init_pose_alignment/init_pose_alignment.cpp")

ament_auto_add_library(${ALIGNMENT_NODE} SHARED
    ${ALIGNMENT_NODE_SRCS}
)

target_link_libraries(${ALIGNMENT_NODE} 
    ${PCL_LIBRARIES} Eigen3::Eigen
)

rclcpp_components_register_node(${ALIGNMENT_NODE}
    PLUGIN "tars_alignment::InitPoseAlignment"
    EXECUTABLE ${ALIGNMENT_NODE}_node
)

# ******************** Init TF Publisher ********************
set(TF_PUBLISHER_NODE "tf_publisher")
file(GLOB_RECURSE TF_PUBLISHER_NODE_SRCS "init_tf_publisher/init_tf_publisher.cpp")

ament_auto_add_library(${TF_PUBLISHER_NODE} SHARED
    ${TF_PUBLISHER_NODE_SRCS}
)

rclcpp_components_register_node(${TF_PUBLISHER_NODE}
    PLUGIN "tars_alignment::InitTFPublisher"
    EXECUTABLE ${TF_PUBLISHER_NODE}_node
)

ament_auto_package(
    INSTALL_TO_SHARE
    config launch pcd
)
