#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <filesystem>
#include <fstream>

#include "../utils/multi_workspace_launcher.hpp"

namespace tars_alignment {

// 配准结果结构体，用于评估不同匹配算法的结果
struct AlignmentResult {
    Eigen::Matrix4f transform;
    double fitness_score;
    double orientation_error;  // 方向误差评估
    bool has_converged;
};

class InitPoseAlignment : public rclcpp::Node 
{
public:
    InitPoseAlignment(const rclcpp::NodeOptions& node_options);

private:
    // 预处理地图点云 - 降采样和过滤
    void preprocessMapCloud();
    
    // 为多分辨率配准准备不同分辨率的地图
    void prepareMultiResolutionMaps();
    
    // 发布地图点云
    void publishMapCloud();
    
    // 初始位姿回调函数
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    // 激光雷达点云回调函数
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // 处理累积的点云，使用笛卡尔网格进行特征提取
    pcl::PointCloud<pcl::PointXYZ>::Ptr processAccumulatedClouds();
    
    // 搜索最佳方向 - 在保持位置不变的情况下尝试不同的旋转
    AlignmentResult searchBestOrientation(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const Eigen::Matrix4f& initial_guess);
    
    // 多分辨率点云配准方法
    Eigen::Matrix4f multiResolutionAlignment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const Eigen::Matrix4f& initial_guess);

    // 执行NDT匹配
    AlignmentResult performNDTAlignment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        int max_iterations,
        double max_correspondence_distance);
    
    // 执行ICP匹配
    AlignmentResult performICPAlignment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        int max_iterations,
        double max_correspondence_distance);

    // 执行GICP匹配
    AlignmentResult performGICPAlignment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        int max_iterations,
        double max_correspondence_distance);

    // 计算内点比例 - 评估匹配质量
    double calculateInlierRatio(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& transform,
        double distance_threshold);

    // 计算方向误差 - 两个变换矩阵之间的方向差异
    double calculateOrientationError(
        const Eigen::Matrix4f& transform1,
        const Eigen::Matrix4f& transform2);

    // 多次匹配尝试，选择最佳结果
    AlignmentResult multipleAlignmentAttempts(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const Eigen::Matrix4f& initial_guess);

    // 发布变换信息到TF
    void publishTransform(const Eigen::Matrix4f& transform);

    // 生成高斯分布的随机数
    float getRandomGaussian(float mean, float stddev);

    // 成员变量
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;                     // 目标点云 (pcd地图)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;  // 累积的源点云
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> multi_res_maps_;      // 多分辨率地图

    Eigen::Matrix4f transform_m2b_;    // 当前 map->base 变换矩阵
    Eigen::Matrix4f transform_o2b_;    // 当前 odom->base 变换矩阵
    Eigen::Matrix4f transform_m2o_;    // 当前 map->odom 变换矩阵
    Eigen::Vector3f initial_position_; // 初始位置

    std::mt19937 rand_gen_;  // 随机数生成器

    int accumulate_time_;
    int orientation_search_steps_;
    int multi_align_attempts_;
    double min_inlier_ratio_;
    double max_distance_;
    double voxel_leaf_size_;
    double fitness_score_threshold_;
    double good_fitness_score_threshold_;

    bool use_preprocess_ = false;

    bool use_radius_filter_ = false;
    double filter_radius_;

    bool use_initial_pose_ = false;

    bool use_multi_resolution_;
    bool use_icp_;

    bool use_cartesian_grid_ = false; 
    double grid_size_;

    bool use_height_threshold_ = false;
    double min_height_;
    double max_height_;

    double ndt_leaf_size_;                    // 0.5
    int ndt_max_iterations_;                  // 30
    double ndt_max_correspondence_distance_;  // 2.0

    double icp_leaf_size_;                    // 0.25
    int icp_max_iterations_;                  // 50
    double icp_max_correspondence_distance_;  // 0.5

    double gicp_leaf_size_;                   // 0.1
    int gicp_max_iterations_;                 // 50
    double gicp_max_correspondence_distance_; // 0.1

    bool has_initial_pose_ = false;           // 是否有初始位姿
    bool has_aligned_ = false;                // 是否已经完成匹配
    bool is_first_time_ = true;               // 是否首次匹配

    std::string map_frame_id_, odom_frame_id_, robot_frame_id_;

    // ROS Publisher
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr get_pose_pub_;

    // ROS Subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    // TF2
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS Timer
    rclcpp::TimerBase::SharedPtr map_timer_;

    MultiWorkspaceLauncher launcher_;
};

}  // namespace tars_alignment

RCLCPP_COMPONENTS_REGISTER_NODE(tars_alignment::InitPoseAlignment)
