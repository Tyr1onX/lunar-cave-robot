#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace tars_pointcloud_tools {

class PointCloudFusion : public rclcpp::Node
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>;

public:
    PointCloudFusion(const rclcpp::NodeOptions& options);

private:
    void syncCallback(
        const LivoxCustomMsg::ConstSharedPtr& msg1,
        const LivoxCustomMsg::ConstSharedPtr& msg2);

    std::shared_ptr<message_filters::Subscriber<LivoxCustomMsg>> lidar1_sub_;
    std::shared_ptr<message_filters::Subscriber<LivoxCustomMsg>> lidar2_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Publisher<LivoxCustomMsg>::SharedPtr fused_custommsg_pub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr fused_pointcloud_pub_;
    
    std::string target_frame_;
    std::string sub_topic1_;
    std::string sub_topic2_;
    std::string pub_custommsg_topic_;
    std::string pub_pointcloud_topic_;
    int queue_size_;

}; // class PointCloudFusion

}  // namespace tars_pointcloud_tools
