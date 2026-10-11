#include "pointcloud_fusion.hpp"

namespace tars_pointcloud_tools {

PointCloudFusion::PointCloudFusion(const rclcpp::NodeOptions& options)
    : Node("pointcloud_fusion", options)
{
    this->declare_parameter("target_frame", "livox_frame");
    this->declare_parameter("sub_topic1", "/livox/lidar_192_168_36_119");
    this->declare_parameter("sub_topic2", "/livox/lidar_192_168_36_128");
    this->declare_parameter("pub_custommsg_topic", "/livox/lidar");
    this->declare_parameter("pub_pointcloud_topic", "/livox/pointcloud");
    this->declare_parameter("queue_size", 10);
    
    target_frame_ = this->get_parameter("target_frame").as_string();
    sub_topic1_ = this->get_parameter("sub_topic1").as_string();
    sub_topic2_ = this->get_parameter("sub_topic2").as_string();
    pub_custommsg_topic_ = this->get_parameter("pub_custommsg_topic").as_string();
    pub_pointcloud_topic_ = this->get_parameter("pub_pointcloud_topic").as_string();
    queue_size_ = this->get_parameter("queue_size").as_int();
    
    lidar1_sub_ = 
        std::make_shared<message_filters::Subscriber<LivoxCustomMsg>>(
        this, sub_topic1_, rmw_qos_profile_sensor_data);

    lidar2_sub_ = 
        std::make_shared<message_filters::Subscriber<LivoxCustomMsg>>(
        this, sub_topic2_, rmw_qos_profile_sensor_data);
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size_), *lidar1_sub_, *lidar2_sub_);
    
    sync_->registerCallback(
        std::bind(&PointCloudFusion::syncCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    fused_custommsg_pub_ = this->create_publisher<LivoxCustomMsg>(pub_custommsg_topic_, 10);
    fused_pointcloud_pub_ = this->create_publisher<PointCloud2>(pub_pointcloud_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "PointCloud Fusion Node initialized!");
}

void PointCloudFusion::syncCallback(
    const LivoxCustomMsg::ConstSharedPtr& msg1,
    const LivoxCustomMsg::ConstSharedPtr& msg2)
{
    RCLCPP_DEBUG(this->get_logger(), "Received synchronized point clouds. Time diff: %f",
                (msg1->header.stamp.sec - msg2->header.stamp.sec) +
                (msg1->header.stamp.nanosec - msg2->header.stamp.nanosec) * 1e-9);

    LivoxCustomMsg custommsg_msg;
    custommsg_msg.header = msg1->header;
    custommsg_msg.timebase = msg1->timebase;
    custommsg_msg.lidar_id = msg1->lidar_id;
    custommsg_msg.rsvd = msg1->rsvd;

    custommsg_msg.points = msg1->points;
    custommsg_msg.points.insert(custommsg_msg.points.end(), msg2->points.begin(), msg2->points.end());
    custommsg_msg.point_num = custommsg_msg.points.size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl_cloud->reserve(custommsg_msg.point_num);
    for (const auto& pt : custommsg_msg.points) {
        pcl_cloud->emplace_back(pcl::PointXYZI{pt.x, pt.y, pt.z, pt.reflectivity});
    }

    PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*pcl_cloud, pointcloud_msg);
    pointcloud_msg.header.stamp = custommsg_msg.header.stamp;
    pointcloud_msg.header.frame_id = target_frame_;

    fused_custommsg_pub_->publish(custommsg_msg);
    fused_pointcloud_pub_->publish(pointcloud_msg);
}

} // namespace tars_pointcloud_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tars_pointcloud_tools::PointCloudFusion)
