#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// #include "dlio/dlio.h"

class SaveMapNode: public rclcpp::Node {

public:

    SaveMapNode() : Node("save_map_node") {
        dlio_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("dlio_map", 10,
                    std::bind(&SaveMapNode::mapCallback, this, std::placeholders::_1));
    };
    ~SaveMapNode() = default;

private:
    void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        pcl::fromROSMsg(*msg, *dlio_map);

        double leaf_size = 0.15;
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.setInputCloud(dlio_map);
        vg.filter(*dlio_map);

        pcl::io::savePCDFileBinary("/home/eatwhat/Documents/dlio_map.pcd", *dlio_map);
        RCLCPP_INFO(this->get_logger(), "Save map!");
        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dlio_map;

};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SaveMapNode>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();

    rclcpp::spin(node);
  
    rclcpp::shutdown();
  
    return 0;
  
  }