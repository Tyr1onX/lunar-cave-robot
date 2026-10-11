#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

class PcdVoxelGridFilter : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // Processing parameters
    std::string frame_id_;
    std::string pcd_file_path_;
    std::string output_pcd_file_path_;
    double publish_rate_;
    float max_height_threshold_;
    float voxel_size_;

    // Debug parameters
    bool publish_input_cloud_;
    bool publish_processed_cloud_;
    
    // Point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_;
    
public:
    PcdVoxelGridFilter() : Node("pcd_voxel_grid_filter"), 
        input_cloud_(new pcl::PointCloud<pcl::PointXYZ>), output_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing point cloud processor...");
        
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("pcd_file_path", "/home/eatwhat/Nav2_rea_ws/src/localization/pcd_processor/pcd/nanling.pcd");
        this->declare_parameter("output_pcd_file_path", "/home/eatwhat/Nav2_rea_ws/src/localization/pcd_processor/pcd/nanling01.pcd");
        this->declare_parameter("publish_rate", 5.0);
        this->declare_parameter("max_height_threshold", 3.0);
        this->declare_parameter("voxel_size", 0.1);

        this->declare_parameter("publish_input_cloud", true);
        this->declare_parameter("publish_processed_cloud", true);

        frame_id_ = this->get_parameter("frame_id").as_string();
        pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();
        output_pcd_file_path_ = this->get_parameter("output_pcd_file_path").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        max_height_threshold_ = this->get_parameter("max_height_threshold").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        
        publish_input_cloud_ = this->get_parameter("publish_input_cloud").as_bool();
        publish_processed_cloud_ = this->get_parameter("publish_processed_cloud").as_bool();
                
        input_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("input_cloud", 5);
        processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_cloud", 5);

        if (!pcd_file_path_.empty()) {
            if (loadPCDFile(pcd_file_path_)) {
                publish_timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(1.0 / publish_rate_),
                    std::bind(&PcdVoxelGridFilter::publishResults, this)
                );
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unable to load PCD file: %s", pcd_file_path_.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "No PCD file path provided!");
        }
    }
    
private:
    bool loadPCDFile(std::string& filepath)
    {
        try {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *input_cloud_) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Cannot read file: %s", filepath.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %zu points", input_cloud_->points.size());
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(input_cloud_);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*filtered_cloud);
            
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(filtered_cloud);
            sor.setMeanK(80);
            sor.setStddevMulThresh(0.8);
            // sor.filter(*filtered_cloud);

            pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
            ror.setInputCloud(filtered_cloud);
            ror.setRadiusSearch(0.1);
            ror.setMinNeighborsInRadius(20);
            // ror.filter(*filtered_cloud);
            
            RCLCPP_INFO(this->get_logger(), "Applied voxel filter, reduced point cloud from %zu to %zu points", 
                        input_cloud_->size(), filtered_cloud->size());
            output_cloud_ = filtered_cloud;

            savePCDFile(output_pcd_file_path_);
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception when loading PCD file: %s", e.what());
            return false;
        }
    }

    // 保存PCD文件
    void savePCDFile(const std::string& filepath)
    {
        try {
            if (pcl::io::savePCDFileBinary(filepath, *output_cloud_) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file: %s", filepath.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Saved processed point cloud to: %s", filepath.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception when saving PCD file: %s", e.what());
        }
    }
    
    void publishResults()
    {   
        try {
            if (publish_input_cloud_) {
                sensor_msgs::msg::PointCloud2 input_cloud_msg;
                pcl::toROSMsg(*input_cloud_, input_cloud_msg);
                input_cloud_msg.header.stamp = this->now();
                input_cloud_msg.header.frame_id = frame_id_;
                input_cloud_pub_->publish(input_cloud_msg);
            }
            
            if (publish_processed_cloud_) {
                sensor_msgs::msg::PointCloud2 output_cloud_msg;
                pcl::toROSMsg(*output_cloud_, output_cloud_msg);
                output_cloud_msg.header.stamp = this->now();
                output_cloud_msg.header.frame_id = frame_id_;
                processed_cloud_pub_->publish(output_cloud_msg);
            }
            
            RCLCPP_INFO(this->get_logger(), "Published %zu processed points", output_cloud_->size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud processing exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdVoxelGridFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}