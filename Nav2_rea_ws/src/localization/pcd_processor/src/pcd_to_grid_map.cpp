#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <mutex>
#include <atomic>
#include <string>

class PcdToGridMap : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // Processing parameters
    int grid_size_;
    double cell_size_;
    std::string frame_id_;
    std::string pcd_file_path_;
    double publish_rate_;
    float max_height_threshold_;      // Maximum height threshold
    float flatness_threshold_;        // Flatness threshold
    bool use_voxel_filter_;
    float voxel_size_;
    float map_resolution_;

    // Debug parameters
    bool publish_input_cloud_;
    bool publish_processed_cloud_;
    
    // Outlier removal parameters
    bool use_outlier_removal_;        // Whether to enable outlier removal
    std::string outlier_removal_method_;  // Outlier removal method: "statistical" or "radius"
    
    // Statistical outlier filter parameters
    int statistical_mean_k_;          // Number of neighbor points for statistics
    float statistical_std_dev_;       // Standard deviation multiplier
    
    // Radius outlier filter parameters
    float radius_search_;             // Search radius
    int min_neighbors_;               // Minimum number of neighbors
    
    // Synchronization protection
    std::mutex processing_mutex_;
    std::atomic<bool> is_processing_{false};
    
    // Point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    
    // Grid cell structure
    struct GridCell {
        float min_height = std::numeric_limits<float>::max();
        float max_height = -std::numeric_limits<float>::max();
        float flatness = 0.0f;           // Flatness, smaller means flatter
        int count = 0;
        std::vector<int> point_indices;  // Store indices of points mapped to this cell
    };
    
public:
    PcdToGridMap() : Node("pcd_to_grid_map"), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing point cloud processor...");
        
        this->declare_parameter("grid_size", 2000);
        this->declare_parameter("cell_size", 0.05);
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("pcd_file_path", "/home/eatwhat/nav_ws/src/preprocess/changzhou01.pcd");
        this->declare_parameter("publish_rate", 5.0);
        this->declare_parameter("max_height_threshold", 2.0);
        this->declare_parameter("flatness_threshold", 0.05);
        this->declare_parameter("use_voxel_filter", true);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("map_resolution", 0.05);
        
        this->declare_parameter("use_outlier_removal", true);
        this->declare_parameter("outlier_removal_method", "statistical");
        
        this->declare_parameter("statistical_mean_k", 50);
        this->declare_parameter("statistical_std_dev", 1.0);
        
        this->declare_parameter("radius_search", 0.08);
        this->declare_parameter("min_neighbors", 5);

        this->declare_parameter("publish_input_cloud", true);
        this->declare_parameter("publish_processed_cloud", true);

        grid_size_ = this->get_parameter("grid_size").as_int();
        cell_size_ = this->get_parameter("cell_size").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        max_height_threshold_ = this->get_parameter("max_height_threshold").as_double();
        flatness_threshold_ = this->get_parameter("flatness_threshold").as_double();
        use_voxel_filter_ = this->get_parameter("use_voxel_filter").as_bool();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        
        use_outlier_removal_ = this->get_parameter("use_outlier_removal").as_bool();
        outlier_removal_method_ = this->get_parameter("outlier_removal_method").as_string();
        
        statistical_mean_k_ = this->get_parameter("statistical_mean_k").as_int();
        statistical_std_dev_ = this->get_parameter("statistical_std_dev").as_double();
        
        radius_search_ = this->get_parameter("radius_search").as_double();
        min_neighbors_ = this->get_parameter("min_neighbors").as_int();
        
        publish_input_cloud_ = this->get_parameter("publish_input_cloud").as_bool();
        publish_processed_cloud_ = this->get_parameter("publish_processed_cloud").as_bool();

        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_qos.reliable();
        map_qos.keep_last(1);
                
        input_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("input_cloud", 5);
        processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_cloud", 5);
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
        
        if (!pcd_file_path_.empty()) {
            if (loadPCDFile(pcd_file_path_)) {
                publish_timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(1.0 / publish_rate_),
                    std::bind(&PcdToGridMap::publishResults, this)
                );
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unable to load PCD file: %s", pcd_file_path_.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "No PCD file path provided!");
        }

        RCLCPP_INFO(this->get_logger(), "Point cloud processor initialization complete (grid size: %d, cell size: %.3f)", 
                   grid_size_, cell_size_);
        RCLCPP_INFO(this->get_logger(), "Height threshold: %.2f, flatness threshold: %.3f", 
                   max_height_threshold_, flatness_threshold_);
        
        if (use_outlier_removal_) {
            RCLCPP_INFO(this->get_logger(), "Outlier removal enabled, method: %s", outlier_removal_method_.c_str());
            if (outlier_removal_method_ == "statistical") {
                RCLCPP_INFO(this->get_logger(), "Statistical outlier filter parameters - MeanK: %d, StdDev: %.2f", 
                           statistical_mean_k_, statistical_std_dev_);
            } else if (outlier_removal_method_ == "radius") {
                RCLCPP_INFO(this->get_logger(), "Radius outlier filter parameters - Radius: %.3f, Min neighbors: %d", 
                           radius_search_, min_neighbors_);
            }
        }
    }
    
private:
    bool loadPCDFile(std::string& filepath)
    {
        try {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud_) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Cannot read file: %s", filepath.c_str());
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %zu points", cloud_->points.size());
            
            if (use_voxel_filter_ && voxel_size_ > 0) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setInputCloud(cloud_);
                voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
                voxel_filter.filter(*filtered_cloud);
                
                RCLCPP_INFO(this->get_logger(), "Applied voxel filter, reduced point cloud from %zu to %zu points", 
                           cloud_->size(), filtered_cloud->size());
                cloud_ = filtered_cloud;
            }
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception when loading PCD file: %s", e.what());
            return false;
        }
    }
    
    void publishResults()
    {
        // Prevent concurrent processing
        if (is_processing_.exchange(true)) {
            RCLCPP_DEBUG(this->get_logger(), "Processing task already in progress, skipping this iteration");
            return;
        }
        
        try {
            if (!cloud_ || cloud_->empty()) {
                RCLCPP_WARN(this->get_logger(), "Point cloud is empty, cannot process");
                is_processing_ = false;
                return;
            }

            auto start_time = std::chrono::high_resolution_clock::now();

            if (publish_input_cloud_) {
                sensor_msgs::msg::PointCloud2 input_cloud_msg;
                pcl::toROSMsg(*cloud_, input_cloud_msg);
                input_cloud_msg.header.stamp = this->now();
                input_cloud_msg.header.frame_id = frame_id_;
                input_cloud_pub_->publish(input_cloud_msg);
            }
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            processPointCloud(cloud_, processed_cloud);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (use_outlier_removal_ && !processed_cloud->empty()) {
                removeOutliers(processed_cloud, outlier_removed_cloud);
            } else {
                outlier_removed_cloud = processed_cloud;
            }
            
            if (!outlier_removed_cloud->empty()) {
                if (publish_processed_cloud_) {
                    sensor_msgs::msg::PointCloud2 cloud_msg;
                    pcl::toROSMsg(*outlier_removed_cloud, cloud_msg);
                    cloud_msg.header.stamp = this->now();
                    cloud_msg.header.frame_id = frame_id_;
                    processed_cloud_pub_->publish(cloud_msg);
                }

                setMapMsg(outlier_removed_cloud, map_msg_);
                map_publisher_->publish(map_msg_);
                
                RCLCPP_INFO(this->get_logger(), "Published %zu processed points", outlier_removed_cloud->size());
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            RCLCPP_INFO(this->get_logger(), "Processing complete: Time: %.2f ms", duration.count() / 1000.0);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud processing exception: %s", e.what());
        }
        
        is_processing_ = false;
    }
    
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
    {
        if (!input_cloud || input_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "processPointCloud: Empty point cloud");
            return;
        }

        output_cloud->clear();
        output_cloud->reserve(input_cloud->size());
        
        std::unordered_map<int, GridCell> grid;
        grid.reserve(input_cloud->size() / 3);
        
        float center_offset_x = grid_size_ / 2.0f * cell_size_;
        float center_offset_y = grid_size_ / 2.0f * cell_size_;
        
        int total_points = 0;
        int filtered_by_height = 0;
        
        for (size_t i = 0; i < input_cloud->points.size(); ++i) {
            const auto& point = input_cloud->points[i];
            
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            
            if (point.z > max_height_threshold_) {
                filtered_by_height++;
                continue;
            }
            
            total_points++;
            
            int grid_x = std::floor((point.x + center_offset_x) / cell_size_);
            int grid_y = std::floor((point.y + center_offset_y) / cell_size_);
            
            if (grid_x >= 0 && grid_x < grid_size_ && grid_y >= 0 && grid_y < grid_size_) {
                int key = grid_y * grid_size_ + grid_x;
                auto& cell = grid[key];
                
                cell.min_height = std::min(cell.min_height, point.z);
                cell.max_height = std::max(cell.max_height, point.z);
                cell.count++;
                cell.point_indices.push_back(i);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Stage 1 - Height filtering: Total points: %zu, Filtered points: %d, Remaining points: %d",
                   input_cloud->size(), filtered_by_height, total_points);
        
        int flat_cells = 0;
        int non_flat_cells = 0;
        
        for (auto& entry : grid) {
            auto& cell = entry.second;
            
            cell.flatness = cell.max_height - cell.min_height;

            if (cell.flatness > 1.2 && cell.flatness < 2.0) {
                cell.flatness = 0.01;
            }
            
            if (cell.flatness < flatness_threshold_) {
                flat_cells++;
            } else {
                non_flat_cells++;
                for (int idx : cell.point_indices) {
                    output_cloud->push_back(input_cloud->points[idx]);
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Stage 2 - Flatness filtering: Flat cells: %d, Non-flat cells: %d, Output points: %zu",
                   flat_cells, non_flat_cells, output_cloud->size());
    }
    
    void removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
    {
        if (!input_cloud || input_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "removeOutliers: Empty point cloud");
            output_cloud = input_cloud;
            return;
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();
        size_t original_size = input_cloud->size();
        
        try {
            if (outlier_removal_method_ == "statistical") {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(input_cloud);
                sor.setMeanK(statistical_mean_k_);
                sor.setStddevMulThresh(statistical_std_dev_);
                sor.filter(*output_cloud);

                pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
                ror.setInputCloud(output_cloud);
                ror.setRadiusSearch(radius_search_);
                ror.setMinNeighborsInRadius(min_neighbors_);
                ror.filter(*output_cloud);
                
                RCLCPP_INFO(this->get_logger(), "Applied statistical outlier filter (MeanK: %d, StdDev: %.2f)",
                           statistical_mean_k_, statistical_std_dev_);
            }
            else if (outlier_removal_method_ == "radius") {
                pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
                ror.setInputCloud(input_cloud);
                ror.setRadiusSearch(radius_search_);
                ror.setMinNeighborsInRadius(min_neighbors_);
                ror.filter(*output_cloud);
                
                RCLCPP_INFO(this->get_logger(), "Applied radius outlier filter (Radius: %.3f, Min neighbors: %d)",
                           radius_search_, min_neighbors_);
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Unknown outlier removal method: %s, using original point cloud", 
                           outlier_removal_method_.c_str());
                output_cloud = input_cloud;
                return;
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            
            RCLCPP_INFO(this->get_logger(), "Outlier removal: Reduced from %zu to %zu points (Removed %zu points, Time: %.2f ms)",
                       original_size, output_cloud->size(), 
                       original_size - output_cloud->size(),
                       duration.count() / 1000.0);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Outlier removal exception: %s, using original point cloud", e.what());
            output_cloud = input_cloud;
        }
    }

    void setMapMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
    {
        msg.header.stamp = now();
        msg.header.frame_id = "map";

        msg.info.map_load_time = now();
        msg.info.resolution = map_resolution_;

        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::lowest();
        double y_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::lowest();

        if (cloud->points.empty()) {
            RCLCPP_WARN(get_logger(), "Point cloud is empty!");
            return;
        }

        for (const auto & point : cloud->points) {
            x_min = std::min(x_min, static_cast<double>(point.x));
            x_max = std::max(x_max, static_cast<double>(point.x));
            y_min = std::min(y_min, static_cast<double>(point.y));
            y_max = std::max(y_max, static_cast<double>(point.y));
        }

        msg.info.origin.position.x = x_min;
        msg.info.origin.position.y = y_min;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
        msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
        msg.data.assign(msg.info.width * msg.info.height, 0);

        for (const auto & point : cloud->points) {
            int i = std::floor((point.x - x_min) / map_resolution_);
            int j = std::floor((point.y - y_min) / map_resolution_);

            if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
            msg.data[i + j * msg.info.width] = 100;
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdToGridMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}