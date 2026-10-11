#include <math.h>
#include <deque>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TerrainAnalysisNode : public rclcpp::Node {
public:
  TerrainAnalysisNode() : Node("terrain_analysis") {
    // 声明并获取参数
    this->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
    this->declare_parameter<double>("decayTime", decayTime);
    this->declare_parameter<double>("noDecayDis", noDecayDis);
    this->declare_parameter<double>("clearingDis", clearingDis);
    this->declare_parameter<bool>("useSorting", useSorting);
    this->declare_parameter<double>("quantileZ", quantileZ);
    this->declare_parameter<bool>("considerDrop", considerDrop);
    this->declare_parameter<bool>("limitGroundLift", limitGroundLift);
    this->declare_parameter<double>("maxGroundLift", maxGroundLift);
    this->declare_parameter<bool>("clearDyObs", clearDyObs);
    this->declare_parameter<double>("minDyObsDis", minDyObsDis);
    this->declare_parameter<double>("minDyObsAngle", minDyObsAngle);
    this->declare_parameter<double>("minDyObsRelZ", minDyObsRelZ);
    this->declare_parameter<double>("absDyObsRelZThre", absDyObsRelZThre);
    this->declare_parameter<double>("minDyObsVFOV", minDyObsVFOV);
    this->declare_parameter<double>("maxDyObsVFOV", maxDyObsVFOV);
    this->declare_parameter<int>("minDyObsPointNum", minDyObsPointNum);
    this->declare_parameter<bool>("noDataObstacle", noDataObstacle);
    this->declare_parameter<int>("noDataBlockSkipNum", noDataBlockSkipNum);
    this->declare_parameter<int>("minBlockPointNum", minBlockPointNum);
    this->declare_parameter<double>("vehicleHeight", vehicleHeight);
    this->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
    this->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
    this->declare_parameter<double>("minRelZ", minRelZ);
    this->declare_parameter<double>("maxRelZ", maxRelZ);
    this->declare_parameter<double>("disRatioZ", disRatioZ);

    this->get_parameter("scanVoxelSize", scanVoxelSize);
    this->get_parameter("decayTime", decayTime);
    this->get_parameter("noDecayDis", noDecayDis);
    this->get_parameter("clearingDis", clearingDis);
    this->get_parameter("useSorting", useSorting);
    this->get_parameter("quantileZ", quantileZ);
    this->get_parameter("considerDrop", considerDrop);
    this->get_parameter("limitGroundLift", limitGroundLift);
    this->get_parameter("maxGroundLift", maxGroundLift);
    this->get_parameter("clearDyObs", clearDyObs);
    this->get_parameter("minDyObsDis", minDyObsDis);
    this->get_parameter("minDyObsAngle", minDyObsAngle);
    this->get_parameter("minDyObsRelZ", minDyObsRelZ);
    this->get_parameter("absDyObsRelZThre", absDyObsRelZThre);
    this->get_parameter("minDyObsVFOV", minDyObsVFOV);
    this->get_parameter("maxDyObsVFOV", maxDyObsVFOV);
    this->get_parameter("minDyObsPointNum", minDyObsPointNum);
    this->get_parameter("noDataObstacle", noDataObstacle);
    this->get_parameter("noDataBlockSkipNum", noDataBlockSkipNum);
    this->get_parameter("minBlockPointNum", minBlockPointNum);
    this->get_parameter("vehicleHeight", vehicleHeight);
    this->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
    this->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
    this->get_parameter("minRelZ", minRelZ);
    this->get_parameter("maxRelZ", maxRelZ);
    this->get_parameter("disRatioZ", disRatioZ);

    // 初始化订阅者
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 5, std::bind(&TerrainAnalysisNode::odometryHandler, this, std::placeholders::_1));

    laser_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_registered", 5, std::bind(&TerrainAnalysisNode::laserCloudHandler, this, std::placeholders::_1));

    joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 5, std::bind(&TerrainAnalysisNode::joystickHandler, this, std::placeholders::_1));

    clearing_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "map_clearing", 5, std::bind(&TerrainAnalysisNode::clearingHandler, this, std::placeholders::_1));

    // 初始化发布者
    terrain_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 2);

    // 初始化点云对象
    laserCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDwz.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudElev.reset(new pcl::PointCloud<pcl::PointXYZI>());
    obstacleCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // 初始化地形体素点云
    for (int i = 0; i < kTerrainVoxelNum; i++) {
      terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    // 设置下采样滤波器
    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

    RCLCPP_INFO(this->get_logger(), "TerrainAnalysisNode 初始化完成");
  }

private:
  // 参数
  double scanVoxelSize = 0.05;
  double decayTime = 2.0;
  double noDecayDis = 4.0;
  double clearingDis = 8.0;
  bool clearingCloud = false;
  bool useSorting = true;
  double quantileZ = 0.25;
  bool considerDrop = false;
  bool limitGroundLift = false;
  double maxGroundLift = 0.15;
  bool clearDyObs = false;
  double minDyObsDis = 0.3;
  double minDyObsAngle = 0;
  double minDyObsRelZ = -0.5;
  double absDyObsRelZThre = 0.2;
  double minDyObsVFOV = -16.0;
  double maxDyObsVFOV = 16.0;
  int minDyObsPointNum = 1;
  bool noDataObstacle = false;
  int noDataBlockSkipNum = 0;
  int minBlockPointNum = 10;
  double vehicleHeight = 1.5;
  int voxelPointUpdateThre = 100;
  double voxelTimeUpdateThre = 2.0;
  double minRelZ = -1.5;
  double maxRelZ = 0.2;
  double disRatioZ = 0.2;

  // 用于累积点云的变量
  const int accumulateFrameNum = 10;
  std::deque<pcl::PointCloud<pcl::PointXYZI>> accumulatedObstacleClouds;
  double lastPublishTime = 0.0;

  // terrain voxel parameters
  float terrainVoxelSize = 0.5;
  int terrainVoxelShiftX = 0;
  int terrainVoxelShiftY = 0;
  const int terrainVoxelWidth = 41;  //65
  int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
  static constexpr int kTerrainVoxelNum = 41 * 41;

  // planar voxel parameters
  float planarVoxelSize = 0.1;
  const int planarVoxelWidth = 201;  //321
  int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
  static constexpr int kPlanarVoxelNum = 201 * 201;

  // 点云对象
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev;
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulatedCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[41 * 41];

  // 数组
  int terrainVoxelUpdateNum[41 * 41] = {0};
  float terrainVoxelUpdateTime[41 * 41] = {0};
  float planarVoxelElev[201 * 201] = {0};
  int planarVoxelEdge[201 * 201] = {0};
  int planarVoxelDyObs[201 * 201] = {0};
  std::vector<float> planarPointElev[201 * 201];

  // 时间和状态变量
  double laserCloudTime = 0;
  bool newlaserCloud = false;
  double systemInitTime = 0;
  bool systemInited = false;
  int noDataInited = 0;

  // 位姿相关变量
  float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
  float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
  float vehicleXRec = 0, vehicleYRec = 0;
  float sinVehicleRoll = 0, cosVehicleRoll = 0;
  float sinVehiclePitch = 0, cosVehiclePitch = 0;
  float sinVehicleYaw = 0, cosVehicleYaw = 0;

  // 滤波器
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

  // 订阅者和发布者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr clearing_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_cloud_pub_;

  // 回调函数：处理里程计数据
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
        .getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    sinVehicleRoll = sin(vehicleRoll);
    cosVehicleRoll = cos(vehicleRoll);
    sinVehiclePitch = sin(vehiclePitch);
    cosVehiclePitch = cos(vehiclePitch);
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);

    if (noDataInited == 0) {
      vehicleXRec = vehicleX;
      vehicleYRec = vehicleY;
      noDataInited = 1;
    }
    if (noDataInited == 1) {
      float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                      (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
      if (dis >= noDecayDis)
        noDataInited = 2;
    }
  }

  // 回调函数：处理激光点云数据
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
    if (!systemInited) {
      systemInitTime = laserCloudTime;
      systemInited = true;
    }

    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                      (pointY - vehicleY) * (pointY - vehicleY));
      if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
          pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
          dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = laserCloudTime - systemInitTime;
        laserCloudCrop->push_back(point);
      }
    }

    // 处理点云数据 - 原main函数中的处理流程
    processPointCloud();
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // std::cout << "点云处理时间:" << static_cast<double>(duration.count()) << " us" << std::endl;
  }

  // 回调函数：处理手柄数据
  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
    if (joy->buttons[5] > 0.5) {
      noDataInited = 0;
      clearingCloud = true;
    }
  }

  // 回调函数：处理地图清除数据
  void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
    noDataInited = 0;
    clearingDis = dis->data;
    clearingCloud = true;
  }

  // 处理点云数据的主函数 - 从main函数移植过来的处理流程
  void processPointCloud() {
    // terrain voxel roll over
    float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

    while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                              indY];
        for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
        }
        terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
        terrainVoxelCloud[indY]->clear();
      }
      terrainVoxelShiftX--;
      terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    }

    while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[indY];
        for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
        }
        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                          indY] = terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
            ->clear();
      }
      terrainVoxelShiftX++;
      terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    }

    while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
      for (int indX = 0; indX < terrainVoxelWidth; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX +
                              (terrainVoxelWidth - 1)];
        for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
        }
        terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
      }
      terrainVoxelShiftY--;
      terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
    }

    while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
      for (int indX = 0; indX < terrainVoxelWidth; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX];
        for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
        }
        terrainVoxelCloud[terrainVoxelWidth * indX +
                          (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
            ->clear();
      }
      terrainVoxelShiftY++;
      terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
    }

    // stack registered laser scans
    pcl::PointXYZI point;
    int laserCloudCropSize = laserCloudCrop->points.size();
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = laserCloudCrop->points[i];

      int indX =
          static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2) /
                          terrainVoxelSize) +
          terrainVoxelHalfWidth;
      int indY =
          static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) /
                          terrainVoxelSize) +
          terrainVoxelHalfWidth;

      if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
        indY--;

      if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
          indY < terrainVoxelWidth) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
        terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
      }
    }

    for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
      if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
          laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
              voxelTimeUpdateThre ||
          clearingCloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[ind];

        laserCloudDwz->clear();
        // downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
        // downSizeFilter.filter(*laserCloudDwz);

        terrainVoxelCloudPtr->clear();
        int laserCloudDwzSize = terrainVoxelCloudPtr->points.size();
        for (int i = 0; i < laserCloudDwzSize; i++) {
          point = terrainVoxelCloudPtr->points[i];
          float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                          (point.y - vehicleY) * (point.y - vehicleY));
          if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
              point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
              (laserCloudTime - systemInitTime - point.intensity <
                   decayTime ||
               dis < noDecayDis) &&
              !(dis < clearingDis && clearingCloud)) {
            terrainVoxelCloudPtr->push_back(point);
          }
        }

        // int laserCloudDwzSize = laserCloudDwz->points.size();
        // for (int i = 0; i < laserCloudDwzSize; i++) {
        //   point = laserCloudDwz->points[i];
        //   float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
        //                   (point.y - vehicleY) * (point.y - vehicleY));
        //   if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
        //       point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
        //       (laserCloudTime - systemInitTime - point.intensity <
        //            decayTime ||
        //        dis < noDecayDis) &&
        //       !(dis < clearingDis && clearingCloud)) {
        //     terrainVoxelCloudPtr->push_back(point);
        //   }
        // }

        terrainVoxelUpdateNum[ind] = 0;
        terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
      }
    }

    terrainCloud->clear();
    for (int indX = terrainVoxelHalfWidth - 15;
         indX <= terrainVoxelHalfWidth + 15; indX++) {
      for (int indY = terrainVoxelHalfWidth - 15;
           indY <= terrainVoxelHalfWidth + 15; indY++) {
        *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
      }
    }

    // estimate ground and compute elevation for each point
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      planarVoxelElev[i] = 0;
      planarVoxelEdge[i] = 0;
      planarVoxelDyObs[i] = 0;
      planarPointElev[i].clear();
    }

    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                  planarVoxelSize) +
                 planarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                  planarVoxelSize) +
                 planarVoxelHalfWidth;

      if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;

      if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
        for (int dX = -1; dX <= 1; dX++) {
          for (int dY = -1; dY <= 1; dY++) {
            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                indY + dY >= 0 && indY + dY < planarVoxelWidth) {
              planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                  .push_back(point.z);
            }
          }
        }
      }

      if (clearDyObs) {
        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
            indY < planarVoxelWidth) {
          float pointX1 = point.x - vehicleX;
          float pointY1 = point.y - vehicleY;
          float pointZ1 = point.z - vehicleZ;

          float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
          float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / M_PI;
          if (angle1 > minDyObsAngle) {
            planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
          }
        }
      }
    }

    if (useSorting) {
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        int planarPointElevSize = planarPointElev[i].size();
        if (planarPointElevSize > 0) {
          sort(planarPointElev[i].begin(), planarPointElev[i].end());

          int quantileID = static_cast<int>(quantileZ * planarPointElevSize);
          if (quantileID < 0)
            quantileID = 0;
          else if (quantileID >= planarPointElevSize)
            quantileID = planarPointElevSize - 1;

          if (planarPointElev[i][quantileID] >
                  planarPointElev[i][0] + maxGroundLift &&
              limitGroundLift) {
            planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
          } else {
            planarVoxelElev[i] = planarPointElev[i][quantileID];
          }
        }
      }
    } else {
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        int planarPointElevSize = planarPointElev[i].size();
        if (planarPointElevSize > 0) {
          float minZ = 1000.0;
          int minID = -1;
          for (int j = 0; j < planarPointElevSize; j++) {
            if (planarPointElev[i][j] < minZ) {
              minZ = planarPointElev[i][j];
              minID = j;
            }
          }

          if (minID != -1) {
            planarVoxelElev[i] = planarPointElev[i][minID];
          }
        }
      }
    }

    terrainCloudElev->clear();
    int terrainCloudElevSize = 0;

    obstacleCloud->clear();
    int obstacleCloudSize = 0;

    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];
      if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
        int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                  planarVoxelHalfWidth;
        int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                  planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
            indY < planarVoxelWidth) {
          if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                  minDyObsPointNum ||
              !clearDyObs) {
            float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
            int planarPointElevSize =
                planarPointElev[planarVoxelWidth * indX + indY].size();
                
            if (disZ > 0.1 && disZ < vehicleHeight && 
                planarPointElevSize >= minBlockPointNum) {
              point.intensity = disZ;
              obstacleCloud->push_back(point);
              obstacleCloudSize++;
            }
          }
        }
      }
    }

    if (noDataObstacle && noDataInited == 2) {
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        int planarPointElevSize = planarPointElev[i].size();
        if (planarPointElevSize < minBlockPointNum) {
          planarVoxelEdge[i] = 1;
        }
      }

      for (int noDataBlockSkipCount = 0;
           noDataBlockSkipCount < noDataBlockSkipNum;
           noDataBlockSkipCount++) {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          if (planarVoxelEdge[i] >= 1) {
            int indX = static_cast<int>(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;
            bool edgeVoxel = false;
            for (int dX = -1; dX <= 1; dX++) {
              for (int dY = -1; dY <= 1; dY++) {
                if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                    indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                  if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                      dY] < planarVoxelEdge[i]) {
                    edgeVoxel = true;
                  }
                }
              }
            }

            if (!edgeVoxel)
              planarVoxelEdge[i]++;
          }
        }
      }

      for (int i = 0; i < kPlanarVoxelNum; i++) {
        if (planarVoxelEdge[i] > noDataBlockSkipNum) {
          int indX = static_cast<int>(i / planarVoxelWidth);
          int indY = i % planarVoxelWidth;

          point.x =
              planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
          point.y =
              planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
          point.z = vehicleZ;
          point.intensity = vehicleHeight;

          point.x -= planarVoxelSize / 4.0;
          point.y -= planarVoxelSize / 4.0;
          terrainCloudElev->push_back(point);

          point.x += planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);

          point.y += planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);

          point.x -= planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);
        }
      }
    }

    clearingCloud = false;

    // 实现点云累积功能
    accumulatedObstacleClouds.emplace_back(*obstacleCloud);
    
    if (accumulatedObstacleClouds.size() >= static_cast<size_t>(accumulateFrameNum)) {
      accumulatedObstacleClouds.pop_front();
    }
    if (clearingCloud) {
      accumulatedObstacleClouds.clear();
    }

    if (!accumulatedObstacleClouds.empty()) {
      accumulatedCloud->clear();
      for (const auto& cloud : accumulatedObstacleClouds) {
        *accumulatedCloud += cloud;
      }
    }
      
    // 发布合并后的点云
    sensor_msgs::msg::PointCloud2 obstacleCloud2;
    pcl::toROSMsg(*accumulatedCloud, obstacleCloud2);
    obstacleCloud2.header.stamp = 
        rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
    obstacleCloud2.header.frame_id = "odom_init";
    terrain_cloud_pub_->publish(obstacleCloud2);
    
    lastPublishTime = laserCloudTime;
    
    // 对于清除事件，直接发布当前帧
    if (clearingCloud) {
      sensor_msgs::msg::PointCloud2 obstacleCloud2;
      pcl::toROSMsg(*obstacleCloud, obstacleCloud2);
      obstacleCloud2.header.stamp =
          rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
      obstacleCloud2.header.frame_id = "odom_init";
      terrain_cloud_pub_->publish(obstacleCloud2);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainAnalysisNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}