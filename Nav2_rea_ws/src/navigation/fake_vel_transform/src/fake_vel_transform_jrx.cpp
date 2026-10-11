#include "fake_vel_transform/fake_vel_transform.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <atomic>
#include <mutex>
#include <deque>
#include <cmath>

namespace fake_vel_transform
{

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");
  
  // 参数声明与获取
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("robot_base_frame", "chassis_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "chassis_link_fake");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel_chassis");
  this->declare_parameter<float>("spin_speed", 0.0);
  this->declare_parameter<double>("tf_update_rate", 100.0);
  this->declare_parameter<int>("angle_filter_window", 5);
  this->declare_parameter<bool>("use_internal_angle_tracking", true);
  this->declare_parameter<double>("max_tf_age", 0.1); // 最大TF老化时间，秒
  this->declare_parameter<double>("velocity_ramp_time", 0.2); // 速度渐变时间，秒
  
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("spin_speed", spin_speed_);
  this->get_parameter("angle_filter_window", angle_filter_window_);
  this->get_parameter("use_internal_angle_tracking", use_internal_angle_tracking_);
  this->get_parameter("max_tf_age", max_tf_age_);
  this->get_parameter("velocity_ramp_time", velocity_ramp_time_);
  
  double tf_update_rate;
  this->get_parameter("tf_update_rate", tf_update_rate);
  
  spin_speed_storage = spin_speed_;
  current_robot_base_angle_ = 0.0;
  target_spin_speed_ = spin_speed_;
  current_spin_speed_ = 0.0;
  
  // 初始化所有时间为当前节点时间
  last_update_time_ = this->now();
  last_cmd_time_ = this->now();
  last_control_time_ = this->now();
  last_tf_time_ = this->now();
  spin_speed_change_time_ = this->now();
  
  // 初始化TF相关组件
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  // 初始化发布者、订阅者和服务
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
  
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  
  ai_tuoluo_sub_ = this->create_subscription<std_msgs::msg::Int8>(
    "/ai_tuoluo", 10,
    std::bind(&FakeVelTransform::tuoluoCallback, this, std::placeholders::_1));
  
  // 健康检查服务
  health_check_service_ = this->create_service<std_srvs::srv::Trigger>(
    "fake_vel_transform/health_check",
    std::bind(&FakeVelTransform::healthCheckCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  // 重置服务
  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "fake_vel_transform/reset",
    std::bind(&FakeVelTransform::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  // 创建定时器
  int update_period_ms = static_cast<int>(1000.0 / tf_update_rate);
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_period_ms),
    std::bind(&FakeVelTransform::timerCallback, this));
  
  // 更高频率的控制循环
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), // 100Hz控制循环
    std::bind(&FakeVelTransform::controlTimerCallback, this));
}

void FakeVelTransform::timerCallback()
{
  try
  {
    geometry_msgs::msg::TransformStamped t;
    // 使用最新的变换，而不是零时间
    t = tf2_buffer_->lookupTransform(map_frame_, robot_base_frame_, tf2::TimePointZero);
    
    // 获取当前时间并计算时间差
    auto current_time = this->now();
    // double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    
    // 获取当前角度
    float new_angle = tf2::getYaw(t.transform.rotation);
    
    // 使用滑动窗口过滤角度
    angle_history_.push_back(new_angle);
    if (angle_history_.size() > static_cast<size_t>(angle_filter_window_)) {
      angle_history_.pop_front();
    }
    
    // 计算平均角度（简单滤波）
    if (angle_history_.size() > 0) {
      float sum = 0.0;
      for (const auto& angle : angle_history_) {
        sum += angle;
      }
      current_robot_base_angle_ = sum / angle_history_.size();
    }
    
    // 更新TF成功状态和时间戳
    last_tf_success_ = true;
    last_tf_time_ = current_time;
    
    // 发布变换
    t.header.stamp = current_time;
    t.header.frame_id = robot_base_frame_;
    t.child_frame_id = fake_robot_base_frame_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, -current_robot_base_angle_);
    t.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_->sendTransform(t);
  }
  catch(const std::exception& e)
  {
    last_tf_success_ = false;
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Could not transform %s to %s: %s", 
      map_frame_.c_str(), robot_base_frame_.c_str(), e.what());
  }
}

// 高频控制循环
void FakeVelTransform::controlTimerCallback()
{
  // 获取当前时间并计算时间差
  auto current_time = this->now();
  double dt = (current_time - last_control_time_).seconds();
  last_control_time_ = current_time;
  
  // 检查TF数据是否过期
  double tf_age = (current_time - last_tf_time_).seconds();
  bool tf_valid = last_tf_success_ && (tf_age < max_tf_age_);
  
  // 内部角度追踪（当启用时）
  if (use_internal_angle_tracking_ && dt > 0.0) {
    if (tf_valid) {
      // 当TF有效时，将内部角度与TF角度对齐
      internal_angle_ = current_robot_base_angle_;
    } else {
      // 当TF无效时，使用速度积分更新内部角度
      internal_angle_ += current_spin_speed_ * dt;
      
      // 规范化角度到[-π, π]
      while (internal_angle_ > M_PI) internal_angle_ -= 2.0 * M_PI;
      while (internal_angle_ < -M_PI) internal_angle_ += 2.0 * M_PI;
    }
  }
  
  // 平滑过渡到目标速度
  if (velocity_ramp_time_ > 0.0 && dt > 0.0) {
    double max_change = dt / velocity_ramp_time_ * std::abs(target_spin_speed_);
    if (current_spin_speed_ < target_spin_speed_) {
      current_spin_speed_ = std::min(float(current_spin_speed_ + max_change), target_spin_speed_);
    } else if (current_spin_speed_ > target_spin_speed_) {
      current_spin_speed_ = std::max(float(current_spin_speed_ - max_change), target_spin_speed_);
    }
  } else {
    current_spin_speed_ = target_spin_speed_;
  }
  
  // 仅当有最近的速度命令时处理
  double cmd_age = (current_time - last_cmd_time_).seconds();
  if (cmd_age < 0.5 && !last_cmd_vel_.empty()) {
    processAndPublishVelocity();
  }
}

void FakeVelTransform::processAndPublishVelocity()
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  
  if (last_cmd_vel_.empty()) {
    return;
  }
  
  auto msg = last_cmd_vel_.front();
  
  // 确定使用哪个角度（TF或内部追踪）
  float angle_to_use = current_robot_base_angle_;
  if (use_internal_angle_tracking_ && !last_tf_success_) {
    angle_to_use = internal_angle_;
  }
  
  // 转换速度
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = current_spin_speed_; // 使用平滑后的速度
  aft_tf_vel.linear.x = msg->linear.x * cos(angle_to_use) + msg->linear.y * sin(angle_to_use);
  aft_tf_vel.linear.y = -msg->linear.x * sin(angle_to_use) + msg->linear.y * cos(angle_to_use);
  
  // 发布转换后的速度命令
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  
  // 保存当前时间
  last_cmd_time_ = this->now();
  
  // 保存最新的速度命令
  last_cmd_vel_.push_back(msg);
  if (last_cmd_vel_.size() > 5) { // 保持一个小的缓冲区
    last_cmd_vel_.pop_front();
  }
  
  // 立即处理速度以减少延迟
  processAndPublishVelocity();
}

void FakeVelTransform::tuoluoCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  if(msg->data == 1) {
    target_spin_speed_ = spin_speed_storage;
  } 
  else if(msg->data == 0) {
    target_spin_speed_ = 0.0;
  } 
  else {
    target_spin_speed_ = 0.0;
  }
  
  // 记录变化时间以便速度平滑
  spin_speed_change_time_ = this->now();
}

// 健康检查服务回调
void FakeVelTransform::healthCheckCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // 未使用
  
  auto current_time = this->now();
  double tf_age = (current_time - last_tf_time_).seconds();
  
  if (last_tf_success_ && tf_age < max_tf_age_) {
    response->success = true;
    response->message = "FakeVelTransform is healthy. Last TF age: " + std::to_string(tf_age) + " seconds";
  } else {
    response->success = false;
    response->message = "FakeVelTransform is unhealthy. TF success: " + 
                        std::to_string(last_tf_success_) + 
                        ", TF age: " + std::to_string(tf_age) + " seconds";
  }
}

// 重置服务回调
void FakeVelTransform::resetCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // 未使用
  
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  
  // 清除历史数据
  angle_history_.clear();
  last_cmd_vel_.clear();
  
  // 重置速度和角度
  current_spin_speed_ = 0.0;
  target_spin_speed_ = spin_speed_;
  internal_angle_ = 0.0;
  
  // 重置所有时间为当前节点时间
  auto current_time = this->now();
  last_update_time_ = current_time;
  last_cmd_time_ = current_time;
  last_control_time_ = current_time;
  last_tf_time_ = current_time;
  spin_speed_change_time_ = current_time;
  
  response->success = true;
  response->message = "FakeVelTransform has been reset successfully";
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)