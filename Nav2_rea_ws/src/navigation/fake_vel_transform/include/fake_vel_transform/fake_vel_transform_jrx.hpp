#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <memory>
#include <string>
#include <deque>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void timerCallback();
  void controlTimerCallback();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void tuoluoCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void healthCheckCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void processAndPublishVelocity();

  // TF相关
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 话题发布和订阅
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr ai_tuoluo_sub_;
  
  // 服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr health_check_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  // 参数
  std::string map_frame_;
  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  float spin_speed_;
  float spin_speed_storage;
  int angle_filter_window_{5};
  bool use_internal_angle_tracking_{true};
  double max_tf_age_{0.1};
  double velocity_ramp_time_{0.2};
  
  // 角度和速度状态
  float current_robot_base_angle_{0.0};
  float internal_angle_{0.0};
  float current_spin_speed_{0.0};
  float target_spin_speed_{0.0};
  
  // 状态跟踪
  rclcpp::Time last_update_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_tf_time_;
  rclcpp::Time spin_speed_change_time_;
  bool last_tf_success_{false};
  
  // 数据缓冲
  std::deque<float> angle_history_;
  std::deque<geometry_msgs::msg::Twist::SharedPtr> last_cmd_vel_;
  std::mutex cmd_vel_mutex_;
};
}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_