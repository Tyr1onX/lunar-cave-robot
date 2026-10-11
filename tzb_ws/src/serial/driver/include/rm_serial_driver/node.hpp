#ifndef _RM_SERIAL_DRIVER_NODE_HPP
#define _RM_SERIAL_DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rcpputils/filesystem_helper.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <cstdint>
#include <unordered_set>
#include <atomic>
#include <chrono>
#include <vector>
#include <mutex>
#include <string>

#include <rm_serial_driver/packet.hpp>

namespace rm::serial
{
class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  ~DriverNode() override;

private:
  enum class ArmSafetyState { kDisarmed, kArmedWaitCommand, kActive, kFaultLatched };

  drivers::serial_driver::SerialPortConfig getDeviceConfig();
  
  void navCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void armCommandCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void gripperCommandCallback(std_msgs::msg::Int8::ConstSharedPtr msg);
  void trajectoryGripperCommandCallback(std_msgs::msg::String::ConstSharedPtr msg);
  void trajectoryCancelCallback(std_msgs::msg::String::ConstSharedPtr msg);
  void timerCallback();
  void statusTimerCallback();
  void resetSafetyCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void enableSafetyCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void startSerialReceive();
  void serialReceiveCallback(const std::vector<uint8_t>& data, std::size_t size);
  void publishArmState(const ReceivePacket& packet);

  void reopenPort();
  void updateArmCommandLocked(double dt_sec);
  void latchFaultLocked(const std::string& reason);
  bool persistTrajectoryEpochLocked(uint64_t epoch);
  bool commandSourceQuietLocked(std::chrono::steady_clock::time_point now) const;
  bool feedbackFreshLocked(std::chrono::steady_clock::time_point now) const;
  static const char* safetyStateName(ArmSafetyState state);
  float angleToSerialUnit(float angle_rad, std::size_t joint_index) const;
  float angleFromSerialUnit(float serial_angle, std::size_t joint_index) const;

  // ******************** ROS Subscription ********************
  // Nav Subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_gripper_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_cancel_sub_;

  // ******************** ROS Publisher ********************
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_cancel_ack_pub_;

  // ******************** ROS Timer ********************
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_safety_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_safety_service_;

  // ******************** Send ********************
  NavPacket nav_packet_;
  ArmJointPacket arm_packet_;
  std::array<float, kArmJointCount> arm_target_position_{};
  std::array<float, kArmJointCount> arm_command_position_rad_{};
  std::array<float, kArmJointCount> arm_measured_position_rad_{};
  std::array<float, kArmJointCount> max_arm_command_velocity_rad_s_{};
  std::array<float, kArmJointCount> arm_joint_lower_limits_rad_{};
  std::array<float, kArmJointCount> arm_joint_upper_limits_rad_{};
  std::vector<std::string> arm_joint_names_;
  uint32_t arm_sequence_{0};
  bool arm_serial_enabled_{true};
  bool include_arm_extension_{true};
  bool suppress_expected_serial_abort_logs_{true};
  bool arm_serial_angles_in_degrees_{false};
  bool have_arm_command_{false};
  bool have_arm_feedback_{false};
  ArmSafetyState arm_safety_state_{ArmSafetyState::kDisarmed};
  std::string arm_fault_reason_{"startup requires explicit enable"};
  std::string last_safety_event_{"startup"};
  bool logged_first_arm_command_{false};
  bool logged_first_enabled_serial_packet_{false};
  int8_t gripper_command_{0};
  double gripper_command_log_interval_sec_{1.0};
  double arm_command_publish_hz_{100.0};
  double arm_command_speed_scale_{1.0};
  double arm_command_goal_tolerance_rad_{0.005};
  double arm_command_lease_sec_{0.1};
  double arm_command_max_age_sec_{0.1};
  double arm_command_future_tolerance_sec_{0.05};
  double arm_feedback_timeout_sec_{0.5};
  double arm_rearm_quiet_sec_{0.1};
  double safety_status_publish_hz_{2.0};
  std::string trajectory_epoch_state_path_;
  std::string trajectory_gripper_topic_{"/p6/arm_task/gripper_command"};
  std::string trajectory_cancel_topic_{"/p6/arm_task/cancel"};
  std::string trajectory_cancel_ack_topic_{"/serial/arm_task/cancel_ack"};
  uint64_t trajectory_epoch_high_water_{0};
  uint64_t accepted_trajectory_epoch_{0};
  uint64_t last_trajectory_sequence_{0};
  std::string accepted_trajectory_task_id_;
  std::unordered_set<std::string> cancelled_trajectory_tasks_;
  int64_t arm_enable_epoch_stamp_ns_{0};
  int64_t last_accepted_command_stamp_ns_{0};
  std::chrono::steady_clock::time_point last_command_rx_time_{};
  std::chrono::steady_clock::time_point last_valid_command_time_{};
  std::chrono::steady_clock::time_point last_valid_feedback_time_{};
  uint64_t invalid_feedback_count_{0};
  uint64_t feedback_sequence_gap_count_{0};
  std::chrono::steady_clock::time_point last_send_time_{};
  std::chrono::steady_clock::time_point last_gripper_command_log_time_{};

  // ******************** Mutex ********************
  std::mutex nav_mutex_;
  std::mutex arm_mutex_;
  
  // ******************** Serial Port ********************
  std::mutex serial_port_mutex_;
  std::mutex receive_parser_mutex_;
  ReceiveFrameDecoder receive_frame_decoder_;
  ReceiveSequenceTracker receive_sequence_tracker_;
  std::atomic_bool serial_receive_enabled_{false};
  std::atomic_bool serial_reopen_in_progress_{false};
  drivers::common::IoContext owned_ctx_;
  drivers::serial_driver::SerialDriver serial_driver_;
};
}

#endif
