// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <serial_driver/serial_driver.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cstdint>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

#include <rm_serial_driver/node.hpp>

namespace rm::serial
{
namespace
{
std::string normalizeAngleUnit(std::string unit)
{
  std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return unit;
}

bool isDegreeUnit(const std::string& unit)
{
  return unit == "deg" || unit == "degree" || unit == "degrees";
}

bool isRadianUnit(const std::string& unit)
{
  return unit == "rad" || unit == "radian" || unit == "radians";
}
struct TrajectoryCommandContext { std::string task_id; uint64_t epoch{0}; uint64_t sequence{0}; };
bool parseUnsigned(const std::string& text, uint64_t& value)
{
  if (text.empty() || !std::all_of(text.begin(), text.end(), [](unsigned char c) {
        return std::isdigit(c);
      })) return false;
  try { std::size_t used=0; auto n=std::stoull(text,&used,10); if (used!=text.size()) return false; value=static_cast<uint64_t>(n); return true; }
  catch (const std::exception&) { return false; }
}
bool parseTrajectoryContext(const std::string& frame_id, TrajectoryCommandContext& out)
{
  if (frame_id.rfind("p4f:",0)!=0) return false;
  const auto a=frame_id.find(':',4), b=(a==std::string::npos ? a : frame_id.find(':',a+1));
  if (a==std::string::npos || b==std::string::npos || frame_id.find(':',b+1)!=std::string::npos) return false;
  out.task_id=frame_id.substr(4,a-4);
  return !out.task_id.empty() && parseUnsigned(frame_id.substr(a+1,b-a-1),out.epoch) &&
    parseUnsigned(frame_id.substr(b+1),out.sequence) && out.epoch>0 && out.sequence>0;
}
std::string trajectoryKey(const std::string& id, uint64_t epoch) { return id+"|"+std::to_string(epoch); }
}  // namespace

DriverNode::DriverNode(const rclcpp::NodeOptions& options)
: Node("serial_driver", options),
  owned_ctx_(2), serial_driver_(owned_ctx_)
{
  using namespace std::placeholders;

  auto device_name = declare_parameter<std::string>("device_name");
  auto device_config = getDeviceConfig();
  arm_serial_enabled_ = declare_parameter<bool>("tzb.arm_serial_enabled", true);
  include_arm_extension_ = declare_parameter<bool>("tzb.include_arm_serial_extension", true);
  suppress_expected_serial_abort_logs_ = declare_parameter<bool>(
    "tzb.suppress_expected_serial_abort_logs", true);
  if (suppress_expected_serial_abort_logs_)
  {
    // serial_driver logs this expected cancellation directly from its dependency.
    const auto logging_result = rcutils_logging_set_logger_level(
      "SerialPort::async_receive_handler", RCUTILS_LOG_SEVERITY_FATAL);
    if (logging_result != RCUTILS_RET_OK)
    {
      rcutils_reset_error();
    }
  }
  const auto arm_serial_angle_unit =
    normalizeAngleUnit(declare_parameter<std::string>("tzb.arm_serial_angle_unit", "rad"));
  if (!isRadianUnit(arm_serial_angle_unit) && !isDegreeUnit(arm_serial_angle_unit))
  {
    throw std::runtime_error("tzb.arm_serial_angle_unit must be 'rad' or 'deg'");
  }
  if (isDegreeUnit(arm_serial_angle_unit))
  {
    throw std::runtime_error(
      "The current STM32 Host protocol carries encoder radians; degree wire units are incompatible");
  }
  arm_serial_angles_in_degrees_ = false;
  arm_command_publish_hz_ = declare_parameter<double>("tzb.arm_command_publish_hz", 100.0);
  arm_command_speed_scale_ = declare_parameter<double>("tzb.arm_command_speed_scale", 1.0);
  if (!std::isfinite(arm_command_publish_hz_) || arm_command_publish_hz_ <= 0.0 ||
      !std::isfinite(arm_command_speed_scale_) || arm_command_speed_scale_ <= 0.0)
  {
    throw std::runtime_error("arm command publish rate and speed scale must be finite and positive");
  }
  arm_command_goal_tolerance_rad_ =
    std::max(0.0, declare_parameter<double>("tzb.arm_command_goal_tolerance_rad", 0.005));
  arm_command_lease_sec_ = declare_parameter<double>("tzb.arm_command_lease_sec", 0.1);
  arm_command_max_age_sec_ = declare_parameter<double>("tzb.arm_command_max_age_sec", 0.1);
  arm_command_future_tolerance_sec_ =
    declare_parameter<double>("tzb.arm_command_future_tolerance_sec", 0.05);
  arm_feedback_timeout_sec_ = declare_parameter<double>("tzb.arm_feedback_timeout_sec", 0.5);
  arm_rearm_quiet_sec_ = declare_parameter<double>(
    "tzb.arm_rearm_quiet_sec", arm_command_lease_sec_);
  safety_status_publish_hz_ = declare_parameter<double>("tzb.arm_safety_status_hz", 2.0);
  if (!std::isfinite(arm_command_lease_sec_) || arm_command_lease_sec_ <= 0.0 ||
      !std::isfinite(arm_command_max_age_sec_) || arm_command_max_age_sec_ <= 0.0 ||
      !std::isfinite(arm_command_future_tolerance_sec_) || arm_command_future_tolerance_sec_ < 0.0 ||
      !std::isfinite(arm_feedback_timeout_sec_) || arm_feedback_timeout_sec_ <= 0.0 ||
      !std::isfinite(arm_rearm_quiet_sec_) || arm_rearm_quiet_sec_ < arm_command_lease_sec_ ||
      !std::isfinite(safety_status_publish_hz_) || safety_status_publish_hz_ <= 0.0)
  {
    throw std::runtime_error("arm safety timing parameters are invalid");
  }
  gripper_command_log_interval_sec_ = std::max(
    0.0,
    declare_parameter<double>("tzb.gripper_command_log_interval_sec", 1.0));
  const char* home = std::getenv("HOME");
  trajectory_epoch_state_path_ = declare_parameter<std::string>(
    "tzb.trajectory_epoch_state_path", std::string(home ? home : "/tmp") +
    (home ? "/.ros/lunar_cave_arm_trajectory_epoch" : "/lunar_cave_arm_trajectory_epoch"));
  trajectory_gripper_topic_ = declare_parameter<std::string>("tzb.trajectory_gripper_topic", "/p6/arm_task/gripper_command");
  trajectory_cancel_topic_ = declare_parameter<std::string>("tzb.trajectory_cancel_topic", "/p6/arm_task/cancel");
  trajectory_cancel_ack_topic_ = declare_parameter<std::string>("tzb.trajectory_cancel_ack_topic", "/serial/arm_task/cancel_ack");
  try {
    const std::filesystem::path path(trajectory_epoch_state_path_);
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ifstream file(trajectory_epoch_state_path_);
    if (file.good()) { std::string value; file >> value; if (!parseUnsigned(value,trajectory_epoch_high_water_)) throw std::runtime_error("malformed epoch high-water"); }
  } catch (const std::exception& e) { throw std::runtime_error(std::string("cannot load epoch high-water: ")+e.what()); }
  arm_joint_names_ = declare_parameter<std::vector<std::string>>(
    "tzb.arm_joint_names",
    std::vector<std::string>{
      "J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"});
  const std::vector<std::string> expected_arm_joint_names{
    "J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"};
  if (arm_joint_names_ != expected_arm_joint_names)
  {
    throw std::runtime_error(
      "tzb.arm_joint_names must match the fixed wire order J1_joint through J6_joint");
  }
  const auto lower_limits = declare_parameter<std::vector<double>>(
    "tzb.arm_joint_lower_limits_rad",
    std::vector<double>{-1.5708, -1.1345, -2.618, -2.8798, -1.578, -2.8798});
  const auto upper_limits = declare_parameter<std::vector<double>>(
    "tzb.arm_joint_upper_limits_rad",
    std::vector<double>{3.6652, 1.1345, 2.618, 2.8798, 1.5708, 2.8798});
  if (lower_limits.size() != kArmJointCount || upper_limits.size() != kArmJointCount)
  {
    throw std::runtime_error("arm joint limit parameters must contain exactly 6 values");
  }
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    if (!std::isfinite(lower_limits[i]) || !std::isfinite(upper_limits[i]) ||
        lower_limits[i] >= upper_limits[i])
    {
      throw std::runtime_error("arm joint limits must be finite and lower < upper");
    }
    arm_joint_lower_limits_rad_[i] = static_cast<float>(lower_limits[i]);
    arm_joint_upper_limits_rad_[i] = static_cast<float>(upper_limits[i]);
    if (!std::isfinite(arm_joint_lower_limits_rad_[i]) ||
        !std::isfinite(arm_joint_upper_limits_rad_[i]) ||
        arm_joint_lower_limits_rad_[i] >= arm_joint_upper_limits_rad_[i])
    {
      throw std::runtime_error("arm joint limits are invalid after float conversion");
    }
  }
  const auto max_velocities = declare_parameter<std::vector<double>>(
    "tzb.max_arm_command_velocity_rad_s",
    std::vector<double>{1.2, 1.2, 1.2, 1.8, 1.8, 2.2});
  if (max_velocities.size() != kArmJointCount)
  {
    throw std::runtime_error("tzb.max_arm_command_velocity_rad_s must contain exactly 6 values");
  }
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    if (!std::isfinite(max_velocities[i]) || max_velocities[i] <= 0.0 ||
        !std::isfinite(static_cast<float>(max_velocities[i])) ||
        static_cast<float>(max_velocities[i]) <= 0.0F)
    {
      throw std::runtime_error("arm velocity limits must be finite and positive");
    }
    max_arm_command_velocity_rad_s_[i] = static_cast<float>(max_velocities[i]);
  }
  (void)declare_parameter<bool>("tzb.arm_command_publish_during_replay", false);
  (void)declare_parameter<bool>("tzb.sequence_loop", false);
  (void)declare_parameter<double>("tzb.loop_pause_sec", 0.5);

  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    declare_parameter<std::string>(
      "tzb.legacy_chassis_command_topic", "/tzb/legacy_chassis_cmd"),
    rclcpp::SensorDataQoS(),
    std::bind(&DriverNode::navCallback, this, _1));
  arm_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    declare_parameter<std::string>("tzb.arm_command_topic", "/tzb_catch/arm_joint_command"),
    rclcpp::SensorDataQoS(),
    std::bind(&DriverNode::armCommandCallback, this, _1));
  gripper_command_sub_ = create_subscription<std_msgs::msg::Int8>(
    declare_parameter<std::string>(
      "tzb.gripper_command_topic", "/tzb_catch/gripper_command"),
    rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&DriverNode::gripperCommandCallback, this, _1));
  trajectory_gripper_command_sub_ = create_subscription<std_msgs::msg::String>(
    trajectory_gripper_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&DriverNode::trajectoryGripperCommandCallback, this, _1));
  trajectory_cancel_sub_ = create_subscription<std_msgs::msg::String>(
    trajectory_cancel_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&DriverNode::trajectoryCancelCallback, this, _1));
  trajectory_cancel_ack_pub_ = create_publisher<std_msgs::msg::String>(
    trajectory_cancel_ack_topic_, rclcpp::QoS(10).reliable().durability_volatile());
  arm_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    declare_parameter<std::string>("tzb.arm_state_topic", "/serial/arm_joint_state"),
    rclcpp::SensorDataQoS());
  safety_status_pub_ = create_publisher<std_msgs::msg::String>(
    declare_parameter<std::string>(
      "tzb.arm_safety_status_topic", "/serial/arm_safety/status"),
    rclcpp::QoS(10).reliable().durability_volatile());
  reset_safety_service_ = create_service<std_srvs::srv::Trigger>(
    declare_parameter<std::string>(
      "tzb.arm_safety_reset_service", "/serial/arm_safety/reset"),
    std::bind(&DriverNode::resetSafetyCallback, this, _1, _2));
  enable_safety_service_ = create_service<std_srvs::srv::Trigger>(
    declare_parameter<std::string>(
      "tzb.arm_safety_enable_service", "/serial/arm_safety/enable"),
    std::bind(&DriverNode::enableSafetyCallback, this, _1, _2));

  while (rclcpp::ok())
  {
    try
    {
      if (!rcpputils::fs::exists(rcpputils::fs::path(device_name)))
      {
        RCLCPP_WARN(get_logger(), "Serial device %s not found, retrying...", device_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      serial_driver_.init_port(device_name, device_config);

      if (!serial_driver_.port()->is_open())
      {
        serial_driver_.port()->open();
      }

      if (serial_driver_.port()->is_open())
      {
        {
          std::scoped_lock parser_lock{receive_parser_mutex_};
          receive_frame_decoder_.Reset();
          receive_sequence_tracker_.Reset();
        }
        serial_receive_enabled_.store(true);
        startSerialReceive();
        break;
      }
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error with serial port %s: %s", device_name.c_str(), ex.what());
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  const auto send_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / arm_command_publish_hz_));
  timer_ = create_wall_timer(send_period, std::bind(&DriverNode::timerCallback, this));
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / safety_status_publish_hz_)),
    std::bind(&DriverNode::statusTimerCallback, this));
  RCLCPP_INFO(
    get_logger(),
    "Started serial driver: arm_serial=%s arm_extension=%s arm_angle_unit=%s arm_hz=%.2f speed_scale=%.3f packet_size=%zu safety=DISARMED lease=%.3fs(candidate)",
    arm_serial_enabled_ ? "true" : "false",
    include_arm_extension_ ? "true" : "false",
    arm_serial_angles_in_degrees_ ? "deg" : "rad",
    arm_command_publish_hz_,
    arm_command_speed_scale_,
    SendPacketSize(include_arm_extension_ && arm_serial_enabled_),
    arm_command_lease_sec_);
}

DriverNode::~DriverNode()
{
  SendPacket shutdown_packet;
  {
    std::scoped_lock lock{arm_mutex_};
    latchFaultLocked("ROS node shutdown");
    shutdown_packet.arm_sequence = arm_sequence_++;
    shutdown_packet.arm_command_enabled = 0;
    shutdown_packet.gripper_command = 0;
    shutdown_packet.arm = arm_packet_;
  }
  try
  {
    std::scoped_lock lock{serial_port_mutex_};
    if (serial_driver_.port()->is_open())
    {
      // Protocol bit 0 means "do not adopt this frame's arm target"; it is not an E-stop.
      serial_driver_.port()->send(
        ToVector(shutdown_packet, include_arm_extension_ && arm_serial_enabled_));
    }
  }
  catch (const std::exception&)
  {
    // Best-effort final inhibit only; shutdown must not block on an unavailable port.
  }
  {
    serial_receive_enabled_.store(false);
    std::scoped_lock lock{serial_port_mutex_};
    if (serial_driver_.port()->is_open())
    {
      serial_driver_.port()->close();
    }
  }

  owned_ctx_.waitForExit();
}

void DriverNode::navCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  try
  {
    std::scoped_lock lock{nav_mutex_};
    nav_packet_.linear_x = static_cast<float>(msg->linear.x);
    nav_packet_.linear_y = static_cast<float>(-msg->linear.y);
    nav_packet_.angular_z = static_cast<float>(msg->angular.z);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error from nav callback: %s", e.what());
  }
}

void DriverNode::armCommandCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  if (!arm_serial_enabled_)
  {
    return;
  }

  try
  {
    const auto steady_now = std::chrono::steady_clock::now();
    std::array<float, kArmJointCount> parsed_positions{};
    std::string rejection_reason;
    TrajectoryCommandContext context;
    if (!parseTrajectoryContext(msg->header.frame_id, context)) rejection_reason="missing/malformed p4f context";
    const auto now_epoch = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
    if (rejection_reason.empty() && context.epoch > now_epoch + 60000000000ULL)
      rejection_reason="trajectory epoch is implausibly far in the future";
    const auto stamp = msg->header.stamp;
    int64_t stamp_ns = 0;
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
        (stamp.sec == 0 && stamp.nanosec == 0))
    {
      rejection_reason = "invalid or zero ROS timestamp";
    }
    else
    {
      stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    }

    const int64_t now_ros_ns = get_clock()->now().nanoseconds();
    const double stamp_age_sec = static_cast<double>(now_ros_ns - stamp_ns) / 1.0e9;
    if (rejection_reason.empty() && stamp_age_sec > arm_command_max_age_sec_)
    {
      rejection_reason = "stale ROS timestamp";
    }
    if (rejection_reason.empty() && stamp_age_sec < -arm_command_future_tolerance_sec_)
    {
      rejection_reason = "ROS timestamp is too far in the future";
    }
    if (rejection_reason.empty() &&
        (msg->name.size() != kArmJointCount || msg->position.size() != kArmJointCount))
    {
      rejection_reason = "JointState must contain exactly six names and six positions";
    }
    if (rejection_reason.empty())
    {
      const auto validation = ParseArmJointCommand(
        arm_joint_names_, msg->name, msg->position,
        arm_joint_lower_limits_rad_, arm_joint_upper_limits_rad_, parsed_positions);
      if (validation != ArmJointCommandError::kNone)
      {
        rejection_reason = "joint integrity/finite/limit validation failed: " +
          std::to_string(static_cast<int>(validation));
      }
    }

    std::scoped_lock lock{arm_mutex_};
    last_command_rx_time_ = steady_now;
    if (arm_safety_state_ == ArmSafetyState::kDisarmed)
    {
      // Never stage commands received before an operator enables this process epoch.
      if (!rejection_reason.empty())
      {
        last_safety_event_ = "ignored while disarmed: " + rejection_reason;
      }
      return;
    }
    if (arm_safety_state_ == ArmSafetyState::kFaultLatched)
    {
      return;
    }
    if (!rejection_reason.empty())
    {
      latchFaultLocked(rejection_reason);
      RCLCPP_ERROR(get_logger(), "Arm command rejected; safety fault latched: %s", rejection_reason.c_str());
      return;
    }
    if (stamp_ns <= arm_enable_epoch_stamp_ns_ ||
        (last_accepted_command_stamp_ns_ != 0 && stamp_ns <= last_accepted_command_stamp_ns_)) {
      rejection_reason="command timestamp is not newer than this enable epoch/previous command";
      latchFaultLocked(rejection_reason);
      RCLCPP_ERROR(get_logger(),"Arm command rejected; safety fault latched: %s",rejection_reason.c_str());
      return;
    }
    if (context.epoch < trajectory_epoch_high_water_ ||
        cancelled_trajectory_tasks_.count(trajectoryKey(context.task_id,context.epoch))) {
      last_safety_event_="rejected old epoch or cancelled task";
      RCLCPP_WARN(get_logger(),"Rejected replay task=%s epoch=%llu seq=%llu",context.task_id.c_str(),
        (unsigned long long)context.epoch,(unsigned long long)context.sequence);
      return;
    }
    if (context.epoch == trajectory_epoch_high_water_ &&
        (accepted_trajectory_epoch_ != context.epoch || accepted_trajectory_task_id_ != context.task_id)) {
      last_safety_event_="rejected epoch replay after lifecycle barrier";
      RCLCPP_WARN(get_logger(),"Rejected inactive epoch replay %llu",(unsigned long long)context.epoch);
      return;
    }
    if (context.epoch == accepted_trajectory_epoch_ &&
        (accepted_trajectory_task_id_ != context.task_id || context.sequence <= last_trajectory_sequence_)) {
      last_safety_event_="rejected duplicate/backward trajectory sequence";
      RCLCPP_WARN(get_logger(),"Rejected duplicate/backward seq task=%s epoch=%llu seq=%llu last=%llu",
        context.task_id.c_str(),(unsigned long long)context.epoch,(unsigned long long)context.sequence,
        (unsigned long long)last_trajectory_sequence_);
      return;
    }
    if (context.epoch > trajectory_epoch_high_water_) {
      if (!persistTrajectoryEpochLocked(context.epoch)) {
        rejection_reason="cannot persist epoch high-water"; latchFaultLocked(rejection_reason);
        RCLCPP_ERROR(get_logger(),"Rejected arm command: %s",rejection_reason.c_str()); return;
      }
      trajectory_epoch_high_water_=context.epoch;
      accepted_trajectory_epoch_=context.epoch;
      accepted_trajectory_task_id_=context.task_id;
      last_trajectory_sequence_=0;
      RCLCPP_INFO(get_logger(),"Accepted trajectory task=%s epoch=%llu",context.task_id.c_str(),
        (unsigned long long)context.epoch);
    }
    for (std::size_t i = 0; i < kArmJointCount; ++i)
    {
      arm_target_position_[i] = parsed_positions[i];
    }
    last_accepted_command_stamp_ns_ = stamp_ns;
    last_trajectory_sequence_ = context.sequence;
    last_valid_command_time_ = steady_now;
    have_arm_command_ = true;
    if (arm_safety_state_ == ArmSafetyState::kArmedWaitCommand)
    {
      arm_safety_state_ = ArmSafetyState::kActive;
      arm_fault_reason_.clear();
      last_safety_event_ = "first fresh command accepted after explicit enable";
      RCLCPP_INFO(get_logger(), "Arm safety state ACTIVE_HOST: new command accepted after enable.");
    }
    if (!logged_first_arm_command_)
    {
      logged_first_arm_command_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Received first arm command on [%s] in an enabled safety epoch.",
        arm_command_sub_->get_topic_name());
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error from arm command callback: %s", e.what());
  }
}

void DriverNode::gripperCommandCallback(std_msgs::msg::Int8::ConstSharedPtr msg)
{
  std::scoped_lock lock{arm_mutex_};
  if (trajectory_epoch_high_water_ > 0) {
    last_safety_event_ = "rejected untagged legacy gripper command after P4f lifecycle activation";
    RCLCPP_WARN(get_logger(), "Rejected untagged gripper command after P4f lifecycle activation.");
    return;
  }
  if (arm_safety_state_ != ArmSafetyState::kActive)
  {
    last_safety_event_ = "ignored gripper command outside ACTIVE state";
    return;
  }
  if (!IsValidGripperCommand(msg->data))
  {
    latchFaultLocked("invalid gripper command");
    RCLCPP_ERROR(get_logger(), "Invalid gripper command; safety fault latched: %d", static_cast<int>(msg->data));
    return;
  }
  gripper_command_ = msg->data;
  const char* state = gripper_command_ > 0 ? "closed" :
    (gripper_command_ < 0 ? "open" : "hold");
  RCLCPP_INFO(
    get_logger(),
    "Gripper serial command updated: %d (%s)",
    static_cast<int>(gripper_command_), state);
}

void DriverNode::trajectoryGripperCommandCallback(std_msgs::msg::String::ConstSharedPtr msg)
{
  const auto first=msg->data.find('|');
  const auto second=first==std::string::npos ? first : msg->data.find('|',first+1);
  const auto third=second==std::string::npos ? second : msg->data.find('|',second+1);
  TrajectoryCommandContext context;
  int value=0;
  bool parsed=first!=std::string::npos && first>0 && second!=std::string::npos &&
    third!=std::string::npos && msg->data.find('|',third+1)==std::string::npos;
  if (parsed) {
    context.task_id=msg->data.substr(0,first);
    parsed=context.task_id.find(':')==std::string::npos &&
      parseUnsigned(msg->data.substr(first+1,second-first-1),context.epoch) &&
      parseUnsigned(msg->data.substr(second+1,third-second-1),context.sequence);
    try {
      std::size_t used=0;
      value=std::stoi(msg->data.substr(third+1),&used,10);
      parsed=parsed && used==msg->data.size()-third-1;
    } catch (const std::exception&) { parsed=false; }
  }
  std::scoped_lock lock{arm_mutex_};
  if (arm_safety_state_ != ArmSafetyState::kActive) {
    last_safety_event_="ignored tagged gripper command outside ACTIVE state";
    return;
  }
  if (!parsed || context.epoch==0 || context.sequence==0 ||
      (value != -1 && value != 1)) {
    last_safety_event_="rejected malformed tagged gripper command";
    RCLCPP_WARN(get_logger(),"Rejected malformed P4f gripper command.");
    return;
  }
  const auto now_epoch=static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
  if (context.epoch > now_epoch + 60000000000ULL ||
      context.epoch < trajectory_epoch_high_water_ ||
      cancelled_trajectory_tasks_.count(trajectoryKey(context.task_id,context.epoch))) {
    last_safety_event_="rejected stale/cancelled tagged gripper command";
    RCLCPP_WARN(get_logger(),"Rejected stale/cancelled P4f gripper task=%s epoch=%llu seq=%llu",
      context.task_id.c_str(),(unsigned long long)context.epoch,(unsigned long long)context.sequence);
    return;
  }
  if (context.epoch == trajectory_epoch_high_water_ &&
      (accepted_trajectory_epoch_ != context.epoch || accepted_trajectory_task_id_ != context.task_id)) {
    last_safety_event_="rejected inactive tagged gripper epoch";
    RCLCPP_WARN(get_logger(),"Rejected inactive P4f gripper epoch %llu",
      (unsigned long long)context.epoch);
    return;
  }
  if (context.epoch == accepted_trajectory_epoch_ &&
      (accepted_trajectory_task_id_ != context.task_id || context.sequence <= last_trajectory_sequence_)) {
    last_safety_event_="rejected duplicate/backward tagged gripper sequence";
    RCLCPP_WARN(get_logger(),"Rejected duplicate/backward P4f gripper sequence task=%s epoch=%llu seq=%llu",
      context.task_id.c_str(),(unsigned long long)context.epoch,(unsigned long long)context.sequence);
    return;
  }
  if (context.epoch > trajectory_epoch_high_water_) {
    if (!persistTrajectoryEpochLocked(context.epoch)) {
      latchFaultLocked("cannot persist gripper trajectory epoch high-water");
      return;
    }
    trajectory_epoch_high_water_=context.epoch;
    accepted_trajectory_epoch_=context.epoch;
    accepted_trajectory_task_id_=context.task_id;
    last_trajectory_sequence_=0;
  }
  if (context.epoch != accepted_trajectory_epoch_ || context.task_id != accepted_trajectory_task_id_ ||
      context.sequence <= last_trajectory_sequence_) {
    last_safety_event_="rejected unmatched tagged gripper command";
    return;
  }
  last_trajectory_sequence_=context.sequence;
  gripper_command_=static_cast<int8_t>(value);
  last_safety_event_="accepted P4f task-scoped gripper command";
  const char* state=gripper_command_>0 ? "closed" : (gripper_command_<0 ? "open" : "hold");
  RCLCPP_INFO(get_logger(),"P4f gripper task=%s epoch=%llu seq=%llu command=%d (%s)",
    context.task_id.c_str(),(unsigned long long)context.epoch,
    (unsigned long long)context.sequence,static_cast<int>(gripper_command_),state);
}

void DriverNode::trajectoryCancelCallback(std_msgs::msg::String::ConstSharedPtr msg)
{
  const auto sep=msg->data.rfind('|');
  const std::string id=sep==std::string::npos ? "" : msg->data.substr(0,sep);
  uint64_t epoch=0;
  const bool valid=sep!=std::string::npos && sep>0 &&
    parseUnsigned(msg->data.substr(sep+1),epoch) && epoch>0 &&
    id.find(':')==std::string::npos && id.find('|')==std::string::npos;
  const auto now_epoch = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
  const bool plausible = valid && epoch <= now_epoch + 60000000000ULL;
  bool applied=false;
  {
    std::scoped_lock lock{arm_mutex_};
    const bool identity_matches_active = accepted_trajectory_epoch_ == 0 ||
      (accepted_trajectory_epoch_ == epoch && accepted_trajectory_task_id_ == id);
    if (plausible && identity_matches_active && epoch >= trajectory_epoch_high_water_) {
      if (epoch > trajectory_epoch_high_water_) {
        applied=persistTrajectoryEpochLocked(epoch);
        if (applied) trajectory_epoch_high_water_=epoch;
      } else {
        applied=true;
      }
      if (applied) {
        cancelled_trajectory_tasks_.insert(trajectoryKey(id,epoch));
        if (accepted_trajectory_epoch_ > 0 && accepted_trajectory_epoch_ <= epoch) {
          cancelled_trajectory_tasks_.insert(trajectoryKey(
            accepted_trajectory_task_id_, accepted_trajectory_epoch_));
          accepted_trajectory_epoch_=0;
          accepted_trajectory_task_id_.clear();
          last_trajectory_sequence_=0;
          have_arm_command_=false;
          gripper_command_=0;
          if (have_arm_feedback_) for (std::size_t i=0;i<kArmJointCount;++i) {
            arm_target_position_[i]=arm_measured_position_rad_[i];
            arm_command_position_rad_[i]=arm_measured_position_rad_[i];
          }
        }
        last_safety_event_="ROS host cancelled trajectory; STM32 stop is unconfirmed";
      }
    } else {
      last_safety_event_=identity_matches_active ?
        "rejected stale or malformed trajectory cancellation" :
        "rejected cancellation identity mismatch for active trajectory";
    }
  }
  std_msgs::msg::String ack;
  ack.data=id+"|"+std::to_string(epoch)+"|"+
    (applied ? "ROS_HOST_CANCELLED_STM32_UNCONFIRMED" : "REJECTED_STALE_OR_MALFORMED");
  if (trajectory_cancel_ack_pub_) trajectory_cancel_ack_pub_->publish(ack);
  RCLCPP_WARN(get_logger(), applied ?
    "ROS host cancellation barrier applied for task=%s epoch=%llu; STM32 stop remains unconfirmed." :
    "Cancellation rejected as stale/malformed [%s].",
    id.c_str(),(unsigned long long)epoch);
}

bool DriverNode::persistTrajectoryEpochLocked(uint64_t epoch)
{
  try {
    const std::string tmp=trajectory_epoch_state_path_+".tmp";
    { std::ofstream file(tmp,std::ios::trunc); if (!file.good()) return false; file<<epoch<<"\n"; file.flush(); if (!file.good()) return false; }
    std::filesystem::rename(tmp,trajectory_epoch_state_path_);
    return true;
  } catch (const std::exception&) { return false; }
}

void DriverNode::resetSafetyCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool port_open = false;
  {
    std::scoped_lock lock{serial_port_mutex_};
    port_open = serial_driver_.port()->is_open();
  }
  std::scoped_lock lock{arm_mutex_};
  const auto now = std::chrono::steady_clock::now();
  if (arm_safety_state_ != ArmSafetyState::kFaultLatched)
  {
    response->success = false;
    response->message = "reset is only valid while a fault is latched";
    return;
  }
  if (!port_open || !feedbackFreshLocked(now))
  {
    response->success = false;
    response->message = "serial port and recent valid arm feedback are required before reset";
    return;
  }
  if (!commandSourceQuietLocked(now))
  {
    response->success = false;
    response->message = "stop/cancel the upstream command source and wait for its quiet interval";
    return;
  }
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    if (!std::isfinite(arm_measured_position_rad_[i]) ||
        arm_measured_position_rad_[i] < arm_joint_lower_limits_rad_[i] ||
        arm_measured_position_rad_[i] > arm_joint_upper_limits_rad_[i])
    {
      response->success = false;
      response->message = "measured joint feedback is non-finite or outside configured limits";
      return;
    }
  }
  have_arm_command_ = false;
  gripper_command_ = 0;
  arm_safety_state_ = ArmSafetyState::kDisarmed;
  arm_fault_reason_.clear();
  last_accepted_command_stamp_ns_ = 0;
  arm_enable_epoch_stamp_ns_ = 0;
  last_safety_event_ = "operator reset accepted; still disarmed";
  response->success = true;
  response->message = "fault cleared to DISARMED; call enable separately, then send a new valid command";
  RCLCPP_WARN(get_logger(), "Arm fault reset by operator; motion remains disarmed.");
}

void DriverNode::enableSafetyCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool port_open = false;
  {
    std::scoped_lock lock{serial_port_mutex_};
    port_open = serial_driver_.port()->is_open();
  }
  std::scoped_lock lock{arm_mutex_};
  const auto now = std::chrono::steady_clock::now();
  if (arm_safety_state_ != ArmSafetyState::kDisarmed)
  {
    response->success = false;
    response->message = "enable is only valid from DISARMED; reset a latched fault first";
    return;
  }
  if (!arm_serial_enabled_ || !port_open || !feedbackFreshLocked(now))
  {
    response->success = false;
    response->message = "arm serial is disabled or recent valid feedback/serial port is unavailable";
    return;
  }
  if (!commandSourceQuietLocked(now))
  {
    response->success = false;
    response->message = "upstream command source must be stopped for the configured quiet interval";
    return;
  }
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    if (!std::isfinite(arm_measured_position_rad_[i]) ||
        arm_measured_position_rad_[i] < arm_joint_lower_limits_rad_[i] ||
        arm_measured_position_rad_[i] > arm_joint_upper_limits_rad_[i])
    {
      response->success = false;
      response->message = "measured joint feedback is non-finite or outside configured limits";
      return;
    }
  }
  have_arm_command_ = false;
  gripper_command_ = 0;
  accepted_trajectory_epoch_ = 0;
  accepted_trajectory_task_id_.clear();
  last_trajectory_sequence_ = 0;
  arm_enable_epoch_stamp_ns_ = get_clock()->now().nanoseconds();
  last_accepted_command_stamp_ns_ = 0;
  arm_safety_state_ = ArmSafetyState::kArmedWaitCommand;
  last_safety_event_ = "operator enabled; waiting for a newer valid JointState";
  response->success = true;
  response->message = "enabled in wait-for-command state; no serial motion enable until a newer valid command arrives";
  RCLCPP_WARN(get_logger(), "Arm safety explicitly enabled; waiting for a fresh command.");
}

void DriverNode::latchFaultLocked(const std::string& reason)
{
  if (arm_safety_state_ != ArmSafetyState::kFaultLatched)
  {
    arm_safety_state_ = ArmSafetyState::kFaultLatched;
    arm_fault_reason_ = reason;
  }
  last_safety_event_ = reason;
  have_arm_command_ = false;
  gripper_command_ = 0;
  last_accepted_command_stamp_ns_ = 0;
  last_valid_command_time_ = std::chrono::steady_clock::time_point{};
  accepted_trajectory_epoch_=0;
  accepted_trajectory_task_id_.clear();
  last_trajectory_sequence_=0;
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    if (have_arm_feedback_)
    {
      arm_target_position_[i] = arm_measured_position_rad_[i];
      arm_command_position_rad_[i] = arm_measured_position_rad_[i];
      arm_packet_.position[i] = angleToSerialUnit(arm_measured_position_rad_[i], i);
    }
    else
    {
      arm_target_position_[i] = 0.0F;
      arm_command_position_rad_[i] = 0.0F;
      arm_packet_.position[i] = 0.0F;
    }
  }
}

bool DriverNode::commandSourceQuietLocked(std::chrono::steady_clock::time_point now) const
{
  return last_command_rx_time_ == std::chrono::steady_clock::time_point{} ||
         std::chrono::duration<double>(now - last_command_rx_time_).count() >=
         arm_rearm_quiet_sec_;
}

bool DriverNode::feedbackFreshLocked(std::chrono::steady_clock::time_point now) const
{
  return have_arm_feedback_ &&
         last_valid_feedback_time_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::duration<double>(now - last_valid_feedback_time_).count() <=
         arm_feedback_timeout_sec_;
}

const char* DriverNode::safetyStateName(ArmSafetyState state)
{
  switch (state)
  {
    case ArmSafetyState::kDisarmed: return "DISARMED";
    case ArmSafetyState::kArmedWaitCommand: return "ARMED_WAIT_COMMAND";
    case ArmSafetyState::kActive: return "ACTIVE_HOST";
    case ArmSafetyState::kFaultLatched: return "FAULT_LATCHED";
  }
  return "UNKNOWN";
}

void DriverNode::statusTimerCallback()
{
  const auto now = std::chrono::steady_clock::now();
  std::ostringstream status;
  bool fault_latched = false;
  {
    std::scoped_lock lock{arm_mutex_};
    const auto feedback_age = last_valid_feedback_time_ == std::chrono::steady_clock::time_point{} ?
      -1.0 : std::chrono::duration<double>(now - last_valid_feedback_time_).count();
    const auto command_age = last_valid_command_time_ == std::chrono::steady_clock::time_point{} ?
      -1.0 : std::chrono::duration<double>(now - last_valid_command_time_).count();
    fault_latched = arm_safety_state_ == ArmSafetyState::kFaultLatched;
    status << std::fixed << std::setprecision(3)
           << "state=" << safetyStateName(arm_safety_state_)
           << " fault=" << (arm_fault_reason_.empty() ? "none" : arm_fault_reason_)
           << " event=" << last_safety_event_
           << " feedback_age_s=" << feedback_age
           << " feedback_timeout_s=" << arm_feedback_timeout_sec_
           << " command_age_s=" << command_age
           << " command_lease_s=" << arm_command_lease_sec_ << "(candidate)"
           << " command_stamp_max_age_s=" << arm_command_max_age_sec_
           << " rearm_quiet_s=" << arm_rearm_quiet_sec_
           << " invalid_feedback_frames=" << invalid_feedback_count_
           << " feedback_sequence_gaps=" << feedback_sequence_gap_count_
           << " trajectory_epoch_high_water=" << trajectory_epoch_high_water_
           << " trajectory_epoch=" << accepted_trajectory_epoch_
           << " trajectory_sequence=" << last_trajectory_sequence_
           << " trajectory_task=" << (accepted_trajectory_task_id_.empty() ? "none" : accepted_trajectory_task_id_)
           << " per_joint_feedback_health=unavailable_in_46B_response";
  }
  bool port_open = false;
  {
    std::scoped_lock lock{serial_port_mutex_};
    port_open = serial_driver_.port()->is_open();
  }
  status << " serial_open=" << (port_open ? "true" : "false");
  std_msgs::msg::String message;
  message.data = status.str();
  if (safety_status_pub_)
  {
    safety_status_pub_->publish(message);
  }
  if (fault_latched)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Arm safety: %s", message.data.c_str());
  }
}

void DriverNode::timerCallback()
{
  try
  {
    SendPacket send_packet;
    {
      std::scoped_lock lock{nav_mutex_};
      send_packet.nav = nav_packet_;
    }
    {
      std::scoped_lock lock{arm_mutex_};
      const auto now = std::chrono::steady_clock::now();
      if ((arm_safety_state_ == ArmSafetyState::kArmedWaitCommand ||
           arm_safety_state_ == ArmSafetyState::kActive) && !feedbackFreshLocked(now))
      {
        latchFaultLocked("arm feedback timeout");
      }
      if (arm_safety_state_ == ArmSafetyState::kActive &&
          (last_valid_command_time_ == std::chrono::steady_clock::time_point{} ||
           std::chrono::duration<double>(now - last_valid_command_time_).count() >
           arm_command_lease_sec_))
      {
        latchFaultLocked("arm command lease expired");
      }
      double dt_sec = 1.0 / arm_command_publish_hz_;
      if (last_send_time_ != std::chrono::steady_clock::time_point{})
      {
        dt_sec = std::max(1.0e-3, std::chrono::duration<double>(now - last_send_time_).count());
      }
      last_send_time_ = now;
      updateArmCommandLocked(dt_sec);
      send_packet.arm_command_enabled =
        (arm_serial_enabled_ && arm_safety_state_ == ArmSafetyState::kActive &&
        have_arm_command_) ? 1 : 0;
      send_packet.arm_sequence = arm_sequence_++;
      send_packet.arm = arm_packet_;
      send_packet.gripper_command =
        arm_safety_state_ == ArmSafetyState::kActive ? gripper_command_ : 0;
      const bool should_log_gripper_command =
        gripper_command_log_interval_sec_ <= 0.0 ||
        last_gripper_command_log_time_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::duration<double>(now - last_gripper_command_log_time_).count() >=
        gripper_command_log_interval_sec_;
      if (should_log_gripper_command)
      {
        last_gripper_command_log_time_ = now;
        const char* gripper_state = send_packet.gripper_command > 0 ? "closed" :
          (send_packet.gripper_command < 0 ? "open" : "hold");
        RCLCPP_INFO(
          get_logger(),
          "Sending gripper_command to lower controller: %d (%s), arm_sequence=%u, log_interval=%.2f s.",
          static_cast<int>(send_packet.gripper_command),
          gripper_state,
          send_packet.arm_sequence,
          gripper_command_log_interval_sec_);
      }
      if (send_packet.arm_command_enabled != 0 && !logged_first_enabled_serial_packet_)
      {
        logged_first_enabled_serial_packet_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Sending enabled arm serial packets: sequence=%u joints=[%.4f %.4f %.4f %.4f %.4f %.4f] gripper=%d packet_size=%zu.",
          send_packet.arm_sequence,
          send_packet.arm.position[0],
          send_packet.arm.position[1],
          send_packet.arm.position[2],
          send_packet.arm.position[3],
          send_packet.arm.position[4],
          send_packet.arm.position[5],
          static_cast<int>(send_packet.gripper_command),
          SendPacketSize(include_arm_extension_ && arm_serial_enabled_));
      }
    }
    {
      std::scoped_lock lock{serial_port_mutex_};
      if (!serial_driver_.port()->is_open())
      {
        throw std::runtime_error("serial port is closed");
      }
      serial_driver_.port()->send(
        ToVector(send_packet, include_arm_extension_ && arm_serial_enabled_));
    }
  }
  catch (const std::exception& e)
  {
    {
      std::scoped_lock lock{arm_mutex_};
      latchFaultLocked(std::string("serial send failure: ") + e.what());
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Serial send unavailable; waiting for one reconnect attempt: %s", e.what());
    reopenPort();
  }
}

void DriverNode::startSerialReceive()
{
  if (!serial_receive_enabled_.load() || !rclcpp::ok())
  {
    return;
  }

  using namespace std::placeholders;
  std::scoped_lock lock{serial_port_mutex_};
  if (!serial_receive_enabled_.load() || !serial_driver_.port()->is_open())
  {
    return;
  }
  serial_driver_.port()->async_receive(std::bind(&DriverNode::serialReceiveCallback, this, _1, _2));
}

void DriverNode::serialReceiveCallback(const std::vector<uint8_t>& data, std::size_t size)
{
  std::unique_lock<std::mutex> parser_lock{receive_parser_mutex_};
  const auto frames = receive_frame_decoder_.Feed(data, size);
  for (const auto& frame : frames)
  {
    if (!HasValidArmReceiveExtension(
        frame.packet, frame.size, frame.arm_offset, frame.protocol_version))
    {
      {
        std::scoped_lock lock{arm_mutex_};
        ++invalid_feedback_count_;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Arm feedback extension invalid: size=%zu arm_offset=%zu arm_header=0x%02X valid=%u gripper=0x%02X",
        frame.size,
        frame.arm_offset,
        frame.packet.arm_header,
        static_cast<unsigned int>(frame.packet.arm_state_valid),
        static_cast<unsigned int>(frame.packet.arm_feedback_gripper));
      continue;
    }
    if (!HasFiniteArmPosition(frame.packet))
    {
      {
        std::scoped_lock lock{arm_mutex_};
        ++invalid_feedback_count_;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting arm feedback with non-finite joint position (sequence=%u)",
        static_cast<unsigned int>(frame.packet.arm_sequence));
      continue;
    }
    uint32_t missed = 0;
    const auto sequence_result = receive_sequence_tracker_.Observe(
      frame.packet.arm_sequence, missed);
    if (sequence_result == ReceiveSequenceResult::kDuplicate)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring duplicate arm feedback sequence=%u",
        static_cast<unsigned int>(frame.packet.arm_sequence));
      continue;
    }
    if (sequence_result == ReceiveSequenceResult::kGap)
    {
      {
        std::scoped_lock lock{arm_mutex_};
        feedback_sequence_gap_count_ += missed;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Arm feedback sequence gap: missed=%u latest=%u",
        missed,
        static_cast<unsigned int>(frame.packet.arm_sequence));
    }
    else if (sequence_result == ReceiveSequenceResult::kControllerRestarted)
    {
      {
        std::scoped_lock lock{arm_mutex_};
        if (arm_safety_state_ == ArmSafetyState::kActive ||
            arm_safety_state_ == ArmSafetyState::kArmedWaitCommand)
        {
          latchFaultLocked("STM32 feedback sequence restarted");
        }
        else
        {
          last_safety_event_ = "STM32 feedback sequence restarted while disarmed";
        }
      }
      RCLCPP_WARN(
        get_logger(),
        "Arm feedback sequence moved backwards; motion is inhibited pending manual recovery, sequence=%u",
        static_cast<unsigned int>(frame.packet.arm_sequence));
    }
    publishArmState(frame.packet);
  }
  parser_lock.unlock();
  startSerialReceive();
}

void DriverNode::publishArmState(const ReceivePacket& packet)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Raw lower-arm feedback before angle conversion: sequence=%u unit=%s "
    "J1=%.6f J2=%.6f J3=%.6f J4=%.6f J5=%.6f J6=%.6f",
    static_cast<unsigned int>(packet.arm_sequence),
    arm_serial_angles_in_degrees_ ? "deg" : "rad",
    static_cast<double>(packet.arm.position[0]),
    static_cast<double>(packet.arm.position[1]),
    static_cast<double>(packet.arm.position[2]),
    static_cast<double>(packet.arm.position[3]),
    static_cast<double>(packet.arm.position[4]),
    static_cast<double>(packet.arm.position[5]));

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = arm_joint_names_;
  msg.position.resize(kArmJointCount);
  {
    std::scoped_lock lock{arm_mutex_};
    have_arm_feedback_ = true;
    last_valid_feedback_time_ = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < kArmJointCount; ++i)
    {
      const float measured = angleFromSerialUnit(packet.arm.position[i], i);
      arm_measured_position_rad_[i] = measured;
      msg.position[i] = measured;
      if (measured < arm_joint_lower_limits_rad_[i] ||
          measured > arm_joint_upper_limits_rad_[i])
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Arm feedback for %s is outside the configured URDF range: %.5f rad "
          "(%.5f..%.5f); publishing the measured value for diagnosis.",
          arm_joint_names_[i].c_str(),
          static_cast<double>(measured),
          static_cast<double>(arm_joint_lower_limits_rad_[i]),
          static_cast<double>(arm_joint_upper_limits_rad_[i]));
      }
      if (arm_safety_state_ != ArmSafetyState::kActive || !have_arm_command_)
      {
        arm_target_position_[i] = measured;
        arm_command_position_rad_[i] = measured;
        arm_packet_.position[i] = angleToSerialUnit(measured, i);
      }
    }
  }
  if (arm_state_pub_)
  {
    arm_state_pub_->publish(msg);
  }
}

drivers::serial_driver::SerialPortConfig DriverNode::getDeviceConfig()
{
  using namespace drivers::serial_driver;

  static const std::unordered_map<std::string, FlowControl> fc_map{
    {"none", FlowControl::NONE}, {"hardware", FlowControl::HARDWARE}, {"software", FlowControl::SOFTWARE}};

  static const std::unordered_map<std::string, Parity> pt_map{
    {"none", Parity::NONE}, {"odd", Parity::ODD}, {"even", Parity::EVEN}};

  static const std::unordered_map<std::string, StopBits> sb_map{
    {"1", StopBits::ONE}, {"1.0", StopBits::ONE}, {"1.5", StopBits::ONE_POINT_FIVE}, {"2", StopBits::TWO}, {"2.0", StopBits::TWO}};

  return {
    static_cast<uint32_t>(declare_parameter<int>("baud_rate")),
    fc_map.at(declare_parameter<std::string>("flow_control")),
    pt_map.at(declare_parameter<std::string>("parity")),
    sb_map.at(declare_parameter<std::string>("stop_bits"))};
}

void DriverNode::reopenPort()
{
  if (serial_reopen_in_progress_.exchange(true))
  {
    return;
  }

  {
    std::scoped_lock lock{arm_mutex_};
    latchFaultLocked("serial disconnect/reconnect; manual reset and enable required");
    have_arm_feedback_ = false;
    last_valid_feedback_time_ = std::chrono::steady_clock::time_point{};
    arm_target_position_.fill(0.0F);
    arm_command_position_rad_.fill(0.0F);
    arm_packet_ = ArmJointPacket{};
  }

  serial_receive_enabled_.store(false);
  while (rclcpp::ok())
  {
    try
    {
      {
        std::scoped_lock lock{serial_port_mutex_};
        if (serial_driver_.port()->is_open())
        {
          serial_driver_.port()->close();
        }
        serial_driver_.port()->open();
      }

      {
        std::scoped_lock parser_lock{receive_parser_mutex_};
        receive_frame_decoder_.Reset();
        receive_sequence_tracker_.Reset();
      }
      serial_receive_enabled_.store(true);
      startSerialReceive();
      serial_reopen_in_progress_.store(false);
      RCLCPP_INFO(get_logger(), "Serial port reopened; receive loop resumed.");
      return;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Serial port is unavailable; retrying reconnect: %s", ex.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  serial_reopen_in_progress_.store(false);
}

void DriverNode::updateArmCommandLocked(double dt_sec)
{
  const double scaled_dt = std::max(1.0e-3, dt_sec) * arm_command_speed_scale_;
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    const float target = arm_target_position_[i];
    const float previous = arm_command_position_rad_[i];
    const float error = target - previous;
    const float abs_error = std::abs(error);
    if (!have_arm_command_ || abs_error <= static_cast<float>(arm_command_goal_tolerance_rad_))
    {
      arm_command_position_rad_[i] = have_arm_command_ ? target : previous;
      arm_packet_.position[i] = angleToSerialUnit(arm_command_position_rad_[i], i);
      continue;
    }

    const float max_step =
      max_arm_command_velocity_rad_s_[i] <= 0.0F ?
      abs_error :
      max_arm_command_velocity_rad_s_[i] * static_cast<float>(scaled_dt);
    const float step = std::clamp(error, -max_step, max_step);
    arm_command_position_rad_[i] = previous + step;
    arm_packet_.position[i] = angleToSerialUnit(arm_command_position_rad_[i], i);
  }
}

float DriverNode::angleToSerialUnit(float angle_rad, std::size_t joint_index) const
{
  return ArmRosAngleToSerialUnit(angle_rad, joint_index, arm_serial_angles_in_degrees_);
}

float DriverNode::angleFromSerialUnit(float serial_angle, std::size_t joint_index) const
{
  return ArmSerialUnitToRosAngle(serial_angle, joint_index, arm_serial_angles_in_degrees_);
}
}  // namespace rm::serial

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(rm::serial::DriverNode)
