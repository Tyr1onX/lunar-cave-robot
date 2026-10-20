#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include "chassis_serial_protocol.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace
{
using chassis_serial_bridge::protocol::Frame;

bool finiteTwist(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
         std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
}

class ChassisSerialBridge final : public rclcpp::Node
{
public:
  ChassisSerialBridge()
  : Node("chassis_serial_bridge")
  {
    device_name_ = declare_parameter<std::string>("device_name", "");
    const int baud_rate = declare_parameter<int>("baud_rate", 115200);
    command_topic_ = declare_parameter<std::string>("command_topic", "/cmd_vel");
    link_status_topic_ = declare_parameter<std::string>(
      "link_status_topic", "/chassis/serial_link_status");
    pusher_command_topic_ = declare_parameter<std::string>(
      "pusher_command_topic", "/chassis/pusher_command");
    pusher_status_topic_ = declare_parameter<std::string>(
      "pusher_status_topic", "/chassis/pusher_status");
    pusher_commands_enabled_ = declare_parameter<bool>("pusher_commands_enabled", false);
    const int tx_period_ms = declare_parameter<int>("tx_period_ms", 20);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 100);
    pusher_command_timeout_ms_ = declare_parameter<int>("pusher_command_timeout_ms", 500);
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.6);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 1.5);
    max_linear_y_abs_ = declare_parameter<double>("max_linear_y_abs", 0.01);
    unsupported_axis_tolerance_ =
      declare_parameter<double>("unsupported_axis_tolerance", 0.01);

    if (baud_rate != 115200) {
      throw std::invalid_argument("STM32 USART1 bridge baud_rate must be 115200");
    }
    if (command_topic_.empty() || pusher_command_topic_.empty() ||
        command_topic_ == pusher_command_topic_ ||
        tx_period_ms <= 0 || tx_period_ms > 1000 || command_timeout_ms_ <= 0 ||
        pusher_command_timeout_ms_ <= 0 || pusher_command_timeout_ms_ > 2000 ||
        !std::isfinite(max_linear_x_) || max_linear_x_ <= 0.0 ||
        !std::isfinite(max_angular_z_) || max_angular_z_ <= 0.0 ||
        !std::isfinite(max_linear_y_abs_) || max_linear_y_abs_ < 0.0 ||
        !std::isfinite(unsupported_axis_tolerance_) ||
        unsupported_axis_tolerance_ < 0.0)
    {
      throw std::invalid_argument("invalid chassis bridge safety parameter");
    }
    command_state_.setTimeouts(
      std::chrono::milliseconds(command_timeout_ms_),
      std::chrono::milliseconds(pusher_command_timeout_ms_));

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, rclcpp::QoS(10),
      std::bind(&ChassisSerialBridge::commandCallback, this, std::placeholders::_1));
    pusher_command_sub_ = create_subscription<std_msgs::msg::String>(
      pusher_command_topic_, rclcpp::QoS(10),
      std::bind(&ChassisSerialBridge::pusherCommandCallback, this, std::placeholders::_1));
    link_status_pub_ = create_publisher<std_msgs::msg::String>(link_status_topic_, rclcpp::QoS(10));
    pusher_status_pub_ = create_publisher<std_msgs::msg::String>(pusher_status_topic_, rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(tx_period_ms),
      std::bind(&ChassisSerialBridge::timerCallback, this));

    if (device_name_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "device_name is empty; serial output is disabled until configured with an explicit path");
    }
    RCLCPP_INFO(
      get_logger(),
      "Configured: device='%s' baud=115200 topic='%s' tx=%dms freshness=%dms "
      "limits=(vx %.3f m/s, wz %.3f rad/s)",
      device_name_.c_str(), command_topic_.c_str(), tx_period_ms, command_timeout_ms_,
      max_linear_x_, max_angular_z_);
  }

  ~ChassisSerialBridge() override
  {
    if (serial_fd_ >= 0) {
      const Frame stop = chassis_serial_bridge::protocol::encodeFrame(
        0.0F, 0.0F, chassis_serial_bridge::protocol::kStopVY);
      (void)::write(serial_fd_, stop.data(), stop.size());
      (void)::tcdrain(serial_fd_);
      (void)::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  using Clock = std::chrono::steady_clock;

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const auto reject = [this](const char * reason) {
      command_state_.rejectAll();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Rejecting /cmd_vel and commanding zero: %s", reason);
    };

    if (!finiteTwist(*msg)) {
      reject("NaN or Inf in Twist");
      return;
    }
    if (std::abs(msg->linear.x) > max_linear_x_ ||
        std::abs(msg->angular.z) > max_angular_z_)
    {
      reject("linear.x or angular.z exceeds configured chassis limit");
      return;
    }
    if (std::abs(msg->linear.y) > max_linear_y_abs_ ||
        std::abs(msg->linear.z) > unsupported_axis_tolerance_ ||
        std::abs(msg->angular.x) > unsupported_axis_tolerance_ ||
        std::abs(msg->angular.y) > unsupported_axis_tolerance_)
    {
      reject("nonzero unsupported differential-drive axis");
      return;
    }

    if (!command_state_.setMotion(
        static_cast<float>(msg->linear.x), static_cast<float>(msg->angular.z), Clock::now()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejecting nonzero /cmd_vel while the pusher is active or its stop frame is pending");
    }
  }

  void pusherCommandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    chassis_serial_bridge::protocol::PusherCommand command;
    if (!chassis_serial_bridge::protocol::parsePusherCommand(msg->data, command)) {
      command_state_.rejectAll();
      publishPusherStatus("REJECTED_INVALID_SAFE_STOP");
      RCLCPP_WARN(get_logger(), "Invalid pusher command; clearing pending chassis and pusher commands");
      return;
    }

    if (command == chassis_serial_bridge::protocol::PusherCommand::Stop) {
      (void)command_state_.requestPusher(command, Clock::now());
      publishPusherStatus("STOPPED_HOST_COMMAND");
      return;
    }

    if (!pusher_commands_enabled_) {
      command_state_.rejectAll();
      publishPusherStatus("REJECTED_DISABLED");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Pusher command rejected: pusher_commands_enabled is false");
      return;
    }
    if (serial_fd_ < 0) {
      command_state_.rejectAll();
      publishPusherStatus("REJECTED_SERIAL_DISCONNECTED");
      return;
    }

    const auto result = command_state_.requestPusher(command, Clock::now());
    if (result == chassis_serial_bridge::protocol::PusherRequestResult::Accepted) {
      publishPusherStatus(
        command == chassis_serial_bridge::protocol::PusherCommand::Extend ?
        "HOST_COMMAND_ACCEPTED_EXTEND" : "HOST_COMMAND_ACCEPTED_RETRACT");
    } else {
      publishPusherStatus("REJECTED_STOP_INTERLOCK");
      RCLCPP_WARN(
        get_logger(), "Pusher direction change rejected until a stop frame has been sent");
    }
  }

  void publishPusherStatus(const char * status)
  {
    std_msgs::msg::String message;
    message.data = status;
    pusher_status_pub_->publish(message);
    RCLCPP_INFO(
      get_logger(), "Pusher host-side status: %s (not actuator acknowledgement)", status);
  }

  void timerCallback()
  {
    const auto now = Clock::now();
    if (serial_fd_ < 0) {
      if (device_name_.empty() || now < next_open_attempt_) {
        publishLinkStatus("DISCONNECTED");
        return;
      }
      next_open_attempt_ = now + std::chrono::seconds(1);
      if (!openPort()) {
        publishLinkStatus("DISCONNECTED");
        return;
      }
    }

    if (command_state_.expirePusher(now)) {
      publishPusherStatus("AUTO_STOPPED_TIMEOUT");
    }
    const Frame frame = command_state_.nextFrame(now);
    if (!writeFrame(frame)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Serial write failed; cached command cleared and port will reconnect");
      command_state_.resetAfterDisconnect();
      closePort();
      publishLinkStatus("DISCONNECTED");
      return;
    }
    // This confirms only that the host OS accepted the complete frame on the
    // local TTY. The 14-byte protocol has no STM32 acknowledgement.
    publishLinkStatus("HOST_WRITE_OK");
  }

  void publishLinkStatus(const char * status)
  {
    std_msgs::msg::String message;
    message.data = status;
    link_status_pub_->publish(message);
    if (last_link_status_ != status) {
      RCLCPP_INFO(
        get_logger(), "Host serial link status: %s (no STM32 acknowledgement semantics)", status);
      last_link_status_ = status;
    }
  }

  bool openPort()
  {
    const int fd = ::open(
      device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unable to open serial device '%s': %s", device_name_.c_str(), std::strerror(errno));
      return false;
    }

    termios options{};
    if (::tcgetattr(fd, &options) != 0) {
      const int error = errno;
      (void)::close(fd);
      RCLCPP_ERROR(
        get_logger(), "Unable to read serial settings for '%s': %s",
        device_name_.c_str(), std::strerror(error));
      return false;
    }
    ::cfmakeraw(&options);
    options.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE));
    options.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
    options.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::cfsetispeed(&options, B115200) != 0 ||
        ::cfsetospeed(&options, B115200) != 0 ||
        ::tcsetattr(fd, TCSANOW, &options) != 0)
    {
      const int error = errno;
      (void)::close(fd);
      RCLCPP_ERROR(
        get_logger(), "Unable to configure 115200 8N1 on '%s': %s",
        device_name_.c_str(), std::strerror(error));
      return false;
    }

    (void)::tcflush(fd, TCIOFLUSH);
    serial_fd_ = fd;
    command_state_.resetAfterDisconnect();
    RCLCPP_INFO(get_logger(), "Serial device opened: %s (115200 8N1)", device_name_.c_str());
    return true;
  }

  bool writeFrame(const Frame & frame)
  {
    std::size_t offset = 0;
    while (offset < frame.size()) {
      const ssize_t count = ::write(
        serial_fd_, frame.data() + offset, frame.size() - offset);
      if (count > 0) {
        offset += static_cast<std::size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    return true;
  }

  void closePort()
  {
    if (serial_fd_ >= 0) {
      (void)::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  std::string device_name_;
  std::string command_topic_;
  std::string link_status_topic_;
  std::string pusher_command_topic_;
  std::string pusher_status_topic_;
  std::string last_link_status_;
  bool pusher_commands_enabled_{false};
  int command_timeout_ms_{100};
  int pusher_command_timeout_ms_{500};
  double max_linear_x_{0.6};
  double max_angular_z_{1.5};
  double max_linear_y_abs_{0.01};
  double unsupported_axis_tolerance_{0.01};
  int serial_fd_{-1};
  Clock::time_point next_open_attempt_{};
  chassis_serial_bridge::CommandState command_state_{
    std::chrono::milliseconds(100), std::chrono::milliseconds(500)};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pusher_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pusher_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ChassisSerialBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("chassis_serial_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
