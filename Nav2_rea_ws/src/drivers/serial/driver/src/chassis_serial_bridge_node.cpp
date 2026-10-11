#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
constexpr std::size_t kFrameSize = 14;
constexpr std::uint8_t kFrameHeader = 0xA5;
constexpr std::uint8_t kFrameTail = 0x4A;

using Frame = std::array<std::uint8_t, kFrameSize>;

void writeFloatLe(Frame & frame, std::size_t offset, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "STM32 protocol requires 32-bit float");
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t i = 0; i < sizeof(bits); ++i) {
    frame[offset + i] = static_cast<std::uint8_t>((bits >> (8U * i)) & 0xFFU);
  }
}

Frame encodeFrame(float vx, float vy, float wz)
{
  Frame frame{};
  frame[0] = kFrameHeader;
  writeFloatLe(frame, 1, vx);
  writeFloatLe(frame, 5, vy);
  writeFloatLe(frame, 9, wz);
  frame[13] = kFrameTail;
  return frame;
}

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
    const int tx_period_ms = declare_parameter<int>("tx_period_ms", 20);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 100);
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.6);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 1.5);
    max_linear_y_abs_ = declare_parameter<double>("max_linear_y_abs", 0.01);
    unsupported_axis_tolerance_ =
      declare_parameter<double>("unsupported_axis_tolerance", 0.01);

    if (baud_rate != 115200) {
      throw std::invalid_argument("STM32 USART1 bridge baud_rate must be 115200");
    }
    if (tx_period_ms <= 0 || tx_period_ms > 1000 || command_timeout_ms_ <= 0 ||
        !std::isfinite(max_linear_x_) || max_linear_x_ <= 0.0 ||
        !std::isfinite(max_angular_z_) || max_angular_z_ <= 0.0 ||
        !std::isfinite(max_linear_y_abs_) || max_linear_y_abs_ < 0.0 ||
        !std::isfinite(unsupported_axis_tolerance_) ||
        unsupported_axis_tolerance_ < 0.0)
    {
      throw std::invalid_argument("invalid chassis bridge safety parameter");
    }

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, rclcpp::QoS(10),
      std::bind(&ChassisSerialBridge::commandCallback, this, std::placeholders::_1));
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
      const Frame stop = encodeFrame(0.0F, 0.0F, 0.0F);
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
      command_valid_ = false;
      latest_vx_ = 0.0F;
      latest_wz_ = 0.0F;
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

    latest_vx_ = static_cast<float>(msg->linear.x);
    latest_wz_ = static_cast<float>(msg->angular.z);
    command_received_at_ = Clock::now();
    command_valid_ = true;
  }

  void timerCallback()
  {
    const auto now = Clock::now();
    if (serial_fd_ < 0) {
      if (device_name_.empty() || now < next_open_attempt_) {
        return;
      }
      next_open_attempt_ = now + std::chrono::seconds(1);
      if (!openPort()) {
        return;
      }
    }

    const bool fresh = command_valid_ &&
      (now - command_received_at_) <= std::chrono::milliseconds(command_timeout_ms_);
    const float vx = fresh ? latest_vx_ : 0.0F;
    const float wz = fresh ? latest_wz_ : 0.0F;
    const Frame frame = encodeFrame(vx, 0.0F, wz);
    if (!writeFrame(frame)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Serial write failed; cached command cleared and port will reconnect");
      command_valid_ = false;
      latest_vx_ = 0.0F;
      latest_wz_ = 0.0F;
      closePort();
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
    command_valid_ = false;
    latest_vx_ = 0.0F;
    latest_wz_ = 0.0F;
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
  int command_timeout_ms_{100};
  double max_linear_x_{0.6};
  double max_angular_z_{1.5};
  double max_linear_y_abs_{0.01};
  double unsupported_axis_tolerance_{0.01};
  int serial_fd_{-1};
  Clock::time_point next_open_attempt_{};
  Clock::time_point command_received_at_{};
  float latest_vx_{0.0F};
  float latest_wz_{0.0F};
  bool command_valid_{false};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
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
