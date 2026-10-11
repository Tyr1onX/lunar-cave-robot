// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <unordered_map>

#include <rm_serial_driver/node.hpp>

namespace rm::serial
{
DriverNode::DriverNode(const rclcpp::NodeOptions& options)
: Node("serial_driver", options),
  owned_ctx_(2), serial_driver_(owned_ctx_)
{
  using namespace std::placeholders;

  auto device_name = declare_parameter<std::string>("device_name");
  auto device_config = getDeviceConfig();

  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_chassis", rclcpp::SensorDataQoS(),
    std::bind(&DriverNode::navCallback, this, _1));

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

  timer_ = create_wall_timer(std::chrono::milliseconds(4), std::bind(&DriverNode::timerCallback, this));
  RCLCPP_INFO(get_logger(), "Started!");
}

DriverNode::~DriverNode()
{
  if (serial_driver_.port()->is_open())
  {
    serial_driver_.port()->close();
  }

  owned_ctx_.waitForExit();
}

void DriverNode::navCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  try
  {
    std::scoped_lock lock{nav_mutex_};
    nav_packet_.linear_x = static_cast<float>(msg->linear.x);
    nav_packet_.linear_y = -static_cast<float>(msg->linear.y);
    nav_packet_.angular_z = static_cast<float>(msg->angular.z);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error from nav callback: %s", e.what());
  }
}

void DriverNode::timerCallback()
{
  try
  {
    SendPacket send_packet;
    std::scoped_lock lock{nav_mutex_};
    send_packet.nav = nav_packet_;
    serial_driver_.port()->send(ToVector(send_packet));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error from timer callback: %s", e.what());
    reopenPort();
  }
}

void DriverNode::startSerialReceive()
{
  using namespace std::placeholders;
  serial_driver_.port()->async_receive(std::bind(&DriverNode::serialReceiveCallback, this, _1, _2));
}

void DriverNode::serialReceiveCallback(const std::vector<uint8_t>& data, std::size_t size)
{
  if (size < sizeof(ReceivePacket))
  {
    RCLCPP_WARN(get_logger(), "Serial receive incomplete");
    startSerialReceive();
    return;
  }

  if (data.front() != 0x5A)
  {
    RCLCPP_WARN(get_logger(), "Invalid header: %02X", data.front());
    startSerialReceive();
    return;
  }

  const ReceivePacket packet = FromVector(data);
  if (packet.end_frame != 0x4A)
  {
    RCLCPP_WARN(get_logger(), "Invalid end frame: %02X", packet.end_frame);
  }

  // Reserved receive-side hook for future fields / publishers.
  startSerialReceive();
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
  while (rclcpp::ok())
  {
    try
    {
      if (serial_driver_.port()->is_open())
      {
        serial_driver_.port()->close();
      }

      serial_driver_.port()->open();
      startSerialReceive();
      return;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}
}  // namespace rm::serial

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(rm::serial::DriverNode)
