#ifndef _RM_SERIAL_DRIVER_NODE_HPP
#define _RM_SERIAL_DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rcpputils/filesystem_helper.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>
#include <mutex>

#include <rm_serial_driver/packet.hpp>

namespace rm::serial
{
class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  ~DriverNode() override;

private:
  drivers::serial_driver::SerialPortConfig getDeviceConfig();
  
  void navCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void timerCallback();
  
  void startSerialReceive();
  void serialReceiveCallback(const std::vector<uint8_t>& data, std::size_t size);

  void reopenPort();

  // ******************** ROS Subscription ********************
  // Nav Subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;

  // ******************** ROS Timer ********************
  rclcpp::TimerBase::SharedPtr timer_;

  // ******************** Send ********************
  NavPacket nav_packet_;

  // ******************** Mutex ********************
  std::mutex nav_mutex_;
  
  // ******************** Serial Port ********************
  drivers::common::IoContext owned_ctx_;
  drivers::serial_driver::SerialDriver serial_driver_;
};
}

#endif
