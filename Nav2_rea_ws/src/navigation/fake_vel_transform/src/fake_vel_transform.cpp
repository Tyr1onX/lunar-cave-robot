// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "base_frame");
  this->declare_parameter<std::string>("fake_robot_base_frame", "base_frame_fake");
  this->declare_parameter<std::string>("odom_topic", "/odometry");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel_chassis");
  this->declare_parameter<float>("spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("spin_speed", spin_speed_);

  spin_speed_storage = spin_speed_;
  spin_speed_ = 0.0;
  current_robot_base_angle_ = 0.0;
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&FakeVelTransform::odomCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 1,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  ai_tuoluo_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/ai_tuoluo", 1,
      std::bind(&FakeVelTransform::tuoluoCallback, this, std::placeholders::_1));
}

void FakeVelTransform::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
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

// Transform the velocity from `robot_base_frame` to `fake_robot_base_frame`
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double angle_diff = current_robot_base_angle_;

  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = msg->angular.z;
  aft_tf_vel.linear.x = (msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff))*chengshu;
  aft_tf_vel.linear.y = (-msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff))*chengshu;
  //aft_tf_vel.linear.y = (msg->linear.x * sin(angle_diff) - msg->linear.y * cos(angle_diff))*chengshu;

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::tuoluoCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "ai_tuoluo: %d", msg->data);
  if(msg->data == 1) { 
    spin_speed_ = spin_speed_storage;
    chengshu = 1.0;
  } 
  else if(msg->data == 0) {
    spin_speed_ = 0.0;
    chengshu = 1.0;
  } 
  else { // diankong tuoluo
    spin_speed_ = 0.0;
    chengshu = 1.0;
  }
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
