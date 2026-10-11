#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <string>

namespace lite_motion_planner {

inline bool validatePoseEstimatorTimestamp(
  const geometry_msgs::msg::PoseStamped& pose,
  const rclcpp::Time& now,
  double max_age_sec,
  double future_tolerance_sec,
  std::string* reason = nullptr)
{
  const auto fail = [reason](const std::string& message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };

  if (pose.header.frame_id.empty()) {
    return fail("pose header has an empty frame_id");
  }
  if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {
    return fail("pose header has a zero timestamp");
  }

  try {
    const rclcpp::Time pose_time(pose.header.stamp, now.get_clock_type());
    const double age_sec = (now - pose_time).seconds();
    if (!std::isfinite(age_sec)) {
      return fail("pose timestamp age is not finite");
    }
    if (age_sec > max_age_sec) {
      return fail("pose timestamp is stale (age=" + std::to_string(age_sec) + " s)");
    }
    if (age_sec < -future_tolerance_sec) {
      return fail("pose timestamp is in the future (offset=" +
                  std::to_string(-age_sec) + " s)");
    }
  } catch (const std::exception& error) {
    return fail(std::string("pose timestamp clock mismatch: ") + error.what());
  }
  return true;
}

inline bool transformPoseAtTimestamp(
  const tf2_ros::Buffer& buffer,
  const geometry_msgs::msg::PoseStamped& input,
  const std::string& target_frame,
  rcl_clock_type_t clock_type,
  double lookup_timeout_sec,
  geometry_msgs::msg::PoseStamped& output,
  std::string* reason = nullptr)
{
  const auto fail = [reason](const std::string& message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };

  if (input.header.frame_id.empty()) {
    return fail("pose header has an empty frame_id");
  }
  if (target_frame.empty()) {
    return fail("planning frame is empty");
  }
  if (input.header.stamp.sec == 0 && input.header.stamp.nanosec == 0) {
    return fail("pose header has a zero timestamp");
  }
  if (input.header.frame_id == target_frame) {
    output = input;
    return true;
  }

  try {
    const rclcpp::Time observation_time(input.header.stamp, clock_type);
    const auto transform = buffer.lookupTransform(
      target_frame,
      input.header.frame_id,
      observation_time,
      rclcpp::Duration::from_seconds(lookup_timeout_sec));
    tf2::doTransform(input, output, transform);
    // Keep the observation time explicit; the transform is evaluated at this
    // same time and must not be mistaken for a newly observed target.
    output.header.stamp = input.header.stamp;
    output.header.frame_id = target_frame;
  } catch (const tf2::TransformException& error) {
    return fail(std::string("no transform at observation timestamp: ") + error.what());
  } catch (const std::exception& error) {
    return fail(std::string("pose transform failed: ") + error.what());
  }
  return true;
}

}  // namespace lite_motion_planner
