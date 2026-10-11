#include "lite_motion_planner/pose_transform.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::TransformStamped makeTransform(
  const std::string& parent,
  const std::string& child,
  int32_t sec,
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.header.stamp.sec = sec;
  transform.child_frame_id = child;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  const Eigen::Quaterniond q(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  return transform;
}

geometry_msgs::msg::PoseStamped makePose(
  const std::string& frame,
  int32_t sec,
  double x,
  double y,
  double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp.sec = sec;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

std::shared_ptr<tf2_ros::Buffer> makeBuffer()
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto buffer = std::make_shared<tf2_ros::Buffer>(clock);
  buffer->setUsingDedicatedThread(true);
  return buffer;
}

TEST(PoseTransform, AppliesOpticalRotationAndTimestampMatchedArmPose)
{
  auto buffer = makeBuffer();

  // Synthetic convention-only transform from the Orbbec driver: no translation
  // is asserted here, and these values are not a hand-eye calibration.
  auto optical = makeTransform(
    "camera_link", "camera_color_optical_frame", 0, 0.0, 0.0, 0.0,
    -kPi / 2.0, 0.0, -kPi / 2.0);
  ASSERT_TRUE(buffer->setTransform(optical, "p3_test", true));

  // Two synthetic dynamic TF samples model two different arm poses.
  auto arm_pose_1 = makeTransform(
    "base_link", "camera_link", 10, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  auto arm_pose_2 = makeTransform(
    "base_link", "camera_link", 20, 2.0, 0.0, 0.0, 0.0, 0.0, kPi / 2.0);
  ASSERT_TRUE(buffer->setTransform(arm_pose_1, "p3_test", false));
  ASSERT_TRUE(buffer->setTransform(arm_pose_2, "p3_test", false));

  geometry_msgs::msg::PoseStamped out_1;
  geometry_msgs::msg::PoseStamped out_2;
  std::string reason;
  ASSERT_TRUE(lite_motion_planner::transformPoseAtTimestamp(
    *buffer, makePose("camera_color_optical_frame", 10, 1.0, 0.0, 0.0),
    "base_link", RCL_SYSTEM_TIME, 0.0, out_1, &reason)) << reason;
  ASSERT_TRUE(lite_motion_planner::transformPoseAtTimestamp(
    *buffer, makePose("camera_color_optical_frame", 20, 1.0, 0.0, 0.0),
    "base_link", RCL_SYSTEM_TIME, 0.0, out_2, &reason)) << reason;

  // Optical +X maps to camera_link -Y under the driver convention.
  EXPECT_NEAR(out_1.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(out_1.pose.position.y, -1.0, 1e-6);
  EXPECT_NEAR(out_1.pose.position.z, 0.0, 1e-6);
  // The second arm TF is rotated 90 degrees and must produce a different base pose.
  EXPECT_NEAR(out_2.pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(out_2.pose.position.y, 0.0, 1e-6);
  EXPECT_NEAR(out_2.pose.position.z, 0.0, 1e-6);
  EXPECT_EQ(out_1.header.frame_id, "base_link");
  EXPECT_EQ(out_2.header.frame_id, "base_link");
  EXPECT_EQ(out_1.header.stamp.sec, 10);
  EXPECT_EQ(out_2.header.stamp.sec, 20);
}

TEST(PoseTransform, DoesNotUseLatestTfWhenObservationTimeIsUnavailable)
{
  auto buffer = makeBuffer();
  auto optical = makeTransform(
    "camera_link", "camera_color_optical_frame", 0, 0.0, 0.0, 0.0,
    -kPi / 2.0, 0.0, -kPi / 2.0);
  auto arm_pose = makeTransform(
    "base_link", "camera_link", 10, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(buffer->setTransform(optical, "p3_test", true));
  ASSERT_TRUE(buffer->setTransform(arm_pose, "p3_test", false));

  geometry_msgs::msg::PoseStamped output;
  std::string reason;
  EXPECT_FALSE(lite_motion_planner::transformPoseAtTimestamp(
    *buffer,
    makePose("camera_color_optical_frame", 20, 0.1, 0.2, 0.3),
    "base_link", RCL_SYSTEM_TIME, 0.0, output, &reason));
  EXPECT_NE(reason.find("no transform at observation timestamp"), std::string::npos);
}

TEST(PoseTransform, RejectsMissingOrDisconnectedTf)
{
  auto buffer = makeBuffer();
  auto pose = makePose("unknown_camera_frame", 30, 0.1, 0.2, 0.3);
  geometry_msgs::msg::PoseStamped output;
  std::string reason;

  EXPECT_FALSE(lite_motion_planner::transformPoseAtTimestamp(
    *buffer, pose, "base_link", RCL_SYSTEM_TIME, 0.0, output, &reason));
  EXPECT_NE(reason.find("no transform at observation timestamp"), std::string::npos);
}

TEST(PoseTransform, RejectsStaleZeroAndFutureTimestamps)
{
  const rclcpp::Time now(100, 0, RCL_SYSTEM_TIME);
  std::string reason;

  auto stale = makePose("camera_color_optical_frame", 99, 0.0, 0.0, 0.0);
  EXPECT_FALSE(lite_motion_planner::validatePoseEstimatorTimestamp(
    stale, now, 0.5, 0.05, &reason));
  EXPECT_NE(reason.find("stale"), std::string::npos);

  auto zero = makePose("camera_color_optical_frame", 0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(lite_motion_planner::validatePoseEstimatorTimestamp(
    zero, now, 0.5, 0.05, &reason));
  EXPECT_NE(reason.find("zero timestamp"), std::string::npos);

  auto future = makePose("camera_color_optical_frame", 101, 0.0, 0.0, 0.0);
  EXPECT_FALSE(lite_motion_planner::validatePoseEstimatorTimestamp(
    future, now, 0.5, 0.05, &reason));
  EXPECT_NE(reason.find("future"), std::string::npos);

  auto fresh = makePose("camera_color_optical_frame", 100, 0.0, 0.0, 0.0);
  EXPECT_TRUE(lite_motion_planner::validatePoseEstimatorTimestamp(
    fresh, now, 0.5, 0.05, &reason)) << reason;
}

}  // namespace
