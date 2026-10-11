#include <rm_serial_driver/packet.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm::serial
{
namespace
{
constexpr std::array<float, kArmJointCount> kExpectedAtRosZero{
  -0.22473F, -3.46819F, -1.62393F, -2.7583F, -0.7566F, -1.46542F};
constexpr std::array<float, kArmJointCount> kExpectedSlopes{
  1.0F, 1.0F, -1.0F, 1.0F, 1.0F, 1.0F};
constexpr std::array<float, kArmJointCount> kLower{
  -1.5708F, -1.1345F, -2.618F, -2.8798F, -1.578F, -2.8798F};
constexpr std::array<float, kArmJointCount> kUpper{
  3.6652F, 1.1345F, 2.618F, 2.8798F, 1.5708F, 2.8798F};

float readFloatLe(const std::vector<uint8_t>& bytes, std::size_t offset)
{
  uint32_t bits = static_cast<uint32_t>(bytes[offset]) |
    (static_cast<uint32_t>(bytes[offset + 1]) << 8U) |
    (static_cast<uint32_t>(bytes[offset + 2]) << 16U) |
    (static_cast<uint32_t>(bytes[offset + 3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

TEST(ArmJointMapping, ZeroOffsetsAndDirectionMatchCurrentDriverEquations)
{
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    EXPECT_NEAR(ArmRosAngleToSerialUnit(0.0F, joint, false), kExpectedAtRosZero[joint], 1.0e-6F);
    EXPECT_NEAR(
      ArmSerialUnitToRosAngle(kExpectedAtRosZero[joint], joint, false), 0.0F, 1.0e-6F);
    EXPECT_NEAR(
      ArmRosAngleToSerialUnit(0.2F, joint, false),
      kExpectedAtRosZero[joint] + kExpectedSlopes[joint] * 0.2F, 1.0e-6F);
  }
  // Source behavior only: the slope signs do not establish physical motor direction.
}

TEST(ArmJointMapping, RoundTripsZeroSignedAnglesAndUrdfEndpointsInRadiansAndDegrees)
{
  constexpr float kPi = 3.14159265358979323846F;
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    const std::array<float, 5> samples{
      kLower[joint], -0.37F, 0.0F, 0.42F, kUpper[joint]};
    for (const float ros_rad : samples)
    {
      for (const bool serial_degrees : {false, true})
      {
        const float wire = ArmRosAngleToSerialUnit(ros_rad, joint, serial_degrees);
        const float recovered = ArmSerialUnitToRosAngle(wire, joint, serial_degrees);
        EXPECT_NEAR(recovered, ros_rad, 2.0e-5F) << "joint=" << joint;
        if (serial_degrees)
        {
          const float wire_rad = wire * (kPi / 180.0F);
          EXPECT_NEAR(
            wire_rad, ArmRosAngleToSerialUnit(ros_rad, joint, false), 2.0e-5F)
            << "joint=" << joint;
        }
      }
    }
  }
}

TEST(ArmJointMapping, EncodedControlAndDecodedFeedbackUseTheSameJointOrderAndFloatUnits)
{
  const std::array<float, kArmJointCount> target{
    -0.8F, 0.6F, -1.2F, 1.1F, -0.5F, 2.2F};
  SendPacket outgoing;
  outgoing.arm_command_enabled = 1;
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    outgoing.arm.position[joint] = ArmRosAngleToSerialUnit(target[joint], joint, false);
  }
  const auto bytes = ToVector(outgoing);
  ASSERT_EQ(bytes.size(), 46U);
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    EXPECT_FLOAT_EQ(readFloatLe(bytes, 20U + 4U * joint), outgoing.arm.position[joint]);
  }

  std::vector<uint8_t> feedback(35U, 0U);
  feedback[0] = 0x5A;
  feedback[1] = 0x01;
  feedback[2] = 0x4A;
  feedback[3] = 0xB5;
  feedback[4] = 1;
  feedback[8] = 1;
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    const float raw = ArmRosAngleToSerialUnit(target[joint], joint, false);
    uint32_t bits = 0;
    std::memcpy(&bits, &raw, sizeof(bits));
    const std::size_t offset = 9U + 4U * joint;
    feedback[offset] = static_cast<uint8_t>(bits & 0xFFU);
    feedback[offset + 1] = static_cast<uint8_t>((bits >> 8U) & 0xFFU);
    feedback[offset + 2] = static_cast<uint8_t>((bits >> 16U) & 0xFFU);
    feedback[offset + 3] = static_cast<uint8_t>((bits >> 24U) & 0xFFU);
  }
  feedback[34] = 0x4B;
  ReceiveFrameDecoder decoder;
  const auto frames = decoder.Feed(feedback, feedback.size());
  ASSERT_EQ(frames.size(), 1U);
  ASSERT_TRUE(HasValidArmReceiveExtension(
    frames[0].packet, frames[0].size, frames[0].arm_offset, frames[0].protocol_version));
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    EXPECT_NEAR(
      ArmSerialUnitToRosAngle(frames[0].packet.arm.position[joint], joint, false),
      target[joint], 2.0e-5F) << "joint=" << joint;
  }
}
}  // namespace rm::serial
