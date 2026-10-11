#include <rm_serial_driver/packet.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace rm::serial
{
namespace
{
constexpr std::array<const char*, kArmJointCount> kJointNames{
  "J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"};

void WriteU32LE(std::vector<uint8_t>& bytes, std::size_t offset, uint32_t value)
{
  bytes[offset] = static_cast<uint8_t>(value & 0xFFU);
  bytes[offset + 1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  bytes[offset + 2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  bytes[offset + 3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

void WriteFloatLE(std::vector<uint8_t>& bytes, std::size_t offset, float value)
{
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  WriteU32LE(bytes, offset, bits);
}

std::vector<uint8_t> MakeFirmwareFeedback(
  uint32_t sequence, int8_t gripper = -1, uint8_t initialized = 1)
{
  // Mirrors Host.cpp: [5A, version, 4A, B5, inited, countLE, 6 float32(rad), gripper, 4B].
  std::vector<uint8_t> bytes(35, 0);
  bytes[0] = 0x5A;
  bytes[1] = 0x01;
  bytes[2] = 0x4A;
  bytes[3] = 0xB5;
  bytes[4] = initialized;
  WriteU32LE(bytes, 5, sequence);
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    WriteFloatLE(bytes, 9 + 4 * joint, static_cast<float>(joint) + 0.25F);
  }
  bytes[33] = static_cast<uint8_t>(gripper);
  bytes[34] = 0x4B;
  return bytes;
}

std::vector<std::string> ConfiguredJointNames()
{
  std::vector<std::string> result;
  for (const auto* name : kJointNames)
  {
    result.emplace_back(name);
  }
  return result;
}
}  // namespace

TEST(ArmSerialProtocol, ControlFrameMatchesFirmwareOffsetsAndLittleEndianFields)
{
  static_assert(sizeof(SendPacket) == 46);
  static_assert(offsetof(SendPacket, end_frame) == 13);
  static_assert(offsetof(SendPacket, arm_header) == 14);
  static_assert(offsetof(SendPacket, arm_command_enabled) == 15);
  static_assert(offsetof(SendPacket, arm_sequence) == 16);
  static_assert(offsetof(SendPacket, arm) == 20);
  static_assert(offsetof(SendPacket, gripper_command) == 44);
  static_assert(offsetof(SendPacket, arm_end_frame) == 45);

  SendPacket packet;
  packet.nav.linear_x = 1.0F;
  packet.nav.linear_y = -2.0F;
  packet.nav.angular_z = 0.5F;
  packet.arm_command_enabled = 1;
  packet.arm_sequence = 0x12345678U;
  packet.arm.position[0] = 1.25F;
  packet.arm.position[5] = -2.5F;
  packet.gripper_command = -1;
  const auto bytes = ToVector(packet);

  ASSERT_EQ(bytes.size(), 46U);
  EXPECT_EQ(bytes[0], 0xA5);
  EXPECT_EQ(bytes[13], 0x4A);
  EXPECT_EQ(bytes[14], 0xB5);
  EXPECT_EQ(bytes[1], 0x00);
  EXPECT_EQ(bytes[2], 0x00);
  EXPECT_EQ(bytes[3], 0x80);
  EXPECT_EQ(bytes[4], 0x3F);
  EXPECT_EQ(bytes[5], 0x00);
  EXPECT_EQ(bytes[6], 0x00);
  EXPECT_EQ(bytes[7], 0x00);
  EXPECT_EQ(bytes[8], 0xC0);
  EXPECT_EQ(bytes[9], 0x00);
  EXPECT_EQ(bytes[10], 0x00);
  EXPECT_EQ(bytes[11], 0x00);
  EXPECT_EQ(bytes[12], 0x3F);
  EXPECT_EQ(bytes[15], 1);
  EXPECT_EQ(bytes[16], 0x78);
  EXPECT_EQ(bytes[17], 0x56);
  EXPECT_EQ(bytes[18], 0x34);
  EXPECT_EQ(bytes[19], 0x12);
  EXPECT_EQ(bytes[44], 0xFF);
  EXPECT_EQ(bytes[45], 0x4B);
  EXPECT_EQ(bytes[20], 0x00);
  EXPECT_EQ(bytes[21], 0x00);
  EXPECT_EQ(bytes[22], 0xA0);
  EXPECT_EQ(bytes[23], 0x3F);
  EXPECT_EQ(bytes[40], 0x00);
  EXPECT_EQ(bytes[41], 0x00);
  EXPECT_EQ(bytes[42], 0x20);
  EXPECT_EQ(bytes[43], 0xC0);
}

TEST(ArmSerialProtocol, DefaultControlFrameHoldsGripper)
{
  SendPacket packet;
  const auto bytes = ToVector(packet);
  ASSERT_EQ(bytes.size(), 46U);
  EXPECT_EQ(packet.arm_command_enabled, 0);
  EXPECT_EQ(packet.gripper_command, 0);
  EXPECT_EQ(bytes[15], 0);
  EXPECT_EQ(bytes[44], 0);
  EXPECT_TRUE(IsValidGripperCommand(-1));
  EXPECT_TRUE(IsValidGripperCommand(0));
  EXPECT_TRUE(IsValidGripperCommand(1));
  EXPECT_FALSE(IsValidGripperCommand(2));
  EXPECT_FALSE(IsValidGripperCommand(-2));
}

TEST(ArmSerialProtocol, FirmwareFeedbackFieldsDecodeAtExactOffsets)
{
  static_assert(sizeof(ReceivePacket) == 34);
  const auto bytes = MakeFirmwareFeedback(0x78563412U, -1);
  ReceiveFrameDecoder decoder;
  const auto frames = decoder.Feed(bytes, bytes.size());

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].size, 35U);
  EXPECT_EQ(frames[0].arm_offset, 3U);
  EXPECT_EQ(frames[0].protocol_version, kReceiveProtocolVersion);
  EXPECT_TRUE(HasValidArmReceiveExtension(
    frames[0].packet, 35U, 3U, frames[0].protocol_version));
  EXPECT_EQ(frames[0].packet.arm_sequence, 0x78563412U);
  EXPECT_FLOAT_EQ(frames[0].packet.arm.position[0], 0.25F);
  EXPECT_FLOAT_EQ(frames[0].packet.arm.position[5], 5.25F);
  EXPECT_EQ(frames[0].packet.arm_feedback_gripper, 0xFF);
  EXPECT_TRUE(HasFiniteArmPosition(frames[0].packet));
}

TEST(ArmSerialProtocol, RandomUsbChunksAndSeveralFramesAreReassembled)
{
  std::vector<uint8_t> stream;
  for (uint32_t seq = 10; seq < 18; ++seq)
  {
    const auto frame = MakeFirmwareFeedback(seq);
    stream.insert(stream.end(), frame.begin(), frame.end());
  }

  ReceiveFrameDecoder decoder;
  std::mt19937 random(20261011U);
  std::uniform_int_distribution<std::size_t> chunk_size(1U, 19U);
  std::vector<ReceivedArmFrame> decoded;
  std::size_t offset = 0;
  while (offset < stream.size())
  {
    const auto count = std::min(chunk_size(random), stream.size() - offset);
    const std::vector<uint8_t> chunk(stream.begin() + offset, stream.begin() + offset + count);
    const auto new_frames = decoder.Feed(chunk, chunk.size());
    decoded.insert(decoded.end(), new_frames.begin(), new_frames.end());
    offset += count;
  }

  ASSERT_EQ(decoded.size(), 8U);
  for (std::size_t i = 0; i < decoded.size(); ++i)
  {
    EXPECT_EQ(decoded[i].packet.arm_sequence, 10U + i);
    EXPECT_TRUE(HasValidArmReceiveExtension(
      decoded[i].packet, decoded[i].size, decoded[i].arm_offset, decoded[i].protocol_version));
  }
}

TEST(ArmSerialProtocol, EmptyInputProducesNoFeedbackAndResetDropsPartialReconnectData)
{
  ReceiveFrameDecoder decoder;
  EXPECT_TRUE(decoder.Feed({}, 0).empty());
  const auto old_frame = MakeFirmwareFeedback(10);
  std::vector<uint8_t> partial(old_frame.begin(), old_frame.begin() + 11);
  EXPECT_TRUE(decoder.Feed(partial, partial.size()).empty());
  decoder.Reset();
  const auto new_frame = MakeFirmwareFeedback(1);
  const auto decoded = decoder.Feed(new_frame, new_frame.size());
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0].packet.arm_sequence, 1U);
}

TEST(ArmSerialProtocol, TruncatedFrameWaitsForTheRemainingBytes)
{
  const auto bytes = MakeFirmwareFeedback(77);
  ReceiveFrameDecoder decoder;
  std::vector<uint8_t> first(bytes.begin(), bytes.begin() + 12);
  EXPECT_TRUE(decoder.Feed(first, first.size()).empty());
  std::vector<uint8_t> rest(bytes.begin() + 12, bytes.end());
  const auto frames = decoder.Feed(rest, rest.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].packet.arm_sequence, 77U);
}

TEST(ArmSerialProtocol, CorruptPrefixAndTrailerResynchronizeToNextFrame)
{
  auto bad = MakeFirmwareFeedback(100);
  bad[34] = 0x00;
  auto stream = bad;
  const auto good = MakeFirmwareFeedback(101);
  stream.insert(stream.end(), good.begin(), good.end());
  stream.insert(stream.begin(), {0x00, 0x11, 0x5A, 0x02, 0x03});

  ReceiveFrameDecoder decoder;
  const auto frames = decoder.Feed(stream, stream.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].packet.arm_sequence, 101U);
}

TEST(ArmSerialProtocol, InvalidExtensionAndValidityFlagAreRejected)
{
  auto bad_header = MakeFirmwareFeedback(1);
  bad_header[3] = 0xB4;
  ReceiveFrameDecoder decoder;
  auto frames = decoder.Feed(bad_header, bad_header.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_FALSE(HasValidArmReceiveExtension(frames[0].packet, frames[0].size, frames[0].arm_offset));

  decoder.Reset();
  auto wrong_version = MakeFirmwareFeedback(2);
  wrong_version[1] = 0x02;
  frames = decoder.Feed(wrong_version, wrong_version.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_FALSE(HasValidArmReceiveExtension(
    frames[0].packet, frames[0].size, frames[0].arm_offset, frames[0].protocol_version));

  decoder.Reset();
  auto not_initialized = MakeFirmwareFeedback(3, -1, 0);
  frames = decoder.Feed(not_initialized, not_initialized.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_FALSE(HasValidArmReceiveExtension(
    frames[0].packet, frames[0].size, frames[0].arm_offset, frames[0].protocol_version));
}

TEST(ArmSerialProtocol, NonFiniteFeedbackIsRejected)
{
  auto bytes = MakeFirmwareFeedback(8);
  WriteFloatLE(bytes, 9 + 2 * 4, std::numeric_limits<float>::quiet_NaN());
  ReceiveFrameDecoder decoder;
  const auto frames = decoder.Feed(bytes, bytes.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_FALSE(HasFiniteArmPosition(frames[0].packet));
}

TEST(ArmSerialProtocol, DuplicateFeedbackSequenceIsIgnoredAndCanReset)
{
  ReceiveSequenceTracker tracker;
  uint32_t missed = 0;
  EXPECT_EQ(tracker.Observe(0xFFFFFFFEU, missed), ReceiveSequenceResult::kFirst);
  EXPECT_EQ(tracker.Observe(0xFFFFFFFFU, missed), ReceiveSequenceResult::kInOrder);
  EXPECT_EQ(tracker.Observe(0xFFFFFFFFU, missed), ReceiveSequenceResult::kDuplicate);
  EXPECT_EQ(tracker.Observe(0U, missed), ReceiveSequenceResult::kInOrder);
  EXPECT_EQ(tracker.Observe(3U, missed), ReceiveSequenceResult::kGap);
  EXPECT_EQ(missed, 2U);
  EXPECT_EQ(tracker.Observe(0U, missed), ReceiveSequenceResult::kControllerRestarted);
  tracker.Reset();
  EXPECT_EQ(tracker.Observe(0U, missed), ReceiveSequenceResult::kFirst);
}

TEST(ArmSerialProtocol, JointStateNamesAreMappedRegardlessOfInputOrder)
{
  const auto configured = ConfiguredJointNames();
  const std::vector<std::string> names{
    "J3_joint", "J1_joint", "J6_joint", "J2_joint", "J5_joint", "J4_joint"};
  const std::vector<double> positions{3.0, 1.0, 6.0, 2.0, 5.0, 4.0};
  std::array<float, kArmJointCount> parsed{};
  EXPECT_EQ(ParseArmJointCommand(configured, names, positions, parsed), ArmJointCommandError::kNone);
  EXPECT_FLOAT_EQ(parsed[0], 1.0F);
  EXPECT_FLOAT_EQ(parsed[1], 2.0F);
  EXPECT_FLOAT_EQ(parsed[2], 3.0F);
  EXPECT_FLOAT_EQ(parsed[3], 4.0F);
  EXPECT_FLOAT_EQ(parsed[4], 5.0F);
  EXPECT_FLOAT_EQ(parsed[5], 6.0F);
}

TEST(ArmSerialProtocol, IncompleteDuplicateAndNonFiniteJointCommandsAreRejected)
{
  const auto configured = ConfiguredJointNames();
  std::array<float, kArmJointCount> parsed{};
  auto names = configured;
  std::vector<double> positions(kArmJointCount, 0.1);

  names.pop_back();
  EXPECT_EQ(ParseArmJointCommand(configured, names, positions, parsed), ArmJointCommandError::kNamePositionSizeMismatch);

  names = configured;
  names.back() = names.front();
  EXPECT_EQ(ParseArmJointCommand(configured, names, positions, parsed), ArmJointCommandError::kDuplicateJoint);

  names = configured;
  names.pop_back();
  positions.pop_back();
  EXPECT_EQ(ParseArmJointCommand(configured, names, positions, parsed), ArmJointCommandError::kMissingJoint);

  names = configured;
  positions.assign(kArmJointCount, 0.1);
  positions[3] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(ParseArmJointCommand(configured, names, positions, parsed), ArmJointCommandError::kNonFinitePosition);

  positions[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ParseArmJointCommand(configured, names, positions, parsed), ArmJointCommandError::kNonFinitePosition);

  positions[3] = std::numeric_limits<double>::max();
  EXPECT_EQ(ParseArmJointCommand(configured, names, positions, parsed), ArmJointCommandError::kNonFinitePosition);
}
}  // namespace rm::serial
