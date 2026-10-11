#include <rm_serial_driver/packet.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rm::serial
{
namespace
{
constexpr std::size_t kPacketArmOffset = offsetof(ReceivePacket, arm_header);
constexpr std::size_t kReceiveBytesBeforeGripper =
  offsetof(ReceivePacket, arm_feedback_gripper) - kPacketArmOffset;
constexpr std::size_t kReceiveArmExtensionMinSize =
  kReceiveBytesBeforeGripper + sizeof(ReceivePacket::arm_end_frame);
constexpr std::size_t kReceiveArmExtensionFullSize =
  kReceiveBytesBeforeGripper + sizeof(ReceivePacket::arm_feedback_gripper) +
  sizeof(ReceivePacket::arm_end_frame);
constexpr double kPi = 3.14159265358979323846;
}

ReceivePacket FromVector(const std::vector<uint8_t>& data)
{
  return FromVectorWithArmOffset(data, kLegacyReceiveFrameSize);
}

std::vector<ReceivedArmFrame> ReceiveFrameDecoder::Feed(
  const std::vector<uint8_t>& data, std::size_t size)
{
  const std::size_t bytes_to_append = std::min(size, data.size());
  buffer_.insert(buffer_.end(), data.begin(), data.begin() + bytes_to_append);

  std::vector<ReceivedArmFrame> frames;
  while (!buffer_.empty())
  {
    if (buffer_.front() != kReceiveHeader)
    {
      buffer_.erase(buffer_.begin());
      continue;
    }
    if (buffer_.size() < kLegacyReceiveFrameSize)
    {
      break;
    }

    std::size_t arm_offset = 0;
    if (buffer_[1] == kLegacyEndFrame)
    {
      arm_offset = kLegacyReceiveFrameSize;
    }
    else if (buffer_.size() < kStatusReceiveFrameSize)
    {
      break;
    }
    else if (buffer_[2] == kLegacyEndFrame)
    {
      arm_offset = kStatusReceiveFrameSize;
    }
    else
    {
      buffer_.erase(buffer_.begin());
      continue;
    }

    // The paired STM32 firmware sends the complete extension, including the
    // gripper echo byte, so the full-frame length is unambiguous.
    const std::size_t frame_size = arm_offset + kReceiveArmExtensionFullSize;
    if (buffer_.size() < frame_size)
    {
      break;
    }
    if (buffer_[frame_size - 1] != kArmExtensionEndFrame)
    {
      buffer_.erase(buffer_.begin());
      continue;
    }

    std::vector<uint8_t> frame_bytes(buffer_.begin(), buffer_.begin() + frame_size);
    const uint8_t protocol_version =
      arm_offset == kStatusReceiveFrameSize ? buffer_[1] : 0U;
    frames.push_back({
      FromVectorWithArmOffset(frame_bytes, arm_offset), frame_size, arm_offset, protocol_version});
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  }
  return frames;
}

void ReceiveFrameDecoder::Reset()
{
  buffer_.clear();
}

ReceiveSequenceResult ReceiveSequenceTracker::Observe(uint32_t sequence, uint32_t& missed)
{
  missed = 0;
  if (!initialized_)
  {
    initialized_ = true;
    last_sequence_ = sequence;
    return ReceiveSequenceResult::kFirst;
  }

  const uint32_t delta = sequence - last_sequence_;
  if (delta == 0U)
  {
    return ReceiveSequenceResult::kDuplicate;
  }
  last_sequence_ = sequence;
  if (delta == 1U)
  {
    return ReceiveSequenceResult::kInOrder;
  }
  if (delta < 0x80000000U)
  {
    missed = delta - 1U;
    return ReceiveSequenceResult::kGap;
  }
  return ReceiveSequenceResult::kControllerRestarted;
}

void ReceiveSequenceTracker::Reset()
{
  initialized_ = false;
  last_sequence_ = 0;
}

float ArmRosAngleToSerialUnit(
  float angle_rad, std::size_t joint_index, bool serial_in_degrees)
{
  float serial_angle_rad = angle_rad;
  switch (joint_index)
  {
    case 0: serial_angle_rad = angle_rad - 0.22473F; break;
    case 1: serial_angle_rad = angle_rad - 3.46819F; break;
    case 2: serial_angle_rad = -angle_rad - 1.62393F; break;
    case 3: serial_angle_rad = angle_rad - 2.7583F; break;
    case 4: serial_angle_rad = angle_rad - 0.7566F; break;
    case 5: serial_angle_rad = angle_rad - 1.46542F; break;
    case 6: serial_angle_rad = angle_rad - 0.19149F; break;  // legacy, outside current 6-joint packet
    default: break;
  }
  return serial_in_degrees ?
    serial_angle_rad * static_cast<float>(180.0 / kPi) : serial_angle_rad;
}

float ArmSerialUnitToRosAngle(
  float serial_angle, std::size_t joint_index, bool serial_in_degrees)
{
  const float serial_angle_rad = serial_in_degrees ?
    serial_angle * static_cast<float>(kPi / 180.0) : serial_angle;
  switch (joint_index)
  {
    case 0: return serial_angle_rad + 0.22473F;
    case 1: return serial_angle_rad + 3.46819F;
    case 2: return -serial_angle_rad - 1.62393F;
    case 3: return serial_angle_rad + 2.7583F;
    case 4: return serial_angle_rad + 0.7566F;
    case 5: return serial_angle_rad + 1.46542F;
    case 6: return serial_angle_rad + 0.19149F;  // legacy, outside current 6-joint packet
    default: return serial_angle_rad;
  }
}

ArmJointCommandError ParseArmJointCommand(
  const std::vector<std::string>& configured_names,
  const std::vector<std::string>& names,
  const std::vector<double>& positions,
  const std::array<float, kArmJointCount>& lower_limits_rad,
  const std::array<float, kArmJointCount>& upper_limits_rad,
  std::array<float, kArmJointCount>& result)
{
  if (configured_names.size() != kArmJointCount)
  {
    return ArmJointCommandError::kInvalidConfiguredJoints;
  }
  for (std::size_t i = 0; i < configured_names.size(); ++i)
  {
    if (configured_names[i].empty() ||
        std::find(configured_names.begin(), configured_names.begin() + i, configured_names[i]) !=
        configured_names.begin() + i ||
        !std::isfinite(lower_limits_rad[i]) || !std::isfinite(upper_limits_rad[i]) ||
        lower_limits_rad[i] >= upper_limits_rad[i])
    {
      return ArmJointCommandError::kInvalidConfiguredJoints;
    }
  }
  if (names.size() != positions.size())
  {
    return ArmJointCommandError::kNamePositionSizeMismatch;
  }

  std::array<float, kArmJointCount> parsed{};
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
  {
    std::size_t found = names.size();
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      if (names[i] == configured_names[joint])
      {
        if (found != names.size())
        {
          return ArmJointCommandError::kDuplicateJoint;
        }
        found = i;
      }
    }
    if (found == names.size())
    {
      return ArmJointCommandError::kMissingJoint;
    }
    if (!std::isfinite(positions[found]))
    {
      return ArmJointCommandError::kNonFinitePosition;
    }
    parsed[joint] = static_cast<float>(positions[found]);
    if (!std::isfinite(parsed[joint]))
    {
      return ArmJointCommandError::kNonFinitePosition;
    }
    if (parsed[joint] < lower_limits_rad[joint] ||
        parsed[joint] > upper_limits_rad[joint])
    {
      return ArmJointCommandError::kOutOfRangePosition;
    }
  }
  result = parsed;
  return ArmJointCommandError::kNone;
}

bool HasFiniteArmPosition(const ReceivePacket& packet)
{
  for (float position : packet.arm.position)
  {
    if (!std::isfinite(position))
    {
      return false;
    }
  }
  return true;
}

bool IsValidGripperCommand(int8_t command)
{
  return command == 1 || command == 0 || command == -1;
}

ReceivePacket FromVectorWithArmOffset(const std::vector<uint8_t>& data, std::size_t arm_offset)
{
  ReceivePacket packet;
  if (data.empty())
  {
    return packet;
  }

  packet.header = data[0];
  if (arm_offset > 0 && data.size() >= arm_offset)
  {
    packet.end_frame = data[arm_offset - 1];
  }
  else if (data.size() >= kLegacyReceiveFrameSize)
  {
    packet.end_frame = data[1];
  }

  if (data.size() > arm_offset)
  {
    const std::size_t bytes_to_copy = std::min(
      data.size() - arm_offset,
      kReceiveBytesBeforeGripper);
    std::memcpy(
      reinterpret_cast<uint8_t *>(&packet) + kPacketArmOffset,
      data.data() + arm_offset,
      bytes_to_copy);

    const std::size_t arm_payload_size = data.size() - arm_offset;
    if (arm_payload_size >= kReceiveArmExtensionFullSize)
    {
      packet.arm_feedback_gripper =
        data[arm_offset + kReceiveBytesBeforeGripper];
      packet.arm_end_frame =
        data[arm_offset + kReceiveBytesBeforeGripper + 1];
    }
    else if (arm_payload_size >= kReceiveArmExtensionMinSize)
    {
      packet.arm_feedback_gripper = 0;
      packet.arm_end_frame =
        data[arm_offset + kReceiveBytesBeforeGripper];
    }
  }
  return packet;
}

std::vector<uint8_t> ToVector(const SendPacket& data, bool include_arm_extension)
{
  std::vector<uint8_t> packet(SendPacketSize(include_arm_extension));
  std::memcpy(packet.data(), &data, packet.size());
  return packet;
}

std::size_t SendPacketSize(bool include_arm_extension)
{
  return include_arm_extension ? sizeof(SendPacket) : LegacySendPacketSize();
}

std::size_t LegacySendPacketSize()
{
  return sizeof(uint8_t) + sizeof(NavPacket) + sizeof(uint8_t);
}

std::size_t ReceiveArmExtensionOffset(const std::vector<uint8_t>& data, std::size_t size)
{
  if (size >= kLegacyReceiveFrameSize &&
      data[0] == kReceiveHeader &&
      data[1] == kLegacyEndFrame)
  {
    return kLegacyReceiveFrameSize;
  }
  if (size >= kStatusReceiveFrameSize &&
      data[0] == kReceiveHeader &&
      data[2] == kLegacyEndFrame)
  {
    return kStatusReceiveFrameSize;
  }
  return 0;
}

std::size_t ReceivePacketSize(std::size_t arm_offset)
{
  return arm_offset + kReceiveArmExtensionMinSize;
}

bool HasValidArmReceiveExtension(
  const ReceivePacket& packet,
  std::size_t size,
  std::size_t arm_offset,
  uint8_t protocol_version)
{
  return (arm_offset != kStatusReceiveFrameSize || protocol_version == kReceiveProtocolVersion) &&
         size >= ReceivePacketSize(arm_offset) &&
         packet.header == kReceiveHeader &&
         packet.end_frame == kLegacyEndFrame &&
         packet.arm_header == kArmExtensionHeader &&
         packet.arm_end_frame == kArmExtensionEndFrame &&
         packet.arm_state_valid != 0;
}
} 
