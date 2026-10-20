#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace chassis_serial_bridge
{
namespace protocol
{
constexpr std::size_t kFrameSize = 14;
constexpr std::uint8_t kFrameHeader = 0xA5;
constexpr std::uint8_t kFrameTail = 0x4A;

using Frame = std::array<std::uint8_t, kFrameSize>;
using VYBytes = std::array<std::uint8_t, 4>;

enum class PusherCommand
{
  Stop,
  Retract,
  Extend
};

enum class PusherRequestResult
{
  Accepted,
  Stopped,
  RejectedInterlock
};

constexpr VYBytes kStopVY{0x00, 0x00, 0x00, 0x00};
constexpr VYBytes kRetractVY{0xFF, 0xFF, 0xFF, 0xFF};
constexpr VYBytes kExtendVY{0xEE, 0xEE, 0xEE, 0xEE};

inline void writeFloatLe(Frame & frame, std::size_t offset, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "protocol requires float32");
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t i = 0; i < sizeof(bits); ++i) {
    frame[offset + i] = static_cast<std::uint8_t>((bits >> (8U * i)) & 0xFFU);
  }
}

inline Frame encodeFrame(float vx, float vz, const VYBytes & vy)
{
  Frame frame{};
  frame[0] = kFrameHeader;
  writeFloatLe(frame, 1, vx);
  for (std::size_t i = 0; i < vy.size(); ++i) {
    // VY may contain protocol command bytes. Copy bytes directly; never turn
    // the FF/EE tokens into a floating-point NaN or other float value.
    frame[5 + i] = vy[i];
  }
  writeFloatLe(frame, 9, vz);
  frame[13] = kFrameTail;
  return frame;
}

inline VYBytes pusherBytes(PusherCommand command)
{
  switch (command) {
    case PusherCommand::Retract:
      return kRetractVY;
    case PusherCommand::Extend:
      return kExtendVY;
    case PusherCommand::Stop:
    default:
      return kStopVY;
  }
}

inline bool parsePusherCommand(const std::string & input, PusherCommand & command)
{
  if (input == "STOP") {
    command = PusherCommand::Stop;
    return true;
  }
  if (input == "RETRACT") {
    command = PusherCommand::Retract;
    return true;
  }
  if (input == "EXTEND") {
    command = PusherCommand::Extend;
    return true;
  }
  return false;
}
}  // namespace protocol

class CommandState final
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  CommandState(std::chrono::milliseconds motion_timeout,
               std::chrono::milliseconds pusher_timeout)
  : motion_timeout_(motion_timeout), pusher_timeout_(pusher_timeout)
  {}

  void setTimeouts(std::chrono::milliseconds motion_timeout,
                   std::chrono::milliseconds pusher_timeout)
  {
    motion_timeout_ = motion_timeout;
    pusher_timeout_ = pusher_timeout;
  }

  bool setMotion(float vx, float wz, TimePoint now)
  {
    if (pusherActive() || stop_fence_pending_) {
      clearMotion();
      return std::abs(vx) <= 1.0e-6F && std::abs(wz) <= 1.0e-6F;
    }
    latest_vx_ = vx;
    latest_wz_ = wz;
    motion_received_at_ = now;
    motion_valid_ = true;
    return true;
  }

  protocol::PusherRequestResult requestPusher(protocol::PusherCommand command, TimePoint now)
  {
    if (command == protocol::PusherCommand::Stop) {
      const bool was_active = pusherActive();
      pusher_command_ = protocol::PusherCommand::Stop;
      if (was_active) {
        clearMotion();
        stop_fence_pending_ = true;
      }
      return protocol::PusherRequestResult::Stopped;
    }

    if (stop_fence_pending_) {
      return protocol::PusherRequestResult::RejectedInterlock;
    }
    if (pusherActive() && pusher_command_ != command) {
      // Reversals require a serialized zero-VY stop frame first.
      pusher_command_ = protocol::PusherCommand::Stop;
      clearMotion();
      stop_fence_pending_ = true;
      return protocol::PusherRequestResult::RejectedInterlock;
    }

    pusher_command_ = command;
    pusher_deadline_ = now + pusher_timeout_;
    clearMotion();
    return protocol::PusherRequestResult::Accepted;
  }

  void rejectAll()
  {
    const bool was_active = pusherActive();
    pusher_command_ = protocol::PusherCommand::Stop;
    clearMotion();
    stop_fence_pending_ = stop_fence_pending_ || was_active;
  }

  bool expirePusher(TimePoint now)
  {
    if (pusherActive() && now >= pusher_deadline_) {
      pusher_command_ = protocol::PusherCommand::Stop;
      clearMotion();
      stop_fence_pending_ = true;
      return true;
    }
    return false;
  }

  protocol::Frame nextFrame(TimePoint now)
  {
    if (stop_fence_pending_) {
      stop_fence_pending_ = false;
      return protocol::encodeFrame(0.0F, 0.0F, protocol::kStopVY);
    }
    if (pusherActive()) {
      return protocol::encodeFrame(0.0F, 0.0F, protocol::pusherBytes(pusher_command_));
    }
    const bool motion_fresh = motion_valid_ &&
      (now - motion_received_at_) <= motion_timeout_;
    const float vx = motion_fresh ? latest_vx_ : 0.0F;
    const float wz = motion_fresh ? latest_wz_ : 0.0F;
    return protocol::encodeFrame(vx, wz, protocol::kStopVY);
  }

  void resetAfterDisconnect()
  {
    pusher_command_ = protocol::PusherCommand::Stop;
    stop_fence_pending_ = false;
    clearMotion();
  }

  bool pusherActive() const
  {
    return pusher_command_ != protocol::PusherCommand::Stop;
  }

private:
  void clearMotion()
  {
    motion_valid_ = false;
    latest_vx_ = 0.0F;
    latest_wz_ = 0.0F;
  }

  std::chrono::milliseconds motion_timeout_;
  std::chrono::milliseconds pusher_timeout_;
  TimePoint motion_received_at_{};
  TimePoint pusher_deadline_{};
  float latest_vx_{0.0F};
  float latest_wz_{0.0F};
  bool motion_valid_{false};
  bool stop_fence_pending_{false};
  protocol::PusherCommand pusher_command_{protocol::PusherCommand::Stop};
};
}  // namespace chassis_serial_bridge
