#ifndef _RM_SERIAL_DRIVER_PACKET_HPP
#define _RM_SERIAL_DRIVER_PACKET_HPP

#include <cstdint>
#include <vector>

namespace rm::serial
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  // Reserve receive-side extension point for future use.
  uint8_t end_frame;                                 
} __attribute__((packed));

struct NavPacket
{
  float linear_x = 0.0;
  float linear_y = 0.0;
  float angular_z = 0.0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  NavPacket nav;
  uint8_t end_frame = 0x4A;
} __attribute__((packed));

ReceivePacket FromVector(const std::vector<uint8_t>& data);
std::vector<uint8_t> ToVector(const SendPacket& data);
}

#endif
