#include <rm_serial_driver/packet.hpp>

#include <cstring>

namespace rm::serial
{
ReceivePacket FromVector(const std::vector<uint8_t>& data)
{
  ReceivePacket packet;
  std::memcpy(&packet, data.data(), sizeof(packet));
  return packet;
}

std::vector<uint8_t> ToVector(const SendPacket& data)
{
  std::vector<uint8_t> packet(sizeof(data));
  std::memcpy(packet.data(), &data, sizeof(data));
  return packet;
}
}