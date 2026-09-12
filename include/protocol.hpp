#pragma once

extern "C"
{
#include "lwip/udp.h"
}

#include <cstdint>
#include <cstring>
#include <string_view>
#include "user_config.hpp"

namespace zt
{

#pragma pack(push, 1)
struct HandshakePacketIn
{
  std::uint8_t packet_type;
  char magic[sizeof(HANDSHAKE_MAGIC) - 1];

  constexpr bool is_valid() const noexcept
  {
    return
      packet_type == PACKET_TYPE_HANDSHAKE &&
      std::string_view(std::begin(magic), std::end(magic)) == HANDSHAKE_MAGIC;
  }
};
static_assert(sizeof(HandshakePacketIn) == 1+sizeof(HANDSHAKE_MAGIC)-1);

struct HandshakePacketOut
{
  std::uint8_t padding;
  std::uint8_t packet_type;
  std::uint32_t packet_number;
  char magic[sizeof(HANDSHAKE_MAGIC) - 1u];
  std::uint8_t mac[6];

  static HandshakePacketOut make() noexcept
  {
    HandshakePacketOut p;
    p.padding = 0;
    p.packet_type = PACKET_TYPE_HANDSHAKE;
    p.packet_number = 0;
    std::memcpy(p.magic, HANDSHAKE_MAGIC, sizeof(p.magic));
    wifi_get_macaddr(STATION_IF, p.mac);

    return p;
  }

  constexpr bool is_valid() const noexcept
  {
    return
      padding == 0 &&
      packet_type == PACKET_TYPE_HANDSHAKE &&
      packet_number == 0 &&
      std::string_view(std::begin(magic), std::end(magic)) == HANDSHAKE_MAGIC;
  }
};
static_assert(sizeof(HandshakePacketOut) == 1+1+4+sizeof(HANDSHAKE_MAGIC)-1+6);

struct RotationPacket
{
#if !IMUCAL_RECORDING
  std::uint8_t  packet_type_padding;
  std::uint8_t  packet_type;
  std::uint32_t packet_number;
  float x;
  float y;
  float z;
  float w;
#else
  std::uint32_t packet_number;
  std::uint32_t timestamp_cycles;
  int16_t temp;
  int16_t gyr[3];
  int16_t acc[3];
#endif
};
static_assert(sizeof(RotationPacket) == 22);

#pragma pack(pop)

} //namespace zt
