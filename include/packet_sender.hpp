#pragma once

extern "C"
{
#include "lwip/udp.h"
}

#include <cstdint>
#include <cstring>
#include "error.hpp"
#include "user_config.hpp"

namespace zt
{

template<typename T>
class PacketSender
{
  udp_pcb* m_udp;
  pbuf* m_pbuf;
  static inline std::uint8_t m_buf[sizeof(T)];

public:
  void begin(ip_addr_t ip)
  {
    err_t ret;

    m_udp = udp_new();
    ERROR_CHECK(m_udp != nullptr, "Failed to create udp socket");

    ret = udp_bind(m_udp, IP_ADDR_ANY, SERVER_PORT);
    ERROR_CHECK(ret == ERR_OK, "Failed to bind port %u, err_t: %d", SERVER_PORT, ret);

    ret = udp_connect(m_udp, &ip, SERVER_PORT);
    ERROR_CHECK(ret == ERR_OK, "Failed to connect, err_t: %d", ret);

    m_pbuf = pbuf_alloc_reference(m_buf, sizeof(m_buf), PBUF_REF);
    ERROR_CHECK(m_pbuf != nullptr, "Failed to alloc pbuf for PacketSender");
  }

  void send(const T& t)
  {
    std::memcpy(m_buf, &t, sizeof(T));
    udp_send(m_udp, m_pbuf);
  }
};

} //namespace zt
