#pragma once

#include "protocol.hpp"
#include "error.hpp"

namespace zt
{

void discovery_recv_callback(
  void* arg, udp_pcb* pcb, pbuf* p, const ip_addr_t* addr, u16_t port)
{
  if(port != SERVER_PORT || p->tot_len != sizeof(HandshakePacketIn))
  {
    pbuf_free(p);
    return;
  }

  HandshakePacketIn packet;
  pbuf_copy_partial(p, &packet, sizeof(packet), 0);

  pbuf_free(p);

  if(packet.is_valid())
    *reinterpret_cast<ip_addr_t*>(arg) = *addr;
}
  
static inline HandshakePacketOut g_handshake_out = HandshakePacketOut::make();

ip_addr_t server_discovery()
{
  err_t ret;

  udp_pcb* pcb = udp_new();
  ERROR_CHECK(pcb != nullptr, "Failed to create udp socket");

  ret = udp_bind(pcb, IP_ADDR_ANY, SERVER_PORT);
  ERROR_CHECK(ret == ERR_OK, "Failed to bind port %u, err_t: %d", SERVER_PORT, ret);

  ip_addr_t server_addr{IPADDR_BROADCAST};
  udp_recv(pcb, discovery_recv_callback, &server_addr);

  pbuf* p = pbuf_alloc_reference(&g_handshake_out, sizeof(g_handshake_out), PBUF_REF);

  ERROR_CHECK(p != nullptr, "Failed to allocate pbuf");

  while(server_addr.addr == IPADDR_BROADCAST)
  {
    ret = udp_sendto(pcb, p, &server_addr, SERVER_PORT);

    ERROR_CHECK(ret == ERR_OK, "Failed to send handshake, err_t: %d", ret);

    delay(HANDSHAKE_RETRY_MS);
  }

  pbuf_free(p);
  udp_remove(pcb);

  return server_addr;
}

} //namespace zt
