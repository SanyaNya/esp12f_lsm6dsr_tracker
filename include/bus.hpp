#pragma once

#include <Wire.h>
#include "types.hpp"
#include "error.hpp"

namespace zt
{

class Bus
{
  std::uint8_t m_addr;

  bool check_bus()
  {
    Wire.beginTransmission(m_addr);
    return Wire.endTransmission() == 0;
  }

public:
  Bus(std::uint8_t addr, Pin sda, Pin scl, std::uint32_t freq) :
    m_addr(addr)
  {
    Wire.begin(sda, scl);
    Wire.setClock(freq);

    ERROR_CHECK(check_bus(),
      "[Bus] Failed to initialize with\n"
      "addr: %04X\n"
      "sda: %u\n"
      "scl: %u\n"
      "frequency: %uHZ",
      addr, sda, scl, freq);
  }

  template<typename T>
  void write(std::uint8_t reg, const T& t)
  {
    std::uint8_t ret;

    Wire.beginTransmission(m_addr);
    Wire.write(reg);
    if constexpr(sizeof(T) > 1)
      Wire.write(reinterpret_cast<const char*>(&t), sizeof(T));
    else
      Wire.write(reinterpret_cast<std::uint8_t*>(&t)[0]);
    ret = Wire.endTransmission();

    ERROR_CHECK(ret == 0, "[Bus] Failed to write with reg: %02X and size: %u", reg, sizeof(T));
  }

  template<typename T>
  T read(std::uint8_t reg)
  {
    std::uint8_t ret;
    T t;

    Wire.beginTransmission(m_addr);
    Wire.write(reg);
    ret = Wire.endTransmission(false);

    ERROR_CHECK(ret == 0, "[Bus] Failed to write reg: %02X", reg);

    ret = Wire.requestFrom(m_addr, sizeof(T));

    ERROR_CHECK(ret == sizeof(T),
      "[Bus] Failed to read with reg: %02X and size: %u, bytes_read: %u", reg, sizeof(T), ret);

    for(std::size_t i = 0; i != sizeof(T); ++i)
      reinterpret_cast<std::uint8_t*>(&t)[i] = static_cast<std::uint8_t>(Wire.read());

    return t;
  }
};

} //namespace zt
