#pragma once

#include <cstdint>
#include <Arduino.h>

namespace zt
{

class CycleCounter64
{
  std::uint32_t m_last = 0;
  std::uint64_t m_acc = 0;

public:
  std::uint64_t get()
  {
    const std::uint32_t c = ESP.getCycleCount();
    const std::uint32_t delta = c - m_last;

    m_acc += delta;
    m_last = c;

    return m_acc;
  }
};

class PreciseTimer
{
  CycleCounter64 m_cycle_counter;

public:
  double now()
  {
    return double(m_cycle_counter.get()) / F_CPU;
  }
};

} //namespace zt
