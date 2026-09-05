#pragma once

#include <Arduino.h>
#include "user_config.hpp"

namespace zt
{

class Logger
{
public:
  void begin()
  {
  #if !DISABLE_LOGGING
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);
    Serial.println("\n=== ESP-12F LSM6DSR Tracker ===");
  #endif
  }

  void println(const char* str)
  {
  #if !DISABLE_LOGGING
    Serial.println(str);
  #endif
  }

  template<typename... Args>
  void printf(const char* fmt, Args&&... args)
  {
  #if !DISABLE_LOGGING
    Serial.printf(fmt, args...);
  #endif
  }
};

};
