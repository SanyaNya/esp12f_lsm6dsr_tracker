#pragma once

#include <Arduino.h>

#if !DISABLE_ERROR_HANDLING
#define ERROR_CHECK(expr, fmt, ...)                                            \
  do                                                                           \
  {                                                                            \
    if(!(expr)) [[unlikely]]                                                   \
    {                                                                          \
      Serial.printf(                                                           \
        "FATAL_ERROR\nin function: %s\nin location: %s:%d:\n" fmt "\n",        \
        __PRETTY_FUNCTION__, __FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__);    \
      Serial.flush();                                                          \
      ESP.restart();                                                           \
    }                                                                          \
  } while(0)
#else
#define ERROR_CHECK(expr, fmt, ...) do { if(!(expr)) __builtin_unreachable(); } while(0)
#endif
