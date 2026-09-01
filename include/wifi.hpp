#pragma once

#include <ESP8266WiFi.h>
#include "user_config.hpp"

namespace zt
{

void wifi_init()
{
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(20.5);
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.hostname("ZT Tracker");

	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED) delay(100);
}

} //namespace zt
