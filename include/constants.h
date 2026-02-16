// Networking and application constants
#pragma once

#include <Arduino.h>

namespace constants {
  
  static const uint16_t HTTP_PORT = 80;
  static const char OTA_PATH[] = "/update";
  static const char UPLOAD_NAME[] = "FlasherX.ino.hex";
  static const unsigned long LINK_TIMEOUT_MS = 5000UL;

  static const IPAddress APP_IP = IPAddress(192,168,4,70);
  static const IPAddress APP_NETMASK = IPAddress(255,255,255,0);
  static const IPAddress APP_GW = IPAddress(192,168,4,1);
  static const IPAddress APP_DNS = IPAddress(192,168,4,1);
}
