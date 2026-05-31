#pragma once

#include <Arduino.h>
#include <stdint.h>

class DwinLink {
public:
  explicit DwinLink(HardwareSerial& serial);

  void begin(uint32_t baud);
  void writeU16(uint16_t vp, uint16_t value);

private:
  HardwareSerial& _serial;
};

