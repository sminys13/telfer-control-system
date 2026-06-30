#pragma once

#include <Arduino.h>
#include <stdint.h>

class DwinLink {
public:
  explicit DwinLink(HardwareSerial& serial);

  void begin(uint32_t baud);

  void writeU16(uint16_t vp, uint16_t value);
  void writeI16(uint16_t vp, int16_t value);

  // Reads DWIN touch/key command frames:
  // 5A A5 06 83 VP_H VP_L 01 DATA_H DATA_L
  // Returns true when a command for expectedVp is received.
  bool pollCommand(uint16_t expectedVp, uint16_t& outCmd);

  void clearCommand(uint16_t cmdVp);

private:
  bool readFrame(uint8_t* payload, uint8_t& lenOut);

private:
  HardwareSerial& _serial;
};
