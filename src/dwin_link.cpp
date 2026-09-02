#include "dwin_link.h"

DwinLink::DwinLink(HardwareSerial& serial)
  : _serial(serial) {}

void DwinLink::begin(uint32_t baud) {
  _serial.begin(baud);
}

void DwinLink::writeU16(uint16_t vp, uint16_t value) {
  uint8_t frame[8];
  frame[0] = 0x5A;
  frame[1] = 0xA5;
  frame[2] = 0x05;
  frame[3] = 0x82;
  frame[4] = (uint8_t)(vp >> 8);
  frame[5] = (uint8_t)(vp & 0xFF);
  frame[6] = (uint8_t)(value >> 8);
  frame[7] = (uint8_t)(value & 0xFF);
  _serial.write(frame, sizeof(frame));
}

void DwinLink::writeI16(uint16_t vp, int16_t value) {
  writeU16(vp, (uint16_t)value);
}

void DwinLink::clearCommand(uint16_t cmdVp) {
  writeU16(cmdVp, 0);
}

bool DwinLink::readFrame(uint8_t* payload, uint8_t& lenOut) {
  static uint8_t state = 0;
  static uint8_t len = 0;
  static uint8_t pos = 0;

  while (_serial.available()) {
    const uint8_t b = (uint8_t)_serial.read();

    switch (state) {
      case 0:
        if (b == 0x5A) state = 1;
        break;

      case 1:
        if (b == 0xA5) state = 2;
        else state = 0;
        break;

      case 2:
        len = b;
        pos = 0;
        if (len == 0 || len > 32) {
          state = 0;
        } else {
          state = 3;
        }
        break;

      case 3:
        payload[pos++] = b;
        if (pos >= len) {
          lenOut = len;
          state = 0;
          return true;
        }
        break;
    }
  }

  return false;
}

bool DwinLink::pollWriteU16(uint16_t& outVp, uint16_t& outValue) {
  uint8_t payload[32];
  uint8_t len = 0;

  while (readFrame(payload, len)) {
    // Expected auto-upload/read-response payload:
    // payload[0] = 0x83
    // payload[1..2] = VP
    // payload[3] = word count
    // payload[4..5] = first value
    if (len < 6) continue;
    if (payload[0] != 0x83) continue;
    if (payload[3] < 1) continue;

    outVp = ((uint16_t)payload[1] << 8) | payload[2];
    outValue = ((uint16_t)payload[4] << 8) | payload[5];
    return true;
  }

  return false;
}

bool DwinLink::pollCommand(uint16_t expectedVp, uint16_t& outCmd) {
  uint16_t vp = 0;
  uint16_t value = 0;
  while (pollWriteU16(vp, value)) {
    if (vp == expectedVp) {
      outCmd = value;
      return true;
    }
  }
  return false;
}
