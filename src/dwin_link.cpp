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

