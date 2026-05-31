#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

class Sc16Is752 {
public:
  enum class Channel : uint8_t {
    A = 0,
    B = 1
  };

  explicit Sc16Is752(uint8_t csPin, uint32_t xtalHz);

  void begin();
  bool selfTest(Channel ch);
  void initUart(Channel ch, uint16_t divisor);

  void writeReg(Channel ch, uint8_t reg, uint8_t value);
  uint8_t readReg(Channel ch, uint8_t reg);

  bool txReady(Channel ch);
  uint8_t rxCount(Channel ch);
  void flushRx(Channel ch);

  void writeByte(Channel ch, uint8_t value);
  void writeBuffer(Channel ch, const uint8_t* data, size_t len);
  int readByte(Channel ch);

private:
  uint8_t makeAddr(uint8_t reg, Channel ch, bool isRead) const;

private:
  uint8_t _csPin;
  uint32_t _xtalHz;
};

namespace sc16reg {
  static constexpr uint8_t RHR_THR = 0x00;
  static constexpr uint8_t IER     = 0x01;
  static constexpr uint8_t FCR_IIR = 0x02;
  static constexpr uint8_t LCR     = 0x03;
  static constexpr uint8_t LSR     = 0x05;
  static constexpr uint8_t SPR     = 0x07;
  static constexpr uint8_t TXLVL   = 0x08;
  static constexpr uint8_t RXLVL   = 0x09;
  static constexpr uint8_t EFCR    = 0x0F;
  static constexpr uint8_t DLL     = 0x00;
  static constexpr uint8_t DLH     = 0x01;
}

