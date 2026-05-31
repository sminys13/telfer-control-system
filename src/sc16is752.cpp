#include "sc16is752.h"

Sc16Is752::Sc16Is752(uint8_t csPin, uint32_t xtalHz)
  : _csPin(csPin), _xtalHz(xtalHz) {}

void Sc16Is752::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  pinMode(53, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
}

bool Sc16Is752::selfTest(Channel ch) {
  writeReg(ch, sc16reg::SPR, 0x55);
  const uint8_t a = readReg(ch, sc16reg::SPR);
  writeReg(ch, sc16reg::SPR, 0xAA);
  const uint8_t b = readReg(ch, sc16reg::SPR);
  return (a == 0x55) && (b == 0xAA);
}

void Sc16Is752::initUart(Channel ch, uint16_t divisor) {
  writeReg(ch, sc16reg::IER, 0x00);
  writeReg(ch, sc16reg::FCR_IIR, 0x07);

  writeReg(ch, sc16reg::LCR, 0x80); // DLAB = 1
  writeReg(ch, sc16reg::DLL, (uint8_t)(divisor & 0xFF));
  writeReg(ch, sc16reg::DLH, (uint8_t)(divisor >> 8));

  writeReg(ch, sc16reg::LCR, 0x03); // 8N1
  writeReg(ch, sc16reg::EFCR, 0x00);
  flushRx(ch);
}

void Sc16Is752::writeReg(Channel ch, uint8_t reg, uint8_t value) {
  digitalWrite(_csPin, LOW);
  SPI.transfer(makeAddr(reg, ch, false));
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH);
}

uint8_t Sc16Is752::readReg(Channel ch, uint8_t reg) {
  digitalWrite(_csPin, LOW);
  SPI.transfer(makeAddr(reg, ch, true));
  const uint8_t value = SPI.transfer(0xFF);
  digitalWrite(_csPin, HIGH);
  return value;
}

bool Sc16Is752::txReady(Channel ch) {
  return readReg(ch, sc16reg::TXLVL) > 0;
}

uint8_t Sc16Is752::rxCount(Channel ch) {
  return readReg(ch, sc16reg::RXLVL);
}

void Sc16Is752::flushRx(Channel ch) {
  while (rxCount(ch) > 0) {
    (void)readReg(ch, sc16reg::RHR_THR);
  }
}

void Sc16Is752::writeByte(Channel ch, uint8_t value) {
  while (!txReady(ch)) {
    delayMicroseconds(20);
  }
  writeReg(ch, sc16reg::RHR_THR, value);
}

void Sc16Is752::writeBuffer(Channel ch, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    writeByte(ch, data[i]);
  }
}

int Sc16Is752::readByte(Channel ch) {
  if (rxCount(ch) == 0) return -1;
  return readReg(ch, sc16reg::RHR_THR);
}

uint8_t Sc16Is752::makeAddr(uint8_t reg, Channel ch, bool isRead) const {
  uint8_t a = 0;
  if (isRead) a |= 0x80;
  a |= (reg & 0x0F) << 3;
  a |= ((uint8_t)ch & 0x03) << 1;
  return a;
}

