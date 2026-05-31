#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "sc16is752.h"

struct LaserReading {
  int32_t mm = 0;
  bool valid = false;
  uint32_t lastUpdateMs = 0;
};

class LaserSensor {
public:
  LaserSensor(Sc16Is752& bus, Sc16Is752::Channel ch);

  bool begin();
  bool readFilteredMm(int32_t& outMm);
  const LaserReading& last() const { return _last; }

private:
  bool laserInit();
  bool readSingleMmRobust(int32_t& outMm);
  bool findValidMeasureFrame(const uint8_t* buf, size_t len, int32_t& outMm) const;
  uint8_t calcChecksum(const uint8_t* data, size_t lenWithoutChecksum) const;
  bool isAsciiDigitOrDot(uint8_t c) const;
  int32_t median3(int32_t a, int32_t b, int32_t c) const;

private:
  Sc16Is752& _bus;
  Sc16Is752::Channel _ch;
  LaserReading _last{};
};

