#include "laser_sensor.h"
#include "config_v6_bringup.h"

static const uint8_t CMD_LASER_ON[] = {0x80, 0x06, 0x05, 0x01, 0x74};
static const uint8_t CMD_SINGLE[]   = {0x80, 0x06, 0x02, 0x78};

LaserSensor::LaserSensor(Sc16Is752& bus, Sc16Is752::Channel ch)
  : _bus(bus), _ch(ch) {}

bool LaserSensor::begin() {
  _bus.initUart(_ch, SC16_DIV_9600);
  return laserInit();
}

bool LaserSensor::laserInit() {
  _bus.flushRx(_ch);
  _bus.writeBuffer(_ch, CMD_LASER_ON, sizeof(CMD_LASER_ON));
  delay(150);

  uint8_t rx[32];
  size_t n = 0;
  uint32_t t0 = millis();

  while (millis() - t0 < 200 && n < sizeof(rx)) {
    const int b = _bus.readByte(_ch);
    if (b >= 0) {
      rx[n++] = (uint8_t)b;
      t0 = millis();
    }
  }

  for (size_t i = 0; i + 5 <= n; ++i) {
    if (rx[i + 0] == 0x80 &&
        rx[i + 1] == 0x06 &&
        rx[i + 2] == 0x85 &&
        rx[i + 3] == 0x01 &&
        rx[i + 4] == 0xF4) {
      return true;
    }
  }
  return false;
}

bool LaserSensor::readFilteredMm(int32_t& outMm) {
  int32_t values[3] = {0, 0, 0};
  int okCount = 0;

  for (int i = 0; i < 3; ++i) {
    int32_t mm = 0;
    if (readSingleMmRobust(mm)) {
      values[okCount++] = mm;
    }
  }

  if (okCount == 0) return false;
  if (okCount == 1) outMm = values[0];
  else if (okCount == 2) outMm = (values[0] + values[1]) / 2;
  else outMm = median3(values[0], values[1], values[2]);

  _last.mm = outMm;
  _last.valid = true;
  _last.lastUpdateMs = millis();
  return true;
}

bool LaserSensor::readSingleMmRobust(int32_t& outMm) {
  for (uint8_t attempt = 0; attempt < LASER_RETRY_COUNT; ++attempt) {
    _bus.flushRx(_ch);
    _bus.writeBuffer(_ch, CMD_SINGLE, sizeof(CMD_SINGLE));
    delay(180);

    uint8_t rx[64];
    size_t n = 0;
    uint32_t t0 = millis();

    while (millis() - t0 < 120 && n < sizeof(rx)) {
      const int b = _bus.readByte(_ch);
      if (b >= 0) {
        rx[n++] = (uint8_t)b;
        t0 = millis();
      }
    }

    if (findValidMeasureFrame(rx, n, outMm)) {
      return true;
    }

    delay(20);
  }

  return false;
}

bool LaserSensor::findValidMeasureFrame(const uint8_t* buf, size_t len, int32_t& outMm) const {
  for (size_t i = 0; i + 11 <= len; ++i) {
    if (buf[i + 0] != 0x80) continue;
    if (buf[i + 1] != 0x06) continue;
    if (buf[i + 2] != 0x82) continue;

    bool asciiOk = true;
    for (size_t k = 3; k <= 9; ++k) {
      if (!isAsciiDigitOrDot(buf[i + k])) {
        asciiOk = false;
        break;
      }
    }
    if (!asciiOk) continue;

    const uint8_t chk = calcChecksum(&buf[i], 10);
    if (chk != buf[i + 10]) continue;

    const int metersInt = (buf[i + 3] - '0') * 100 +
                          (buf[i + 4] - '0') * 10 +
                          (buf[i + 5] - '0');

    const int fracMm = (buf[i + 7] - '0') * 100 +
                       (buf[i + 8] - '0') * 10 +
                       (buf[i + 9] - '0');

    outMm = metersInt * 1000 + fracMm;
    if (outMm < 0 || outMm > LASER_MAX_MM) return false;
    return true;
  }
  return false;
}

uint8_t LaserSensor::calcChecksum(const uint8_t* data, size_t lenWithoutChecksum) const {
  uint8_t sum = 0;
  for (size_t i = 0; i < lenWithoutChecksum; ++i) {
    sum = (uint8_t)(sum + data[i]);
  }
  return (uint8_t)(~sum + 1);
}

bool LaserSensor::isAsciiDigitOrDot(uint8_t c) const {
  return (c >= '0' && c <= '9') || (c == '.');
}

int32_t LaserSensor::median3(int32_t a, int32_t b, int32_t c) const {
  if (a > b) { const int32_t t = a; a = b; b = t; }
  if (b > c) { const int32_t t = b; b = c; c = t; }
  if (a > b) { const int32_t t = a; a = b; b = t; }
  return b;
}

