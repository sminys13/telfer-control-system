#include "fast_laser_sensor.h"
#include "config_v6_bringup.h"
#include <string.h>

static const uint8_t CMD_LASER_ON[]      = {0x80, 0x06, 0x05, 0x01, 0x74};
static const uint8_t CMD_RANGE_10M[]     = {0x04, 0x09, 0x0A, 0xEF};
static const uint8_t CMD_RESOLUTION_1MM[]= {0x04, 0x0C, 0x01, 0xF5};
static const uint8_t CMD_FREQ_10HZ[]     = {0x04, 0x0A, 0x0A, 0xEE};
static const uint8_t CMD_FREQ_20HZ[]     = {0x04, 0x0A, 0x14, 0xE4};
static const uint8_t CMD_CONTINUOUS[]    = {0x80, 0x06, 0x03, 0x77};

FastLaserSensor::FastLaserSensor(Sc16Is752& bus, Sc16Is752::Channel ch)
  : _bus(bus), _ch(ch) {}

void FastLaserSensor::begin(uint32_t now, uint16_t samplePeriodMs, uint16_t phaseMs) {
  _bus.initUart(_ch, SC16_DIV_9600);
  _samplePeriodMs = samplePeriodMs;
  _state = State::Start;
  _nextActionMs = now + phaseMs;
  _lastContinuousStartMs = 0;
  _rxLen = 0;
  _reading = FastLaserReading{};
  flushRx();
}

bool FastLaserSensor::valid(uint32_t now, uint16_t staleTimeoutMs) const {
  return _reading.valid && ((uint32_t)(now - _reading.lastUpdateMs) <= staleTimeoutMs);
}

bool FastLaserSensor::consumeUpdated() {
  const bool v = _reading.updated;
  _reading.updated = false;
  return v;
}

void FastLaserSensor::service(uint32_t now) {
  // Always drain RX first. In true continuous mode the sensor pushes frames itself.
  readIncoming(now);

  switch (_state) {
    case State::Start:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendLaserOn(now);
        _state = State::WaitAfterLaserOn;
        _nextActionMs = now + FAST_CONTINUOUS_INIT_GAP_MS;
      }
      break;

    case State::WaitAfterLaserOn:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendRange10m(now);
        _state = State::WaitAfterRange;
        _nextActionMs = now + FAST_CONTINUOUS_INIT_GAP_MS;
      }
      break;

    case State::WaitAfterRange:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendResolution1mm(now);
        _state = State::WaitAfterResolution;
        _nextActionMs = now + FAST_CONTINUOUS_INIT_GAP_MS;
      }
      break;

    case State::WaitAfterResolution:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendFrequency(now);
        _state = State::WaitAfterFrequency;
        _nextActionMs = now + FAST_CONTINUOUS_INIT_GAP_MS;
      }
      break;

    case State::WaitAfterFrequency:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendContinuous(now);
        _state = State::WaitAfterContinuous;
        _nextActionMs = now + FAST_CONTINUOUS_INIT_GAP_MS;
      }
      break;

    case State::WaitAfterContinuous:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        _state = State::Running;
        _lastContinuousStartMs = now;
      }
      break;

    case State::Running:
      // True passive continuous mode.
      // Do NOT restart the laser automatically while the hoist is moving.
      // Automatic restarts caused visible laser blinking and made measurements worse
      // after a temporary loss of target/valid frames. Stale handling is done in
      // main_v6_fast.cpp via ERROR mask, while the last valid coordinate remains
      // on DWIN. Manual/power-cycle restart can be added later as a separate command.
      break;
  }
}

void FastLaserSensor::sendLaserOn(uint32_t now) {
  (void)now;
  flushRx();
  _bus.writeBuffer(_ch, CMD_LASER_ON, sizeof(CMD_LASER_ON));
}

void FastLaserSensor::sendRange10m(uint32_t now) {
  (void)now;
  flushRx();
  if (FAST_LASER_SET_RANGE_10M) {
    _bus.writeBuffer(_ch, CMD_RANGE_10M, sizeof(CMD_RANGE_10M));
  }
}

void FastLaserSensor::sendResolution1mm(uint32_t now) {
  (void)now;
  flushRx();
  if (FAST_LASER_SET_RESOLUTION_1MM) {
    _bus.writeBuffer(_ch, CMD_RESOLUTION_1MM, sizeof(CMD_RESOLUTION_1MM));
  }
}

void FastLaserSensor::sendFrequency(uint32_t now) {
  (void)now;
  flushRx();
  if (FAST_LASER_SET_FREQ_20HZ) {
    _bus.writeBuffer(_ch, CMD_FREQ_20HZ, sizeof(CMD_FREQ_20HZ));
  } else if (FAST_LASER_SET_FREQ_10HZ) {
    _bus.writeBuffer(_ch, CMD_FREQ_10HZ, sizeof(CMD_FREQ_10HZ));
  }
}

void FastLaserSensor::sendContinuous(uint32_t now) {
  (void)now;
  flushRx();
  _bus.writeBuffer(_ch, CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
}

void FastLaserSensor::restartContinuous(uint32_t now) {
  flushRx();
  _bus.writeBuffer(_ch, CMD_LASER_ON, sizeof(CMD_LASER_ON));
  _state = State::WaitAfterLaserOn;
  _nextActionMs = now + FAST_CONTINUOUS_RESTART_GAP_MS;
  _lastContinuousStartMs = now;
}

void FastLaserSensor::flushRx() {
  _bus.flushRx(_ch);
  _rxLen = 0;
}

void FastLaserSensor::readIncoming(uint32_t now) {
  while (true) {
    const int b = _bus.readByte(_ch);
    if (b < 0) break;
    appendRx((uint8_t)b);
  }

  bool progress = true;
  while (progress) {
    progress = false;
    int32_t mm = 0;
    const int8_t rc = tryConsumeFrame(mm);
    if (rc == 1) {
      if (mm > 0 && mm <= LASER_MAX_MM) {
        _reading.mm = mm;
        _reading.valid = true;
        _reading.updated = true;
        _reading.lastUpdateMs = now;
      }
      progress = true;
    } else if (rc == 0) {
      progress = true;
    }
  }
}

void FastLaserSensor::appendRx(uint8_t b) {
  if (_rxLen < sizeof(_rx)) {
    _rx[_rxLen++] = b;
    return;
  }
  memmove(_rx, _rx + 1, sizeof(_rx) - 1);
  _rx[sizeof(_rx) - 1] = b;
}

int8_t FastLaserSensor::tryConsumeFrame(int32_t& outMm) {
  // ACK frames:
  // LASER ON ack:      80 06 85 01 F4
  // CONTINUOUS ack may be mixed with measurement frames on some modules.
  for (size_t i = 0; i + 5 <= _rxLen; ++i) {
    if (_rx[i + 0] == 0x80 && _rx[i + 1] == 0x06 && _rx[i + 2] == 0x85 &&
        _rx[i + 3] == 0x01 && _rx[i + 4] == 0xF4) {
      const size_t consumed = i + 5;
      memmove(_rx, _rx + consumed, _rxLen - consumed);
      _rxLen -= consumed;
      return 0;
    }
  }

  // Measurement: 80 06 82/83 d d d . d d d chk
  for (size_t i = 0; i + 11 <= _rxLen; ++i) {
    if (_rx[i + 0] != 0x80) continue;
    if (_rx[i + 1] != 0x06) continue;
    if (!(_rx[i + 2] == 0x82 || _rx[i + 2] == 0x83)) continue;

    bool asciiOk = true;
    for (size_t k = 3; k <= 9; ++k) {
      if (!isAsciiDigitOrDot(_rx[i + k])) {
        asciiOk = false;
        break;
      }
    }
    if (!asciiOk) continue;

    const uint8_t chk = calcChecksum(&_rx[i], 10);
    if (chk != _rx[i + 10]) continue;

    const int metersInt = (_rx[i + 3] - '0') * 100 +
                          (_rx[i + 4] - '0') * 10 +
                          (_rx[i + 5] - '0');

    const int fracMm = (_rx[i + 7] - '0') * 100 +
                       (_rx[i + 8] - '0') * 10 +
                       (_rx[i + 9] - '0');

    outMm = metersInt * 1000L + fracMm;

    const size_t consumed = i + 11;
    memmove(_rx, _rx + consumed, _rxLen - consumed);
    _rxLen -= consumed;
    return 1;
  }

  // If buffer starts with garbage before the next possible 0x80, trim it.
  if (_rxLen > 0 && _rx[0] != 0x80) {
    size_t first80 = 0;
    while (first80 < _rxLen && _rx[first80] != 0x80) ++first80;
    if (first80 > 0 && first80 < _rxLen) {
      memmove(_rx, _rx + first80, _rxLen - first80);
      _rxLen -= first80;
      return 0;
    }
  }

  if (_rxLen > 96) {
    memmove(_rx, _rx + (_rxLen - 24), 24);
    _rxLen = 24;
    return 0;
  }

  return -1;
}

uint8_t FastLaserSensor::calcChecksum(const uint8_t* data, size_t lenWithoutChecksum) const {
  uint8_t sum = 0;
  for (size_t i = 0; i < lenWithoutChecksum; ++i) {
    sum = (uint8_t)(sum + data[i]);
  }
  return (uint8_t)(~sum + 1);
}

bool FastLaserSensor::isAsciiDigitOrDot(uint8_t c) const {
  return (c >= '0' && c <= '9') || (c == '.');
}
