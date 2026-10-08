#include "fast_laser_sensor.h"
#include "config_v6_bringup.h"
#include <string.h>

namespace {
static const uint8_t CMD_SHUTDOWN[]       = {0x80, 0x04, 0x02, 0x7A};
static const uint8_t CMD_LASER_ON[]       = {0x80, 0x06, 0x05, 0x01, 0x74};
static const uint8_t CMD_RANGE_10M[]      = {0xFA, 0x04, 0x09, 0x0A, 0xEF};
static const uint8_t CMD_RESOLUTION_1MM[]  = {0xFA, 0x04, 0x0C, 0x01, 0xF5};
static const uint8_t CMD_RESOLUTION_01MM[] = {0xFA, 0x04, 0x0C, 0x02, 0xF4};
static const uint8_t CMD_FREQ_5HZ[]       = {0xFA, 0x04, 0x0A, 0x05, 0xF3};
static const uint8_t CMD_FREQ_10HZ[]      = {0xFA, 0x04, 0x0A, 0x0A, 0xEE};
static const uint8_t CMD_FREQ_20HZ[]      = {0xFA, 0x04, 0x0A, 0x14, 0xE4};
static const uint8_t CMD_CONTINUOUS[]     = {0x80, 0x06, 0x03, 0x77};
static const uint8_t CMD_SINGLE[]         = {0x80, 0x06, 0x02, 0x78};

static const uint8_t ACK_SHUTDOWN[]       = {0x80, 0x04, 0x82, 0xFA};
static const uint8_t ACK_LASER_ON[]       = {0x80, 0x06, 0x85, 0x01, 0xF4};
static const uint8_t ACK_RANGE[]          = {0xFA, 0x04, 0x89, 0x79};
static const uint8_t ACK_FREQUENCY[]      = {0xFA, 0x04, 0x8A, 0x78};
static const uint8_t ACK_RESOLUTION[]     = {0xFA, 0x04, 0x8C, 0x76};

static inline void satInc16(uint16_t& value) {
  if (value != UINT16_MAX) ++value;
}

static inline void satAdd16(uint16_t& value, size_t amount) {
  if (amount >= UINT16_MAX || value > (uint16_t)(UINT16_MAX - amount)) {
    value = UINT16_MAX;
  } else {
    value = (uint16_t)(value + amount);
  }
}
}

FastLaserSensor::FastLaserSensor(Sc16Is752& bus, Sc16Is752::Channel ch)
  : _bus(bus), _ch(ch) {}

void FastLaserSensor::begin(uint32_t now, uint16_t samplePeriodMs,
                            uint16_t phaseMs, uint8_t frequencyHz,
                            FastLaserResolution resolution,
                            FastLaserAcquisitionMode mode) {
  _bus.initUart(_ch, SC16_DIV_9600);
  _samplePeriodMs = samplePeriodMs;
  _reading = FastLaserReading{};
  _diag = FastLaserDiagnostics{};
  _rxLen = 0;
  _rateWindowStartMs = now;
  _rateWindowFrameBase = 0;
  beginConfiguration(now, phaseMs, frequencyHz, resolution, mode);
}

bool FastLaserSensor::requestConfigure(uint32_t now, uint8_t frequencyHz,
                                       FastLaserResolution resolution,
                                       FastLaserAcquisitionMode mode) {
  if (frequencyHz != 5 && frequencyHz != 10 && frequencyHz != 20) return false;
  if (resolution != FastLaserResolution::Mm1 &&
      resolution != FastLaserResolution::TenthMm) return false;
  if (_diag.configuring) return false;
  beginConfiguration(now, 0, frequencyHz, resolution, mode);
  return true;
}

void FastLaserSensor::beginConfiguration(uint32_t now, uint16_t phaseMs,
                                         uint8_t frequencyHz,
                                         FastLaserResolution resolution,
                                         FastLaserAcquisitionMode mode) {
  if (frequencyHz != 5 && frequencyHz != 10 && frequencyHz != 20) {
    frequencyHz = FAST_LASER_DEFAULT_FREQ_HZ;
  }
  if (resolution != FastLaserResolution::Mm1 &&
      resolution != FastLaserResolution::TenthMm) {
    resolution = FastLaserResolution::Mm1;
  }

  flushRx();
  _resolution = resolution;
  _mode = mode;
  _singlePending = false;
  _singleDeadlineMs = 0;
  _diag.ackMask = 0;
  _diag.missingAckMask = 0;
  _diag.nackMask = 0;
  _diag.configuredFrequencyHz = frequencyHz;
  _diag.configuredResolutionCode = (uint8_t)resolution;
  _diag.acquisitionMode = (uint8_t)mode;
  _diag.configuring = true;
  _diag.streamSeen = false;
  _diag.singlePending = false;
  _diag.lastConfigStartMs = now;
  _diag.lastConfigCompleteMs = 0;
  _expectedAckBit = 0;
  _ackDeadlineMs = 0;
  _state = State::StartDelay;
  _nextActionMs = now + phaseMs;
}

bool FastLaserSensor::requestSingleMeasurement(uint32_t now) {
  if (_diag.configuring || _state != State::Running ||
      _mode != FastLaserAcquisitionMode::SingleShot || _singlePending) {
    return false;
  }
  _bus.writeBuffer(_ch, CMD_SINGLE, sizeof(CMD_SINGLE));
  if (_diag.singleRequests != UINT16_MAX) ++_diag.singleRequests;
  _singlePending = true;
  _diag.singlePending = true;
  _singleDeadlineMs = now + FAST_LASER_SINGLE_TIMEOUT_MS;
  return true;
}

void FastLaserSensor::resetDiagnostics(uint32_t now) {
  const uint8_t ackMask = _diag.ackMask;
  const uint8_t missingAckMask = _diag.missingAckMask;
  const uint8_t nackMask = _diag.nackMask;
  const uint8_t frequencyHz = _diag.configuredFrequencyHz;
  const uint8_t resolutionCode = _diag.configuredResolutionCode;
  const uint8_t acquisitionMode = _diag.acquisitionMode;
  const bool configuring = _diag.configuring;
  const bool streamSeen = _diag.streamSeen;
  const bool singlePending = _diag.singlePending;
  const uint32_t configStart = _diag.lastConfigStartMs;
  const uint32_t configComplete = _diag.lastConfigCompleteMs;
  const uint32_t lastByte = _diag.lastByteMs;
  const uint32_t lastFrame = _diag.lastFrameMs;

  _diag = FastLaserDiagnostics{};
  _diag.ackMask = ackMask;
  _diag.missingAckMask = missingAckMask;
  _diag.nackMask = nackMask;
  _diag.configuredFrequencyHz = frequencyHz;
  _diag.configuredResolutionCode = resolutionCode;
  _diag.acquisitionMode = acquisitionMode;
  _diag.configuring = configuring;
  _diag.streamSeen = streamSeen;
  _diag.singlePending = singlePending;
  _diag.lastConfigStartMs = configStart;
  _diag.lastConfigCompleteMs = configComplete;
  _diag.lastByteMs = lastByte;
  _diag.lastFrameMs = lastFrame;
  _rateWindowStartMs = now;
  _rateWindowFrameBase = 0;
}

bool FastLaserSensor::valid(uint32_t now, uint16_t staleTimeoutMs) const {
  return _reading.valid && ((uint32_t)(now - _reading.lastUpdateMs) <= staleTimeoutMs);
}

bool FastLaserSensor::consumeUpdated() {
  const bool updated = _reading.updated;
  _reading.updated = false;
  return updated;
}

void FastLaserSensor::service(uint32_t now) {
  sampleLineStatus();
  readIncoming(now);
  serviceConfiguration(now);
  if (_singlePending && (int32_t)(now - _singleDeadlineMs) >= 0) {
    _singlePending = false;
    _diag.singlePending = false;
    satInc16(_diag.singleTimeouts);
    noteErrorFrame(0xFFFE);
  }
  updateMaximumAge(now);
  updateFrameRate(now);
}

void FastLaserSensor::sampleLineStatus() {
  const uint8_t lsr = _bus.readLineStatus(_ch);
  if (lsr & 0x02) satInc16(_diag.uartOverrunErrors);
  if (lsr & 0x04) satInc16(_diag.uartParityErrors);
  if (lsr & 0x08) satInc16(_diag.uartFramingErrors);
  if (lsr & 0x10) satInc16(_diag.uartBreakErrors);
  if (lsr & 0x80) satInc16(_diag.uartFifoErrors);
}

void FastLaserSensor::serviceConfiguration(uint32_t now) {
  switch (_state) {
    case State::StartDelay:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        _state = State::SendShutdown;
        _nextActionMs = now;
      }
      break;

    case State::SendShutdown:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendCommand(CMD_SHUTDOWN, sizeof(CMD_SHUTDOWN), now,
                    FAST_LASER_ACK_SHUTDOWN, State::WaitShutdownAck);
      }
      break;
    case State::WaitShutdownAck:
      finishWaitOrTimeout(now, FAST_LASER_ACK_SHUTDOWN, State::SendLaserOn);
      break;

    case State::SendLaserOn:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendCommand(CMD_LASER_ON, sizeof(CMD_LASER_ON), now,
                    FAST_LASER_ACK_LASER_ON, State::WaitLaserOnAck);
      }
      break;
    case State::WaitLaserOnAck:
      finishWaitOrTimeout(now, FAST_LASER_ACK_LASER_ON, State::SendRange);
      break;

    case State::SendRange:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        sendCommand(CMD_RANGE_10M, sizeof(CMD_RANGE_10M), now,
                    FAST_LASER_ACK_RANGE, State::WaitRangeAck);
      }
      break;
    case State::WaitRangeAck:
      finishWaitOrTimeout(now, FAST_LASER_ACK_RANGE, State::SendResolution);
      break;

    case State::SendResolution:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        uint8_t len = 0;
        const uint8_t* command = resolutionCommand(_resolution, len);
        sendCommand(command, len, now,
                    FAST_LASER_ACK_RESOLUTION, State::WaitResolutionAck);
      }
      break;
    case State::WaitResolutionAck:
      finishWaitOrTimeout(now, FAST_LASER_ACK_RESOLUTION, State::SendFrequency);
      break;

    case State::SendFrequency:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        uint8_t len = 0;
        const uint8_t* command = frequencyCommand(_diag.configuredFrequencyHz, len);
        sendCommand(command, len, now,
                    FAST_LASER_ACK_FREQUENCY, State::WaitFrequencyAck);
      }
      break;
    case State::WaitFrequencyAck:
      if (!waitFinished(now, FAST_LASER_ACK_FREQUENCY)) break;
      if (!(_diag.ackMask & FAST_LASER_ACK_FREQUENCY) &&
          !(_diag.nackMask & FAST_LASER_ACK_FREQUENCY)) {
        _diag.missingAckMask |= FAST_LASER_ACK_FREQUENCY;
      }
      if (_mode == FastLaserAcquisitionMode::SingleShot) {
        completeSingleShotConfiguration(now);
      } else {
        _state = State::SendContinuous;
        _nextActionMs = now + FAST_LASER_INTER_COMMAND_GAP_MS;
      }
      break;

    case State::SendContinuous:
      if ((int32_t)(now - _nextActionMs) >= 0) {
        // Confirm a stream frame received after this CONTINUOUS command, not a
        // leftover frame that may have arrived before SHUTDOWN took effect.
        _diag.streamSeen = false;
        _diag.ackMask &= (uint8_t)~FAST_LASER_ACK_STREAM;
        _bus.writeBuffer(_ch, CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
        _expectedAckBit = FAST_LASER_ACK_STREAM;
        _ackDeadlineMs = now + FAST_LASER_STREAM_START_TIMEOUT_MS;
        _state = State::WaitStream;
      }
      break;

    case State::WaitStream:
      if (_diag.streamSeen || (int32_t)(now - _ackDeadlineMs) >= 0) {
        if (!_diag.streamSeen) _diag.missingAckMask |= FAST_LASER_ACK_STREAM;
        _diag.configuring = false;
        _diag.lastConfigCompleteMs = now;
        _state = State::Running;
      }
      break;

    case State::Running:
      break;
  }
}

void FastLaserSensor::completeSingleShotConfiguration(uint32_t now) {
  _diag.configuring = false;
  _diag.lastConfigCompleteMs = now;
  _state = State::Running;
  _nextActionMs = now;
}

void FastLaserSensor::sendCommand(const uint8_t* data, size_t len,
                                  uint32_t now, uint8_t expectedAckBit,
                                  State waitState) {
  if (!data || len == 0) return;
  _bus.writeBuffer(_ch, data, len);
  _expectedAckBit = expectedAckBit;
  _ackDeadlineMs = now + FAST_LASER_ACK_TIMEOUT_MS;
  _state = waitState;
}

bool FastLaserSensor::waitFinished(uint32_t now, uint8_t ackBit) const {
  return (_diag.ackMask & ackBit) || (_diag.nackMask & ackBit) ||
         ((int32_t)(now - _ackDeadlineMs) >= 0);
}

void FastLaserSensor::finishWaitOrTimeout(uint32_t now, uint8_t ackBit,
                                          State nextState) {
  if (!waitFinished(now, ackBit)) return;
  if (!(_diag.ackMask & ackBit) && !(_diag.nackMask & ackBit)) {
    _diag.missingAckMask |= ackBit;
  }
  _state = nextState;
  _nextActionMs = now + FAST_LASER_INTER_COMMAND_GAP_MS;
}

const uint8_t* FastLaserSensor::frequencyCommand(uint8_t frequencyHz,
                                                 uint8_t& len) const {
  len = 5;
  if (frequencyHz == 5) return CMD_FREQ_5HZ;
  if (frequencyHz == 20) return CMD_FREQ_20HZ;
  return CMD_FREQ_10HZ;
}

const uint8_t* FastLaserSensor::resolutionCommand(
    FastLaserResolution resolution, uint8_t& len) const {
  len = 5;
  return resolution == FastLaserResolution::TenthMm
             ? CMD_RESOLUTION_01MM
             : CMD_RESOLUTION_1MM;
}

void FastLaserSensor::flushRx() {
  _bus.flushRx(_ch);
  _rxLen = 0;
}

void FastLaserSensor::readIncoming(uint32_t now) {
  while (true) {
    const int value = _bus.readByte(_ch);
    if (value < 0) break;
    ++_diag.bytesRx;
    _diag.lastByteMs = now;
    appendRx((uint8_t)value);
  }

  bool progress = true;
  while (progress) {
    progress = false;
    int32_t mm = 0;
    const int8_t rc = tryConsumeFrame(now, mm);
    if (rc == 1) {
      if (mm > 0 && mm <= LASER_MAX_MM) {
        _reading.mm = mm;
        _reading.valid = true;
        _reading.updated = true;
        _reading.lastUpdateMs = now;
        ++_diag.goodFrames;
        noteGoodFrame();
      } else {
        satInc16(_diag.rangeRejects);
      }
      progress = true;
    } else if (rc == 0) {
      progress = true;
    }
  }
}

void FastLaserSensor::appendRx(uint8_t value) {
  if (_rxLen < sizeof(_rx)) {
    _rx[_rxLen++] = value;
    return;
  }
  satInc16(_diag.softwareOverruns);
  discardFront(1);
  _rx[_rxLen++] = value;
}

void FastLaserSensor::discardFront(size_t count) {
  if (count == 0 || _rxLen == 0) return;
  if (count > _rxLen) count = _rxLen;
  memmove(_rx, _rx + count, _rxLen - count);
  _rxLen -= count;
}

bool FastLaserSensor::consumeExactAck(const uint8_t* frame, uint8_t len,
                                      uint8_t ackBit) {
  if (_rxLen < len || memcmp(_rx, frame, len) != 0) return false;
  _diag.ackMask |= ackBit;
  discardFront(len);
  return true;
}

bool FastLaserSensor::consumeConfigNack(uint8_t functionByte,
                                        uint8_t ackBit) {
  if (_rxLen < 5) return false;
  if (_rx[0] != 0xFA || _rx[1] != 0x84 || _rx[2] != functionByte) return false;
  if (!frameChecksumOk(_rx, 5)) {
    satInc16(_diag.checksumErrors);
    discardFront(1);
    return true;
  }
  _diag.nackMask |= ackBit;
  discardFront(5);
  return true;
}

int8_t FastLaserSensor::tryConsumeFrame(uint32_t now, int32_t& outMm) {
  if (_rxLen == 0) return -1;

  // Synchronise to one of the two valid frame prefixes used by this family.
  size_t start = 0;
  while (start < _rxLen && _rx[start] != 0x80 && _rx[start] != 0xFA) ++start;
  if (start > 0) {
    satAdd16(_diag.discardedBytes, start);
    discardFront(start);
    return 0;
  }

  if (_rx[0] == 0xFA) {
    if (consumeExactAck(ACK_RANGE, sizeof(ACK_RANGE), FAST_LASER_ACK_RANGE)) return 0;
    if (consumeExactAck(ACK_RESOLUTION, sizeof(ACK_RESOLUTION), FAST_LASER_ACK_RESOLUTION)) return 0;
    if (consumeExactAck(ACK_FREQUENCY, sizeof(ACK_FREQUENCY), FAST_LASER_ACK_FREQUENCY)) return 0;
    if (consumeConfigNack(0x89, FAST_LASER_ACK_RANGE)) return 0;
    if (consumeConfigNack(0x8C, FAST_LASER_ACK_RESOLUTION)) return 0;
    if (consumeConfigNack(0x8A, FAST_LASER_ACK_FREQUENCY)) return 0;

    if (_rxLen < 5) return -1;
    satInc16(_diag.malformedFrames);
    satInc16(_diag.discardedBytes);
    discardFront(1);
    return 0;
  }

  if (consumeExactAck(ACK_SHUTDOWN, sizeof(ACK_SHUTDOWN), FAST_LASER_ACK_SHUTDOWN)) return 0;
  if (consumeExactAck(ACK_LASER_ON, sizeof(ACK_LASER_ON), FAST_LASER_ACK_LASER_ON)) return 0;

  if (_rxLen >= 3 && _rx[1] == 0x06 &&
      (_rx[2] == 0x82 || _rx[2] == 0x83)) {
    return consumeMeasurement(now, outMm);
  }

  if (_rxLen < 5) return -1;
  satInc16(_diag.malformedFrames);
  satInc16(_diag.discardedBytes);
  discardFront(1);
  return 0;
}

int8_t FastLaserSensor::consumeMeasurement(uint32_t now, int32_t& outMm) {
  if (_rxLen < 11) return -1;

  const bool valid11 = frameChecksumOk(_rx, 11);
  const bool valid12 = (_rxLen >= 12) && frameChecksumOk(_rx, 12);

  uint8_t totalLen = 0;
  uint8_t dataLen = 0;
  // Prefer the 0.1 mm frame only when its extra data byte is an ASCII digit.
  if (valid12 && isAsciiDigit(_rx[10])) {
    totalLen = 12;
    dataLen = 8;
  } else if (valid11) {
    totalLen = 11;
    dataLen = 7;
  } else if (valid12) {
    totalLen = 12;
    dataLen = 8;
  } else {
    // At 11 bytes this may still be a valid 12-byte 0.1 mm frame.
    if (_rxLen < 12) return -1;
    satInc16(_diag.checksumErrors);
    satInc16(_diag.discardedBytes);
    discardFront(1);
    return 0;
  }

  ++_diag.streamFrames;
  _diag.lastFrameMs = now;
  _diag.streamSeen = true;
  _diag.ackMask |= FAST_LASER_ACK_STREAM;
  if (_rx[2] == 0x82) {
    satInc16(_diag.singleResponses);
    if (_singlePending) {
      _singlePending = false;
      _diag.singlePending = false;
    }
  }

  const uint8_t* data = &_rx[3];
  if (data[0] == 'E' && data[1] == 'R' && data[2] == 'R') {
    satInc16(_diag.sensorErrorFrames);
    _diag.lastSensorError = parseErrorCode(data, dataLen);
    noteErrorFrame(_diag.lastSensorError);
    discardFront(totalLen);
    return 0;
  }

  const bool shapeOk = dataLen >= 7 &&
                       isAsciiDigit(data[0]) && isAsciiDigit(data[1]) &&
                       isAsciiDigit(data[2]) && data[3] == '.' &&
                       isAsciiDigit(data[4]) && isAsciiDigit(data[5]) &&
                       isAsciiDigit(data[6]) &&
                       (dataLen == 7 || isAsciiDigit(data[7]));
  if (!shapeOk) {
    satInc16(_diag.malformedFrames);
    discardFront(totalLen);
    return 0;
  }

  outMm = parseDistanceMm(data, dataLen);
  discardFront(totalLen);
  return 1;
}

void FastLaserSensor::updateMaximumAge(uint32_t now) {
  if (!_reading.valid || _reading.lastUpdateMs == 0) return;
  const uint32_t age = now - _reading.lastUpdateMs;
  if (age > _diag.maxAgeMs) _diag.maxAgeMs = age;
}

void FastLaserSensor::noteErrorFrame(uint16_t code) {
  if (code == 15) satInc16(_diag.errorCode15Frames);
  else if (code == 16) satInc16(_diag.errorCode16Frames);
  else if (code != 0xFFFE) satInc16(_diag.otherSensorErrorFrames);
  satInc16(_diag.currentErrorStreak);
  if (_diag.currentErrorStreak > _diag.maxErrorStreak) {
    _diag.maxErrorStreak = _diag.currentErrorStreak;
  }
}

void FastLaserSensor::noteGoodFrame() {
  _diag.currentErrorStreak = 0;
}

void FastLaserSensor::updateFrameRate(uint32_t now) {
  const uint32_t elapsed = now - _rateWindowStartMs;
  if (elapsed < 1000U) return;
  const uint32_t delta = _diag.streamFrames - _rateWindowFrameBase;
  uint32_t rateX10 = elapsed ? ((delta * 10000UL) / elapsed) : 0;
  if (rateX10 > UINT16_MAX) rateX10 = UINT16_MAX;
  _diag.frameRateX10 = (uint16_t)rateX10;
  _rateWindowStartMs = now;
  _rateWindowFrameBase = _diag.streamFrames;
}

uint8_t FastLaserSensor::calcChecksum(const uint8_t* data,
                                      size_t lenWithoutChecksum) const {
  uint8_t sum = 0;
  for (size_t i = 0; i < lenWithoutChecksum; ++i) {
    sum = (uint8_t)(sum + data[i]);
  }
  return (uint8_t)(~sum + 1);
}

bool FastLaserSensor::frameChecksumOk(const uint8_t* data, size_t totalLen) const {
  return data && totalLen >= 2 &&
         calcChecksum(data, totalLen - 1) == data[totalLen - 1];
}

bool FastLaserSensor::isAsciiDigit(uint8_t value) const {
  return value >= '0' && value <= '9';
}

uint16_t FastLaserSensor::parseErrorCode(const uint8_t* data,
                                         uint8_t dataLen) const {
  if (!data || dataLen < 2) return 0xFFFF;
  const uint8_t hi = data[dataLen - 2];
  const uint8_t lo = data[dataLen - 1];
  if (isAsciiDigit(hi) && isAsciiDigit(lo)) {
    return (uint16_t)((hi - '0') * 10U + (lo - '0'));
  }
  return (uint16_t)(((uint16_t)hi << 8) | lo);
}

int32_t FastLaserSensor::parseDistanceMm(const uint8_t* data,
                                         uint8_t dataLen) const {
  const int32_t metres = (int32_t)(data[0] - '0') * 100L +
                         (int32_t)(data[1] - '0') * 10L +
                         (int32_t)(data[2] - '0');
  int32_t millimetres = (int32_t)(data[4] - '0') * 100L +
                        (int32_t)(data[5] - '0') * 10L +
                        (int32_t)(data[6] - '0');
  if (dataLen == 8 && data[7] >= '5') ++millimetres;
  return metres * 1000L + millimetres;
}
