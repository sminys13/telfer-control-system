#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "sc16is752.h"

struct FastLaserReading {
  int32_t mm = 0;
  bool valid = false;
  bool updated = false;
  uint32_t lastUpdateMs = 0;
};

class FastLaserSensor {
public:
  FastLaserSensor(Sc16Is752& bus, Sc16Is752::Channel ch);

  // Continuous mode: LASER ON -> CONTINUOUS -> passive stream parsing.
  // samplePeriodMs is kept for compatibility but is not used to trigger SINGLE shots.
  void begin(uint32_t now, uint16_t samplePeriodMs, uint16_t phaseMs);
  void service(uint32_t now);

  bool valid(uint32_t now, uint16_t staleTimeoutMs) const;
  bool consumeUpdated();
  int32_t rawMm() const { return _reading.mm; }
  uint32_t lastUpdateMs() const { return _reading.lastUpdateMs; }

private:
  enum class State : uint8_t {
    Start,
    WaitAfterLaserOn,
    WaitAfterRange,
    WaitAfterResolution,
    WaitAfterFrequency,
    WaitAfterContinuous,
    Running
  };

  void sendLaserOn(uint32_t now);
  void sendRange10m(uint32_t now);
  void sendResolution1mm(uint32_t now);
  void sendFrequency(uint32_t now);
  void sendContinuous(uint32_t now);
  void readIncoming(uint32_t now);
  void appendRx(uint8_t b);
  void flushRx();
  void restartContinuous(uint32_t now);

  // returns: 1 measurement, 0 consumed non-measurement frame, -1 nothing
  int8_t tryConsumeFrame(int32_t& outMm);
  uint8_t calcChecksum(const uint8_t* data, size_t lenWithoutChecksum) const;
  bool isAsciiDigitOrDot(uint8_t c) const;

private:
  Sc16Is752& _bus;
  Sc16Is752::Channel _ch;

  State _state = State::Start;

  uint16_t _samplePeriodMs = 120; // reserved/compatibility
  uint32_t _nextActionMs = 0;
  uint32_t _lastContinuousStartMs = 0;

  uint8_t _rx[128];
  size_t _rxLen = 0;

  FastLaserReading _reading;
};
