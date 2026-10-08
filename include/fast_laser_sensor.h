#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "sc16is752.h"

// Configuration acknowledgement bits from the SEN0366-compatible protocol.
enum FastLaserAckBits : uint8_t {
  FAST_LASER_ACK_SHUTDOWN   = 1u << 0,
  FAST_LASER_ACK_LASER_ON   = 1u << 1,
  FAST_LASER_ACK_RANGE      = 1u << 2,
  FAST_LASER_ACK_RESOLUTION = 1u << 3,
  FAST_LASER_ACK_FREQUENCY  = 1u << 4,
  FAST_LASER_ACK_STREAM     = 1u << 5
};

enum class FastLaserResolution : uint8_t {
  Mm1 = 1,
  TenthMm = 2
};

enum class FastLaserAcquisitionMode : uint8_t {
  Continuous = 0,
  SingleShot = 1
};

struct FastLaserReading {
  int32_t mm = 0;
  bool valid = false;
  bool updated = false;
  uint32_t lastUpdateMs = 0;
};

// Compact counters intended for field diagnostics on Arduino Mega.
// Distances remain integer millimetres even when the sensor replies in 0.1 mm
// mode; the fourth decimal digit is rounded to the nearest millimetre.
struct FastLaserDiagnostics {
  uint32_t bytesRx = 0;
  uint32_t streamFrames = 0;
  uint32_t goodFrames = 0;
  uint16_t sensorErrorFrames = 0;
  uint16_t checksumErrors = 0;
  uint16_t malformedFrames = 0;
  uint16_t rangeRejects = 0;
  uint16_t discardedBytes = 0;
  uint16_t softwareOverruns = 0;

  uint16_t uartOverrunErrors = 0;
  uint16_t uartParityErrors = 0;
  uint16_t uartFramingErrors = 0;
  uint16_t uartBreakErrors = 0;
  uint16_t uartFifoErrors = 0;

  uint16_t lastSensorError = 0;
  uint16_t errorCode15Frames = 0;
  uint16_t errorCode16Frames = 0;
  uint16_t otherSensorErrorFrames = 0;
  uint16_t currentErrorStreak = 0;
  uint16_t maxErrorStreak = 0;

  uint16_t singleRequests = 0;
  uint16_t singleResponses = 0;
  uint16_t singleTimeouts = 0;
  uint16_t frameRateX10 = 0; // frames/s multiplied by 10
  uint32_t maxAgeMs = 0;

  uint32_t lastByteMs = 0;
  uint32_t lastFrameMs = 0;
  uint32_t lastConfigStartMs = 0;
  uint32_t lastConfigCompleteMs = 0;

  uint8_t ackMask = 0;
  uint8_t missingAckMask = 0;
  uint8_t nackMask = 0;
  uint8_t configuredFrequencyHz = 10;
  uint8_t configuredResolutionCode = (uint8_t)FastLaserResolution::Mm1;
  uint8_t acquisitionMode = (uint8_t)FastLaserAcquisitionMode::Continuous;
  bool configuring = false;
  bool streamSeen = false;
  bool singlePending = false;
};

class FastLaserSensor {
public:
  FastLaserSensor(Sc16Is752& bus, Sc16Is752::Channel ch);

  // Full startup sequence:
  // SHUTDOWN -> LASER ON -> RANGE 10 m -> RESOLUTION -> FREQUENCY ->
  // CONTINUOUS, or READY FOR EXTERNAL SINGLE-SHOT REQUESTS.
  void begin(uint32_t now, uint16_t samplePeriodMs, uint16_t phaseMs,
             uint8_t frequencyHz = 10,
             FastLaserResolution resolution = FastLaserResolution::Mm1,
             FastLaserAcquisitionMode mode = FastLaserAcquisitionMode::Continuous);
  void service(uint32_t now);

  // Re-run the full configuration without reinitialising the SC16 UART.
  // Returns false for unsupported parameters or while already configuring.
  bool requestConfigure(uint32_t now, uint8_t frequencyHz,
                        FastLaserResolution resolution = FastLaserResolution::Mm1,
                        FastLaserAcquisitionMode mode = FastLaserAcquisitionMode::Continuous);

  // Valid only after configuration in SingleShot mode. The caller may issue
  // the next request when singleRequestPending() becomes false.
  bool requestSingleMeasurement(uint32_t now);
  bool singleRequestPending() const { return _singlePending; }

  void resetDiagnostics(uint32_t now);

  bool valid(uint32_t now, uint16_t staleTimeoutMs) const;
  bool consumeUpdated();
  int32_t rawMm() const { return _reading.mm; }
  uint32_t lastUpdateMs() const { return _reading.lastUpdateMs; }
  const FastLaserDiagnostics& diagnostics() const { return _diag; }
  bool configurationBusy() const { return _diag.configuring; }
  uint8_t configuredFrequencyHz() const { return _diag.configuredFrequencyHz; }
  FastLaserResolution configuredResolution() const { return _resolution; }
  FastLaserAcquisitionMode acquisitionMode() const { return _mode; }

private:
  enum class State : uint8_t {
    StartDelay,
    SendShutdown,
    WaitShutdownAck,
    SendLaserOn,
    WaitLaserOnAck,
    SendRange,
    WaitRangeAck,
    SendResolution,
    WaitResolutionAck,
    SendFrequency,
    WaitFrequencyAck,
    SendContinuous,
    WaitStream,
    Running
  };

  void beginConfiguration(uint32_t now, uint16_t phaseMs, uint8_t frequencyHz,
                          FastLaserResolution resolution,
                          FastLaserAcquisitionMode mode);
  void completeSingleShotConfiguration(uint32_t now);
  void sendCommand(const uint8_t* data, size_t len, uint32_t now,
                   uint8_t expectedAckBit, State waitState);
  void serviceConfiguration(uint32_t now);
  bool waitFinished(uint32_t now, uint8_t ackBit) const;
  void finishWaitOrTimeout(uint32_t now, uint8_t ackBit, State nextState);

  void readIncoming(uint32_t now);
  void sampleLineStatus();
  void appendRx(uint8_t b);
  void flushRx();
  void updateFrameRate(uint32_t now);
  void updateMaximumAge(uint32_t now);
  void noteErrorFrame(uint16_t code);
  void noteGoodFrame();

  // returns: 1 measurement, 0 consumed non-measurement frame, -1 need data
  int8_t tryConsumeFrame(uint32_t now, int32_t& outMm);
  bool consumeExactAck(const uint8_t* frame, uint8_t len, uint8_t ackBit);
  bool consumeConfigNack(uint8_t functionByte, uint8_t ackBit);
  int8_t consumeMeasurement(uint32_t now, int32_t& outMm);
  void discardFront(size_t count);

  uint8_t calcChecksum(const uint8_t* data, size_t lenWithoutChecksum) const;
  bool frameChecksumOk(const uint8_t* data, size_t totalLen) const;
  bool isAsciiDigit(uint8_t c) const;
  uint16_t parseErrorCode(const uint8_t* data, uint8_t dataLen) const;
  int32_t parseDistanceMm(const uint8_t* data, uint8_t dataLen) const;
  const uint8_t* frequencyCommand(uint8_t frequencyHz, uint8_t& len) const;
  const uint8_t* resolutionCommand(FastLaserResolution resolution,
                                   uint8_t& len) const;

private:
  Sc16Is752& _bus;
  Sc16Is752::Channel _ch;

  State _state = State::StartDelay;
  uint16_t _samplePeriodMs = 120; // retained for compatibility/diagnostics
  uint32_t _nextActionMs = 0;
  uint32_t _ackDeadlineMs = 0;
  uint8_t _expectedAckBit = 0;

  FastLaserResolution _resolution = FastLaserResolution::Mm1;
  FastLaserAcquisitionMode _mode = FastLaserAcquisitionMode::Continuous;
  bool _singlePending = false;
  uint32_t _singleDeadlineMs = 0;

  uint8_t _rx[128];
  size_t _rxLen = 0;

  uint32_t _rateWindowStartMs = 0;
  uint32_t _rateWindowFrameBase = 0;

  FastLaserReading _reading;
  FastLaserDiagnostics _diag;
};
