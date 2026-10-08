#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config_v6_bringup.h"
#include "motor_control_v6.h"
#include "direction_calibration_v6.h"

// Step9I service-cockpit runtime gate. It intentionally sits outside AutoRunner:
// commissioning/manual recovery can be proven without enabling physical AUTO/HOME.
class He200ServiceV6 {
public:
  enum DriveBit : uint8_t {
    BIT_H1 = 1u << DRIVE_H1,
    BIT_H2 = 1u << DRIVE_H2,
    BIT_V1 = 1u << DRIVE_V1,
    BIT_V2 = 1u << DRIVE_V2,
    MASK_H = BIT_H1 | BIT_H2,
    MASK_V = BIT_V1 | BIT_V2,
    MASK_ALL = MASK_H | MASK_V
  };

  enum class AssistAxis : uint8_t { X = 0, Z = 1 };
  enum class AssistState : uint8_t { OFF=0, STARTING, NORMAL, WARNING, DECEL, HOLD, RECOVER, FAULT };

  void begin(MotorControlV6& motor);
  void resetGates();
  uint8_t calibratedMask() const { uint8_t mask=0; for(uint8_t i=0;i<4;i++)if(_calibration.ready(i))mask|=1u<<i; return mask; }
  int8_t forwardSensorSign(uint8_t i) const { return _calibration.physicalSign(i,true); }
  void restoreDirection(uint8_t i,int8_t sign) { _calibration.restore(i,sign); if(_calibration.ready(i)){_confirmPositiveMask|=1u<<i;_confirmNegativeMask|=1u<<i;} }
  bool calibrationSamplesReady(uint32_t now) const { for(uint8_t i=0;i<4;i++)if(!_calibration.samples[i].stable(now))return false;return true; }
  void setPreflight(bool pass, const __FlashStringHelper* reason = nullptr);
  void markProtocol(uint8_t driveIndex, bool pass);
  void confirmDirection(uint8_t driveIndex, bool positive, bool pass = true);
  void printGateStatus() const;

  bool startPulse(uint8_t driveMask, bool positive, uint8_t pct, uint16_t durationMs, uint32_t nowMs, bool rawCalibration = false);
  void observeSensors(const int32_t rawMm[4], const uint32_t sampleMs[4]);
  bool startAssist(AssistAxis axis, bool positive, uint8_t basePct, uint32_t nowMs,
                   const uint32_t ageMs[4], const uint32_t lastSampleMs[4]);
  void stopAll(const __FlashStringHelper* reason);
  void stopAssist(const __FlashStringHelper* reason);
  void clearAssistFault();

  // MCU input state cannot prove independence of the external safety chain.
  // The diagnostic release keeps all physical writes locked separately.
  void service(uint32_t nowMs, bool safetyBlocked,
               const uint32_t ageMs[4], const uint32_t lastSampleMs[4]);

  bool preflightPassed() const { return _preflight; }
  uint8_t protocolMask() const { return _protocolMask; }
  bool pulseActive() const { return _pulse.active; }
  bool assistActive() const { return _assist.state != AssistState::OFF && _assist.state != AssistState::FAULT; }
  AssistState assistState() const { return _assist.state; }

  static const char* assistStateName(AssistState state);

private:
  struct PulseState {
    bool active = false;
    uint8_t mask = 0;
    bool positive = true;
    uint8_t pct = 10;
    uint16_t durationMs = 1200;
    uint32_t startMs = 0;
    uint32_t stopMs = 0;
    uint32_t lastPollMs = 0;
    bool stopIssued = false;
    uint8_t sawRunningMask = 0;
    bool rawCalibration = false;
    bool forward = false;
    uint8_t drive = 0;
  };

  struct AssistRuntime {
    AssistAxis axis = AssistAxis::X;
    AssistState state = AssistState::OFF;
    bool positive = true;
    uint8_t basePct = 40;
    uint8_t currentPct = 0;
    uint32_t lastRampMs = 0;
    uint32_t lastPrintMs = 0;
    uint8_t freshPairCount = 0;
    uint8_t freshSeenMask = 0;
    uint32_t seenSampleA = 0;
    uint32_t seenSampleB = 0;
    bool stoppedBySupervisor = false;
    uint16_t decelEvents = 0;
    uint16_t recoverEvents = 0;
    uint16_t holdEvents = 0;
  };

  MotorControlV6* _motor = nullptr;
  bool _preflight = false;
  uint8_t _protocolMask = 0;
  uint8_t _pulsePositiveMask = 0;
  uint8_t _pulseNegativeMask = 0;
  uint8_t _confirmPositiveMask = 0;
  uint8_t _confirmNegativeMask = 0;
  uint8_t _pairPassMask = 0; // bit0=X, bit1=Z
  DirectionCalibrationV6 _calibration;
  PulseState _pulse;
  AssistRuntime _assist;

  static uint8_t clampPct(uint8_t pct);
  static uint8_t bitForDrive(uint8_t driveIndex);
  static bool maskIsSingle(uint8_t mask);
  bool protocolReadyForMask(uint8_t mask) const;
  bool directionConfirmedForMask(uint8_t mask, bool positive) const;
  void targetsForMask(uint8_t mask, bool positive, uint8_t pct,
                      int16_t& h1, int16_t& h2, int16_t& v1, int16_t& v2) const;
  bool requestMask(uint8_t mask, bool positive, uint8_t pct);
  void pollMask(uint8_t mask);
  bool maskStopped(uint8_t mask) const;
  void finishPulse(bool pass, const __FlashStringHelper* reason);
  void servicePulse(uint32_t nowMs, bool safetyBlocked);

  uint8_t assistMask() const;
  void assistSensorIndices(uint8_t& a, uint8_t& b) const;
  void noteFreshPair(const uint32_t ageMs[4], const uint32_t lastSampleMs[4]);
  void setAssistState(AssistState next, uint32_t nowMs, uint32_t ageA, uint32_t ageB);
  void applyAssistPct(uint8_t pct, uint32_t nowMs, uint32_t ageA, uint32_t ageB);
  void serviceAssist(uint32_t nowMs, bool safetyBlocked,
                     const uint32_t ageMs[4], const uint32_t lastSampleMs[4]);
};
