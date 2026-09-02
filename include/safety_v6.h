#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config_v6_bringup.h"

class SafetyV6 {
public:
  enum LimitBits : uint8_t {
    LIM_H1_LEFT  = 0x01,
    LIM_H1_RIGHT = 0x02,
    LIM_H2_LEFT  = 0x04,
    LIM_H2_RIGHT = 0x08
  };

  void begin();
  bool service();

  bool estopActive() const { return _estopActive; }
  bool estopLatched() const { return _estopLatched; }
  uint8_t limitMask() const { return _limitMask; }
  bool estopEnabled() const { return _estopEnabled; }
  bool limitsEnabled() const { return _limitsEnabled; }

  uint16_t stateWord() const {
    // Preserve Step8A meaning for logs/DWIN: bit0=active, bit1=latched.
    // Enable/disable states have their own VPs 0x1052/0x1054.
    return (_estopActive ? 0x0001 : 0) | (_estopLatched ? 0x0002 : 0);
  }

  // Runtime safety configuration. Disabling is accepted only in BENCH build.
  bool setEstopEnabled(bool enabled);
  bool setLimitsEnabled(bool enabled);

  // Clear only the latch. An actually active enabled E-stop can never be cleared.
  bool clearEstopLatch();

  // motorStateCode uses MOTOR_STATE_* constants from config_v6_bringup.h.
  bool blocksMotorState(uint16_t motorStateCode) const;
  const __FlashStringHelper* blockReason(uint16_t motorStateCode) const;

private:
  bool readActive(uint8_t pin, bool activeLow) const;
  uint8_t readLimitMask() const;
  void refreshInputs();

private:
  bool _estopEnabled = ENABLE_ESTOP;
  bool _limitsEnabled = ENABLE_LIMIT_SWITCHES;
  bool _estopActive = false;
  bool _estopLatched = false;
  uint8_t _limitMask = 0;
};
