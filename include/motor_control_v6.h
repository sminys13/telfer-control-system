#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "system_state.h"
#include "config_v6_bringup.h"
#include "settings_v6.h"
#include "vfd_driver_v6.h"

enum class MotorAxisV6 : uint8_t
{
  H1,
  H2,
  H_BOTH,
  V1,
  V2,
  V_BOTH
};

enum class MotorDirV6 : int8_t
{
  NEGATIVE = -1,
  POSITIVE = 1
};

enum class MotorStateV6 : uint16_t
{
  IDLE        = MOTOR_STATE_IDLE,
  H1_FWD      = MOTOR_STATE_H1_FWD,
  H1_BWD      = MOTOR_STATE_H1_BWD,
  H2_FWD      = MOTOR_STATE_H2_FWD,
  H2_BWD      = MOTOR_STATE_H2_BWD,
  H_BOTH_FWD  = MOTOR_STATE_H_BOTH_FWD,
  H_BOTH_BWD  = MOTOR_STATE_H_BOTH_BWD,
  V1_UP       = MOTOR_STATE_V1_UP,
  V1_DOWN     = MOTOR_STATE_V1_DOWN,
  V2_UP       = MOTOR_STATE_V2_UP,
  V2_DOWN     = MOTOR_STATE_V2_DOWN,
  V_BOTH_UP   = MOTOR_STATE_V_BOTH_UP,
  V_BOTH_DOWN = MOTOR_STATE_V_BOTH_DOWN,
  STOPPED     = MOTOR_STATE_STOPPED,
  BLOCKED     = MOTOR_STATE_BLOCKED
};

class MotorControlV6
{
public:
  void begin(const SettingsV6& settings);
  void applySettings(const SettingsV6& settings);

  // Returns true when state/status changed and DWIN must be refreshed.
  bool service(uint32_t nowMs);

  void stopAll(const __FlashStringHelper* reason);
  void onModeChanged(SystemModeV6 mode);

  bool requestManualMove(SystemModeV6 currentMode, MotorAxisV6 axis, MotorDirV6 dir);
  // AUTO/HOME target interface. Percent signs use logical coordinates:
  // H + = right, V + = up. Manual watchdog is intentionally not armed here.
  bool requestAutoTargets(SystemModeV6 currentMode, int16_t h1RightPct, int16_t h2RightPct,
                          int16_t v1UpPct, int16_t v2UpPct);
  void autoStop(const __FlashStringHelper* reason);
  void manualStop(const __FlashStringHelper* reason);
  void testVfdConnection(uint8_t driveIndex);

  bool isMotionActive() const { return _motionActive; }
  uint32_t lastManualCmdMs() const { return _lastManualCmdMs; }
  uint16_t manualJogTimeoutMs() const { return _manualJogTimeoutMs; }
  MotorStateV6 state() const { return _state; }
  uint16_t stateCode() const { return static_cast<uint16_t>(_state); }
  uint16_t vfdStatusCode() const { return _vfd.statusCode(); }
  uint8_t vfdConnectedCount() const { return _vfd.connectedCount(); }
  bool vfdHasPendingWork() const { return _vfd.hasPendingWork(); }
  const __FlashStringHelper* stateName() const;

private:
  const __FlashStringHelper* axisName(MotorAxisV6 axis) const;
  const __FlashStringHelper* dirName(MotorDirV6 dir) const;
  MotorStateV6 stateForMove(MotorAxisV6 axis, MotorDirV6 dir) const;
  MotorStateV6 stateForAutoTargets(int16_t h1, int16_t h2, int16_t v1Up, int16_t v2Up) const;
  bool stateMeansMovement() const;

private:
  bool _motionActive = false;
  MotorStateV6 _state = MotorStateV6::IDLE;
  uint32_t _lastManualCmdMs = 0;
  uint16_t _manualJogTimeoutMs = MANUAL_JOG_TIMEOUT_DEFAULT_MS;
  VfdDriverV6 _vfd;
};
