#include "motor_control_v6.h"

void MotorControlV6::begin(const SettingsV6& settings)
{
  _motionActive = false;
  _state = MotorStateV6::IDLE;
  _lastManualCmdMs = 0;
  _manualJogTimeoutMs = settings.vfd.manualJogTimeoutMs;
  if (!VFD_RS485_ENABLED)
    Serial.println(F("MotorControlV6: safe control layer initialized, NE200 dry-run scheduler enabled"));
  else if (!VFD_WRITE_COMMANDS_ENABLED)
    Serial.println(F("MotorControlV6: safe control layer initialized, physical RS485 READ-ONLY"));
  else
    Serial.println(F("MotorControlV6: safe control layer initialized, physical RS485 writes enabled"));
  _vfd.begin(settings);
}

void MotorControlV6::applySettings(const SettingsV6& settings)
{
  _manualJogTimeoutMs = settings.vfd.manualJogTimeoutMs;
  _vfd.applySettings(settings);
  Serial.print(F("MotorControlV6: manual watchdog applied="));
  Serial.print(_manualJogTimeoutMs);
  Serial.println(F("ms"));
}

bool MotorControlV6::service(uint32_t nowMs)
{
  bool changed = _vfd.service(nowMs);

  if (MANUAL_JOG_TIMEOUT_ENABLED && _motionActive && _lastManualCmdMs != 0)
  {
    if ((uint32_t)(nowMs - _lastManualCmdMs) > _manualJogTimeoutMs)
    {
      _motionActive = false;
      _state = MotorStateV6::STOPPED;
      _vfd.stopAll(F("manual watchdog"));
      Serial.print(F("MANUAL JOG WATCHDOG STOP timeout="));
      Serial.print(_manualJogTimeoutMs);
      Serial.println(F("ms"));
      return true;
    }
  }
  return changed;
}

void MotorControlV6::stopAll(const __FlashStringHelper* reason)
{
  _motionActive = false;
  _lastManualCmdMs = 0;
  _state = MotorStateV6::STOPPED;
  Serial.print(F("MOTOR STOP ALL"));
  if (reason)
  {
    Serial.print(F(" reason="));
    Serial.print(reason);
  }
  Serial.println();
  _vfd.stopAll(reason);
}

void MotorControlV6::manualStop(const __FlashStringHelper* reason)
{
  _motionActive = false;
  _lastManualCmdMs = 0;
  _state = MotorStateV6::STOPPED;
  Serial.print(F("MANUAL JOG STOP"));
  if (reason)
  {
    Serial.print(F(" reason="));
    Serial.print(reason);
  }
  Serial.println();
  _vfd.stopAll(reason);
}

const __FlashStringHelper* MotorControlV6::axisName(MotorAxisV6 axis) const
{
  switch (axis)
  {
  case MotorAxisV6::H1: return F("H1/X1");
  case MotorAxisV6::H2: return F("H2/X2");
  case MotorAxisV6::H_BOTH: return F("H1+H2");
  case MotorAxisV6::V1: return F("V1/Z1");
  case MotorAxisV6::V2: return F("V2/Z2");
  case MotorAxisV6::V_BOTH: return F("V1+V2");
  }
  return F("?");
}

const __FlashStringHelper* MotorControlV6::dirName(MotorDirV6 dir) const
{
  return (dir == MotorDirV6::POSITIVE) ? F("POS/FWD/UP") : F("NEG/BWD/DOWN");
}

MotorStateV6 MotorControlV6::stateForMove(MotorAxisV6 axis, MotorDirV6 dir) const
{
  const bool pos = (dir == MotorDirV6::POSITIVE);
  switch (axis)
  {
  case MotorAxisV6::H1: return pos ? MotorStateV6::H1_FWD : MotorStateV6::H1_BWD;
  case MotorAxisV6::H2: return pos ? MotorStateV6::H2_FWD : MotorStateV6::H2_BWD;
  case MotorAxisV6::H_BOTH: return pos ? MotorStateV6::H_BOTH_FWD : MotorStateV6::H_BOTH_BWD;
  case MotorAxisV6::V1: return pos ? MotorStateV6::V1_UP : MotorStateV6::V1_DOWN;
  case MotorAxisV6::V2: return pos ? MotorStateV6::V2_UP : MotorStateV6::V2_DOWN;
  case MotorAxisV6::V_BOTH: return pos ? MotorStateV6::V_BOTH_UP : MotorStateV6::V_BOTH_DOWN;
  }
  return MotorStateV6::BLOCKED;
}

bool MotorControlV6::stateMeansMovement() const
{
  const uint16_t code = stateCode();
  return code >= MOTOR_STATE_H1_FWD && code <= MOTOR_STATE_V_BOTH_DOWN;
}

MotorStateV6 MotorControlV6::stateForAutoTargets(int16_t h1, int16_t h2,
                                                       int16_t v1Up, int16_t v2Up) const
{
  if (h1 > 0 && h2 > 0) return MotorStateV6::H_BOTH_FWD;
  if (h1 < 0 && h2 < 0) return MotorStateV6::H_BOTH_BWD;
  if (h1 > 0) return MotorStateV6::H1_FWD;
  if (h1 < 0) return MotorStateV6::H1_BWD;
  if (h2 > 0) return MotorStateV6::H2_FWD;
  if (h2 < 0) return MotorStateV6::H2_BWD;
  if (v1Up > 0 && v2Up > 0) return MotorStateV6::V_BOTH_UP;
  if (v1Up < 0 && v2Up < 0) return MotorStateV6::V_BOTH_DOWN;
  if (v1Up > 0) return MotorStateV6::V1_UP;
  if (v1Up < 0) return MotorStateV6::V1_DOWN;
  if (v2Up > 0) return MotorStateV6::V2_UP;
  if (v2Up < 0) return MotorStateV6::V2_DOWN;
  return MotorStateV6::IDLE;
}

bool MotorControlV6::requestManualMove(SystemModeV6 currentMode, MotorAxisV6 axis, MotorDirV6 dir)
{
  if (VFD_RS485_ENABLED && !VFD_WRITE_COMMANDS_ENABLED)
  {
    _motionActive = false;
    _lastManualCmdMs = 0;
    _state = MotorStateV6::BLOCKED;
    Serial.println(F("MANUAL JOG BLOCKED: physical RS485 build is READ-ONLY"));
    return false;
  }

  if (currentMode != SystemModeV6::MANUAL)
  {
    _motionActive = false;
    _lastManualCmdMs = 0;
    _state = MotorStateV6::BLOCKED;
    Serial.print(F("MANUAL JOG ignored: system mode is "));
    Serial.println(systemModeName(currentMode));
    _vfd.block(F("not MANUAL mode"));
    return false;
  }

  _motionActive = true;
  _lastManualCmdMs = millis();
  _state = stateForMove(axis, dir);

  Serial.print(F("MANUAL JOG REQUEST axis="));
  Serial.print(axisName(axis));
  Serial.print(F(" dir="));
  Serial.print(dirName(dir));
  Serial.print(F(" state="));
  Serial.print(stateCode());
  Serial.print(F(" "));
  Serial.println(stateName());

  _vfd.startByMotorState(stateCode());
  return true;
}

bool MotorControlV6::requestAutoTargets(SystemModeV6 currentMode,
                                             int16_t h1RightPct, int16_t h2RightPct,
                                             int16_t v1UpPct, int16_t v2UpPct)
{
  if (currentMode != SystemModeV6::AUTO && currentMode != SystemModeV6::HOME)
  {
    _motionActive = false;
    _state = MotorStateV6::BLOCKED;
    Serial.println(F("AUTO TARGET ignored: system is not AUTO/HOME"));
    _vfd.block(F("not AUTO/HOME mode"));
    return false;
  }

  if (VFD_RS485_ENABLED && !VFD_WRITE_COMMANDS_ENABLED)
  {
    _motionActive = false;
    _state = MotorStateV6::BLOCKED;
    return _vfd.setAutoLogicalTargets(h1RightPct, h2RightPct, v1UpPct, v2UpPct);
  }

  _lastManualCmdMs = 0; // auto motion must never be killed by the manual jog watchdog
  _motionActive = h1RightPct || h2RightPct || v1UpPct || v2UpPct;
  _state = stateForAutoTargets(h1RightPct, h2RightPct, v1UpPct, v2UpPct);
  return _vfd.setAutoLogicalTargets(h1RightPct, h2RightPct, v1UpPct, v2UpPct);
}

void MotorControlV6::autoStop(const __FlashStringHelper* reason)
{
  const bool announce = _motionActive || _state != MotorStateV6::STOPPED;
  _motionActive = false;
  _lastManualCmdMs = 0;
  _state = MotorStateV6::STOPPED;
  _vfd.setAutoLogicalTargets(0, 0, 0, 0);
  if (announce) {
    Serial.print(F("AUTO MOTION STOP"));
    if (reason) {
      Serial.print(F(" reason="));
      Serial.print(reason);
    }
    Serial.println();
  }
}

void MotorControlV6::onModeChanged(SystemModeV6 mode)
{
  Serial.print(F("MotorControlV6 mode hook: "));
  Serial.println(systemModeName(mode));

  if (mode == SystemModeV6::STOP)
  {
    stopAll(F("STOP mode"));
    return;
  }

  // Safety fix: every other mode transition away from an active movement
  // performs a stop through the VFD abstraction before changing logical state.
  if (_motionActive || stateMeansMovement())
  {
    stopAll(F("mode change"));
  }

  _motionActive = false;
  _lastManualCmdMs = 0;
  _state = MotorStateV6::IDLE;

  switch (mode)
  {
  case SystemModeV6::MANUAL:
    if (!VFD_RS485_ENABLED)
      Serial.println(F("Manual mode selected. Jog commands build real NE200 RTU frames in dry-run."));
    else if (!VFD_WRITE_COMMANDS_ENABLED)
      Serial.println(F("Manual mode selected, but motion is blocked by physical RS485 READ-ONLY build."));
    else
      Serial.println(F("Manual mode selected. Physical NE200 write commands are enabled."));
    break;
  case SystemModeV6::AUTO:
    Serial.println(F("Auto mode selected. AutoRunnerV6 owns the program sequence."));
    break;
  case SystemModeV6::HOME:
    Serial.println(F("Home mode selected. AutoRunnerV6 owns the homing sequence."));
    break;
  case SystemModeV6::SERVICE:
  case SystemModeV6::SETTINGS:
  case SystemModeV6::CALIBRATION:
  case SystemModeV6::STOP:
    break;
  }
}

void MotorControlV6::testVfdConnection(uint8_t driveIndex)
{
  _vfd.testConnection(driveIndex);
}

const __FlashStringHelper* MotorControlV6::stateName() const
{
  switch (_state)
  {
  case MotorStateV6::IDLE: return F("IDLE");
  case MotorStateV6::H1_FWD: return F("H1_FWD");
  case MotorStateV6::H1_BWD: return F("H1_BWD");
  case MotorStateV6::H2_FWD: return F("H2_FWD");
  case MotorStateV6::H2_BWD: return F("H2_BWD");
  case MotorStateV6::H_BOTH_FWD: return F("H_BOTH_FWD");
  case MotorStateV6::H_BOTH_BWD: return F("H_BOTH_BWD");
  case MotorStateV6::V1_UP: return F("V1_UP");
  case MotorStateV6::V1_DOWN: return F("V1_DOWN");
  case MotorStateV6::V2_UP: return F("V2_UP");
  case MotorStateV6::V2_DOWN: return F("V2_DOWN");
  case MotorStateV6::V_BOTH_UP: return F("V_BOTH_UP");
  case MotorStateV6::V_BOTH_DOWN: return F("V_BOTH_DOWN");
  case MotorStateV6::STOPPED: return F("STOPPED");
  case MotorStateV6::BLOCKED: return F("BLOCKED");
  }
  return F("?");
}
