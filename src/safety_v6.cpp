#include "safety_v6.h"

bool SafetyV6::readActive(uint8_t pin, bool activeLow) const {
  const bool levelHigh = (digitalRead(pin) == HIGH);
  return activeLow ? !levelHigh : levelHigh;
}

uint8_t SafetyV6::readLimitMask() const {
  if (!_limitsEnabled) return 0;
  uint8_t m = 0;
  if (readActive(PIN_LIM_H1_LEFT,  LIMIT_ACTIVE_LOW)) m |= LIM_H1_LEFT;
  if (readActive(PIN_LIM_H1_RIGHT, LIMIT_ACTIVE_LOW)) m |= LIM_H1_RIGHT;
  if (readActive(PIN_LIM_H2_LEFT,  LIMIT_ACTIVE_LOW)) m |= LIM_H2_LEFT;
  if (readActive(PIN_LIM_H2_RIGHT, LIMIT_ACTIVE_LOW)) m |= LIM_H2_RIGHT;
  return m;
}

void SafetyV6::refreshInputs() {
  _estopActive = _estopEnabled ? readActive(PIN_ESTOP, ESTOP_ACTIVE_LOW) : false;
  if (_estopActive) _estopLatched = true;
  _limitMask = readLimitMask();
}

void SafetyV6::begin() {
  if (ENABLE_ESTOP) pinMode(PIN_ESTOP, INPUT_PULLUP);
  if (ENABLE_LIMIT_SWITCHES) {
    pinMode(PIN_LIM_H1_LEFT, INPUT_PULLUP);
    pinMode(PIN_LIM_H1_RIGHT, INPUT_PULLUP);
    pinMode(PIN_LIM_H2_LEFT, INPUT_PULLUP);
    pinMode(PIN_LIM_H2_RIGHT, INPUT_PULLUP);
  }

  _estopEnabled = ENABLE_ESTOP;
  _limitsEnabled = ENABLE_LIMIT_SWITCHES;
  _estopLatched = false;
  refreshInputs();

  Serial.print(F("SafetyV6: mode="));
  Serial.print(SAFETY_BENCH_MODE ? F("BENCH") : F("FIELD/NC"));
  Serial.print(F(" estop="));
  Serial.print(_estopActive ? F("ACTIVE") : F("OK"));
  Serial.print(F(" limits=0x"));
  Serial.println(_limitMask, HEX);
}

bool SafetyV6::setEstopEnabled(bool enabled) {
  if (!enabled && !SAFETY_BENCH_MODE) {
    Serial.println(F("SAFETY CONFIG rejected: E-stop cannot be disabled in FIELD mode"));
    return false;
  }
  _estopEnabled = ENABLE_ESTOP && enabled;
  if (!_estopEnabled) {
    _estopActive = false;
    _estopLatched = false;
    Serial.println(F("SAFETY E-STOP monitoring DISABLED (BENCH)"));
  } else {
    refreshInputs();
    Serial.println(F("SAFETY E-STOP monitoring ENABLED"));
  }
  return true;
}

bool SafetyV6::setLimitsEnabled(bool enabled) {
  // Controller-side limit monitoring is optional: some cabinets enforce end
  // limits entirely in the hardwired safety/VFD circuit. E-stop is different
  // and remains mandatory in FIELD mode.
  _limitsEnabled = ENABLE_LIMIT_SWITCHES && enabled;
  _limitMask = readLimitMask();
  Serial.print(F("SAFETY LIMIT monitoring "));
  Serial.println(_limitsEnabled ? F("ENABLED") : F("DISABLED (BENCH)"));
  return true;
}

bool SafetyV6::service() {
  const bool oldActive = _estopActive;
  const bool oldLatched = _estopLatched;
  const uint8_t oldLimits = _limitMask;

  refreshInputs();

  if (_estopActive != oldActive) {
    Serial.print(F("SAFETY E-STOP "));
    Serial.println(_estopActive ? F("ACTIVE") : F("released; latch remains until CLEAR"));
  }
  if (_limitMask != oldLimits) {
    Serial.print(F("SAFETY LIMIT mask=0x"));
    Serial.println(_limitMask, HEX);
  }

  return (_estopActive != oldActive) || (_estopLatched != oldLatched) || (_limitMask != oldLimits);
}

bool SafetyV6::clearEstopLatch() {
  if (_estopEnabled && _estopActive) {
    Serial.println(F("SAFETY CLEAR rejected: E-stop is still active"));
    return false;
  }
  if (_estopLatched) Serial.println(F("SAFETY E-stop latch cleared"));
  _estopLatched = false;
  return true;
}

bool SafetyV6::blocksMotorState(uint16_t state) const {
  if ((_estopEnabled && _estopActive) || _estopLatched) return true;
  if (!_limitsEnabled) return false;

  switch (state) {
    case MOTOR_STATE_H1_FWD:     return (_limitMask & LIM_H1_RIGHT) != 0;
    case MOTOR_STATE_H1_BWD:     return (_limitMask & LIM_H1_LEFT) != 0;
    case MOTOR_STATE_H2_FWD:     return (_limitMask & LIM_H2_RIGHT) != 0;
    case MOTOR_STATE_H2_BWD:     return (_limitMask & LIM_H2_LEFT) != 0;
    case MOTOR_STATE_H_BOTH_FWD: return (_limitMask & (LIM_H1_RIGHT | LIM_H2_RIGHT)) != 0;
    case MOTOR_STATE_H_BOTH_BWD: return (_limitMask & (LIM_H1_LEFT  | LIM_H2_LEFT)) != 0;
    default:                     return false;
  }
}

const __FlashStringHelper* SafetyV6::blockReason(uint16_t state) const {
  if (_estopEnabled && _estopActive) return F("E-STOP active");
  if (_estopLatched) return F("E-STOP latched");
  if (_limitsEnabled && blocksMotorState(state)) return F("horizontal limit switch");
  return F("none");
}
