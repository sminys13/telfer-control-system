#include "he200_service_v6.h"

#include <limits.h>

void He200ServiceV6::begin(MotorControlV6& motor) {
  _motor = &motor;
  resetGates();
  _pulse = PulseState{};
  _assist = AssistRuntime{};
  Serial.println(F("HE200 ServiceV6: runtime commissioning gates initialized"));
}

void He200ServiceV6::resetGates() {
  _calibration.reset();
  _preflight = false;
  _protocolMask = 0;
  _pulsePositiveMask = 0;
  _pulseNegativeMask = 0;
  _confirmPositiveMask = 0;
  _confirmNegativeMask = 0;
  _pairPassMask = 0;
  Serial.println(F("@GATE name=ALL result=RESET"));
}

uint8_t He200ServiceV6::clampPct(uint8_t pct) {
  if (pct < SENSOR_ASSIST_MIN_RUNNING_PCT) return SENSOR_ASSIST_MIN_RUNNING_PCT;
  if (pct > 80) return 80; // service tool never commands >80%
  return pct;
}

uint8_t He200ServiceV6::bitForDrive(uint8_t driveIndex) {
  return driveIndex < DRIVE_COUNT_V6 ? (uint8_t)(1u << driveIndex) : 0;
}

bool He200ServiceV6::maskIsSingle(uint8_t mask) {
  return mask && !(mask & (uint8_t)(mask - 1u));
}

void He200ServiceV6::setPreflight(bool pass, const __FlashStringHelper* reason) {
  _preflight = pass;
  Serial.print(F("@GATE name=PREFLIGHT result=")); Serial.print(pass ? F("PASS") : F("FAIL"));
  if (reason) { Serial.print(F(" reason=")); Serial.print(reason); }
  Serial.println();
}

void He200ServiceV6::markProtocol(uint8_t driveIndex, bool pass) {
  const uint8_t bit = bitForDrive(driveIndex);
  if (!bit) return;
  if (pass) _protocolMask |= bit; else _protocolMask &= (uint8_t)~bit;
  Serial.print(F("@GATE name=PROTOCOL drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
  Serial.print(F(" result=")); Serial.println(pass ? F("PASS") : F("FAIL"));
}

void He200ServiceV6::confirmDirection(uint8_t driveIndex, bool positive, bool pass) {
  const uint8_t bit = bitForDrive(driveIndex);
  if (!bit) return;
  (void)pass;
  Serial.print(F("@GATE name=DIRECTION drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
  Serial.print(F(" dir=")); Serial.print(positive ? F("POS") : F("NEG"));
  Serial.println(F(" result=REJECTED reason=MEASURED_CALIBRATION_REQUIRED"));
}

void He200ServiceV6::printGateStatus() const {
  Serial.print(F("@GATE_STATE preflight=")); Serial.print(_preflight ? 1 : 0);
  Serial.print(F(" protocol=0x")); Serial.print(_protocolMask, HEX);
  Serial.print(F(" pulsePos=0x")); Serial.print(_pulsePositiveMask, HEX);
  Serial.print(F(" pulseNeg=0x")); Serial.print(_pulseNegativeMask, HEX);
  Serial.print(F(" confirmPos=0x")); Serial.print(_confirmPositiveMask, HEX);
  Serial.print(F(" confirmNeg=0x")); Serial.print(_confirmNegativeMask, HEX);
  Serial.print(F(" pair=0x")); Serial.print(_pairPassMask, HEX);
  Serial.print(F(" pulseActive=")); Serial.print(_pulse.active ? 1 : 0);
  Serial.print(F(" assist=")); Serial.println(assistStateName(_assist.state));
}

bool He200ServiceV6::protocolReadyForMask(uint8_t mask) const {
  return _preflight && mask && ((_protocolMask & mask) == mask);
}

bool He200ServiceV6::directionConfirmedForMask(uint8_t mask, bool positive) const {
  const uint8_t confirmed = positive ? _confirmPositiveMask : _confirmNegativeMask;
  return mask && ((confirmed & mask) == mask);
}

void He200ServiceV6::targetsForMask(uint8_t mask, bool positive, uint8_t pct,
                                    int16_t& h1, int16_t& h2, int16_t& v1, int16_t& v2) const {
  h1 = h2 = v1 = v2 = 0;
  int16_t raw[4]={0,0,0,0};
  for (uint8_t i=0;i<4;++i) if (mask & bitForDrive(i)) {
    const int8_t sign=_pulse.rawCalibration ? (_pulse.forward ? 1 : -1) : _calibration.physicalSign(i,positive);
    raw[i]=(int16_t)(sign*pct);
  }
  h1=raw[0]; h2=raw[1]; v1=raw[2]; v2=raw[3];
}

bool He200ServiceV6::requestMask(uint8_t mask, bool positive, uint8_t pct) {
  if (!_motor) return false;
  int16_t h1, h2, v1, v2;
  targetsForMask(mask, positive, pct, h1, h2, v1, v2);
  return _motor->requestServiceRawTargets(h1, h2, v1, v2);
}

void He200ServiceV6::pollMask(uint8_t mask) {
  if (!_motor) return;
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i)
    if (mask & bitForDrive(i)) _motor->testVfdConnection(i);
}

bool He200ServiceV6::maskStopped(uint8_t mask) const {
  if (!_motor) return false;
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    if (!(mask & bitForDrive(i))) continue;
    const DriveTelemetry& t = _motor->vfdTelemetry(i);
    if (!t.connected || (int32_t)(t.lastOkMs-_pulse.stopMs)<=0 || (uint32_t)(millis()-t.lastOkMs)>1000 || t.faultCode != 0 || t.runningFreq001Hz > 20 || t.statusWord != 0) return false;
  }
  return true;
}

bool He200ServiceV6::startPulse(uint8_t driveMask, bool positive, uint8_t pct,
                                uint16_t durationMs, uint32_t nowMs, bool rawCalibration) {
  if (HE200_DIAGNOSTIC_LOCK) {
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=DIAGNOSTIC_LOCK"));
    return false;
  }
  if (!_motor || _pulse.active || assistActive() || _motor->auditActive()) return false;
  if (!HE200_FIELD_SERVICE || !protocolReadyForMask(driveMask)) {
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=GATE_NOT_READY"));
    return false;
  }
  if (!maskIsSingle(driveMask) && !directionConfirmedForMask(driveMask, positive)) {
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=DIRECTION_NOT_CONFIRMED"));
    return false;
  }
  if ((driveMask & ~MASK_ALL) || (!maskIsSingle(driveMask) &&
      (rawCalibration || (driveMask!=MASK_H && driveMask!=MASK_V) || (calibratedMask()&driveMask)!=driveMask))) {
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=PAIRED_MOTION_LOCKED")); return false;
  }
  uint8_t drive=0; while (!(driveMask & bitForDrive(drive))) ++drive;
  if (!rawCalibration && !_calibration.ready(drive)) {
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=CALIBRATION_REQUIRED")); return false;
  }
  if (maskIsSingle(driveMask) && !_calibration.begin(drive,nowMs)) {
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=STABLE_SENSORS_REQUIRED")); return false;
  }
  _pulse = PulseState{};
  _pulse.rawCalibration=rawCalibration;
  _pulse.drive=drive;
  _pulse.forward=rawCalibration ? positive : _calibration.physicalSign(drive,positive)>0;
  _pulse.active = true;
  _pulse.mask = driveMask;
  _pulse.positive = positive;
  _pulse.pct = clampPct(pct);
  if (_pulse.pct > 20) _pulse.pct = 20; // pulse tests are intentionally low speed
  _pulse.durationMs = durationMs < 500 ? 500 : (durationMs > 3000 ? 3000 : durationMs);
  _pulse.startMs = nowMs;
  _pulse.lastPollMs = 0;
  if (!requestMask(driveMask, positive, _pulse.pct)) {
    _pulse.active = false;
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=QUEUE_START"));
    return false;
  }
  Serial.print(F("@SERVICE_PULSE state=START mask=0x")); Serial.print(driveMask, HEX);
  Serial.print(F(" dir=")); Serial.print(positive ? F("POS") : F("NEG"));
  Serial.print(F(" pct=")); Serial.print(_pulse.pct);
  Serial.print(F(" durationMs=")); Serial.println(_pulse.durationMs);
  return true;
}

void He200ServiceV6::finishPulse(bool pass, const __FlashStringHelper* reason) {
  const uint8_t mask = _pulse.mask;
  const bool positive = _pulse.positive;
  int32_t delta=0;
  const bool measured=maskIsSingle(mask) ? _calibration.finish(_pulse.drive,_pulse.forward,millis(),_pulse.stopMs,pass,delta) : pass;
  if (maskIsSingle(mask) && (!measured || (!_pulse.rawCalibration && (delta>0)!=positive))) {
    _calibration.revoke(_pulse.drive);
    _confirmPositiveMask &= (uint8_t)~mask; _confirmNegativeMask &= (uint8_t)~mask;
  } else if (_calibration.ready(_pulse.drive)) {
    _confirmPositiveMask |= mask; _confirmNegativeMask |= mask;
  }
  Serial.print(F("@CAL drive=")); Serial.print(Drives::driveName((DriveId)_pulse.drive));
  Serial.print(F(" command=")); Serial.print(_pulse.forward ? F("FWD") : F("REV"));
  Serial.print(F(" deltaMm=")); Serial.print(delta);
  Serial.print(F(" measured=")); Serial.print(measured ? 1 : 0);
  Serial.print(F(" ready=")); Serial.print(_calibration.ready(_pulse.drive) ? 1 : 0);
  Serial.print(F(" plusCommand=")); Serial.println(_calibration.ready(_pulse.drive) ?
      (_calibration.physicalSign(_pulse.drive,true)>0 ? F("FWD") : F("REV")) : F("UNKNOWN"));
  if (pass && !_pulse.rawCalibration) {
    if (positive) _pulsePositiveMask |= mask; else _pulseNegativeMask |= mask;
    if (mask == MASK_H) _pairPassMask |= 0x01;
    if (mask == MASK_V) _pairPassMask |= 0x02;
  }
  Serial.print(F("@SERVICE_PULSE state=DONE mask=0x")); Serial.print(mask, HEX);
  Serial.print(F(" dir=")); Serial.print(positive ? F("POS") : F("NEG"));
  Serial.print(F(" result=")); Serial.print(pass ? F("PASS") : F("FAIL"));
  if (reason) { Serial.print(F(" reason=")); Serial.print(reason); }
  Serial.println();
  _pulse.active = false;
  printGateStatus();
}

void He200ServiceV6::servicePulse(uint32_t nowMs, bool safetyBlocked) {
  if (!_pulse.active || !_motor) return;
  if (safetyBlocked) {
    _motor->serviceDecelStop(F("service pulse safety"));
    finishPulse(false, F("SAFETY"));
    return;
  }

  if (nowMs - _pulse.lastPollMs >= 250) {
    _pulse.lastPollMs = nowMs;
    pollMask(_pulse.mask);
  }

  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    const uint8_t bit = bitForDrive(i);
    if (!(_pulse.mask & bit)) continue;
    const DriveTelemetry& t = _motor->vfdTelemetry(i);
    if (t.connected && (int32_t)(t.lastOkMs-_pulse.startMs)>0 && (uint32_t)(nowMs-t.lastOkMs)<=1000 && t.runningFreq001Hz > 50 && t.statusWord != 0) _pulse.sawRunningMask |= bit;
  }

  if (!_pulse.stopIssued && (uint32_t)(nowMs - _pulse.startMs) >= _pulse.durationMs) {
    _motor->serviceDecelStopMask(_pulse.mask, F("service pulse timed stop"));
    _pulse.stopIssued = true;
    _pulse.stopMs = nowMs;
    Serial.println(F("@SERVICE_PULSE state=DECEL_STOP"));
  }

  if (_pulse.stopIssued && (uint32_t)(nowMs - _pulse.stopMs) >= 900) {
    pollMask(_pulse.mask);
    if (maskStopped(_pulse.mask)) {
      const bool ran = (_pulse.sawRunningMask & _pulse.mask) == _pulse.mask;
      finishPulse(ran, ran ? F("RUN_AND_STOP_VERIFIED") : F("NO_CONFIRMED_RUN"));
      return;
    }
  }

  if ((uint32_t)(nowMs - _pulse.startMs) > 7000) {
    _motor->serviceDecelStopMask(_pulse.mask, F("service pulse timeout"));
    finishPulse(false, F("TIMEOUT"));
  }
}

uint8_t He200ServiceV6::assistMask() const {
  return _assist.axis == AssistAxis::X ? MASK_H : MASK_V;
}

void He200ServiceV6::assistSensorIndices(uint8_t& a, uint8_t& b) const {
  if (_assist.axis == AssistAxis::X) { a = SENSOR_X1; b = SENSOR_X2; }
  else { a = SENSOR_Z1; b = SENSOR_Z2; }
}

void He200ServiceV6::noteFreshPair(const uint32_t ageMs[4], const uint32_t lastSampleMs[4]) {
  uint8_t a, b; assistSensorIndices(a, b);
  if (ageMs[a] > SENSOR_GUARD_WARNING_MS || ageMs[b] > SENSOR_GUARD_WARNING_MS) {
    _assist.freshPairCount = 0;
    _assist.freshSeenMask = 0;
    return;
  }
  if (lastSampleMs[a] && lastSampleMs[a] != _assist.seenSampleA) {
    _assist.seenSampleA = lastSampleMs[a];
    _assist.freshSeenMask |= 0x01;
  }
  if (lastSampleMs[b] && lastSampleMs[b] != _assist.seenSampleB) {
    _assist.seenSampleB = lastSampleMs[b];
    _assist.freshSeenMask |= 0x02;
  }
  if (_assist.freshSeenMask == 0x03) {
    if (_assist.freshPairCount < 255) ++_assist.freshPairCount;
    _assist.freshSeenMask = 0;
  }
}

const char* He200ServiceV6::assistStateName(AssistState state) {
  switch (state) {
    case AssistState::OFF: return "OFF";
    case AssistState::STARTING: return "STARTING";
    case AssistState::NORMAL: return "NORMAL";
    case AssistState::WARNING: return "WARNING";
    case AssistState::DECEL: return "DECEL";
    case AssistState::HOLD: return "HOLD";
    case AssistState::RECOVER: return "RECOVER";
    case AssistState::FAULT: return "FAULT";
  }
  return "?";
}

void He200ServiceV6::setAssistState(AssistState next, uint32_t nowMs,
                                    uint32_t ageA, uint32_t ageB) {
  if (_assist.state == next) return;
  _assist.state = next;
  _assist.lastPrintMs = nowMs;
  Serial.print(F("@ASSIST axis=")); Serial.print(_assist.axis == AssistAxis::X ? 'X' : 'Z');
  Serial.print(F(" state=")); Serial.print(assistStateName(next));
  Serial.print(F(" pct=")); Serial.print(_assist.currentPct);
  Serial.print(F(" ageA=")); Serial.print(ageA == UINT32_MAX ? 999999UL : ageA);
  Serial.print(F(" ageB=")); Serial.print(ageB == UINT32_MAX ? 999999UL : ageB);
  Serial.print(F(" freshPairs=")); Serial.println(_assist.freshPairCount);
}

void He200ServiceV6::applyAssistPct(uint8_t pct, uint32_t nowMs,
                                    uint32_t ageA, uint32_t ageB) {
  if (pct == _assist.currentPct) return;
  _assist.currentPct = pct;
  if (pct == 0) {
    _motor->serviceDecelStopMask(assistMask(), F("sensor assist hold"));
    _assist.stoppedBySupervisor = true;
  } else {
    (void)requestMask(assistMask(), _assist.positive, pct);
  }
  Serial.print(F("@ASSIST axis=")); Serial.print(_assist.axis == AssistAxis::X ? 'X' : 'Z');
  Serial.print(F(" state=")); Serial.print(assistStateName(_assist.state));
  Serial.print(F(" pct=")); Serial.print(_assist.currentPct);
  Serial.print(F(" ageA=")); Serial.print(ageA == UINT32_MAX ? 999999UL : ageA);
  Serial.print(F(" ageB=")); Serial.print(ageB == UINT32_MAX ? 999999UL : ageB);
  Serial.print(F(" decelEvents=")); Serial.print(_assist.decelEvents);
  Serial.print(F(" recoverEvents=")); Serial.println(_assist.recoverEvents);
  _assist.lastPrintMs = nowMs;
}

bool He200ServiceV6::startAssist(AssistAxis axis, bool positive, uint8_t basePct,
                                 uint32_t nowMs, const uint32_t ageMs[4],
                                 const uint32_t lastSampleMs[4]) {
  if (HE200_DIAGNOSTIC_LOCK) {
    Serial.println(F("@SERVICE_PULSE result=FAIL reason=DIAGNOSTIC_LOCK"));
    return false;
  }
  if (!_motor || _pulse.active || assistActive() || _motor->auditActive()) return false;
  _assist = AssistRuntime{};
  _assist.axis = axis;
  _assist.positive = positive;
  _assist.basePct = clampPct(basePct);
  const uint8_t mask = assistMask();
  const uint8_t pairBit = axis == AssistAxis::X ? 0x01 : 0x02;
  if (!protocolReadyForMask(mask) || !directionConfirmedForMask(mask, positive) || !(_pairPassMask & pairBit)) {
    Serial.println(F("@ASSIST state=REJECT reason=GATES"));
    _assist.state = AssistState::OFF;
    return false;
  }
  uint8_t a, b; assistSensorIndices(a, b);
  if (ageMs[a] > SENSOR_GUARD_WARNING_MS || ageMs[b] > SENSOR_GUARD_WARNING_MS ||
      lastSampleMs[a] == 0 || lastSampleMs[b] == 0) {
    Serial.println(F("@ASSIST state=REJECT reason=SENSORS_NOT_FRESH"));
    _assist.state = AssistState::OFF;
    return false;
  }
  _assist.seenSampleA = lastSampleMs[a];
  _assist.seenSampleB = lastSampleMs[b];
  _assist.currentPct = _assist.basePct < 10 ? _assist.basePct : 10;
  _assist.state = AssistState::STARTING;
  _assist.lastRampMs = nowMs;
  if (!requestMask(mask, positive, _assist.currentPct)) {
    _assist.state = AssistState::OFF;
    return false;
  }
  Serial.print(F("@ASSIST axis=")); Serial.print(axis == AssistAxis::X ? 'X' : 'Z');
  Serial.print(F(" state=STARTING dir=")); Serial.print(positive ? F("POS") : F("NEG"));
  Serial.print(F(" basePct=")); Serial.print(_assist.basePct);
  Serial.print(F(" pct=")); Serial.println(_assist.currentPct);
  return true;
}

void He200ServiceV6::serviceAssist(uint32_t nowMs, bool safetyBlocked,
                                   const uint32_t ageMs[4], const uint32_t lastSampleMs[4]) {
  if (!_motor || _assist.state == AssistState::OFF || _assist.state == AssistState::FAULT) return;
  uint8_t a, b; assistSensorIndices(a, b);
  const uint32_t ageA = ageMs[a];
  const uint32_t ageB = ageMs[b];
  const uint32_t worst = ageA > ageB ? ageA : ageB;
  noteFreshPair(ageMs, lastSampleMs);

  if (safetyBlocked) {
    _motor->serviceDecelStop(F("sensor assist safety"));
    _assist.currentPct = 0;
    setAssistState(AssistState::FAULT, nowMs, ageA, ageB);
    return;
  }
  if (worst > SENSOR_ASSIST_FAULT_MS) {
    _motor->serviceDecelStopMask(assistMask(), F("sensor assist timeout fault"));
    _assist.currentPct = 0;
    setAssistState(AssistState::FAULT, nowMs, ageA, ageB);
    return;
  }

  if ((uint32_t)(nowMs - _assist.lastRampMs) < SENSOR_ASSIST_RAMP_PERIOD_MS) return;
  _assist.lastRampMs = nowMs;

  if (worst > SENSOR_ASSIST_HOLD_MS) {
    if (_assist.state != AssistState::HOLD) { ++_assist.holdEvents; setAssistState(AssistState::HOLD, nowMs, ageA, ageB); }
    if (_assist.currentPct > 0) {
      const uint8_t next = _assist.currentPct > SENSOR_ASSIST_RAMP_STEP_PCT
          ? (uint8_t)(_assist.currentPct - SENSOR_ASSIST_RAMP_STEP_PCT) : 0;
      applyAssistPct(next, nowMs, ageA, ageB);
    }
    return;
  }

  if (worst > SENSOR_ASSIST_DECEL_START_MS) {
    if (_assist.state != AssistState::DECEL) { ++_assist.decelEvents; setAssistState(AssistState::DECEL, nowMs, ageA, ageB); }
    const uint8_t floorPct = SENSOR_ASSIST_MIN_RUNNING_PCT;
    if (_assist.currentPct > floorPct) {
      uint8_t next = _assist.currentPct > SENSOR_ASSIST_RAMP_STEP_PCT
          ? (uint8_t)(_assist.currentPct - SENSOR_ASSIST_RAMP_STEP_PCT) : floorPct;
      if (next < floorPct) next = floorPct;
      applyAssistPct(next, nowMs, ageA, ageB);
    }
    return;
  }

  if (worst > SENSOR_GUARD_WARNING_MS) {
    setAssistState(AssistState::WARNING, nowMs, ageA, ageB);
    return;
  }

  const uint8_t requiredFresh = _assist.stoppedBySupervisor
      ? SENSOR_ASSIST_RECOVERY_STOPPED_FRAMES : SENSOR_ASSIST_RECOVERY_MOVING_FRAMES;
  if ((_assist.state == AssistState::DECEL || _assist.state == AssistState::HOLD ||
       _assist.state == AssistState::WARNING || _assist.state == AssistState::RECOVER) &&
      _assist.freshPairCount < requiredFresh) {
    setAssistState(AssistState::RECOVER, nowMs, ageA, ageB);
    return;
  }

  if (_assist.state != AssistState::NORMAL && _assist.state != AssistState::STARTING) ++_assist.recoverEvents;
  setAssistState(AssistState::NORMAL, nowMs, ageA, ageB);
  if (_assist.stoppedBySupervisor && _assist.currentPct == 0) {
    _assist.currentPct = SENSOR_ASSIST_MIN_RUNNING_PCT;
    _assist.stoppedBySupervisor = false;
    (void)requestMask(assistMask(), _assist.positive, _assist.currentPct);
    _assist.freshPairCount = 0;
    return;
  }
  if (_assist.currentPct < _assist.basePct) {
    uint8_t next = (uint8_t)(_assist.currentPct + SENSOR_ASSIST_RAMP_STEP_PCT);
    if (next > _assist.basePct) next = _assist.basePct;
    applyAssistPct(next, nowMs, ageA, ageB);
  }
}

void He200ServiceV6::service(uint32_t nowMs, bool safetyBlocked,
                             const uint32_t ageMs[4], const uint32_t lastSampleMs[4]) {
  servicePulse(nowMs, safetyBlocked);
  serviceAssist(nowMs, safetyBlocked, ageMs, lastSampleMs);
}

void He200ServiceV6::stopAll(const __FlashStringHelper* reason) {
  if (_motor) _motor->serviceDecelStop(reason);
  _pulse.active = false;
  if (_assist.state != AssistState::OFF) {
    _assist.currentPct = 0;
    _assist.state = AssistState::OFF;
    Serial.println(F("@ASSIST state=OFF reason=SERVICE_STOP"));
  }
}

void He200ServiceV6::stopAssist(const __FlashStringHelper* reason) {
  if (!_motor || _assist.state == AssistState::OFF) return;
  const uint8_t mask = assistMask();
  _motor->serviceDecelStopMask(mask, reason);
  _assist.currentPct = 0;
  _assist.state = AssistState::OFF;
  Serial.print(F("@ASSIST axis=")); Serial.print(_assist.axis == AssistAxis::X ? 'X' : 'Z');
  Serial.println(F(" state=OFF reason=ASSIST_STOP"));
}

void He200ServiceV6::clearAssistFault() {
  if (_assist.state == AssistState::FAULT) {
    _assist = AssistRuntime{};
    Serial.println(F("@ASSIST state=OFF reason=FAULT_CLEARED"));
  }
}

void He200ServiceV6::observeSensors(const int32_t rawMm[4], const uint32_t sampleMs[4]) {
  for (uint8_t i=0;i<4;++i) _calibration.samples[i].add(rawMm[i],sampleMs[i]);
}
