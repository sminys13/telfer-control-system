#include "auto_runner_v6.h"
#include "safety_v6.h"

static constexpr uint32_t AUTO_MOVE_TIMEOUT_MS_V6 = 180000UL;
static constexpr uint16_t AUTO_SIM_MM_PER_S_100_V6 = 4000;

static inline int32_t abs32v6(int32_t v) { return v < 0 ? -v : v; }
static inline uint8_t minU8v6(uint8_t a, uint8_t b) { return a < b ? a : b; }

void AutoRunnerV6::begin(MotorControlV6& motor, const SettingsV6& settings) {
  _motor = &motor;
  _settings = &settings;
  resetRuntime();
}

void AutoRunnerV6::resetRuntime() {
  _phase = Phase::IDLE;
  _running = false;
  _paused = false;
  _simulation = false;
  _waitOperator = false;
  _operatorNext = false;
  _dryAlarm = false;
  _stateChanged = false;
  _homeOnly = false;
  _motorMode = SystemModeV6::AUTO;
  _error = AUTO_ERR_NONE;
  _orderIndex = 0;
  _zoneIndex = 0;
  _cycleZoneOrdinal = 0;
  _lowSide = 0;
  _stagingZone = 0;
  _baseZ[0] = _baseZ[1] = 0;
  _lowTiltTarget = 0;
  _highLiftTarget = 0;
  _runStartMs = 0;
  _runEndMs = 0;
  _phaseStartMs = 0;
  _pauseStartMs = 0;
  _pausedAccumMs = 0;
  _dryTimerStartMs = 0;
  _simLastMs = 0;
  _knownTimedSeconds = 0;
  _enabledZoneCount = 0;
}

const __FlashStringHelper* AutoRunnerV6::phaseName() const {
  switch (_phase) {
    case Phase::IDLE: return F("IDLE");
    case Phase::PREP_TRAVEL: return F("PREP_TRAVEL");
    case Phase::MOVE_ZONE_H: return F("MOVE_ZONE_H");
    case Phase::LOWER_TILT: return F("LOWER_TILT");
    case Phase::WAIT_AFTER_TILT: return F("WAIT_AFTER_TILT");
    case Phase::LOWER_BASE: return F("LOWER_BASE");
    case Phase::WAIT_DIP: return F("WAIT_DIP");
    case Phase::RAISE_TILT_HIGH: return F("RAISE_TILT_HIGH");
    case Phase::WAIT_DRAIN: return F("WAIT_DRAIN");
    case Phase::RAISE_TRAVEL: return F("RAISE_TRAVEL");
    case Phase::NEXT_ZONE: return F("NEXT_ZONE");
    case Phase::DRY_GOTO_STAGING: return F("DRY_GOTO_STAGING");
    case Phase::DRY_WAIT_OPEN: return F("DRY_WAIT_OPEN");
    case Phase::DRY_GOTO_DRY: return F("DRY_GOTO_DRY");
    case Phase::DRY_LOWER_DROP: return F("DRY_LOWER_DROP");
    case Phase::DRY_WAIT_DETACH: return F("DRY_WAIT_DETACH");
    case Phase::DRY_RAISE_TRAVEL: return F("DRY_RAISE_TRAVEL");
    case Phase::DRY_WAIT_START: return F("DRY_WAIT_START");
    case Phase::DRY_WAIT_TIMER: return F("DRY_WAIT_TIMER");
    case Phase::DRY_LOWER_PICK: return F("DRY_LOWER_PICK");
    case Phase::DRY_WAIT_ATTACH: return F("DRY_WAIT_ATTACH");
    case Phase::DRY_RAISE_TRAVEL2: return F("DRY_RAISE_TRAVEL2");
    case Phase::DRY_GOTO_STAGING2: return F("DRY_GOTO_STAGING2");
    case Phase::DRY_WAIT_CLOSE: return F("DRY_WAIT_CLOSE");
    case Phase::MOVE_HOME_Z: return F("MOVE_HOME_Z");
    case Phase::MOVE_HOME_X: return F("MOVE_HOME_X");
    case Phase::DONE: return F("DONE");
    case Phase::FAULT: return F("FAULT");
  }
  return F("?");
}

void AutoRunnerV6::setPhase(Phase p, uint32_t nowMs) {
  if (_phase == p) return;
  _phase = p;
  _stateChanged = true;
  _phaseStartMs = nowMs;
  _waitOperator = false;
  _operatorNext = false;
  Serial.print(F("AUTO PHASE -> "));
  Serial.print((uint8_t)_phase);
  Serial.print(' ');
  Serial.println(phaseName());
}

void AutoRunnerV6::seedSimulation(const AutoProgramV6& program, const AutoSensorsV6& sensors) {
  // Use live coordinates when all four are available; otherwise start at HOME/TRAVEL.
  if ((sensors.usableMask & 0x0F) == 0x0F) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) _simPos[i] = sensors.mm[i];
  } else {
    _simPos[SENSOR_X1] = program.homeX[0];
    _simPos[SENSOR_X2] = program.homeX[1];
    _simPos[SENSOR_Z1] = program.travelZ[0];
    _simPos[SENSOR_Z2] = program.travelZ[1];
  }
  _simLastMs = millis();
}

bool AutoRunnerV6::realSensorsUsable(const AutoSensorsV6& sensors) const {
  return (sensors.usableMask & 0x0F) == 0x0F;
}

int32_t AutoRunnerV6::pos(const AutoSensorsV6& sensors, SensorIndex idx) const {
  return _simulation ? _simPos[idx] : sensors.mm[idx];
}

bool AutoRunnerV6::start(const AutoProgramV6& program, const AutoSensorsV6& sensors,
                         bool simulation, uint32_t nowMs) {
  resetRuntime();
  _program = &program;
  _simulation = simulation;
  _homeOnly = false;
  _motorMode = SystemModeV6::AUTO;
  _lowSide = program.lowSide > 1 ? 0 : program.lowSide;
  _stagingZone = program.stagingZone < program.zoneCount ? program.stagingZone : 0;

  ProgramStorageV6 validator;
  if (!validator.readyForAuto(program)) {
    fail(AUTO_ERR_PROGRAM, F("program not calibrated/valid"));
    return false;
  }
  if (!simulation && !realSensorsUsable(sensors)) {
    fail(AUTO_ERR_SENSORS, F("X1/X2/Z1/Z2 are not all usable"));
    return false;
  }
  if (!simulation && (!VFD_RS485_ENABLED || !VFD_WRITE_COMMANDS_ENABLED)) {
    fail(VFD_RS485_ENABLED ? AUTO_ERR_READONLY : AUTO_ERR_OUTPUT_DISABLED,
         VFD_RS485_ENABLED ? F("physical RS485 build is READ-ONLY") : F("physical RS485 output disabled; use SIM"));
    return false;
  }
  if (!simulation && !AUTO_PHYSICAL_ENABLED) {
    fail(AUTO_ERR_OUTPUT_DISABLED, F("physical AUTO/HOME interlock disabled in this build"));
    return false;
  }
  if (simulation && VFD_RS485_ENABLED && VFD_WRITE_COMMANDS_ENABLED) {
    fail(AUTO_ERR_OUTPUT_DISABLED, F("SIM is forbidden in physical WRITE build"));
    return false;
  }

  _knownTimedSeconds = validator.knownTimedSeconds(program);
  _enabledZoneCount = validator.enabledZoneCount(program);
  _runStartMs = nowMs;
  _phaseStartMs = nowMs;
  _running = true;
  if (simulation) seedSimulation(program, sensors);

  _orderIndex = 0;
  if (!advanceToEnabledZone()) {
    fail(AUTO_ERR_PROGRAM, F("program has no enabled zone"));
    return false;
  }
  setPhase(Phase::PREP_TRAVEL, nowMs);
  Serial.print(F("AUTO START program="));
  Serial.print(program.name);
  Serial.print(F(" mode="));
  Serial.println(simulation ? F("SIMULATION") : F("PHYSICAL"));
  return true;
}

bool AutoRunnerV6::startHome(const AutoProgramV6& program, const AutoSensorsV6& sensors,
                             bool simulation, uint32_t nowMs) {
  resetRuntime();
  _program = &program;
  _simulation = simulation;
  _homeOnly = true;
  _motorMode = SystemModeV6::HOME;
  const uint8_t required = AUTO_PROGRAM_VALID_HOME_X | AUTO_PROGRAM_VALID_TRAVEL_Z;
  if ((program.validMask & required) != required) {
    fail(AUTO_ERR_PROGRAM, F("HOME/TRAVEL not calibrated"));
    return false;
  }
  if (!simulation && !realSensorsUsable(sensors)) {
    fail(AUTO_ERR_SENSORS, F("X1/X2/Z1/Z2 are not all usable"));
    return false;
  }
  if (!simulation && (!VFD_RS485_ENABLED || !VFD_WRITE_COMMANDS_ENABLED)) {
    fail(VFD_RS485_ENABLED ? AUTO_ERR_READONLY : AUTO_ERR_OUTPUT_DISABLED,
         VFD_RS485_ENABLED ? F("physical RS485 build is READ-ONLY") : F("physical RS485 output disabled; use SIM"));
    return false;
  }
  if (!simulation && !AUTO_PHYSICAL_ENABLED) {
    fail(AUTO_ERR_OUTPUT_DISABLED, F("physical AUTO/HOME interlock disabled in this build"));
    return false;
  }
  if (simulation && VFD_RS485_ENABLED && VFD_WRITE_COMMANDS_ENABLED) {
    fail(AUTO_ERR_OUTPUT_DISABLED, F("SIM is forbidden in physical WRITE build"));
    return false;
  }

  _runStartMs = nowMs;
  _phaseStartMs = nowMs;
  _running = true;
  if (simulation) seedSimulation(program, sensors);
  setPhase(Phase::MOVE_HOME_Z, nowMs);
  Serial.println(simulation ? F("HOME START SIMULATION") : F("HOME START PHYSICAL"));
  return true;
}

void AutoRunnerV6::fail(AutoErrorV6 err, const __FlashStringHelper* reason) {
  _error = err;
  _running = false;
  _paused = false;
  _waitOperator = false;
  _phase = Phase::FAULT;
  _runEndMs = millis();
  _stateChanged = true;
  stopMotion();
  Serial.print(F("AUTO FAULT code="));
  Serial.print((uint16_t)err);
  Serial.print(F(" reason="));
  Serial.println(reason ? reason : F("?"));
}

void AutoRunnerV6::stop(const __FlashStringHelper* reason) {
  if (_running || _phase != Phase::IDLE) {
    Serial.print(F("AUTO STOP"));
    if (reason) { Serial.print(F(" reason=")); Serial.print(reason); }
    Serial.println();
  }
  stopMotion();
  resetRuntime();
}

void AutoRunnerV6::pause() {
  if (!_running || _paused) return;
  _paused = true;
  _pauseStartMs = millis();
  stopMotion();
  Serial.println(F("AUTO PAUSED"));
}

void AutoRunnerV6::resume(uint32_t nowMs) {
  if (!_running || !_paused) return;
  _paused = false;
  const uint32_t pauseDur = nowMs - _pauseStartMs;
  _pausedAccumMs += pauseDur;
  _phaseStartMs += pauseDur;
  if (_dryTimerStartMs) _dryTimerStartMs += pauseDur;
  _simLastMs = nowMs;
  Serial.println(F("AUTO RESUMED"));
}

void AutoRunnerV6::operatorNext() {
  if (!_running) return;
  _operatorNext = true;
  Serial.println(F("AUTO OPERATOR NEXT"));
}

bool AutoRunnerV6::advanceToEnabledZone() {
  if (!_program) return false;
  while (_orderIndex < _program->zoneCount) {
    const uint8_t zid = _program->order[_orderIndex];
    if (zid < _program->zoneCount && _program->zones[zid].enabled) {
      _zoneIndex = zid;
      return true;
    }
    ++_orderIndex;
  }
  return false;
}

bool AutoRunnerV6::checkMovementTimeout(uint32_t nowMs) {
  // A command can start a phase between two timestamp samples in the main loop.
  // If the caller supplies a timestamp only a few ms older than phaseStart,
  // unsigned subtraction would look like an almost-49-day elapsed interval.
  // Treat only a small backwards delta as a stale sample. A real millis() wrap
  // still produces the correct small unsigned elapsed value below.
  if (nowMs < _phaseStartMs && (uint32_t)(_phaseStartMs - nowMs) <= 1000UL)
    return false;

  if ((uint32_t)(nowMs - _phaseStartMs) <= AUTO_MOVE_TIMEOUT_MS_V6) return false;
  fail(AUTO_ERR_MOVE_TIMEOUT, F("movement phase timeout"));
  return true;
}

int16_t AutoRunnerV6::targetPct(SensorIndex idx, int32_t cur, int32_t target, uint8_t capPct) const {
  if (!_settings) return 0;
  const uint8_t drive = (idx == SENSOR_X1) ? DRIVE_H1 :
                        (idx == SENSOR_X2) ? DRIVE_H2 :
                        (idx == SENSOR_Z1) ? DRIVE_V1 : DRIVE_V2;
  const DriveProfileV6& p = _settings->drive[drive];
  const int32_t diff = target - cur;
  if (abs32v6(diff) <= (int32_t)p.stopToleranceMm) return 0;
  uint8_t cap = capPct;
  if (cap < 1 || cap > 100) cap = p.maxPercent;
  cap = minU8v6(cap, p.maxPercent);
  uint8_t use = cap;
  if (abs32v6(diff) <= (int32_t)p.slowdownDistanceMm) use = minU8v6(p.slowPercent, cap);
  if (use < 1) use = 1;
  return diff > 0 ? (int16_t)use : (int16_t)-use;
}

void AutoRunnerV6::integrateSimulation(uint32_t nowMs, int16_t h1, int16_t h2, int16_t v1Up, int16_t v2Up) {
  if (!_simulation) return;
  if (_simLastMs == 0) _simLastMs = nowMs;
  uint32_t dt = nowMs - _simLastMs;
  if (dt > 200) dt = 200;
  _simLastMs = nowMs;
  const int16_t pct[SENSOR_COUNT] = {h1, h2, v1Up, v2Up};
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (pct[i] == 0) continue;
    int32_t step = (int32_t)AUTO_SIM_MM_PER_S_100_V6 * (int32_t)abs(pct[i]) * (int32_t)dt / 100000L;
    if (step < 1) step = 1;
    _simPos[i] += pct[i] > 0 ? step : -step;
  }
}

void AutoRunnerV6::commandTargets(int16_t h1, int16_t h2, int16_t v1Up, int16_t v2Up) {
  if (_simulation) return;
  if (_motor) (void)_motor->requestAutoTargets(_motorMode, h1, h2, v1Up, v2Up);
}

void AutoRunnerV6::stopMotion() {
  if (_simulation) return;
  if (_motor) _motor->autoStop(F("auto phase stop"));
}

bool AutoRunnerV6::moveHorizontal(uint32_t nowMs, const AutoSensorsV6& sensors,
                                  int32_t x1, int32_t x2, uint8_t capPct, uint8_t limitMask) {
  if (checkMovementTimeout(nowMs)) return false;
  int16_t h1 = targetPct(SENSOR_X1, pos(sensors, SENSOR_X1), x1, capPct);
  int16_t h2 = targetPct(SENSOR_X2, pos(sensors, SENSOR_X2), x2, capPct);

  if ((h1 > 0 && (limitMask & SafetyV6::LIM_H1_RIGHT)) ||
      (h1 < 0 && (limitMask & SafetyV6::LIM_H1_LEFT)) ||
      (h2 > 0 && (limitMask & SafetyV6::LIM_H2_RIGHT)) ||
      (h2 < 0 && (limitMask & SafetyV6::LIM_H2_LEFT))) {
    fail(AUTO_ERR_LIMIT, F("horizontal limit blocks automatic direction"));
    return false;
  }

  integrateSimulation(nowMs, h1, h2, 0, 0);
  // Re-evaluate after the virtual movement so simulation cannot oscillate around the target.
  h1 = targetPct(SENSOR_X1, pos(sensors, SENSOR_X1), x1, capPct);
  h2 = targetPct(SENSOR_X2, pos(sensors, SENSOR_X2), x2, capPct);
  commandTargets(h1, h2, 0, 0);
  return h1 == 0 && h2 == 0;
}

bool AutoRunnerV6::moveVertical(uint32_t nowMs, const AutoSensorsV6& sensors,
                                int32_t z1, int32_t z2, uint8_t capPct) {
  if (checkMovementTimeout(nowMs)) return false;
  int16_t v1 = targetPct(SENSOR_Z1, pos(sensors, SENSOR_Z1), z1, capPct);
  int16_t v2 = targetPct(SENSOR_Z2, pos(sensors, SENSOR_Z2), z2, capPct);
  integrateSimulation(nowMs, 0, 0, v1, v2);
  v1 = targetPct(SENSOR_Z1, pos(sensors, SENSOR_Z1), z1, capPct);
  v2 = targetPct(SENSOR_Z2, pos(sensors, SENSOR_Z2), z2, capPct);
  commandTargets(0, 0, v1, v2);
  return v1 == 0 && v2 == 0;
}

bool AutoRunnerV6::moveOneVertical(uint32_t nowMs, const AutoSensorsV6& sensors,
                                   uint8_t side, int32_t target, uint8_t capPct) {
  if (checkMovementTimeout(nowMs)) return false;
  const SensorIndex idx = side == 0 ? SENSOR_Z1 : SENSOR_Z2;
  int16_t v = targetPct(idx, pos(sensors, idx), target, capPct);
  integrateSimulation(nowMs, 0, 0, side == 0 ? v : 0, side == 1 ? v : 0);
  v = targetPct(idx, pos(sensors, idx), target, capPct);
  commandTargets(0, 0, side == 0 ? v : 0, side == 1 ? v : 0);
  return v == 0;
}

uint32_t AutoRunnerV6::elapsedSeconds(uint32_t nowMs) const {
  if (_runStartMs == 0) return 0;
  const uint32_t effectiveNow = (!_running && _runEndMs != 0) ? _runEndMs : nowMs;
  uint32_t paused = _pausedAccumMs;
  if (_paused) paused += effectiveNow - _pauseStartMs;
  const uint32_t total = effectiveNow - _runStartMs;
  return total > paused ? (total - paused) / 1000UL : 0;
}

uint16_t AutoRunnerV6::remainingWaitSeconds(uint32_t nowMs) const {
  if (!_program || !_running) return 0xFFFF;
  uint32_t total = 0;
  switch (_phase) {
    case Phase::WAIT_AFTER_TILT: total = (uint32_t)_program->zones[_zoneIndex].stepWaitS * 1000UL; break;
    case Phase::WAIT_DIP: total = (uint32_t)_program->zones[_zoneIndex].dipTimeS * 1000UL; break;
    case Phase::WAIT_DRAIN: total = (uint32_t)_program->dripWaitS * 1000UL; break;
    case Phase::DRY_WAIT_TIMER: total = (uint32_t)_program->dryingTimeS * 1000UL; break;
    default: return 0xFFFF;
  }
  const uint32_t started = (_phase == Phase::DRY_WAIT_TIMER && _dryTimerStartMs) ? _dryTimerStartMs : _phaseStartMs;
  const uint32_t effectiveNow = _paused ? _pauseStartMs : nowMs;
  const uint32_t elapsed = effectiveNow - started;
  if (elapsed >= total) return 0;
  const uint32_t rem = (total - elapsed + 999UL) / 1000UL;
  return rem > 65535UL ? 65535 : (uint16_t)rem;
}

uint16_t AutoRunnerV6::currentStep() const {
  if (!_running && _phase != Phase::DONE) return 0;
  if (_homeOnly) {
    if (_phase == Phase::MOVE_HOME_Z) return 1;
    if (_phase == Phase::MOVE_HOME_X || _phase == Phase::DONE) return 2;
    return 0;
  }

  // Step 1 is the initial move to transport height. Each enabled process zone
  // then owns exactly nine steps. Disabled zones do not create holes in DWIN.
  const uint16_t zoneBase = 1u + (uint16_t)_cycleZoneOrdinal * 9u;
  const uint16_t afterZones = 1u + (uint16_t)_enabledZoneCount * 9u;
  switch (_phase) {
    case Phase::PREP_TRAVEL: return 1;
    case Phase::MOVE_ZONE_H: return zoneBase + 1;
    case Phase::LOWER_TILT: return zoneBase + 2;
    case Phase::WAIT_AFTER_TILT: return zoneBase + 3;
    case Phase::LOWER_BASE: return zoneBase + 4;
    case Phase::WAIT_DIP: return zoneBase + 5;
    case Phase::RAISE_TILT_HIGH: return zoneBase + 6;
    case Phase::WAIT_DRAIN: return zoneBase + 7;
    case Phase::RAISE_TRAVEL: return zoneBase + 8;
    case Phase::NEXT_ZONE: return zoneBase + 9;
    case Phase::DRY_GOTO_STAGING: return afterZones + 1;
    case Phase::DRY_WAIT_OPEN: return afterZones + 2;
    case Phase::DRY_GOTO_DRY: return afterZones + 3;
    case Phase::DRY_LOWER_DROP: return afterZones + 4;
    case Phase::DRY_WAIT_DETACH: return afterZones + 5;
    case Phase::DRY_RAISE_TRAVEL: return afterZones + 6;
    case Phase::DRY_WAIT_START: return afterZones + 7;
    case Phase::DRY_WAIT_TIMER: return afterZones + 8;
    case Phase::DRY_LOWER_PICK: return afterZones + 9;
    case Phase::DRY_WAIT_ATTACH: return afterZones + 10;
    case Phase::DRY_RAISE_TRAVEL2: return afterZones + 11;
    case Phase::DRY_GOTO_STAGING2: return afterZones + 12;
    case Phase::DRY_WAIT_CLOSE: return afterZones + 13;
    case Phase::MOVE_HOME_Z: return totalSteps() - 1;
    case Phase::MOVE_HOME_X:
    case Phase::DONE: return totalSteps();
    default: return 0;
  }
}

uint16_t AutoRunnerV6::totalSteps() const {
  if (_homeOnly) return 2;
  uint16_t n = 1u + (uint16_t)_enabledZoneCount * 9u; // PREP + process zones
  if (_program && _program->dryingEnabled) n += 13;
  n += 2; // HOME: transport Z then horizontal X
  return n;
}

bool AutoRunnerV6::service(uint32_t nowMs, const AutoSensorsV6& sensors,
                           bool estopBlocked, uint8_t limitMask) {
  _stateChanged = false;
  if (!_running) return false;
  if (_paused) { stopMotion(); return false; }
  if (estopBlocked) { fail(AUTO_ERR_ESTOP, F("E-STOP active/latched")); return true; }
  if (!_simulation && !realSensorsUsable(sensors)) { fail(AUTO_ERR_SENSORS, F("sensor became LOST")); return true; }
  if (!_program) { fail(AUTO_ERR_PROGRAM, F("program pointer missing")); return true; }

  const AutoZoneV6& z = _program->zones[_zoneIndex];
  const uint8_t low = _lowSide;
  const uint8_t high = low ? 0 : 1;

  switch (_phase) {
    case Phase::PREP_TRAVEL:
      if (moveVertical(nowMs, sensors, _program->travelZ[0], _program->travelZ[1], 0)) {
        stopMotion(); setPhase(Phase::MOVE_ZONE_H, nowMs);
      }
      break;

    case Phase::MOVE_ZONE_H:
      if (moveHorizontal(nowMs, sensors, z.xMm[0], z.xMm[1], z.movePercent, limitMask)) {
        stopMotion();
        _baseZ[0] = z.zMm[0]; _baseZ[1] = z.zMm[1];
        const SensorIndex highIdx = high == 0 ? SENSOR_Z1 : SENSOR_Z2;
        const int32_t highPos = pos(sensors, highIdx);
        // Step9D semantics: tiltStepMm is the requested DIFFERENCE |Z1-Z2|,
        // not "move one side by N mm". The low side moves down until its
        // coordinate is N mm below the opposite side, limited by final bath Z.
        int32_t intermediate = highPos - (int32_t)z.tiltStepMm;
        if (intermediate < _baseZ[low]) intermediate = _baseZ[low];
        _lowTiltTarget = intermediate;
        setPhase(Phase::LOWER_TILT, nowMs);
      }
      break;

    case Phase::LOWER_TILT:
      if (moveOneVertical(nowMs, sensors, low, _lowTiltTarget, _program->tiltPercent)) {
        stopMotion(); setPhase(Phase::WAIT_AFTER_TILT, nowMs);
      }
      break;

    case Phase::WAIT_AFTER_TILT:
      stopMotion();
      if ((uint32_t)(nowMs - _phaseStartMs) >= (uint32_t)z.stepWaitS * 1000UL)
        setPhase(Phase::LOWER_BASE, nowMs);
      break;

    case Phase::LOWER_BASE:
      if (moveVertical(nowMs, sensors, _baseZ[0], _baseZ[1], z.verticalPercent)) {
        stopMotion(); setPhase(Phase::WAIT_DIP, nowMs);
      }
      break;

    case Phase::WAIT_DIP:
      stopMotion();
      if ((uint32_t)(nowMs - _phaseStartMs) >= (uint32_t)z.dipTimeS * 1000UL) {
        const SensorIndex lowIdx = low == 0 ? SENSOR_Z1 : SENSOR_Z2;
        // Drain tilt uses the same direct Z1/Z2 differential: raise the high side
        // until it is N mm above the low side, limited by transport height.
        int32_t intermediate = pos(sensors, lowIdx) + (int32_t)z.tiltStepMm;
        if (intermediate > _program->travelZ[high]) intermediate = _program->travelZ[high];
        _highLiftTarget = intermediate;
        setPhase(Phase::RAISE_TILT_HIGH, nowMs);
      }
      break;

    case Phase::RAISE_TILT_HIGH:
      if (moveOneVertical(nowMs, sensors, high, _highLiftTarget, _program->tiltPercent)) {
        stopMotion(); setPhase(Phase::WAIT_DRAIN, nowMs);
      }
      break;

    case Phase::WAIT_DRAIN:
      stopMotion();
      if ((uint32_t)(nowMs - _phaseStartMs) >= (uint32_t)_program->dripWaitS * 1000UL)
        setPhase(Phase::RAISE_TRAVEL, nowMs);
      break;

    case Phase::RAISE_TRAVEL:
      if (moveVertical(nowMs, sensors, _program->travelZ[0], _program->travelZ[1], 0)) {
        stopMotion(); setPhase(Phase::NEXT_ZONE, nowMs);
      }
      break;

    case Phase::NEXT_ZONE:
      ++_cycleZoneOrdinal;
      ++_orderIndex;
      if (advanceToEnabledZone()) setPhase(Phase::MOVE_ZONE_H, nowMs);
      else if (_program->dryingEnabled) setPhase(Phase::DRY_GOTO_STAGING, nowMs);
      else setPhase(Phase::MOVE_HOME_Z, nowMs);
      break;

    case Phase::DRY_GOTO_STAGING: {
      const AutoZoneV6& stg = _program->zones[_stagingZone];
      if (!moveVertical(nowMs, sensors, _program->travelZ[0], _program->travelZ[1], 0)) break;
      if (moveHorizontal(nowMs, sensors, stg.xMm[0], stg.xMm[1], stg.movePercent, limitMask)) {
        stopMotion(); setPhase(Phase::DRY_WAIT_OPEN, nowMs);
      }
    } break;

    case Phase::DRY_WAIT_OPEN:
      stopMotion(); _waitOperator = true;
      if (_operatorNext) setPhase(Phase::DRY_GOTO_DRY, nowMs);
      break;

    case Phase::DRY_GOTO_DRY:
      if (moveHorizontal(nowMs, sensors, _program->dryX[0], _program->dryX[1], 0, limitMask)) {
        stopMotion(); setPhase(Phase::DRY_LOWER_DROP, nowMs);
      }
      break;

    case Phase::DRY_LOWER_DROP:
      if (moveVertical(nowMs, sensors, _program->dryZ[0], _program->dryZ[1], 0)) {
        stopMotion(); setPhase(Phase::DRY_WAIT_DETACH, nowMs);
      }
      break;

    case Phase::DRY_WAIT_DETACH:
      stopMotion(); _waitOperator = true;
      if (_operatorNext) setPhase(Phase::DRY_RAISE_TRAVEL, nowMs);
      break;

    case Phase::DRY_RAISE_TRAVEL:
      if (moveVertical(nowMs, sensors, _program->travelZ[0], _program->travelZ[1], 0)) {
        stopMotion(); setPhase(Phase::DRY_WAIT_START, nowMs);
      }
      break;

    case Phase::DRY_WAIT_START:
      stopMotion(); _waitOperator = true;
      if (_operatorNext) {
        _dryTimerStartMs = nowMs;
        _dryAlarm = false;
        setPhase(Phase::DRY_WAIT_TIMER, nowMs);
      }
      break;

    case Phase::DRY_WAIT_TIMER:
      stopMotion();
      if (_program->dryingTimeS == 0) {
        _waitOperator = true;
        if (_operatorNext) setPhase(Phase::DRY_LOWER_PICK, nowMs);
      } else if ((uint32_t)(nowMs - _dryTimerStartMs) >= (uint32_t)_program->dryingTimeS * 1000UL) {
        _dryAlarm = true;
        _waitOperator = true;
        if (_operatorNext) { _dryAlarm = false; setPhase(Phase::DRY_LOWER_PICK, nowMs); }
      }
      break;

    case Phase::DRY_LOWER_PICK:
      if (moveVertical(nowMs, sensors, _program->dryZ[0], _program->dryZ[1], 0)) {
        stopMotion(); setPhase(Phase::DRY_WAIT_ATTACH, nowMs);
      }
      break;

    case Phase::DRY_WAIT_ATTACH:
      stopMotion(); _waitOperator = true;
      if (_operatorNext) setPhase(Phase::DRY_RAISE_TRAVEL2, nowMs);
      break;

    case Phase::DRY_RAISE_TRAVEL2:
      if (moveVertical(nowMs, sensors, _program->travelZ[0], _program->travelZ[1], 0)) {
        stopMotion(); setPhase(Phase::DRY_GOTO_STAGING2, nowMs);
      }
      break;

    case Phase::DRY_GOTO_STAGING2: {
      const AutoZoneV6& stg = _program->zones[_stagingZone];
      if (moveHorizontal(nowMs, sensors, stg.xMm[0], stg.xMm[1], stg.movePercent, limitMask)) {
        stopMotion(); setPhase(Phase::DRY_WAIT_CLOSE, nowMs);
      }
    } break;

    case Phase::DRY_WAIT_CLOSE:
      stopMotion(); _waitOperator = true;
      if (_operatorNext) setPhase(Phase::MOVE_HOME_Z, nowMs);
      break;

    case Phase::MOVE_HOME_Z:
      if (moveVertical(nowMs, sensors, _program->travelZ[0], _program->travelZ[1], 0)) {
        stopMotion(); setPhase(Phase::MOVE_HOME_X, nowMs);
      }
      break;

    case Phase::MOVE_HOME_X:
      if (moveHorizontal(nowMs, sensors, _program->homeX[0], _program->homeX[1], 0, limitMask)) {
        stopMotion(); setPhase(Phase::DONE, nowMs);
      }
      break;

    case Phase::DONE:
      stopMotion();
      _running = false;
      _runEndMs = nowMs;
      _stateChanged = true;
      Serial.println(F("AUTO PROGRAM DONE"));
      return true;

    case Phase::FAULT:
    case Phase::IDLE:
      break;
  }
  return _stateChanged;
}
