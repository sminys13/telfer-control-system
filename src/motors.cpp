/**
 * @file motors.cpp
 * @brief Four-drive scheduler. Step9E adds HE200 read-only commissioning.
 */
#include "motors.h"
#include "utils.h"

const __FlashStringHelper* Drives::driveName(DriveId id) {
  switch (id) {
    case DriveId::H1: return F("H1");
    case DriveId::H2: return F("H2");
    case DriveId::V1: return F("V1");
    case DriveId::V2: return F("V2");
    case DriveId::COUNT: break;
  }
  return F("?");
}

void Drives::begin(ModbusMasterRTU& mb, const SettingsV6& settings) {
  _mb = &mb;
  for (uint8_t i = 0; i < indexOf(DriveId::COUNT); ++i) {
    _st[i] = DriveState{};
    _tel[i] = DriveTelemetry{};
  }
  _activeDrive = 0xFF;
  _rrStart = 0;
  _lastTransactionMs = 0;
  applySettings(settings);
}

void Drives::applySettings(const SettingsV6& settings) {
  _interRequestMs = settings.vfd.interRequestMs;
  for (uint8_t i = 0; i < indexOf(DriveId::COUNT); ++i) {
    _map[i].addr = settings.vfd.address[i];
    _map[i].invertDir = (settings.vfd.invertDirectionMask & (1U << i)) != 0;
  }

  Serial.print(F("Drives map applied H1/H2/V1/V2="));
  for (uint8_t i = 0; i < indexOf(DriveId::COUNT); ++i) {
    if (i) Serial.print('/');
    Serial.print(_map[i].addr);
    if (_map[i].invertDir) Serial.print(F("i"));
  }
  Serial.print(F(" interRequest="));
  Serial.print(_interRequestMs);
  Serial.println(F("ms"));
}

void Drives::setSpeed(DriveId id, int16_t speedPct) {
  if (!_mb || id == DriveId::COUNT) return;
  const uint8_t i = indexOf(id);
  _st[i].targetPct = clampT<int16_t>(speedPct, -100, 100);
  _st[i].forceStop = false;
}

void Drives::stop(DriveId id) {
  if (id == DriveId::COUNT) return;
  const uint8_t i = indexOf(id);
  _st[i].targetPct = 0;
  _st[i].forceStop = true;
}

void Drives::stopAll() {
  uint8_t firstMoving = 0xFF;
  for (uint8_t i = 0; i < indexOf(DriveId::COUNT); ++i) {
    if (firstMoving == 0xFF &&
        (_st[i].appliedPct != 0 ||
         _st[i].phase == TxPhase::WRITE_SETPOINT ||
         _st[i].phase == TxPhase::WRITE_RUN_COMMAND)) {
      firstMoving = i;
    }
    _st[i].targetPct = 0;
    _st[i].forceStop = true;
  }

  // A drive known to be moving must receive STOP before idle drives.
  if (firstMoving != 0xFF) {
    _rrStart = firstMoving;
    if (_activeDrive >= indexOf(DriveId::COUNT)) _activeDrive = firstMoving;
  }
}

void Drives::resetFault(DriveId id) {
  if (id == DriveId::COUNT) return;
  _st[indexOf(id)].resetRequested = true;
}

void Drives::requestSafeStatusRead(DriveId id) {
  if (id == DriveId::COUNT) return;
  _st[indexOf(id)].statusReadRequested = true;
}

void Drives::requestSafeStatusReadAll() {
  for (uint8_t i = 0; i < indexOf(DriveId::COUNT); ++i)
    _st[i].statusReadRequested = true;
}

int16_t Drives::effectivePercent(uint8_t i, int16_t requestedPct) const {
  int16_t pct = requestedPct;
  if (_map[i].invertDir) pct = (int16_t)-pct;
  return pct;
}

uint16_t Drives::setpointMagnitude(int16_t effectivePct) const {
  int16_t magnitudePct = effectivePct < 0 ? (int16_t)-effectivePct : effectivePct;
  magnitudePct = clampT<int16_t>(magnitudePct, 0, 100);
  const uint16_t sp = (uint16_t)(magnitudePct * 100);
  return clampT<uint16_t>(sp, NE200_SETPOINT_MIN, NE200_SETPOINT_MAX);
}

uint16_t Drives::directionCommand(int16_t effectivePct) const {
  return effectivePct >= 0 ? NE200_CMD_FORWARD : NE200_CMD_REVERSE;
}

void Drives::printPlan(uint8_t i, const __FlashStringHelper* action,
                       uint16_t reg, uint16_t value) const {
  Serial.print(F("NE200 "));
  Serial.print(driveName(driveFromIndex(i)));
  Serial.print(F(" addr="));
  Serial.print(_map[i].addr);
  Serial.print(' ');
  Serial.print(action);
  Serial.print(F(" reg=0x"));
  if (reg < 0x1000) Serial.print('0');
  if (reg < 0x0100) Serial.print('0');
  if (reg < 0x0010) Serial.print('0');
  Serial.print(reg, HEX);
  Serial.print(F(" value=0x"));
  if (value < 0x1000) Serial.print('0');
  if (value < 0x0100) Serial.print('0');
  if (value < 0x0010) Serial.print('0');
  Serial.print(value, HEX);
  Serial.print(F(" ("));
  Serial.print(value);
  Serial.println(')');
}

void Drives::preparePhase(uint8_t i) {
  DriveState& st = _st[i];
  if (st.phase != TxPhase::IDLE) return;

  if (st.resetRequested) {
    st.phase = TxPhase::WRITE_RESET_FAULT;
    return;
  }

  if (st.forceStop || (st.targetPct == 0 && st.appliedPct != 0)) {
    st.phase = TxPhase::WRITE_STOP_COMMAND;
    return;
  }

  if (st.targetPct != st.appliedPct) {
    st.transactionPct = st.targetPct;
    st.phase = (st.targetPct == 0) ? TxPhase::WRITE_STOP_COMMAND
                                   : TxPhase::WRITE_SETPOINT;
    return;
  }

  if (st.statusReadRequested)
    st.phase = TxPhase::READ_SAFE_STATUS;
}

bool Drives::selectNextWork() {
  if (_activeDrive < indexOf(DriveId::COUNT)) return true;

  // Priority 1: stop a drive that is known to be moving before any other
  // drive is allowed to start. This is essential when the operator changes
  // from one jog axis to another.
  for (uint8_t k = 0; k < indexOf(DriveId::COUNT); ++k) {
    const uint8_t i = (uint8_t)((_rrStart + k) % indexOf(DriveId::COUNT));
    DriveState& st = _st[i];
    const bool stopPriority =
        st.phase == TxPhase::WRITE_STOP_COMMAND ||
        st.phase == TxPhase::WRITE_ZERO_SETPOINT ||
        st.forceStop ||
        (st.targetPct == 0 && st.appliedPct != 0) ||
        ((st.phase == TxPhase::WRITE_SETPOINT ||
          st.phase == TxPhase::WRITE_RUN_COMMAND) &&
         st.targetPct == 0);
    if (!stopPriority) continue;

    preparePhase(i);
    _activeDrive = i;
    _rrStart = (uint8_t)((i + 1) % indexOf(DriveId::COUNT));
    return true;
  }

  // Priority 2: all other queued work, round-robin.
  for (uint8_t k = 0; k < indexOf(DriveId::COUNT); ++k) {
    const uint8_t i = (uint8_t)((_rrStart + k) % indexOf(DriveId::COUNT));
    preparePhase(i);
    if (_st[i].phase != TxPhase::IDLE) {
      _activeDrive = i;
      _rrStart = (uint8_t)((i + 1) % indexOf(DriveId::COUNT));
      return true;
    }
  }
  return false;
}

bool Drives::finishTransaction(uint8_t i, const ModbusResult& r,
                               TxPhase nextOnSuccess) {
  DriveState& st = _st[i];
  DriveTelemetry& tel = _tel[i];

  if (r.ok || r.simulated) {
    st.failStreak = 0;
    st.phase = nextOnSuccess;
    if (st.phase == TxPhase::IDLE) _activeDrive = 0xFF;
    return true;
  }

  tel.lastErr = r.error;
  if (st.failStreak < 255) ++st.failStreak;
  Serial.print(F("VFD transaction failed drive="));
  Serial.print(driveName(driveFromIndex(i)));
  Serial.print(F(" error="));
  Serial.println(r.error);

  // Do not hold the whole scheduler forever on one missing drive.
  if (st.failStreak >= 3) {
    st.phase = TxPhase::IDLE;
    st.failStreak = 0;
    _activeDrive = 0xFF;
  }
  return true;
}

bool Drives::processActiveDrive(uint32_t nowMs) {
  if (_activeDrive >= indexOf(DriveId::COUNT) || !_mb) return false;
  const uint8_t i = _activeDrive;
  DriveState& st = _st[i];
  const DriveMap& map = _map[i];

  if (map.addr == 0) {
    st.phase = TxPhase::IDLE;
    _activeDrive = 0xFF;
    return false;
  }

  // A STOP request always preempts an unfinished start sequence.
  if ((st.phase == TxPhase::WRITE_SETPOINT ||
       st.phase == TxPhase::WRITE_RUN_COMMAND) &&
      (st.forceStop || st.targetPct == 0)) {
    st.phase = TxPhase::WRITE_STOP_COMMAND;
  }

  // If the operator changed direction/speed between the two writes, rebuild
  // the setpoint first instead of mixing values from different commands.
  if (st.phase == TxPhase::WRITE_RUN_COMMAND &&
      st.targetPct != st.transactionPct) {
    st.transactionPct = st.targetPct;
    st.phase = (st.targetPct == 0) ? TxPhase::WRITE_STOP_COMMAND
                                   : TxPhase::WRITE_SETPOINT;
  }

  ModbusResult r{false, MODBUS_ERROR_BAD_RESPONSE, false};
  switch (st.phase) {
    case TxPhase::WRITE_SETPOINT: {
      st.transactionPct = st.targetPct;
      const int16_t eff = effectivePercent(i, st.transactionPct);
      const uint16_t sp = setpointMagnitude(eff);
      printPlan(i, F("SETPOINT"), NE200_REG_SETPOINT, sp);
      r = _mb->writeSingleRegister(map.addr, NE200_REG_SETPOINT, sp);
      return finishTransaction(i, r, TxPhase::WRITE_RUN_COMMAND);
    }

    case TxPhase::WRITE_RUN_COMMAND: {
      const int16_t eff = effectivePercent(i, st.transactionPct);
      const uint16_t cmd = directionCommand(eff);
      printPlan(i, eff >= 0 ? F("RUN FWD") : F("RUN REV"),
                NE200_REG_COMMAND, cmd);
      r = _mb->writeSingleRegister(map.addr, NE200_REG_COMMAND, cmd);
      if (r.ok || r.simulated) {
        st.appliedPct = st.transactionPct;
        st.forceStop = false;
      }
      return finishTransaction(i, r, TxPhase::IDLE);
    }

    case TxPhase::WRITE_STOP_COMMAND:
      printPlan(i, F("STOP"), NE200_REG_COMMAND, NE200_CMD_STOP);
      r = _mb->writeSingleRegister(map.addr, NE200_REG_COMMAND, NE200_CMD_STOP);
      return finishTransaction(i, r, TxPhase::WRITE_ZERO_SETPOINT);

    case TxPhase::WRITE_ZERO_SETPOINT:
      printPlan(i, F("ZERO SETPOINT"), NE200_REG_SETPOINT, 0);
      r = _mb->writeSingleRegister(map.addr, NE200_REG_SETPOINT, 0);
      if (r.ok || r.simulated) {
        st.appliedPct = 0;
        st.forceStop = false;
      }
      return finishTransaction(i, r, TxPhase::IDLE);

    case TxPhase::WRITE_RESET_FAULT:
      printPlan(i, F("RESET FAULT"), NE200_REG_COMMAND,
                NE200_CMD_RESET_FAULT);
      r = _mb->writeSingleRegister(map.addr, NE200_REG_COMMAND,
                                   NE200_CMD_RESET_FAULT);
      if (r.ok || r.simulated) st.resetRequested = false;
      return finishTransaction(i, r, TxPhase::IDLE);

    case TxPhase::READ_SAFE_STATUS: {
      if (HE200_COMMISSIONING) {
        uint16_t values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        Serial.print(F("HE200 "));
        Serial.print(driveName(driveFromIndex(i)));
        Serial.print(F(" addr="));
        Serial.print(map.addr);
        Serial.println(F(" READ 0x7000..0x7007 (freq/set/busV/outV/outI/DI)"));
        r = _mb->readHoldingRegisters(map.addr, HE200_REG_RUNNING_FREQ, 8, values);
        if (r.ok) {
          DriveTelemetry& tel = _tel[i];
          tel.runningFreq001Hz = values[0];
          tel.setFreq001Hz = values[1];
          tel.busVoltage01V = values[2];
          tel.outputVoltageV = values[3];
          tel.outputCurrent001A = values[4];
          tel.digitalInputState = values[7];
          tel.connected = true;
          tel.lastErr = MODBUS_ERROR_NONE;
          tel.lastOkMs = nowMs;
          Serial.print(F("HE200 READ OK run="));
          Serial.print(values[0] / 100); Serial.print('.');
          if ((values[0] % 100) < 10) Serial.print('0'); Serial.print(values[0] % 100);
          Serial.print(F("Hz set="));
          Serial.print(values[1] / 100); Serial.print('.');
          if ((values[1] % 100) < 10) Serial.print('0'); Serial.print(values[1] % 100);
          Serial.print(F("Hz bus="));
          Serial.print(values[2] / 10); Serial.print('.'); Serial.print(values[2] % 10);
          Serial.print(F("V outV=")); Serial.print(values[3]);
          Serial.print(F("V outIraw=")); Serial.print(values[4]);
          Serial.print(F(" DI=0x")); Serial.println(values[7], HEX);
        }
        return finishTransaction(i, r, TxPhase::READ_HE200_FAULT);
      }

      // Legacy dry-run/read test retained for old NE200 builds only.
      uint16_t values[2] = {0, 0};
      r = _mb->readHoldingRegisters(map.addr, NE200_REG_STATUS, 2, values);
      st.statusReadRequested = false;
      if (r.ok) {
        _tel[i].statusWord = values[0];
        _tel[i].faultCode = values[1];
        _tel[i].connected = true;
        _tel[i].lastErr = MODBUS_ERROR_NONE;
        _tel[i].lastOkMs = nowMs;
      }
      return finishTransaction(i, r, TxPhase::IDLE);
    }

    case TxPhase::READ_HE200_FAULT: {
      uint16_t value = 0;
      r = _mb->readHoldingRegisters(map.addr, HE200_REG_FAULT_INFO, 1, &value);
      if (r.ok) {
        _tel[i].faultCode = value;
        _tel[i].connected = true;
        _tel[i].lastErr = MODBUS_ERROR_NONE;
        _tel[i].lastOkMs = nowMs;
        Serial.print(F("HE200 fault info 0x702D=0x"));
        Serial.println(value, HEX);
      }
      return finishTransaction(i, r, TxPhase::READ_HE200_STATE);
    }

    case TxPhase::READ_HE200_STATE: {
      uint16_t values[3] = {0, 0, 0};
      r = _mb->readHoldingRegisters(map.addr, HE200_REG_CUR_SET_FREQ, 3, values);
      st.statusReadRequested = false;
      if (r.ok) {
        DriveTelemetry& tel = _tel[i];
        tel.currentSetFreq001Pct = values[0];
        tel.currentRunFreq001Pct = values[1];
        tel.statusWord = values[2];
        tel.connected = true;
        tel.lastErr = MODBUS_ERROR_NONE;
        tel.lastOkMs = nowMs;
        Serial.print(F("HE200 state 0x703B/3C/3D="));
        Serial.print(values[0]); Serial.print('/');
        Serial.print(values[1]); Serial.print(F("/0x")); Serial.println(values[2], HEX);
      }
      return finishTransaction(i, r, TxPhase::IDLE);
    }

    case TxPhase::IDLE:
      _activeDrive = 0xFF;
      break;
  }
  return false;
}

bool Drives::tick(uint32_t nowMs) {
  if (!_mb) return false;
  if ((uint32_t)(nowMs - _lastTransactionMs) < _interRequestMs) return false;
  if (!selectNextWork()) return false;

  const bool didWork = processActiveDrive(nowMs);
  if (didWork) _lastTransactionMs = nowMs;
  return didWork;
}

bool Drives::hasPendingWork() const {
  if (_activeDrive < indexOf(DriveId::COUNT)) return true;
  for (uint8_t i = 0; i < indexOf(DriveId::COUNT); ++i) {
    const DriveState& st = _st[i];
    if (st.phase != TxPhase::IDLE || st.forceStop || st.resetRequested ||
        st.statusReadRequested || st.targetPct != st.appliedPct)
      return true;
  }
  return false;
}
