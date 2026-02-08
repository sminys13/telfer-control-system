\
/**
 * @file motors.cpp
 * @brief Реализация управления приводами.
 */
#include "motors.h"
#include "utils.h"
#include <Arduino.h>

static inline uint8_t idx(DriveId id){ return (uint8_t)id; }

void Drives::begin(ModbusMasterRTU& mb) {
  _mb = &mb;

  // Карта адресов частотников. Поменяйте адреса под вашу сеть RS-485.
  // Рекомендация: адреса 1..4.
  _map[idx(DriveId::H1)] = {1, false};
  _map[idx(DriveId::H2)] = {2, false};
  _map[idx(DriveId::V1)] = {3, false};
  _map[idx(DriveId::V2)] = {4, false};

  for (uint8_t i=0;i<idx(DriveId::COUNT);i++) {
    _st[i] = DriveState{};
    _tel[i] = DriveTelemetry{};
  }

  _rrSend = 0;
  _rrPoll = 0;
}

void Drives::setSpeed(DriveId id, int16_t speedPct) {
  if (!_mb) return;
  speedPct = clampT<int16_t>(speedPct, -100, 100);
  _st[idx(id)].targetPct = speedPct;
}

void Drives::stop(DriveId id) {
  _st[idx(id)].targetPct = 0;
  _st[idx(id)].needStopCmd = true;
}

void Drives::stopAll() {
  for (uint8_t i=0;i<idx(DriveId::COUNT);i++) {
    _st[i].targetPct = 0;
    _st[i].needStopCmd = true;
  }
}

void Drives::resetFault(DriveId id) {
  if (!_mb) return;
  DriveMap m = _map[idx(id)];
  (void)_mb->writeSingleRegister(m.addr, MB_REG_CMD, MB_CMD_RESET_FAULT);
}

void Drives::sendCommand(DriveId id, int16_t pct) {
  if (!_mb) return;
  DriveMap m = _map[idx(id)];
  if (m.addr == 0) return; // адрес 0 = отключено (удобно при отладке 1..2 приводов)

  // направление с учётом инверсии
  int16_t eff = pct;
  if (m.invertDir) eff = -eff;

  if (eff == 0) {
    (void)_mb->writeSingleRegister(m.addr, MB_REG_CMD, MB_CMD_STOP);
    (void)_mb->writeSingleRegister(m.addr, MB_REG_SETPOINT, (uint16_t)0);
    _st[idx(id)].sentPct = 0;
    _st[idx(id)].needStopCmd = false;
    return;
  }

  // setpoint
  int16_t sp = pctToSetpoint(eff);
  sp = clampT<int16_t>(sp, MB_SETPOINT_MIN, MB_SETPOINT_MAX);

  // команда направления
  uint16_t cmd = (eff > 0) ? MB_CMD_FWD : MB_CMD_REV;

  // Порядок: сначала setpoint, потом cmd — чтобы стартовать с нужным заданием
  (void)_mb->writeSingleRegister(m.addr, MB_REG_SETPOINT, (uint16_t)sp);
  (void)_mb->writeSingleRegister(m.addr, MB_REG_CMD, cmd);

  _st[idx(id)].sentPct = pct;
  _st[idx(id)].needStopCmd = false;
}

void Drives::pollTelemetry(DriveId id, uint32_t nowMs) {
  if (!_mb) return;
  DriveMap m = _map[idx(id)];
  if (m.addr == 0) return; // отключено
  auto& st = _st[idx(id)];
  auto& tel = _tel[idx(id)];

  const uint16_t baseReg = MB_REG_MON_RUN_FREQ;
  bool okNow = false;

  auto tryReadMon = [&](bool useInput, uint16_t reg, uint8_t modeCode) -> bool {
    // 0x7000..0x7003: run freq, set freq, DC bus voltage, output voltage
    uint16_t regs[4] = {0, 0, 0, 0};
    ModbusResult r = useInput ? _mb->readInputRegisters(m.addr, reg, 4, regs)
                            : _mb->readHoldingRegisters(m.addr, reg, 4, regs);
    if (r.ok) {
      tel.runFreq01Hz = regs[0];
      tel.setFreq01Hz = regs[1];
      tel.busV01V     = regs[2];
      tel.lastErr = 0;
      tel.lastOkMs = nowMs;
      tel.connected = true;
      st.failStreak = 0;
      st.regMode = modeCode;
      tel.regMode = modeCode;
      return true;
    }
    tel.lastErr = r.error;
    return false;
  };

  // Авто-детект карты/FC: 03/04 и возможный сдвиг адреса на -1 (некоторые мануалы 1-based).
  // ВАЖНО: чтобы не «подвешивать» UI при отсутствии одного из приводов на шине,
  // не делаем 4 запроса подряд. Пробуем по одному режиму за тик (probePhase).
  if (st.regMode == 0) {
    switch (st.probePhase & 0x03u) {
      case 0: okNow = tryReadMon(false, baseReg, 1); break; // 03
      case 1: okNow = tryReadMon(true,  baseReg, 2); break; // 04
      case 2: okNow = (baseReg > 0) && tryReadMon(false, (uint16_t)(baseReg - 1), 3); break; // 03 base-1
      case 3: okNow = (baseReg > 0) && tryReadMon(true,  (uint16_t)(baseReg - 1), 4); break; // 04 base-1
      default: break;
    }
    // если не получилось — в следующий раз попробуем другой режим
    if (!okNow) st.probePhase = (uint8_t)((st.probePhase + 1) & 0x03u);
  } else {
    bool useInput = (st.regMode == 2 || st.regMode == 4);
    uint16_t off = (st.regMode == 3 || st.regMode == 4) ? 1 : 0;
    uint16_t reg = (uint16_t)(baseReg - off);
    okNow = tryReadMon(useInput, reg, st.regMode);
  }

  if (!okNow) {
    st.failStreak++;
    tel.connected = ((uint32_t)(nowMs - tel.lastOkMs) <= 2000);
    if (st.failStreak >= 3) {
      st.regMode = 0;
      tel.regMode = 0;
    }
    return;
  }

  // Диагностика (fault/state) — не чаще 1 раза/сек на привод. Используем тот же режим FC/offset.
  if ((uint32_t)(nowMs - st.lastDiag) < 1000) return;
  st.lastDiag = nowMs;

  bool useInput = (st.regMode == 2 || st.regMode == 4);
  uint16_t off = (st.regMode == 3 || st.regMode == 4) ? 1 : 0;

  uint16_t v = 0;
  if (st.diagPhase == 0) {
    uint16_t rFault = (uint16_t)(MB_REG_MON_FAULT_INFO - off);
    ModbusResult rf = useInput ? _mb->readInputRegisters(m.addr, rFault, 1, &v)
                               : _mb->readHoldingRegisters(m.addr, rFault, 1, &v);
    if (rf.ok) tel.faultInfo = v;
    st.diagPhase = 1;
  } else {
    uint16_t rState = (uint16_t)(MB_REG_MON_RUN_STATE - off);
    ModbusResult rs = useInput ? _mb->readInputRegisters(m.addr, rState, 1, &v)
                               : _mb->readHoldingRegisters(m.addr, rState, 1, &v);
    if (rs.ok) tel.runState = v;
    st.diagPhase = 0;
  }
}

void Drives::tick(uint32_t nowMs) {
  if (!_mb) return;

  // Если давно не было успешного ответа — считаем привод офлайн.
  for (uint8_t i=0;i<idx(DriveId::COUNT);i++) {
    if (_tel[i].connected && (uint32_t)(nowMs - _tel[i].lastOkMs) > 2000) {
      _tel[i].connected = false;
    }
  }

  // 1) Отправка команд: максимум ОДИН привод за тик.
  // Если привод не на связи, не "долбим" его каждой итерацией — пробуем не чаще 1 раза/сек.
  for (uint8_t k=0;k<idx(DriveId::COUNT);k++) {
    const uint8_t i = (uint8_t)((_rrSend + k) % idx(DriveId::COUNT));
    DriveId id = (DriveId)i;
    auto& st = _st[i];
    const bool online = _tel[i].connected;
    const uint16_t minGap = online ? MOTORS_TICK_MS : 1000;
    if ((uint32_t)(nowMs - st.lastSend) < (uint32_t)minGap) continue;

    if (st.needStopCmd) {
      sendCommand(id, 0);
      st.lastSend = nowMs;
      _rrSend = (uint8_t)((i + 1) % idx(DriveId::COUNT));
      break;
    }
    if (st.targetPct != st.sentPct) {
      sendCommand(id, st.targetPct);
      st.lastSend = nowMs;
      _rrSend = (uint8_t)((i + 1) % idx(DriveId::COUNT));
      break;
    }
  }

  // 2) Телеметрия: максимум ОДИН привод за тик.
  // Это защищает UI/датчики от "зависания" при отсутствии Modbus.
  {
    const uint8_t i = _rrPoll;
    auto& st = _st[i];
    // Если привод не на связи, опрашиваем реже, чтобы меню/экран не «тормозили».
    const uint16_t pollGap = _tel[i].connected ? 500 : 2000;
    if ((uint32_t)(nowMs - st.lastPoll) >= pollGap) {
      pollTelemetry((DriveId)i, nowMs);
      st.lastPoll = nowMs;
      _rrPoll = (uint8_t)((i + 1) % idx(DriveId::COUNT));
    }
  }
}

