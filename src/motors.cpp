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
    _tel[i] = DriveTelemetry{0,0,false,0,0};
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

  // 1) Основной опрос: 0x7000..0x7001 (RUN+SET frequency, 0.01Hz)
  uint16_t regs[2] = {0, 0};
  auto r = _mb->readHoldingRegisters(m.addr, MB_REG_MON_RUN_FREQ, 2, regs);
  if (r.ok) {
    _tel[idx(id)].runFreq01Hz = regs[0];
    _tel[idx(id)].setFreq01Hz = regs[1];
    _tel[idx(id)].connected = true;
    _tel[idx(id)].lastErr = 0;
    _tel[idx(id)].lastOkMs = nowMs;
  } else {
    _tel[idx(id)].connected = false;
    _tel[idx(id)].lastErr = r.error;
    return; // если нет связи — не тратим время на diag
  }

  // 2) Диагностика: чередуем faultInfo и runState (не чаще 1 раза/сек на привод)
  auto& st = _st[idx(id)];
  if ((uint32_t)(nowMs - st.lastDiag) < 1000) return;
  st.lastDiag = nowMs;

  uint16_t v = 0;
  if (st.diagPhase == 0) {
    auto rf = _mb->readHoldingRegisters(m.addr, MB_REG_MON_FAULT_INFO, 1, &v);
    if (rf.ok) _tel[idx(id)].faultInfo = v;
    st.diagPhase = 1;
  } else {
    auto rs = _mb->readHoldingRegisters(m.addr, MB_REG_MON_RUN_STATE, 1, &v);
    if (rs.ok) _tel[idx(id)].runState = v;
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
    if ((uint32_t)(nowMs - st.lastPoll) >= 500) {
      pollTelemetry((DriveId)i, nowMs);
      st.lastPoll = nowMs;
      _rrPoll = (uint8_t)((i + 1) % idx(DriveId::COUNT));
    }
  }
}

