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
    _tel[i] = DriveTelemetry{0,0,false};
  }
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

void Drives::pollTelemetry(DriveId id) {
  if (!_mb) return;
  DriveMap m = _map[idx(id)];
  uint16_t regs[2] = {0,0};
  auto r = _mb->readHoldingRegisters(m.addr, MB_REG_STATUS, 2, regs);
  if (r.ok) {
    _tel[idx(id)].statusReg = regs[0];
    _tel[idx(id)].faultCode = regs[1];
    _tel[idx(id)].connected = true;
  } else {
    _tel[idx(id)].connected = false;
  }
}

void Drives::tick(uint32_t nowMs) {
  if (!_mb) return;

  for (uint8_t i=0;i<idx(DriveId::COUNT);i++) {
    DriveId id = (DriveId)i;
    auto& st = _st[i];

    // отправляем команды не чаще MOTORS_TICK_MS
    if ((uint32_t)(nowMs - st.lastSend) >= MOTORS_TICK_MS) {
      if (st.needStopCmd) {
        sendCommand(id, 0);
        st.lastSend = nowMs;
      } else if (st.targetPct != st.sentPct) {
        sendCommand(id, st.targetPct);
        st.lastSend = nowMs;
      }
    }

    // телеметрию читаем реже (пример: 500 мс)
    if ((uint32_t)(nowMs - st.lastPoll) >= 500) {
      pollTelemetry(id);
      st.lastPoll = nowMs;
    }
  }
}

