/**
 * @file app.cpp
 * @brief Главная логика: режимы, безопасность, автомат.
 */

#include "app.h"
#include "utils.h"
#include <Arduino.h>

// ------------------------- локальные утилиты -------------------------

static inline bool withinTol(int32_t v, int32_t target, int16_t tol) {
  const int32_t d = v - target;
  return (d <= tol && d >= -tol);
}

static inline uint8_t otherSide(uint8_t idx) { return idx ? 0 : 1; }

// Движение одного вертикального привода к target по УЗ.
// Возвращает true, когда достигли.
static bool driveVerticalOne(Drives& drives, int16_t& cmdOut,
                            DriveId id, int32_t curUsMm, bool curValid,
                            int32_t targetUsMm, int16_t tolMm, uint8_t maxPct,
                            bool estopOrFault)
{
  if (estopOrFault) {
    drives.stop(id);
    cmdOut = 0;
    return false;
  }
  if (!curValid) {
    drives.stop(id);
    cmdOut = 0;
    return false;
  }
  if (withinTol(curUsMm, targetUsMm, tolMm)) {
    drives.stop(id);
    cmdOut = 0;
    return true;
  }
  // ВАЖНО: V Forward = вниз, а по УЗ "вниз" => расстояние уменьшается.
  // Если cur > target => надо вниз => +pct.
  // Если cur < target => надо вверх => -pct.
  int16_t pct = 0;
  if (curUsMm > targetUsMm) pct = (int16_t)maxPct;
  else pct = (int16_t)-maxPct;

  drives.setSpeed(id, pct);
  cmdOut = pct;
  return false;
}

// Движение одного горизонтального привода к target по лазеру.
static bool driveHorizontalOne(Drives& drives, int16_t& cmdOut,
                              DriveId id, int32_t curMm, bool curValid,
                              int32_t targetMm, int16_t tolMm, uint8_t maxPct,
                              bool estopOrFault)
{
  if (estopOrFault) {
    drives.stop(id);
    cmdOut = 0;
    return false;
  }
  if (!curValid) {
    drives.stop(id);
    cmdOut = 0;
    return false;
  }
  if (withinTol(curMm, targetMm, tolMm)) {
    drives.stop(id);
    cmdOut = 0;
    return true;
  }

  // ВАЖНО: H Forward = вправо.
  int16_t pct = 0;
  if (curMm < targetMm) pct = (int16_t)maxPct;   // нужно вправо
  else pct = (int16_t)-maxPct;                   // нужно влево

  drives.setSpeed(id, pct);
  cmdOut = pct;
  return false;
}

// ------------------------- App -------------------------

void App::setup() {
  Serial.begin(115200);
  delay(50);

  // Датчики и UI
  _sensors.begin();
  _ui.begin();

  // EEPROM
  _storage.begin();
  _rt.activeSlot = _storage.getActiveSlot();
  _storage.loadSettings(_rt.settings);
  _storage.loadActiveProgram(_rt.program);

  // Настройка RS485/Modbus и приводов
  // Параметры RS485/Modbus должны совпадать с настройками ПЧ (группа Fd.xx).
  // Заводская настройка NE200/300: 9600 бод, even parity (Fd.03=0).
  _mb.begin(Serial1, PIN_RS485_DE_RE, BAUD_RS485, 120, SERIAL_8E1);
  _drives.begin(_mb);

  stopAll();

  _rt.mode = RunMode::STOP;
  _rt.error = ErrorCode::NONE;
  _rt.autoRt = AutoRunner{};

  // дефолт синхры ручного горизонтального движения
  _ui.setManualSyncEnabled(_rt.settings.manual_h_sync_default);

  _tUi = _tSensors = _tSafety = millis();
}

void App::loop() {
  const uint32_t now = millis();

  // --- Датчики ---
  if ((uint32_t)(now - _tSensors) >= SENSORS_TICK_MS) {
    _tSensors = now;
    _sensors.tick(now);
  }

  // --- Drives (modbus) ---
  _drives.tick(now);

  // --- Сводка для UI ---
  UiStateSummary st{};
  st.mode = _rt.mode;
  st.autoPaused = _rt.autoRt.paused;
  st.activeSlot = _rt.activeSlot;
  st.autoOrderIndex = _rt.autoRt.orderIndex;
  st.autoZoneIndex = _rt.autoRt.zoneIndex;
  st.error = _rt.error;
  // modbusOk: считаем OK, если все 4 привода отвечают
  const bool m1 = _drives.telemetry(DriveId::H1).connected;
  const bool m2 = _drives.telemetry(DriveId::H2).connected;
  const bool m3 = _drives.telemetry(DriveId::V1).connected;
  const bool m4 = _drives.telemetry(DriveId::V2).connected;
  _rt.modbusOk = (m1 && m2 && m3 && m4);
  st.modbusOk = _rt.modbusOk;

  // --- UI ---
  if ((uint32_t)(now - _tUi) >= UI_TICK_MS) {
    _tUi = now;
    _actions = AppActions{}; // сброс
    _ui.tick(now, _sensors.get(), st, _rt.settings, _rt.program, _actions, _manual);
  }

  // --- Обработка действий UI ---
  applyActions(now);

  // --- Безопасность ---
  if ((uint32_t)(now - _tSafety) >= SAFETY_TICK_MS) {
    _tSafety = now;
    updateSafety(now);
  }

  // --- Режимы ---
  // В STOP тоже разрешаем "джог" с физических кнопок (удобно, чтобы уйти от концевика).
  if (_rt.mode == RunMode::AUTO) {
    updateAuto(now);
  } else {
    updateManual(now);
  }
}

// ------------------------- Helpers -------------------------

void App::stopAll() {
  _drives.stopAll();
  for (uint8_t i=0;i<(uint8_t)DriveId::COUNT;i++) _cmdPct[i] = 0;
}

void App::setError(ErrorCode e) {
  if (_rt.error != ErrorCode::NONE) return; // не перетираем первичную причину
  _rt.error = e;
  stopAll();
  _rt.mode = RunMode::STOP;
  _rt.autoRt.running = false;
  _rt.autoRt.paused = false;
  _rt.autoRt.phase = AutoRunner::Phase::IDLE;
}

void App::applyActions(uint32_t nowMs) {
  // Режимы
  if (_actions.toStop) {
    autoStop();
    _rt.mode = RunMode::STOP;
    stopAll();
  }
  if (_actions.toManual) {
    autoStop();
    _rt.mode = RunMode::MANUAL;
    stopAll();
  }

  // Storage
  if (_actions.factoryReset) {
    _storage.factoryReset();
    _rt.activeSlot = _storage.getActiveSlot();
    _storage.loadSettings(_rt.settings);
    _storage.loadActiveProgram(_rt.program);
    _ui.setManualSyncEnabled(_rt.settings.manual_h_sync_default);
    stopAll();
    _rt.error = ErrorCode::NONE;
  }

  if (_actions.saveSettings) {
    _storage.saveSettings(_rt.settings);
  }

  if (_actions.copySlot) {
    _storage.copySlot(_actions.copyFrom, _actions.copyTo);
  }

  if (_actions.loadSlot) {
    if (_storage.loadProgramSlot(_actions.slot, _rt.program)) {
      _storage.setActiveSlot(_actions.slot);
      _rt.activeSlot = _actions.slot;
    }
  }

  if (_actions.saveSlot) {
    _storage.saveProgramSlot(_actions.slot, _rt.program);
    _storage.setActiveSlot(_actions.slot);
    _rt.activeSlot = _actions.slot;
  }

  // Калибровки
  if (_actions.captureHome) {
    if (!lasersOk()) setError(ErrorCode::LASER1_FAIL);
    else {
      _rt.settings.home_x_mm[0] = _sensors.get().laser[0].mm;
      _rt.settings.home_x_mm[1] = _sensors.get().laser[1].mm;
      _storage.saveSettings(_rt.settings);
    }
  }
  if (_actions.captureTravel) {
    if (!usOk()) setError(ErrorCode::US1_FAIL);
    else {
      _rt.settings.travel_us_mm[0] = _sensors.get().us[0].mm;
      _rt.settings.travel_us_mm[1] = _sensors.get().us[1].mm;
      _storage.saveSettings(_rt.settings);
    }
  }

  if (_actions.captureZoneX) {
    if (!lasersOk()) setError(ErrorCode::LASER1_FAIL);
    else {
      const uint8_t z = _actions.zoneIndex;
      if (z < MAX_ZONES) {
        if (z >= _rt.program.zone_count) {
          // расширим программу
          const uint8_t old = _rt.program.zone_count;
          _rt.program.zone_count = (uint8_t)(z + 1);
          if (_rt.program.zone_count > MAX_ZONES) _rt.program.zone_count = MAX_ZONES;
          for (uint8_t i=old; i<_rt.program.zone_count; i++) {
            _rt.program.order[i] = i;
            _rt.program.zones[i].enabled = true;
          }
        }
        _rt.program.zones[z].x_mm[0] = _sensors.get().laser[0].mm;
        _rt.program.zones[z].x_mm[1] = _sensors.get().laser[1].mm;
        _rt.program.zones[z].enabled = true;
      }
    }
  }

  if (_actions.captureZoneHeight) {
    if (!usOk()) setError(ErrorCode::US1_FAIL);
    else {
      const uint8_t z = _actions.zoneIndex;
      if (z < MAX_ZONES) {
        if (z >= _rt.program.zone_count) {
          const uint8_t old = _rt.program.zone_count;
          _rt.program.zone_count = (uint8_t)(z + 1);
          if (_rt.program.zone_count > MAX_ZONES) _rt.program.zone_count = MAX_ZONES;
          for (uint8_t i=old; i<_rt.program.zone_count; i++) {
            _rt.program.order[i] = i;
            _rt.program.zones[i].enabled = true;
          }
        }
        _rt.program.zones[z].us_target_mm[0] = _sensors.get().us[0].mm;
        _rt.program.zones[z].us_target_mm[1] = _sensors.get().us[1].mm;
        _rt.program.zones[z].enabled = true;
      }
    }
  }

  // Авто
  if (_actions.startAuto) {
    autoStart(nowMs);
  }
  if (_actions.pauseResumeAuto) {
    if (_rt.mode == RunMode::AUTO && _rt.autoRt.running) {
      _rt.autoRt.paused = !_rt.autoRt.paused;
      if (_rt.autoRt.paused) stopAll();
    }
  }
  if (_actions.stopAuto) {
    autoStop();
    stopAll();
    _rt.mode = RunMode::STOP;
  }
  if (_actions.returnHome) {
    autoGotoHome(nowMs);
  }
}

void App::updateSafety(uint32_t) {
  // E-stop
  if (_manual.estop) {
    setError(ErrorCode::ESTOP);
    return;
  }

  // Сброс ошибки по кнопке START (если не E-stop)
  if (_manual.start && _rt.error != ErrorCode::NONE) {
    // Разрешаем сброс, если E-stop отпущен
    _rt.error = ErrorCode::NONE;
  }

  // Ошибки частотников
  for (uint8_t i=0;i<(uint8_t)DriveId::COUNT;i++) {
    const auto& t = _drives.telemetry((DriveId)i);
    if (t.connected && t.faultCode != 0) {
      setError(ErrorCode::DRIVE_FAULT);
      return;
    }
  }

  // Концевики (только горизонтальные NC)
  // Останавливаем при движении "в концевик".
  // H Forward = вправо -> положительная команда = вправо.
  const int16_t h1 = _cmdPct[(uint8_t)DriveId::H1];
  const int16_t h2 = _cmdPct[(uint8_t)DriveId::H2];

  if (_manual.lim_h1_right && h1 > 0) {
    stopAll();
    setError(ErrorCode::LIMIT_SWITCH);
  }
  if (_manual.lim_h1_left && h1 < 0) {
    stopAll();
    setError(ErrorCode::LIMIT_SWITCH);
  }
  if (_manual.lim_h2_right && h2 > 0) {
    stopAll();
    setError(ErrorCode::LIMIT_SWITCH);
  }
  if (_manual.lim_h2_left && h2 < 0) {
    stopAll();
    setError(ErrorCode::LIMIT_SWITCH);
  }
}

void App::updateManual(uint32_t) {
  // При E-stop/фатальной ошибке никаких движений.
  if (_manual.estop) {
    stopAll();
    return;
  }
  if (_rt.error != ErrorCode::NONE && _rt.error != ErrorCode::LIMIT_SWITCH) {
    stopAll();
    return;
  }

  // Кнопка STOP — всегда стоп.
  if (_manual.stop) {
    stopAll();
    _rt.mode = RunMode::STOP;
    return;
  }

  // Если оператор нажал START в STOP — перейдём в MANUAL (удобно).
  if (_manual.start && _rt.mode == RunMode::STOP) {
    _rt.mode = RunMode::MANUAL;
  }

  const uint8_t hPct = _rt.settings.h_speed_pct;
  const uint8_t vPct = _rt.settings.v_speed_pct;

  int16_t cmdH1 = 0, cmdH2 = 0;
  int16_t cmdV1 = 0, cmdV2 = 0;

  // --- Горизонталь ---
  // Приоритет:
  //  1) кнопки "оба"
  //  2) если включена синхронизация — любая из парных кнопок двигает оба
  //  3) иначе независимые
  const bool bothF = _manual.h_both_fwd;
  const bool bothB = _manual.h_both_bwd;
  if (bothF && !bothB) {
    cmdH1 = (int16_t)hPct;
    cmdH2 = (int16_t)hPct;
  } else if (bothB && !bothF) {
    cmdH1 = (int16_t)-hPct;
    cmdH2 = (int16_t)-hPct;
  } else if (_ui.manualSyncEnabled()) {
    const bool anyF = (_manual.h1_fwd || _manual.h2_fwd);
    const bool anyB = (_manual.h1_bwd || _manual.h2_bwd);
    if (anyF && !anyB) {
      cmdH1 = (int16_t)hPct;
      cmdH2 = (int16_t)hPct;
    } else if (anyB && !anyF) {
      cmdH1 = (int16_t)-hPct;
      cmdH2 = (int16_t)-hPct;
    }
  } else {
    if (_manual.h1_fwd && !_manual.h1_bwd) cmdH1 = (int16_t)hPct;
    else if (_manual.h1_bwd && !_manual.h1_fwd) cmdH1 = (int16_t)-hPct;

    if (_manual.h2_fwd && !_manual.h2_bwd) cmdH2 = (int16_t)hPct;
    else if (_manual.h2_bwd && !_manual.h2_fwd) cmdH2 = (int16_t)-hPct;
  }

  // Не позволяем ехать "в концевик"
  if (_manual.lim_h1_right && cmdH1 > 0) cmdH1 = 0;
  if (_manual.lim_h1_left  && cmdH1 < 0) cmdH1 = 0;
  if (_manual.lim_h2_right && cmdH2 > 0) cmdH2 = 0;
  if (_manual.lim_h2_left  && cmdH2 < 0) cmdH2 = 0;

  // --- Вертикаль (кнопки продублированы для каждого тельфера) ---
  // В Forward вниз: DOWN => +pct, UP => -pct.
  if (_manual.v1_down && !_manual.v1_up) cmdV1 = (int16_t)vPct;
  else if (_manual.v1_up && !_manual.v1_down) cmdV1 = (int16_t)-vPct;

  if (_manual.v2_down && !_manual.v2_up) cmdV2 = (int16_t)vPct;
  else if (_manual.v2_up && !_manual.v2_down) cmdV2 = (int16_t)-vPct;

  // Применяем
  _drives.setSpeed(DriveId::H1, cmdH1);
  _drives.setSpeed(DriveId::H2, cmdH2);
  _drives.setSpeed(DriveId::V1, cmdV1);
  _drives.setSpeed(DriveId::V2, cmdV2);

  _cmdPct[(uint8_t)DriveId::H1] = cmdH1;
  _cmdPct[(uint8_t)DriveId::H2] = cmdH2;
  _cmdPct[(uint8_t)DriveId::V1] = cmdV1;
  _cmdPct[(uint8_t)DriveId::V2] = cmdV2;
}

bool App::driveToHorizontal(int32_t targetX1, int32_t targetX2, uint8_t maxPct) {
  const auto& s = _sensors.get();
  const bool stop = (_manual.estop || _rt.error != ErrorCode::NONE);

  int16_t c1=0, c2=0;
  const bool d1 = driveHorizontalOne(_drives, c1, DriveId::H1, s.laser[0].mm, s.laser[0].valid,
                                    targetX1, _rt.settings.h_tol_mm, maxPct, stop);
  const bool d2 = driveHorizontalOne(_drives, c2, DriveId::H2, s.laser[1].mm, s.laser[1].valid,
                                    targetX2, _rt.settings.h_tol_mm, maxPct, stop);

  // концевики: не ехать в них
  if (_manual.lim_h1_right && c1 > 0) { _drives.stop(DriveId::H1); c1 = 0; }
  if (_manual.lim_h1_left  && c1 < 0) { _drives.stop(DriveId::H1); c1 = 0; }
  if (_manual.lim_h2_right && c2 > 0) { _drives.stop(DriveId::H2); c2 = 0; }
  if (_manual.lim_h2_left  && c2 < 0) { _drives.stop(DriveId::H2); c2 = 0; }

  _cmdPct[(uint8_t)DriveId::H1] = c1;
  _cmdPct[(uint8_t)DriveId::H2] = c2;
  return d1 && d2;
}

bool App::driveToVertical(int32_t targetUs1, int32_t targetUs2, uint8_t maxPct) {
  const auto& s = _sensors.get();
  const bool stop = (_manual.estop || _rt.error != ErrorCode::NONE);

  int16_t c1=0, c2=0;
  const bool d1 = driveVerticalOne(_drives, c1, DriveId::V1, s.us[0].mm, s.us[0].valid,
                                  targetUs1, _rt.settings.v_tol_mm, maxPct, stop);
  const bool d2 = driveVerticalOne(_drives, c2, DriveId::V2, s.us[1].mm, s.us[1].valid,
                                  targetUs2, _rt.settings.v_tol_mm, maxPct, stop);

  _cmdPct[(uint8_t)DriveId::V1] = c1;
  _cmdPct[(uint8_t)DriveId::V2] = c2;
  return d1 && d2;
}

// ------------------------- Auto -------------------------

void App::autoStop() {
  _rt.autoRt.running = false;
  _rt.autoRt.paused = false;
  _rt.autoRt.phase = AutoRunner::Phase::IDLE;
}

bool App::autoAdvanceToNextEnabledZone() {
  while (_rt.autoRt.orderIndex < _rt.program.zone_count) {
    const uint8_t zid = _rt.program.order[_rt.autoRt.orderIndex];
    if (zid < MAX_ZONES && _rt.program.zones[zid].enabled) {
      _rt.autoRt.zoneIndex = zid;
      return true;
    }
    _rt.autoRt.orderIndex++;
  }
  return false;
}

void App::autoStart(uint32_t nowMs) {
  if (_rt.error != ErrorCode::NONE) return;
  if (!lasersOk() || !usOk()) {
    setError(ErrorCode::SENSOR_TIMEOUT);
    return;
  }

  _rt.mode = RunMode::AUTO;
  _rt.autoRt = AutoRunner{};
  _rt.autoRt.running = true;
  _rt.autoRt.paused = false;
  _rt.autoRt.phaseStartMs = nowMs;
  _rt.autoRt.lowSide = LOW_SIDE_TELFER_INDEX;
  _rt.autoRt.orderIndex = 0;

  if (!autoAdvanceToNextEnabledZone()) {
    setError(ErrorCode::INVALID_PROGRAM);
    return;
  }
  _rt.autoRt.phase = AutoRunner::Phase::PREP_TRAVEL;
}

void App::autoGotoHome(uint32_t nowMs) {
  if (_rt.error != ErrorCode::NONE) return;

  _rt.mode = RunMode::AUTO;
  _rt.autoRt.running = true;
  _rt.autoRt.paused = false;
  _rt.autoRt.phase = AutoRunner::Phase::MOVE_HOME;
  _rt.autoRt.phaseStartMs = nowMs;
}

void App::updateAuto(uint32_t nowMs) {
  if (!_rt.autoRt.running) return;

  if (_rt.autoRt.paused) {
    stopAll();
    return;
  }

  if (_rt.error != ErrorCode::NONE) {
    autoStop();
    stopAll();
    _rt.mode = RunMode::STOP;
    return;
  }

  // Датчики обязаны быть валидны в авто
  if (!lasersOk() || !usOk()) {
    setError(ErrorCode::SENSOR_TIMEOUT);
    return;
  }

  const auto& s = _sensors.get();

  // Для удобства
  const uint8_t zid = _rt.autoRt.zoneIndex;
  const ZoneConfig& z = _rt.program.zones[zid];
  const uint8_t low = _rt.autoRt.lowSide;
  const uint8_t high = otherSide(low);

  switch (_rt.autoRt.phase) {
    case AutoRunner::Phase::PREP_TRAVEL: {
      if (driveToVertical(_rt.settings.travel_us_mm[0], _rt.settings.travel_us_mm[1], _rt.settings.v_speed_pct)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::MOVE_ZONE_H;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::MOVE_ZONE_H: {
      const uint8_t hSpd = (z.move_speed_pct >= 10 && z.move_speed_pct <= 100) ? z.move_speed_pct : _rt.settings.h_speed_pct;
      if (driveToHorizontal(z.x_mm[0], z.x_mm[1], hSpd)) {
        stopAll();

        // Подготовим цели по УЗ
        _rt.autoRt.baseUsTarget[0] = z.us_target_mm[0];
        _rt.autoRt.baseUsTarget[1] = z.us_target_mm[1];

        // Наклон: опускаем LOW сторону на tilt_step_mm относительно текущей,
        // но не ниже финального target.
        const int32_t curLow = s.us[low].mm;
        int32_t inter = curLow - (int32_t)z.tilt_step_mm;
        if (inter < _rt.autoRt.baseUsTarget[low]) inter = _rt.autoRt.baseUsTarget[low];
        _rt.autoRt.lowTiltTarget = inter;

        _rt.autoRt.phase = AutoRunner::Phase::LOWER_TILT;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::LOWER_TILT: {
      // Двигаем только LOW вертикальный привод вниз/вверх по УЗ
      int16_t dummyCmd = 0;
      const DriveId did = (low == 0) ? DriveId::V1 : DriveId::V2;
      const int32_t cur = s.us[low].mm;
      const bool ok = driveVerticalOne(_drives, dummyCmd, did, cur, s.us[low].valid,
                                       _rt.autoRt.lowTiltTarget, _rt.settings.v_tol_mm,
                                       _rt.settings.v_tilt_speed_pct, false);
      // Второй вертикальный стоп
      _drives.stop((low == 0) ? DriveId::V2 : DriveId::V1);

      _cmdPct[(uint8_t)DriveId::V1] = (low==0) ? dummyCmd : 0;
      _cmdPct[(uint8_t)DriveId::V2] = (low==1) ? dummyCmd : 0;

      if (ok) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::WAIT_AFTER_TILT;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::WAIT_AFTER_TILT: {
      stopAll();
      const uint32_t waitMs = (uint32_t)z.step_wait_s * 1000UL;
      if ((uint32_t)(nowMs - _rt.autoRt.phaseStartMs) >= waitMs) {
        _rt.autoRt.phase = AutoRunner::Phase::LOWER_BASE;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::LOWER_BASE: {
      const uint8_t vSpd = (z.v_speed_pct >= 10 && z.v_speed_pct <= 100) ? z.v_speed_pct : _rt.settings.v_speed_pct;
      if (driveToVertical(_rt.autoRt.baseUsTarget[0], _rt.autoRt.baseUsTarget[1], vSpd)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::WAIT_DIP;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::WAIT_DIP: {
      stopAll();
      const uint32_t waitMs = (uint32_t)z.dip_time_s * 1000UL;
      if ((uint32_t)(nowMs - _rt.autoRt.phaseStartMs) >= waitMs) {
        // Подъём: сначала HIGH сторону
        const int32_t curHigh = s.us[high].mm;
        int32_t inter = curHigh + (int32_t)z.tilt_step_mm;
        // не выше travel
        if (inter > _rt.settings.travel_us_mm[high]) inter = _rt.settings.travel_us_mm[high];
        _rt.autoRt.highLiftTarget = inter;

        _rt.autoRt.phase = AutoRunner::Phase::RAISE_TILT_HIGH;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::RAISE_TILT_HIGH: {
      int16_t dummyCmd = 0;
      const DriveId did = (high == 0) ? DriveId::V1 : DriveId::V2;
      const int32_t cur = s.us[high].mm;
      const bool ok = driveVerticalOne(_drives, dummyCmd, did, cur, s.us[high].valid,
                                       _rt.autoRt.highLiftTarget, _rt.settings.v_tol_mm,
                                       _rt.settings.v_tilt_speed_pct, false);
      _drives.stop((high == 0) ? DriveId::V2 : DriveId::V1);

      _cmdPct[(uint8_t)DriveId::V1] = (high==0) ? dummyCmd : 0;
      _cmdPct[(uint8_t)DriveId::V2] = (high==1) ? dummyCmd : 0;

      if (ok) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::WAIT_DRAIN;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::WAIT_DRAIN: {
      stopAll();
      const uint32_t waitMs = (uint32_t)_rt.settings.drip_wait_s * 1000UL;
      if ((uint32_t)(nowMs - _rt.autoRt.phaseStartMs) >= waitMs) {
        _rt.autoRt.phase = AutoRunner::Phase::RAISE_TRAVEL;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::RAISE_TRAVEL: {
      if (driveToVertical(_rt.settings.travel_us_mm[0], _rt.settings.travel_us_mm[1], _rt.settings.v_speed_pct)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::NEXT_ZONE;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::NEXT_ZONE: {
      // следующий шаг
      _rt.autoRt.orderIndex++;
      if (autoAdvanceToNextEnabledZone()) {
        _rt.autoRt.phase = AutoRunner::Phase::MOVE_ZONE_H;
        _rt.autoRt.phaseStartMs = nowMs;
      } else {
        _rt.autoRt.phase = AutoRunner::Phase::MOVE_HOME;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::MOVE_HOME: {
      // Сначала на travel, затем домой по X
      if (!withinTol(s.us[0].mm, _rt.settings.travel_us_mm[0], _rt.settings.v_tol_mm) ||
          !withinTol(s.us[1].mm, _rt.settings.travel_us_mm[1], _rt.settings.v_tol_mm)) {
        (void)driveToVertical(_rt.settings.travel_us_mm[0], _rt.settings.travel_us_mm[1], _rt.settings.v_speed_pct);
      } else {
        if (driveToHorizontal(_rt.settings.home_x_mm[0], _rt.settings.home_x_mm[1], _rt.settings.h_speed_pct)) {
          stopAll();
          _rt.autoRt.phase = AutoRunner::Phase::DONE;
          _rt.autoRt.phaseStartMs = nowMs;
        }
      }
    } break;

    case AutoRunner::Phase::DONE: {
      stopAll();
      autoStop();
      _rt.mode = RunMode::STOP;
    } break;

    default:
      break;
  }
}
