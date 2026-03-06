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
  _sensors.applySettings(_rt.settings);

  // Настройка RS485/Modbus и приводов.
  // Параметры должны совпадать с настройками ПЧ (группа Fd.xx):
  //   - адрес (например 1..4),
  //   - скорость (Fd.02),
  //   - формат (Fd.03) → в config.h это RS485_SERIAL_CONFIG.
  _mb.begin(Serial1, PIN_RS485_DE_RE, BAUD_RS485, RS485_TIMEOUT_MS, RS485_SERIAL_CONFIG);
  _mb.setInterFrameDelayUs(4500); // 3.5 char times @9600bps ≈ 3.65ms, use a safe margin
  _drives.begin(_mb);

  stopAll();

  // Notifications / signal panel
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  if (ENABLE_BUZZER) {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
  }

  if (ENABLE_SIGNAL_PANEL) {
    pinMode(PIN_LAMP_RED, OUTPUT);
    pinMode(PIN_LAMP_GREEN, OUTPUT);
    pinMode(PIN_LAMP_YELLOW, OUTPUT);
    pinMode(PIN_LAMP_ORANGE, OUTPUT);
    digitalWrite(PIN_LAMP_RED, PANEL_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(PIN_LAMP_GREEN, PANEL_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(PIN_LAMP_YELLOW, PANEL_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(PIN_LAMP_ORANGE, PANEL_ACTIVE_HIGH ? LOW : HIGH);
  }

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
  st.autoRunning = _rt.autoRt.running;
  st.activeSlot = _rt.activeSlot;
  st.autoOrderIndex = _rt.autoRt.orderIndex;
  st.autoZoneIndex = _rt.autoRt.zoneIndex;
  st.autoPhase = (uint8_t)_rt.autoRt.phase;
  st.autoZoneNow = 0;
  st.autoZoneNext = 0;
  st.autoZoneDir = 0;
  st.autoDipRemainS = 0xFFFF;
  st.autoDryRemainS = 0xFFFF;
  st.autoWaitOperator = _rt.autoRt.waitOperator;
  st.autoDryAlarm = _rt.autoRt.dryAlarm;
  st.error = _rt.error;

  // Доп. инфо для статус-экрана: текущая/следующая зона, направление, остатки выдержки/сушки.
  if (_rt.autoRt.running) {
    const uint8_t STG = 0xFD;
    const uint8_t DRY = 0xFE;

    const uint8_t ph = (uint8_t)_rt.autoRt.phase;
    const bool isDry = (ph >= (uint8_t)AutoRunner::Phase::DRY_GOTO_STAGING && ph <= (uint8_t)AutoRunner::Phase::DRY_WAIT_CLOSE);

    auto avgHomeX = [&](){
      return ((int32_t)_rt.settings.home_x_mm[0] + (int32_t)_rt.settings.home_x_mm[1]) / 2;
    };
    auto avgDryX = [&](){
      return ((int32_t)_rt.settings.dry_x_mm[0] + (int32_t)_rt.settings.dry_x_mm[1]) / 2;
    };
    auto avgZoneX = [&](uint8_t zid){
      return ((int32_t)_rt.program.zones[zid].x_mm[0] + (int32_t)_rt.program.zones[zid].x_mm[1]) / 2;
    };
    auto avgX = [&](uint8_t code){
      if (code == 0) return avgHomeX();
      if (code == STG) return avgZoneX(_rt.autoRt.stagingZoneIndex);
      if (code == DRY) return avgDryX();
      const uint8_t zid = (uint8_t)(code - 1);
      if (zid >= MAX_ZONES) return 0L;
      return avgZoneX(zid);
    };

    if (!isDry) {
      const uint8_t curZid = _rt.autoRt.zoneIndex;
      if (curZid < MAX_ZONES) st.autoZoneNow = (uint8_t)(curZid + 1);

      // Следующая включённая зона по порядку программы.
      uint8_t nextZid = 0xFF;
      for (uint8_t j = (uint8_t)(_rt.autoRt.orderIndex + 1); j < _rt.program.zone_count; j++) {
        const uint8_t zid = _rt.program.order[j];
        if (zid < MAX_ZONES && _rt.program.zones[zid].enabled) { nextZid = zid; break; }
      }
      if (nextZid != 0xFF) {
        st.autoZoneNext = (uint8_t)(nextZid + 1);
      } else {
        // Дальше — домой (или переход в сушку)
        st.autoZoneNext = 0;
      }

      const int32_t dx = avgX(st.autoZoneNext) - avgX(st.autoZoneNow);
      st.autoZoneDir = (dx > 0) ? 1 : (dx < 0 ? -1 : 0);

      // Остаток выдержки (только в фазе WAIT_DIP).
      if (_rt.autoRt.phase == AutoRunner::Phase::WAIT_DIP && curZid < MAX_ZONES) {
        const ZoneConfig& z = _rt.program.zones[curZid];
        const uint32_t totalMs = (uint32_t)z.dip_time_s * 1000u;
        const uint32_t elapsed = (uint32_t)(now - _rt.autoRt.phaseStartMs);
        if (elapsed >= totalMs) st.autoDipRemainS = 0;
        else st.autoDipRemainS = (uint16_t)((totalMs - elapsed + 999u) / 1000u);
      }
    } else {
      // DRY sequence: показываем ST/DR/HM
      uint8_t nowCode = STG;
      uint8_t nextCode = DRY;

      switch (_rt.autoRt.phase) {
        case AutoRunner::Phase::DRY_GOTO_STAGING:
        case AutoRunner::Phase::DRY_WAIT_OPEN:
          nowCode = STG; nextCode = DRY; break;
        case AutoRunner::Phase::DRY_GOTO_DRY:
        case AutoRunner::Phase::DRY_LOWER_DROP:
        case AutoRunner::Phase::DRY_WAIT_DETACH:
        case AutoRunner::Phase::DRY_RAISE_TRAVEL:
        case AutoRunner::Phase::DRY_WAIT_START:
        case AutoRunner::Phase::DRY_WAIT_TIMER:
        case AutoRunner::Phase::DRY_LOWER_PICK:
        case AutoRunner::Phase::DRY_WAIT_ATTACH:
          nowCode = DRY; nextCode = STG; break;
        case AutoRunner::Phase::DRY_RAISE_TRAVEL2:
        case AutoRunner::Phase::DRY_GOTO_STAGING2:
        case AutoRunner::Phase::DRY_WAIT_CLOSE:
          nowCode = STG; nextCode = 0; break;
        default: break;
      }

      st.autoZoneNow = nowCode;
      st.autoZoneNext = nextCode;
      const int32_t dx = avgX(nextCode) - avgX(nowCode);
      st.autoZoneDir = (dx > 0) ? 1 : (dx < 0 ? -1 : 0);

      // Остаток сушки (только в фазе DRY_WAIT_TIMER и если задано время)
      if (_rt.autoRt.phase == AutoRunner::Phase::DRY_WAIT_TIMER && _rt.autoRt.dryTimeS > 0) {
        const uint32_t elapsedS = (uint32_t)((now - _rt.autoRt.dryTimerStartMs) / 1000u);
        if (elapsedS >= _rt.autoRt.dryTimeS) st.autoDryRemainS = 0;
        else st.autoDryRemainS = (uint16_t)(_rt.autoRt.dryTimeS - elapsedS);
      }
    }
  }

  // --- Modbus/RS485 (4 привода: H1,H2,V1,V2) ---
  // Собираем краткую телеметрию в UiStateSummary, чтобы UI мог показать состояние связи.
  // Важно: порядок индексов 0..3 жёстко задан и совпадает с маской в ui.h:
  //   0=H1, 1=H2, 2=V1, 3=V2
  const DriveId ids[4] = {DriveId::H1, DriveId::H2, DriveId::V1, DriveId::V2};
  uint8_t mask = 0;
  bool allOk = true;
  for (uint8_t i = 0; i < 4; i++) {
    const DriveTelemetry& t = _drives.telemetry(ids[i]);
    if (t.connected) mask |= (1u << i);
    else allOk = false;
    st.mbRunFreq01Hz[i] = t.runFreq01Hz;
    st.mbSetFreq01Hz[i] = t.setFreq01Hz;
    st.mbBusV01V[i]     = t.busV01V;
  }
  st.mbConnectedMask = mask;
  _rt.modbusOk = allOk;
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

  // --- Signal panel / notifications ---
  updateSignalPanel(now);
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
    _sensors.applySettings(_rt.settings);
    _ui.setManualSyncEnabled(_rt.settings.manual_h_sync_default);
    stopAll();
    _rt.error = ErrorCode::NONE;
  }

  if (_actions.saveSettings) {
    _storage.saveSettings(_rt.settings);
    _sensors.applySettings(_rt.settings);
  }

  if (_actions.applyLaserConfig) {
    // конфиг лазеров: частота/диапазон/разрешение/ноль/автостарт/адрес
    _storage.saveSettings(_rt.settings);
    _sensors.applyLaserDeviceConfig(_rt.settings);
  }

  if (_actions.restartLaserStreaming) {
    // Мягкий запуск лазеров без конфигурации.
    _sensors.restartLaserStreaming();
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

  // DRY zone calibration (special)
  if (_actions.captureDryX) {
    if (!lasersOk()) setError(ErrorCode::LASER1_FAIL);
    else {
      _rt.settings.dry_x_mm[0] = _sensors.get().laser[0].mm;
      _rt.settings.dry_x_mm[1] = _sensors.get().laser[1].mm;
      _rt.settings.dry_valid = true;
      _storage.saveSettings(_rt.settings);
    }
  }
  if (_actions.captureDryHeight) {
    if (!usOk()) setError(ErrorCode::US1_FAIL);
    else {
      _rt.settings.dry_us_mm[0] = _sensors.get().us[0].mm;
      _rt.settings.dry_us_mm[1] = _sensors.get().us[1].mm;
      _rt.settings.dry_valid = true;
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
    if (t.connected && t.faultInfo != 0) {
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
                                    targetX1, _rt.settings.x_tol_mm[0], maxPct, stop);
  const bool d2 = driveHorizontalOne(_drives, c2, DriveId::H2, s.laser[1].mm, s.laser[1].valid,
                                    targetX2, _rt.settings.x_tol_mm[1], maxPct, stop);

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
                                  targetUs1, _rt.settings.us_tol_mm[0], maxPct, stop);
  const bool d2 = driveVerticalOne(_drives, c2, DriveId::V2, s.us[1].mm, s.us[1].valid,
                                  targetUs2, _rt.settings.us_tol_mm[1], maxPct, stop);

  _cmdPct[(uint8_t)DriveId::V1] = c1;
  _cmdPct[(uint8_t)DriveId::V2] = c2;
  return d1 && d2;
}

// ------------------------- Auto -------------------------

void App::autoStop() {
  _rt.autoRt.running = false;
  _rt.autoRt.paused = false;
  _rt.autoRt.phase = AutoRunner::Phase::IDLE;
  _rt.autoRt.waitOperator = false;
  _rt.autoRt.dryAlarm = false;
  _rt.autoRt.dryTimerStartMs = 0;
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

  // DRY flow init
  _rt.autoRt.waitOperator = false;
  _rt.autoRt.dryAlarm = false;
  _rt.autoRt.dryTimerStartMs = 0;
  _rt.autoRt.dryTimeS = _rt.program.drying_time_s;
  _rt.autoRt.stagingZoneIndex = _rt.program.staging_zone;
  if (_rt.autoRt.stagingZoneIndex >= _rt.program.zone_count) _rt.autoRt.stagingZoneIndex = 0;

  if (_rt.program.drying_enabled && !_rt.settings.dry_valid) {
    // Требуется калибровка зоны сушки
    setError(ErrorCode::INVALID_PROGRAM);
    return;
  }

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
                                       _rt.autoRt.lowTiltTarget, _rt.settings.us_tol_mm[high],
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
                                       _rt.autoRt.highLiftTarget, _rt.settings.us_tol_mm[high],
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
        // Конец основного алгоритма
        if (_rt.program.drying_enabled) {
          _rt.autoRt.stagingZoneIndex = _rt.program.staging_zone;
          if (_rt.autoRt.stagingZoneIndex >= _rt.program.zone_count) _rt.autoRt.stagingZoneIndex = 0;
          _rt.autoRt.dryTimeS = _rt.program.drying_time_s;
          _rt.autoRt.waitOperator = false;
          _rt.autoRt.dryAlarm = false;
          _rt.autoRt.dryTimerStartMs = 0;
          _rt.autoRt.phase = AutoRunner::Phase::DRY_GOTO_STAGING;
          _rt.autoRt.phaseStartMs = nowMs;
        } else {
          _rt.autoRt.phase = AutoRunner::Phase::MOVE_HOME;
          _rt.autoRt.phaseStartMs = nowMs;
        }
      }
    } break;

    case AutoRunner::Phase::DRY_GOTO_STAGING: {
      // Движение к предсушке/преддверию (Staging).
      _rt.autoRt.waitOperator = false;

      const uint8_t stg = _rt.autoRt.stagingZoneIndex;
      const ZoneConfig& sz = _rt.program.zones[stg];

      // Сначала на travel, затем в зону STG по X
      if (!withinTol(s.us[0].mm, _rt.settings.travel_us_mm[0], _rt.settings.us_tol_mm[0]) ||
          !withinTol(s.us[1].mm, _rt.settings.travel_us_mm[1], _rt.settings.us_tol_mm[1])) {
        (void)driveToVertical(_rt.settings.travel_us_mm[0], _rt.settings.travel_us_mm[1], _rt.settings.v_speed_pct);
      } else {
        if (driveToHorizontal(sz.x_mm[0], sz.x_mm[1], _rt.settings.h_speed_pct)) {
          stopAll();
          _rt.autoRt.phase = AutoRunner::Phase::DRY_WAIT_OPEN;
          _rt.autoRt.phaseStartMs = nowMs;
        }
      }
    } break;

    case AutoRunner::Phase::DRY_WAIT_OPEN: {
      // Ждём команду оператора: дверь открыта, можно подъезжать к сушке
      stopAll();
      _rt.autoRt.waitOperator = true;
      if (_actions.operatorNext) {
        _rt.autoRt.waitOperator = false;
        _rt.autoRt.phase = AutoRunner::Phase::DRY_GOTO_DRY;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_GOTO_DRY: {
      // Едем к зоне сушки по X (на travel высоте)
      _rt.autoRt.waitOperator = false;
      if (!withinTol(s.us[0].mm, _rt.settings.travel_us_mm[0], _rt.settings.us_tol_mm[0]) ||
          !withinTol(s.us[1].mm, _rt.settings.travel_us_mm[1], _rt.settings.us_tol_mm[1])) {
        (void)driveToVertical(_rt.settings.travel_us_mm[0], _rt.settings.travel_us_mm[1], _rt.settings.v_speed_pct);
      } else {
        if (driveToHorizontal(_rt.settings.dry_x_mm[0], _rt.settings.dry_x_mm[1], _rt.settings.h_speed_pct)) {
          stopAll();
          _rt.autoRt.phase = AutoRunner::Phase::DRY_LOWER_DROP;
          _rt.autoRt.phaseStartMs = nowMs;
        }
      }
    } break;

    case AutoRunner::Phase::DRY_LOWER_DROP: {
      // Опустить груз в сушилку
      _rt.autoRt.waitOperator = false;
      if (driveToVertical(_rt.settings.dry_us_mm[0], _rt.settings.dry_us_mm[1], _rt.settings.v_speed_pct)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::DRY_WAIT_DETACH;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_WAIT_DETACH: {
      // Оператор отцепил груз
      stopAll();
      _rt.autoRt.waitOperator = true;
      if (_actions.operatorNext) {
        _rt.autoRt.waitOperator = false;
        _rt.autoRt.phase = AutoRunner::Phase::DRY_RAISE_TRAVEL;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_RAISE_TRAVEL: {
      // Подняться на транспортную высоту
      _rt.autoRt.waitOperator = false;
      if (driveToVertical(_rt.settings.travel_us_mm[0], _rt.settings.travel_us_mm[1], _rt.settings.v_speed_pct)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::DRY_WAIT_START;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_WAIT_START: {
      // Оператор закрыл дверцу и запустил сушку
      stopAll();
      _rt.autoRt.waitOperator = true;
      if (_actions.operatorNext) {
        _rt.autoRt.waitOperator = false;
        _rt.autoRt.dryAlarm = false;
        _rt.autoRt.dryTimerStartMs = nowMs;
        _rt.autoRt.phase = AutoRunner::Phase::DRY_WAIT_TIMER;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_WAIT_TIMER: {
      // Ждём окончания таймера сушки (и/или команду оператора после окончания)
      stopAll();

      if (_rt.autoRt.dryTimeS == 0) {
        // полностью ручной режим: сразу ждём подтверждение
        _rt.autoRt.waitOperator = true;
        if (_actions.operatorNext) {
          _rt.autoRt.waitOperator = false;
          _rt.autoRt.dryAlarm = false;
          _rt.autoRt.phase = AutoRunner::Phase::DRY_LOWER_PICK;
          _rt.autoRt.phaseStartMs = nowMs;
        }
        break;
      }

      const uint32_t elapsedMs = (uint32_t)(nowMs - _rt.autoRt.dryTimerStartMs);
      const uint32_t totalMs = (uint32_t)_rt.autoRt.dryTimeS * 1000u;
      const bool done = (elapsedMs >= totalMs);

      if (done) {
        _rt.autoRt.dryAlarm = true;
        _rt.autoRt.waitOperator = true;
        if (_actions.operatorNext) {
          _rt.autoRt.waitOperator = false;
          _rt.autoRt.dryAlarm = false;
          _rt.autoRt.phase = AutoRunner::Phase::DRY_LOWER_PICK;
          _rt.autoRt.phaseStartMs = nowMs;
        }
      } else {
        _rt.autoRt.waitOperator = false;
      }
    } break;

    case AutoRunner::Phase::DRY_LOWER_PICK: {
      // Опустить для подцепления груза
      _rt.autoRt.waitOperator = false;
      if (driveToVertical(_rt.settings.dry_us_mm[0], _rt.settings.dry_us_mm[1], _rt.settings.v_speed_pct)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::DRY_WAIT_ATTACH;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_WAIT_ATTACH: {
      // Оператор подцепил груз
      stopAll();
      _rt.autoRt.waitOperator = true;
      if (_actions.operatorNext) {
        _rt.autoRt.waitOperator = false;
        _rt.autoRt.phase = AutoRunner::Phase::DRY_RAISE_TRAVEL2;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_RAISE_TRAVEL2: {
      // Подняться на транспортную высоту
      _rt.autoRt.waitOperator = false;
      if (driveToVertical(_rt.settings.travel_us_mm[0], _rt.settings.travel_us_mm[1], _rt.settings.v_speed_pct)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::DRY_GOTO_STAGING2;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_GOTO_STAGING2: {
      // Вернуться на предсушку/преддверие
      _rt.autoRt.waitOperator = false;
      const uint8_t stg = _rt.autoRt.stagingZoneIndex;
      const ZoneConfig& sz = _rt.program.zones[stg];
      if (driveToHorizontal(sz.x_mm[0], sz.x_mm[1], _rt.settings.h_speed_pct)) {
        stopAll();
        _rt.autoRt.phase = AutoRunner::Phase::DRY_WAIT_CLOSE;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::DRY_WAIT_CLOSE: {
      // Оператор закрыл дверцу сушилки (или подготовил к отъезду)
      stopAll();
      _rt.autoRt.waitOperator = true;
      if (_actions.operatorNext) {
        _rt.autoRt.waitOperator = false;
        _rt.autoRt.phase = AutoRunner::Phase::MOVE_HOME;
        _rt.autoRt.phaseStartMs = nowMs;
      }
    } break;

    case AutoRunner::Phase::MOVE_HOME: {
      // Сначала на travel, затем домой по X
      if (!withinTol(s.us[0].mm, _rt.settings.travel_us_mm[0], _rt.settings.us_tol_mm[0]) ||
          !withinTol(s.us[1].mm, _rt.settings.travel_us_mm[1], _rt.settings.us_tol_mm[1])) {
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
      // completion notification: green blink + beep for a few seconds
      _finishNotifyUntilMs = nowMs + 10000UL;
      autoStop();
      _rt.mode = RunMode::STOP;
    } break;

    default:
      break;
  }
}


// ------------------------- Signal panel / notifications -------------------------

void App::updateSignalPanel(uint32_t nowMs) {
  // helpers
  auto setLamp = [&](uint8_t pin, bool on) {
    if (!ENABLE_SIGNAL_PANEL) return;
    const uint8_t v = (PANEL_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
    digitalWrite(pin, v);
  };

  auto allOff = [&]() {
    setLamp(PIN_LAMP_RED, false);
    setLamp(PIN_LAMP_GREEN, false);
    setLamp(PIN_LAMP_YELLOW, false);
    setLamp(PIN_LAMP_ORANGE, false);
  };

  // blinking base (250ms tick)
  if ((uint32_t)(nowMs - _panelLastMs) >= 250) {
    _panelLastMs = nowMs;
    _panelBlink = !_panelBlink;
  }

  // Determine states
  const bool hasErr = (_rt.error != ErrorCode::NONE);
  const bool finishNotify = ((int32_t)(nowMs - _finishNotifyUntilMs) < 0);
  const bool autoRun = (_rt.mode == RunMode::AUTO && _rt.autoRt.running);
  const bool autoPause = (autoRun && _rt.autoRt.paused);

  // Drying process indicator (yellow): in DRY_WAIT_TIMER or when dryAlarm active
  const bool inDryTimer = autoRun && (_rt.autoRt.phase == AutoRunner::Phase::DRY_WAIT_TIMER);
  const bool dryAlarm = _rt.autoRt.dryAlarm;

  // Manual away-from-home and not moving
  const bool anyMove = (_cmdPct[0] != 0) || (_cmdPct[1] != 0) || (_cmdPct[2] != 0) || (_cmdPct[3] != 0);
  bool atHome = false;
  if (_sensors.get().laser[0].valid && _sensors.get().laser[1].valid) {
    atHome = withinTol(_sensors.get().laser[0].mm, _rt.settings.home_x_mm[0], _rt.settings.x_tol_mm[0]) &&
             withinTol(_sensors.get().laser[1].mm, _rt.settings.home_x_mm[1], _rt.settings.x_tol_mm[1]);
  }
  const bool manualAway = (_rt.mode == RunMode::MANUAL) && (!anyMove) && (!atHome);

  // Clear buzzer by default
  if (ENABLE_BUZZER) noTone(PIN_BUZZER);
  digitalWrite(PIN_STATUS_LED, LOW);

  if (hasErr) {
    // ERROR: all lamps blink 0.5s (toggle every 250ms), buzzer with the blink
    const bool on = _panelBlink;
    setLamp(PIN_LAMP_RED, on);
    setLamp(PIN_LAMP_GREEN, on);
    setLamp(PIN_LAMP_YELLOW, on);
    setLamp(PIN_LAMP_ORANGE, on);
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
    if (ENABLE_BUZZER && on) tone(PIN_BUZZER, 1800);
    return;
  }

  if (finishNotify) {
    // FINISH: green blink 1s (toggle every 500ms -> use _panelBlink every 250ms => two ticks per toggle)
    const bool on = ((nowMs / 500UL) % 2) == 0;
    allOff();
    setLamp(PIN_LAMP_GREEN, on);
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
    if (ENABLE_BUZZER && on) tone(PIN_BUZZER, 1600);
    return;
  }

  if (inDryTimer) {
    allOff();
    // Drying in progress: yellow solid; when timer done (dryAlarm) blink + beep
    if (dryAlarm) {
      const bool on = _panelBlink;
      setLamp(PIN_LAMP_YELLOW, on);
      digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
      if (ENABLE_BUZZER && on) tone(PIN_BUZZER, 1800);
    } else {
      setLamp(PIN_LAMP_YELLOW, true);
      digitalWrite(PIN_STATUS_LED, HIGH);
    }
    return;
  }

  if (autoRun) {
    allOff();
    if (autoPause) {
      // red blink 1s
      const bool on = ((nowMs / 500UL) % 2) == 0;
      setLamp(PIN_LAMP_RED, on);
      digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
    } else {
      setLamp(PIN_LAMP_RED, true);
      digitalWrite(PIN_STATUS_LED, HIGH);
    }
    return;
  }

  if (manualAway) {
    allOff();
    setLamp(PIN_LAMP_ORANGE, true);
    digitalWrite(PIN_STATUS_LED, HIGH);
    return;
  }

  // IDLE: green solid
  allOff();
  setLamp(PIN_LAMP_GREEN, true);
  digitalWrite(PIN_STATUS_LED, HIGH);
}
