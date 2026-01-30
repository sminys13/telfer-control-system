\
/**
 * @file app.h
 * @brief Верхний уровень логики проекта.
 */
#pragma once
#include <stdint.h>
#include "config.h"
#include "ui.h"
#include "sensors.h"
#include "modbus.h"
#include "motors.h"
#include "storage.h"

struct AutoRunner {
  enum class Phase : uint8_t {
    IDLE,
    PREP_TRAVEL,     // поднять на travel height (по УЗ) перед горизонталью
    MOVE_ZONE_H,     // горизонталь к зоне
    LOWER_TILT,      // ступень 1: опускаем только LOW сторону (наклон)
    WAIT_AFTER_TILT, // пауза после наклона, чтобы полости набрали жидкость
    LOWER_BASE,      // ступень 2: опускаем оба до целевого уровня
    WAIT_DIP,        // выдержка в жидкости

    RAISE_TILT_HIGH, // ступень 1 подъёма: приподнять HIGH сторону (по ТЗ)
    WAIT_DRAIN,      // пауза, чтобы стекла жидкость
    RAISE_TRAVEL,    // ступень 2 подъёма: поднять оба на travel

    NEXT_ZONE,       // переход к следующей зоне
    MOVE_HOME,       // домой
    DONE
  };

  Phase phase = Phase::IDLE;
  bool running = false;
  bool paused = false;

  uint8_t orderIndex = 0;   // индекс в program.order
  uint8_t zoneIndex = 0;    // текущая зона (индекс zones[])

  uint32_t phaseStartMs = 0;
  uint32_t lastStepMs = 0;
  uint8_t  levelStepsDone = 0;
  uint8_t  levelStepsTotal = 0;
  uint8_t  lowSide = 0; // 0=telfer1, 1=telfer2
  int32_t  baseUsTarget[2] = {0,0};
  int32_t  lowTiltTarget = 0;
  int32_t  highLiftTarget = 0;
};

struct AppRuntime {
  RunMode mode = RunMode::STOP;
  ErrorCode error = ErrorCode::NONE;

  GlobalSettings settings{};
  ProgramConfig  program{};
  uint8_t activeSlot = 0;

  AutoRunner autoRt{};
  bool modbusOk = false;
};

class App {
public:
  void setup();
  void loop();

private:
  // подсистемы
  Storage _storage;
  Sensors _sensors;
  ModbusMasterRTU _mb;
  Drives _drives;
  UI _ui;

  // runtime
  AppRuntime _rt{};

  // таймеры
  uint32_t _tUi = 0;
  uint32_t _tSensors = 0;
  uint32_t _tSafety = 0;

  ManualButtons _manual{};
  AppActions _actions{};

  // Последние выданные команды на приводы (в процентах, знак = forward/reverse).
  // Нужны для корректной логики концевиков.
  int16_t _cmdPct[(uint8_t)DriveId::COUNT] = {0,0,0,0};

  // ручная синхронизация горизонтали
  bool _syncActive = false;
  int32_t _syncBaseDelta = 0;

  // ---- helpers ----
  void applyActions(uint32_t nowMs);
  void updateSafety(uint32_t nowMs);
  void updateManual(uint32_t nowMs);
  void updateAuto(uint32_t nowMs);

  // движение к цели
  bool driveToHorizontal(int32_t targetX1, int32_t targetX2, uint8_t maxPct);
  bool driveToVertical(int32_t targetUs1, int32_t targetUs2, uint8_t maxPct);

  void stopAll();

  bool lasersOk() const { return _sensors.get().laser[0].valid && _sensors.get().laser[1].valid; }
  bool usOk() const { return _sensors.get().us[0].valid && _sensors.get().us[1].valid; }

  void setError(ErrorCode e);

  // синхр ручной горизонтали
  void manualHorizontalSync(int16_t dirPct, int32_t baseDeltaMm);

  // Хелперы для AutoRunner
  void autoStart(uint32_t nowMs);
  void autoStop();
  void autoGotoHome(uint32_t nowMs);
  bool autoAdvanceToNextEnabledZone();
};