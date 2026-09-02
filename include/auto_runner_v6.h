#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "settings_v6.h"
#include "program_v6.h"
#include "motor_control_v6.h"

struct AutoSensorsV6 {
  int32_t mm[SENSOR_COUNT] = {0,0,0,0};
  uint8_t usableMask = 0; // bit per SensorIndex; usable through configured stale timeout
};

enum AutoErrorV6 : uint16_t {
  AUTO_ERR_NONE = 0,
  AUTO_ERR_PROGRAM = 1,
  AUTO_ERR_SENSORS = 2,
  AUTO_ERR_ESTOP = 3,
  AUTO_ERR_LIMIT = 4,
  AUTO_ERR_MOVE_TIMEOUT = 5,
  AUTO_ERR_OUTPUT_DISABLED = 6,
  AUTO_ERR_READONLY = 7
};

class AutoRunnerV6 {
public:
  enum class Phase : uint8_t {
    IDLE = 0,
    PREP_TRAVEL,
    MOVE_ZONE_H,
    LOWER_TILT,
    WAIT_AFTER_TILT,
    LOWER_BASE,
    WAIT_DIP,
    RAISE_TILT_HIGH,
    WAIT_DRAIN,
    RAISE_TRAVEL,
    NEXT_ZONE,

    DRY_GOTO_STAGING,
    DRY_WAIT_OPEN,
    DRY_GOTO_DRY,
    DRY_LOWER_DROP,
    DRY_WAIT_DETACH,
    DRY_RAISE_TRAVEL,
    DRY_WAIT_START,
    DRY_WAIT_TIMER,
    DRY_LOWER_PICK,
    DRY_WAIT_ATTACH,
    DRY_RAISE_TRAVEL2,
    DRY_GOTO_STAGING2,
    DRY_WAIT_CLOSE,

    MOVE_HOME_Z,
    MOVE_HOME_X,
    DONE,
    FAULT
  };

  void begin(MotorControlV6& motor, const SettingsV6& settings);
  void applySettings(const SettingsV6& settings) { _settings = &settings; }

  bool start(const AutoProgramV6& program, const AutoSensorsV6& sensors,
             bool simulation, uint32_t nowMs);
  bool startHome(const AutoProgramV6& program, const AutoSensorsV6& sensors,
                 bool simulation, uint32_t nowMs);
  void stop(const __FlashStringHelper* reason);
  void pause();
  void resume(uint32_t nowMs);
  void operatorNext();

  // Returns true when visible state changed and the DWIN header should be refreshed.
  bool service(uint32_t nowMs, const AutoSensorsV6& sensors,
               bool estopBlocked, uint8_t limitMask);

  bool running() const { return _running; }
  bool paused() const { return _paused; }
  bool simulation() const { return _simulation; }
  bool waitOperator() const { return _waitOperator; }
  bool dryAlarm() const { return _dryAlarm; }
  Phase phase() const { return _phase; }
  uint8_t zoneIndex() const { return _zoneIndex; }
  uint8_t orderIndex() const { return _orderIndex; }
  uint16_t error() const { return _error; }
  uint32_t elapsedSeconds(uint32_t nowMs) const;
  uint16_t knownTimedSeconds() const { return _knownTimedSeconds; }
  uint16_t remainingWaitSeconds(uint32_t nowMs) const;
  uint16_t currentStep() const;
  uint16_t totalSteps() const;
  const __FlashStringHelper* phaseName() const;

  // Bench simulation coordinates for diagnostics/DWIN. Valid only when simulation()==true.
  int32_t simPosition(SensorIndex idx) const { return idx < SENSOR_COUNT ? _simPos[idx] : 0; }

private:
  void resetRuntime();
  void setPhase(Phase p, uint32_t nowMs);
  void fail(AutoErrorV6 err, const __FlashStringHelper* reason);
  bool advanceToEnabledZone();
  bool checkMovementTimeout(uint32_t nowMs);
  bool realSensorsUsable(const AutoSensorsV6& sensors) const;
  int32_t pos(const AutoSensorsV6& sensors, SensorIndex idx) const;

  int16_t targetPct(SensorIndex idx, int32_t cur, int32_t target, uint8_t capPct) const;
  bool moveHorizontal(uint32_t nowMs, const AutoSensorsV6& sensors,
                      int32_t x1, int32_t x2, uint8_t capPct, uint8_t limitMask);
  bool moveVertical(uint32_t nowMs, const AutoSensorsV6& sensors,
                    int32_t z1, int32_t z2, uint8_t capPct);
  bool moveOneVertical(uint32_t nowMs, const AutoSensorsV6& sensors,
                       uint8_t side, int32_t target, uint8_t capPct);
  void commandTargets(int16_t h1, int16_t h2, int16_t v1Up, int16_t v2Up);
  void stopMotion();
  void integrateSimulation(uint32_t nowMs, int16_t h1, int16_t h2, int16_t v1Up, int16_t v2Up);
  void seedSimulation(const AutoProgramV6& program, const AutoSensorsV6& sensors);

private:
  MotorControlV6* _motor = nullptr;
  const SettingsV6* _settings = nullptr;
  const AutoProgramV6* _program = nullptr;

  Phase _phase = Phase::IDLE;
  bool _running = false;
  bool _paused = false;
  bool _simulation = false;
  bool _waitOperator = false;
  bool _operatorNext = false;
  bool _dryAlarm = false;
  bool _stateChanged = false;
  bool _homeOnly = false;
  SystemModeV6 _motorMode = SystemModeV6::AUTO;
  uint16_t _error = AUTO_ERR_NONE;

  uint8_t _orderIndex = 0;
  uint8_t _zoneIndex = 0;
  uint8_t _cycleZoneOrdinal = 0; // number of enabled zones already completed
  uint8_t _lowSide = 0;
  uint8_t _stagingZone = 0;
  int32_t _baseZ[2] = {0,0};
  int32_t _lowTiltTarget = 0;
  int32_t _highLiftTarget = 0;

  uint32_t _runStartMs = 0;
  uint32_t _runEndMs = 0;
  uint32_t _phaseStartMs = 0;
  uint32_t _pauseStartMs = 0;
  uint32_t _pausedAccumMs = 0;
  uint32_t _dryTimerStartMs = 0;
  uint32_t _simLastMs = 0;
  uint16_t _knownTimedSeconds = 0;
  uint8_t _enabledZoneCount = 0;
  int32_t _simPos[SENSOR_COUNT] = {0,0,0,0};
};
