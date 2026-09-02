/**
 * @file motors.h
 * @brief Four-drive NE200 command scheduler reused by the V6 project.
 *
 * Important Step7B rule: at most ONE Modbus transaction is executed per tick.
 * A run request is therefore split into:
 *   1) write setpoint 0x0002
 *   2) write direction command 0x0001
 * A stop request is split into:
 *   1) write STOP to 0x0001
 *   2) write zero setpoint to 0x0002
 */
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config_v6_bringup.h"
#include "modbus.h"
#include "settings_v6.h"

enum class DriveId : uint8_t { H1 = 0, H2 = 1, V1 = 2, V2 = 3, COUNT = 4 };

struct DriveMap {
  uint8_t addr;
  bool invertDir;
};

struct DriveTelemetry {
  // HE200 read-only monitoring snapshot (Step9E).
  uint16_t runningFreq001Hz = 0;
  uint16_t setFreq001Hz = 0;
  uint16_t busVoltage01V = 0;
  uint16_t outputVoltageV = 0;
  uint16_t outputCurrent001A = 0;
  uint16_t digitalInputState = 0;
  uint16_t faultCode = 0;
  uint16_t currentSetFreq001Pct = 0;
  uint16_t currentRunFreq001Pct = 0;
  uint16_t statusWord = 0; // HE200 running state (0x703D)
  bool connected = false;
  uint8_t lastErr = MODBUS_ERROR_NONE;
  uint32_t lastOkMs = 0;
};

class Drives {
public:
  void begin(ModbusMasterRTU& mb, const SettingsV6& settings);
  void applySettings(const SettingsV6& settings);

  // speedPct: -100..100. Sign selects requested direction.
  void setSpeed(DriveId id, int16_t speedPct);
  void stop(DriveId id);
  void stopAll();
  void resetFault(DriveId id);
  void requestSafeStatusRead(DriveId id);
  void requestSafeStatusReadAll();

  // Executes at most one Modbus transaction.
  bool tick(uint32_t nowMs);

  const DriveTelemetry& telemetry(DriveId id) const { return _tel[indexOf(id)]; }
  int16_t targetPct(DriveId id) const { return _st[indexOf(id)].targetPct; }
  bool hasPendingWork() const;

  static const __FlashStringHelper* driveName(DriveId id);

private:
  enum class TxPhase : uint8_t {
    IDLE = 0,
    WRITE_SETPOINT,
    WRITE_RUN_COMMAND,
    WRITE_STOP_COMMAND,
    WRITE_ZERO_SETPOINT,
    WRITE_RESET_FAULT,
    READ_SAFE_STATUS,
    READ_HE200_FAULT,
    READ_HE200_STATE
  };

  struct DriveState {
    int16_t targetPct = 0;
    int16_t appliedPct = 0;
    int16_t transactionPct = 0;
    TxPhase phase = TxPhase::IDLE;
    uint8_t failStreak = 0;
    bool forceStop = false;
    bool resetRequested = false;
    bool statusReadRequested = false;
  };

  ModbusMasterRTU* _mb = nullptr;
  DriveMap _map[(uint8_t)DriveId::COUNT]{};
  DriveState _st[(uint8_t)DriveId::COUNT]{};
  DriveTelemetry _tel[(uint8_t)DriveId::COUNT]{};
  uint8_t _activeDrive = 0xFF;
  uint8_t _rrStart = 0;
  uint16_t _interRequestMs = 10;
  uint32_t _lastTransactionMs = 0;

  static uint8_t indexOf(DriveId id) { return (uint8_t)id; }
  DriveId driveFromIndex(uint8_t i) const { return (DriveId)i; }

  bool selectNextWork();
  void preparePhase(uint8_t i);
  bool processActiveDrive(uint32_t nowMs);
  bool finishTransaction(uint8_t i, const ModbusResult& r, TxPhase nextOnSuccess);
  uint16_t setpointMagnitude(int16_t effectivePct) const;
  int16_t effectivePercent(uint8_t i, int16_t requestedPct) const;
  uint16_t directionCommand(int16_t effectivePct) const;
  void printPlan(uint8_t i, const __FlashStringHelper* action,
                 uint16_t reg, uint16_t value) const;
};
