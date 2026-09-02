#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config_v6_bringup.h"
#include "modbus.h"
#include "motors.h"
#include "settings_v6.h"

// V6 VFD facade.
// Step7B connects the already existing ModbusMasterRTU and Drives layers, but
// keeps the physical MAX485 transport disabled. Complete NE200 RTU frames are
// printed to the PlatformIO monitor in dry-run mode.
class VfdDriverV6 {
public:
  void begin(const SettingsV6& settings);
  void applySettings(const SettingsV6& settings);
  bool service(uint32_t nowMs);

  void startByMotorState(uint16_t motorStateCode);
  // Automatic controller uses logical coordinates: H positive=right, V positive=UP.
  // The V physical sign conversion (NE200 Forward=DOWN in this installation) stays here.
  bool setAutoLogicalTargets(int16_t h1RightPct, int16_t h2RightPct,
                             int16_t v1UpPct, int16_t v2UpPct);
  void stopAll(const __FlashStringHelper* reason);
  void block(const __FlashStringHelper* reason);
  void testConnection(uint8_t driveIndex);

  uint16_t statusCode() const { return _status; }
  uint8_t address(uint8_t driveIndex) const;
  uint32_t baud() const { return SettingsStorageV6::baudFromCode(_comm.baudCode); }
  bool hasPendingWork() const { return _drives.hasPendingWork(); }
  uint8_t connectedCount() const;

private:
  uint16_t serialMode() const;
  void printConfiguration() const;
  void printMovePlan(uint16_t motorStateCode) const;
  void setExclusiveTargets(int16_t h1, int16_t h2, int16_t v1, int16_t v2);
  int16_t manualPercent(uint8_t driveIndex) const;

private:
  uint16_t _status = VFD_STATUS_DISABLED;
  VfdCommSettingsV6 _comm{};
  DriveProfileV6 _drive[DRIVE_COUNT_V6]{};
  ModbusMasterRTU _modbus;
  Drives _drives;
  int16_t _lastAutoLogical[DRIVE_COUNT_V6] = {0,0,0,0};
  bool _autoTargetKnown = false;
  bool _begun = false;
};
