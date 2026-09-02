#pragma once

#include <Arduino.h>
#include <stdint.h>

enum SensorIndex : uint8_t {
  SENSOR_X1 = 0,
  SENSOR_X2 = 1,
  SENSOR_Z1 = 2,
  SENSOR_Z2 = 3,
  SENSOR_COUNT = 4
};

enum DriveIndexV6 : uint8_t {
  DRIVE_H1 = 0,
  DRIVE_H2 = 1,
  DRIVE_V1 = 2,
  DRIVE_V2 = 3,
  DRIVE_COUNT_V6 = 4
};

enum VfdBaudCodeV6 : uint8_t {
  VFD_BAUD_4800 = 0,
  VFD_BAUD_9600 = 1,
  VFD_BAUD_19200 = 2,
  VFD_BAUD_38400 = 3,
  VFD_BAUD_57600 = 4,
  VFD_BAUD_115200 = 5
};

enum VfdParityV6 : uint8_t {
  VFD_PARITY_NONE = 0,
  VFD_PARITY_EVEN = 1,
  VFD_PARITY_ODD = 2
};

struct SensorCalV6 {
  int16_t offsetMm;
  uint8_t invert;
  uint8_t enabled;
};

enum SafetyDisableBitsV6 : uint8_t {
  SAFETY_DISABLE_NONE   = 0x00,
  SAFETY_DISABLE_ESTOP  = 0x01,
  SAFETY_DISABLE_LIMITS = 0x02
};

struct VfdCommSettingsV6 {
  uint8_t baudCode;
  uint8_t parity;
  uint8_t stopBits;
  uint8_t retries;

  uint16_t responseTimeoutMs;
  uint16_t interRequestMs;
  uint16_t onlinePollMs;
  uint16_t offlinePollMs;

  uint8_t address[DRIVE_COUNT_V6];
  uint8_t invertDirectionMask;
  // Kept in the same byte as legacy reserved0: bit=1 means disabled.
  // Zero in existing EEPROM therefore preserves the original safe behaviour.
  uint8_t safetyDisableMask;
  uint16_t manualJogTimeoutMs;
};

struct DriveProfileV6 {
  uint8_t manualPercent;
  uint8_t maxPercent;
  uint8_t slowPercent;
  uint8_t reserved0;
  uint16_t slowdownDistanceMm;
  uint16_t stopToleranceMm;
};

struct SettingsV6 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  SensorCalV6 sensor[SENSOR_COUNT];
  uint16_t staleTimeoutMs;
  uint16_t samplePeriodMs;

  VfdCommSettingsV6 vfd;
  DriveProfileV6 drive[DRIVE_COUNT_V6];

  uint16_t crc;
};

enum SettingsValidationErrorV6 : uint16_t {
  SETTINGS_VALID = 0x0000,
  SETTINGS_ERR_BAUD = 0x0001,
  SETTINGS_ERR_PARITY = 0x0002,
  SETTINGS_ERR_STOP_BITS = 0x0004,
  SETTINGS_ERR_TIMEOUT = 0x0008,
  SETTINGS_ERR_RETRIES = 0x0010,
  SETTINGS_ERR_INTERVAL = 0x0020,
  SETTINGS_ERR_ADDRESS_RANGE = 0x0040,
  SETTINGS_ERR_ADDRESS_DUPLICATE = 0x0080,
  SETTINGS_ERR_WATCHDOG = 0x0100,
  SETTINGS_ERR_DRIVE_PROFILE = 0x0200,
  SETTINGS_ERR_SENSOR = 0x0400,
  SETTINGS_ERR_EEPROM = 0x0800,
  SETTINGS_ERR_UNSAFE_MODE = 0x8000
};

class SettingsStorageV6 {
public:
  void defaults(SettingsV6& cfg) const;
  void defaultsVfdSection(SettingsV6& cfg) const;

  // Returns true for a valid current record and for a successfully migrated v1 record.
  bool load(SettingsV6& cfg) const;
  void save(SettingsV6& cfg) const;

  bool lastLoadMigrated() const { return _lastLoadMigrated; }
  uint16_t validate(const SettingsV6& cfg) const;

  void resetOffsets(SettingsV6& cfg) const;
  void zeroSensor(SettingsV6& cfg, SensorIndex idx, int32_t rawMm) const;

  int32_t applyCalibration(const SettingsV6& cfg, SensorIndex idx, int32_t rawMm) const;

  static uint32_t baudFromCode(uint8_t baudCode);

private:
  uint16_t calcCrc(const SettingsV6& cfg) const;
  bool hasDuplicateAddresses(const SettingsV6& cfg) const;

private:
  mutable bool _lastLoadMigrated = false;
};
