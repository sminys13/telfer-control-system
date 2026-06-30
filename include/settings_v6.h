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

struct SensorCalV6 {
  int16_t offsetMm;
  uint8_t invert;
  uint8_t enabled;
};

struct SettingsV6 {
  uint32_t magic;
  uint16_t version;
  SensorCalV6 sensor[SENSOR_COUNT];
  uint16_t staleTimeoutMs;
  uint16_t samplePeriodMs;
  uint16_t crc;
};

class SettingsStorageV6 {
public:
  void defaults(SettingsV6& cfg) const;
  bool load(SettingsV6& cfg) const;
  void save(SettingsV6& cfg) const;

  void resetOffsets(SettingsV6& cfg) const;
  void zeroSensor(SettingsV6& cfg, SensorIndex idx, int32_t rawMm) const;

  int32_t applyCalibration(const SettingsV6& cfg, SensorIndex idx, int32_t rawMm) const;

private:
  uint16_t calcCrc(const SettingsV6& cfg) const;
};
