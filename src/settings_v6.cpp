#include "settings_v6.h"
#include <EEPROM.h>

static constexpr uint32_t SETTINGS_MAGIC = 0x54464C36UL; // 'TFL6'
static constexpr uint16_t SETTINGS_VERSION = 1;
static constexpr int EEPROM_ADDR_SETTINGS = 0;

void SettingsStorageV6::defaults(SettingsV6& cfg) const {
  cfg.magic = SETTINGS_MAGIC;
  cfg.version = SETTINGS_VERSION;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    cfg.sensor[i].offsetMm = 0;
    cfg.sensor[i].invert = 0;
    cfg.sensor[i].enabled = 1;
  }

  cfg.staleTimeoutMs = 1200;
  cfg.samplePeriodMs = 180;
  cfg.crc = calcCrc(cfg);
}

uint16_t SettingsStorageV6::calcCrc(const SettingsV6& cfg) const {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&cfg);
  const size_t n = sizeof(SettingsV6) - sizeof(cfg.crc);

  uint16_t crc = 0xA55A;
  for (size_t i = 0; i < n; ++i) {
    crc = (uint16_t)((crc << 5) | (crc >> 11));
    crc ^= p[i];
  }
  return crc;
}

bool SettingsStorageV6::load(SettingsV6& cfg) const {
  SettingsV6 tmp;
  EEPROM.get(EEPROM_ADDR_SETTINGS, tmp);

  if (tmp.magic != SETTINGS_MAGIC) return false;
  if (tmp.version != SETTINGS_VERSION) return false;
  if (tmp.crc != calcCrc(tmp)) return false;

  cfg = tmp;
  return true;
}

void SettingsStorageV6::save(SettingsV6& cfg) const {
  cfg.magic = SETTINGS_MAGIC;
  cfg.version = SETTINGS_VERSION;
  cfg.crc = calcCrc(cfg);
  EEPROM.put(EEPROM_ADDR_SETTINGS, cfg);
}

void SettingsStorageV6::resetOffsets(SettingsV6& cfg) const {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    cfg.sensor[i].offsetMm = 0;
    cfg.sensor[i].invert = 0;
  }
  cfg.crc = calcCrc(cfg);
}

void SettingsStorageV6::zeroSensor(SettingsV6& cfg, SensorIndex idx, int32_t rawMm) const {
  if (idx >= SENSOR_COUNT) return;

  if (cfg.sensor[idx].invert) {
    cfg.sensor[idx].offsetMm = (int16_t)rawMm;
  } else {
    cfg.sensor[idx].offsetMm = (int16_t)(-rawMm);
  }

  cfg.crc = calcCrc(cfg);
}

int32_t SettingsStorageV6::applyCalibration(const SettingsV6& cfg, SensorIndex idx, int32_t rawMm) const {
  if (idx >= SENSOR_COUNT) return rawMm;

  if (cfg.sensor[idx].invert) {
    return -rawMm + cfg.sensor[idx].offsetMm;
  }

  return rawMm + cfg.sensor[idx].offsetMm;
}
