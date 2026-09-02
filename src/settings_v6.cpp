#include "settings_v6.h"
#include <EEPROM.h>
#include <string.h>
#include <stddef.h>

static constexpr uint32_t SETTINGS_MAGIC = 0x54464C36UL; // 'TFL6'
static constexpr uint16_t SETTINGS_VERSION = 2;
static constexpr int EEPROM_ADDR_SETTINGS = 0;

// Exact layout used by system-step6 and earlier sensor-core builds.
struct SettingsV6LegacyV1 {
  uint32_t magic;
  uint16_t version;
  SensorCalV6 sensor[SENSOR_COUNT];
  uint16_t staleTimeoutMs;
  uint16_t samplePeriodMs;
  uint16_t crc;
};

static uint16_t calcRollingCrc(const uint8_t* p, size_t n) {
  uint16_t crc = 0xA55A;
  for (size_t i = 0; i < n; ++i) {
    crc = (uint16_t)((crc << 5) | (crc >> 11));
    crc ^= p[i];
  }
  return crc;
}

static uint16_t calcLegacyCrc(const SettingsV6LegacyV1& cfg) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&cfg);
  return calcRollingCrc(p, offsetof(SettingsV6LegacyV1, crc));
}

void SettingsStorageV6::defaultsVfdSection(SettingsV6& cfg) const {
  cfg.vfd.baudCode = VFD_BAUD_9600;
  cfg.vfd.parity = VFD_PARITY_EVEN;
  cfg.vfd.stopBits = 1;
  cfg.vfd.retries = 1;
  cfg.vfd.responseTimeoutMs = 150;
  cfg.vfd.interRequestMs = 10;
  cfg.vfd.onlinePollMs = 100;
  cfg.vfd.offlinePollMs = 1000;
  cfg.vfd.address[DRIVE_H1] = 1;
  cfg.vfd.address[DRIVE_H2] = 2;
  cfg.vfd.address[DRIVE_V1] = 3;
  cfg.vfd.address[DRIVE_V2] = 4;
  cfg.vfd.invertDirectionMask = 0;
  cfg.vfd.safetyDisableMask = SAFETY_DISABLE_NONE;
  cfg.vfd.manualJogTimeoutMs = 2500;

  cfg.drive[DRIVE_H1] = {20, 80, 10, 0, 500, 10};
  cfg.drive[DRIVE_H2] = {20, 80, 10, 0, 500, 10};
  cfg.drive[DRIVE_V1] = {20, 60, 10, 0, 400, 8};
  cfg.drive[DRIVE_V2] = {20, 60, 10, 0, 400, 8};
}

void SettingsStorageV6::defaults(SettingsV6& cfg) const {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = SETTINGS_MAGIC;
  cfg.version = SETTINGS_VERSION;
  cfg.size = sizeof(SettingsV6);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    cfg.sensor[i].offsetMm = 0;
    cfg.sensor[i].invert = 0;
    cfg.sensor[i].enabled = 1;
  }

  cfg.staleTimeoutMs = 1200;
  cfg.samplePeriodMs = 180;
  defaultsVfdSection(cfg);
  cfg.crc = calcCrc(cfg);
}

uint16_t SettingsStorageV6::calcCrc(const SettingsV6& cfg) const {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&cfg);
  return calcRollingCrc(p, offsetof(SettingsV6, crc));
}

bool SettingsStorageV6::hasDuplicateAddresses(const SettingsV6& cfg) const {
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    for (uint8_t j = i + 1; j < DRIVE_COUNT_V6; ++j) {
      if (cfg.vfd.address[i] == cfg.vfd.address[j]) return true;
    }
  }
  return false;
}

uint16_t SettingsStorageV6::validate(const SettingsV6& cfg) const {
  uint16_t err = SETTINGS_VALID;

  if (cfg.vfd.baudCode > VFD_BAUD_115200) err |= SETTINGS_ERR_BAUD;
  if (cfg.vfd.parity > VFD_PARITY_ODD) err |= SETTINGS_ERR_PARITY;
  if (cfg.vfd.stopBits != 1 && cfg.vfd.stopBits != 2) err |= SETTINGS_ERR_STOP_BITS;
  if (cfg.vfd.responseTimeoutMs < 20 || cfg.vfd.responseTimeoutMs > 5000) err |= SETTINGS_ERR_TIMEOUT;
  if (cfg.vfd.retries > 5) err |= SETTINGS_ERR_RETRIES;
  if (cfg.vfd.interRequestMs > 1000 ||
      cfg.vfd.onlinePollMs < 20 || cfg.vfd.onlinePollMs > 10000 ||
      cfg.vfd.offlinePollMs < 100 || cfg.vfd.offlinePollMs > 60000) {
    err |= SETTINGS_ERR_INTERVAL;
  }

  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    if (cfg.vfd.address[i] < 1 || cfg.vfd.address[i] > 247) {
      err |= SETTINGS_ERR_ADDRESS_RANGE;
    }
  }
  if (hasDuplicateAddresses(cfg)) err |= SETTINGS_ERR_ADDRESS_DUPLICATE;

  if (cfg.vfd.manualJogTimeoutMs < 500 || cfg.vfd.manualJogTimeoutMs > 60000) {
    err |= SETTINGS_ERR_WATCHDOG;
  }

  if (cfg.vfd.safetyDisableMask & (uint8_t)~(SAFETY_DISABLE_ESTOP | SAFETY_DISABLE_LIMITS)) {
    err |= SETTINGS_ERR_UNSAFE_MODE;
  }

  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    const DriveProfileV6& p = cfg.drive[i];
    if (p.manualPercent < 1 || p.manualPercent > 100 ||
        p.maxPercent < 1 || p.maxPercent > 100 ||
        p.slowPercent < 1 || p.slowPercent > p.maxPercent ||
        p.manualPercent > p.maxPercent ||
        p.slowdownDistanceMm < 10 || p.slowdownDistanceMm > 10000 ||
        p.stopToleranceMm < 1 || p.stopToleranceMm > 500) {
      err |= SETTINGS_ERR_DRIVE_PROFILE;
    }
  }

  if (cfg.staleTimeoutMs < 300 || cfg.staleTimeoutMs > 10000 ||
      cfg.samplePeriodMs < 20 || cfg.samplePeriodMs > 2000) {
    err |= SETTINGS_ERR_SENSOR;
  }

  return err;
}

bool SettingsStorageV6::load(SettingsV6& cfg) const {
  _lastLoadMigrated = false;

  uint32_t magic = 0;
  uint16_t version = 0;
  EEPROM.get(EEPROM_ADDR_SETTINGS, magic);
  EEPROM.get(EEPROM_ADDR_SETTINGS + (int)sizeof(magic), version);

  if (magic != SETTINGS_MAGIC) return false;

  if (version == SETTINGS_VERSION) {
    SettingsV6 tmp;
    EEPROM.get(EEPROM_ADDR_SETTINGS, tmp);
    if (tmp.size != sizeof(SettingsV6)) return false;
    if (tmp.crc != calcCrc(tmp)) return false;
    if (validate(tmp) != SETTINGS_VALID) return false;
    cfg = tmp;
    return true;
  }

  if (version == 1) {
    SettingsV6LegacyV1 oldCfg;
    EEPROM.get(EEPROM_ADDR_SETTINGS, oldCfg);
    if (oldCfg.magic != SETTINGS_MAGIC || oldCfg.version != 1) return false;
    if (oldCfg.crc != calcLegacyCrc(oldCfg)) return false;

    defaults(cfg);
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) cfg.sensor[i] = oldCfg.sensor[i];
    cfg.staleTimeoutMs = oldCfg.staleTimeoutMs;
    cfg.samplePeriodMs = oldCfg.samplePeriodMs;

    if (validate(cfg) != SETTINGS_VALID) return false;
    save(cfg); // one-time in-place migration; preserves all calibration offsets
    _lastLoadMigrated = true;
    return true;
  }

  return false;
}

void SettingsStorageV6::save(SettingsV6& cfg) const {
  cfg.magic = SETTINGS_MAGIC;
  cfg.version = SETTINGS_VERSION;
  cfg.size = sizeof(SettingsV6);
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
  return cfg.sensor[idx].invert ? (-rawMm + cfg.sensor[idx].offsetMm)
                                : (rawMm + cfg.sensor[idx].offsetMm);
}

uint32_t SettingsStorageV6::baudFromCode(uint8_t baudCode) {
  switch (baudCode) {
    case VFD_BAUD_4800: return 4800UL;
    case VFD_BAUD_9600: return 9600UL;
    case VFD_BAUD_19200: return 19200UL;
    case VFD_BAUD_38400: return 38400UL;
    case VFD_BAUD_57600: return 57600UL;
    case VFD_BAUD_115200: return 115200UL;
    default: return 9600UL;
  }
}
