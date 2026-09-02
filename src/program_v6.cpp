#include "program_v6.h"
#include <EEPROM.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

static constexpr uint32_t AUTO_PROGRAM_MAGIC_V6 = 0x41503636UL; // 'AP66'
static constexpr uint16_t AUTO_PROGRAM_VERSION_V6 = 1;
static constexpr int EEPROM_AUTO_META_ADDR_V6 = 384;
static constexpr int EEPROM_AUTO_BASE_ADDR_V6 = 512;
static constexpr uint32_t AUTO_META_MAGIC_V6 = 0x41504D36UL; // 'APM6'

struct AutoMetaV6 {
  uint32_t magic;
  uint8_t activeSlot;
  uint8_t reserved;
  uint16_t crc;
};

static uint16_t rollingCrcV6(const uint8_t* p, size_t n) {
  uint16_t crc = 0x6A3D;
  for (size_t i = 0; i < n; ++i) {
    crc = (uint16_t)((crc << 3) | (crc >> 13));
    crc ^= p[i];
    crc = (uint16_t)(crc + 0x31u);
  }
  return crc;
}

static uint16_t metaCrc(const AutoMetaV6& m) {
  return rollingCrcV6(reinterpret_cast<const uint8_t*>(&m), offsetof(AutoMetaV6, crc));
}

int ProgramStorageV6::slotAddress(uint8_t slot) const {
  return EEPROM_AUTO_BASE_ADDR_V6 + (int)slot * (int)sizeof(AutoProgramV6);
}

uint16_t ProgramStorageV6::calcCrc(const AutoProgramV6& p) const {
  return rollingCrcV6(reinterpret_cast<const uint8_t*>(&p), offsetof(AutoProgramV6, crc));
}

void ProgramStorageV6::defaults(AutoProgramV6& p, uint8_t slot) const {
  memset(&p, 0, sizeof(p));
  p.magic = AUTO_PROGRAM_MAGIC_V6;
  p.version = AUTO_PROGRAM_VERSION_V6;
  p.size = sizeof(AutoProgramV6);
  snprintf(p.name, sizeof(p.name), "PROG%u", (unsigned)(slot + 1));
  p.zoneCount = 1;
  for (uint8_t i = 0; i < AUTO_MAX_ZONES_V6; ++i) {
    p.order[i] = i;
    AutoZoneV6& z = p.zones[i];
    z.xMm[0] = 0;
    z.xMm[1] = 0;
    z.zMm[0] = 0;
    z.zMm[1] = 0;
    z.dipTimeS = 60;
    z.tiltStepMm = 30;
    z.stepWaitS = 30;
    z.movePercent = 55;
    z.verticalPercent = 45;
    z.enabled = (i == 0) ? 1 : 0;
    z.validMask = AUTO_ZONE_VALID_NONE; // capture is mandatory before real AUTO
  }
  p.homeX[0] = p.homeX[1] = 0;
  p.travelZ[0] = p.travelZ[1] = 0;
  p.dripWaitS = 45;
  p.tiltPercent = 35;
  p.lowSide = 0;
  p.validMask = AUTO_PROGRAM_VALID_NONE;
  p.dryingEnabled = 0;
  p.dryValid = 0;
  p.stagingZone = 0;
  p.dryingTimeS = 1800;
  p.dryX[0] = p.dryX[1] = 0;
  p.dryZ[0] = p.dryZ[1] = 0;
  p.crc = calcCrc(p);
}

void ProgramStorageV6::demo(AutoProgramV6& p) const {
  defaults(p, 0);
  strncpy(p.name, "SIM-DEMO", sizeof(p.name) - 1);
  p.name[sizeof(p.name) - 1] = '\0';
  p.zoneCount = 2;
  p.homeX[0] = 1000; p.homeX[1] = 1000;
  p.travelZ[0] = 1000; p.travelZ[1] = 1000;
  p.validMask = AUTO_PROGRAM_VALID_HOME_X | AUTO_PROGRAM_VALID_TRAVEL_Z;
  p.dripWaitS = 2;

  p.zones[0].xMm[0] = 2000; p.zones[0].xMm[1] = 2000;
  p.zones[0].zMm[0] = 500;  p.zones[0].zMm[1] = 500;
  p.zones[0].dipTimeS = 3;
  p.zones[0].tiltStepMm = 100;
  p.zones[0].stepWaitS = 2;
  p.zones[0].movePercent = 55;
  p.zones[0].verticalPercent = 45;
  p.zones[0].enabled = 1;
  p.zones[0].validMask = AUTO_ZONE_VALID_ALL;

  p.zones[1].xMm[0] = 3200; p.zones[1].xMm[1] = 3200;
  p.zones[1].zMm[0] = 450;  p.zones[1].zMm[1] = 450;
  p.zones[1].dipTimeS = 3;
  p.zones[1].tiltStepMm = 80;
  p.zones[1].stepWaitS = 2;
  p.zones[1].movePercent = 55;
  p.zones[1].verticalPercent = 45;
  p.zones[1].enabled = 1;
  p.zones[1].validMask = AUTO_ZONE_VALID_ALL;
  p.order[0] = 0;
  p.order[1] = 1;
  p.crc = calcCrc(p);
}

bool ProgramStorageV6::validate(const AutoProgramV6& p) const {
  if (p.magic != AUTO_PROGRAM_MAGIC_V6 || p.version != AUTO_PROGRAM_VERSION_V6 || p.size != sizeof(AutoProgramV6)) return false;
  if (p.crc != calcCrc(p)) return false;
  if (p.zoneCount < 1 || p.zoneCount > AUTO_MAX_ZONES_V6) return false;
  if (p.lowSide > 1) return false;
  if (p.tiltPercent < 1 || p.tiltPercent > 100) return false;
  if (p.stagingZone >= p.zoneCount) return false;
  for (uint8_t i = 0; i < p.zoneCount; ++i) {
    if (p.order[i] >= p.zoneCount) return false;
  }
  for (uint8_t i = 0; i < AUTO_MAX_ZONES_V6; ++i) {
    const AutoZoneV6& z = p.zones[i];
    if (z.movePercent < 1 || z.movePercent > 100) return false;
    if (z.verticalPercent < 1 || z.verticalPercent > 100) return false;
    if (z.tiltStepMm > 5000) return false;
    if (z.stepWaitS > 36000 || z.dipTimeS > 36000) return false;
    if (z.validMask & (uint8_t)~AUTO_ZONE_VALID_ALL) return false;
  }
  return true;
}

uint8_t ProgramStorageV6::enabledZoneCount(const AutoProgramV6& p) const {
  uint8_t n = 0;
  for (uint8_t oi = 0; oi < p.zoneCount; ++oi) {
    const uint8_t zid = p.order[oi];
    if (zid < p.zoneCount && p.zones[zid].enabled) ++n;
  }
  return n;
}

bool ProgramStorageV6::readyForAuto(const AutoProgramV6& p) const {
  AutoProgramV6 tmp = p;
  tmp.magic = AUTO_PROGRAM_MAGIC_V6;
  tmp.version = AUTO_PROGRAM_VERSION_V6;
  tmp.size = sizeof(AutoProgramV6);
  tmp.crc = calcCrc(tmp);
  if (!validate(tmp)) return false;
  const uint8_t required = AUTO_PROGRAM_VALID_HOME_X | AUTO_PROGRAM_VALID_TRAVEL_Z;
  if ((p.validMask & required) != required) return false;
  bool hasZone = false;
  for (uint8_t oi = 0; oi < p.zoneCount; ++oi) {
    const uint8_t zid = p.order[oi];
    if (zid >= p.zoneCount) return false;
    const AutoZoneV6& z = p.zones[zid];
    if (!z.enabled) continue;
    hasZone = true;
    if ((z.validMask & AUTO_ZONE_VALID_ALL) != AUTO_ZONE_VALID_ALL) return false;
  }
  if (!hasZone) return false;
  if (p.dryingEnabled) {
    const uint8_t dryRequired = AUTO_PROGRAM_VALID_DRY_X | AUTO_PROGRAM_VALID_DRY_Z;
    if (!p.dryValid || (p.validMask & dryRequired) != dryRequired) return false;
    // Staging uses the selected zone's X coordinate even if that process zone
    // itself is disabled, therefore its X pair must be explicitly calibrated.
    if (p.stagingZone >= p.zoneCount || (p.zones[p.stagingZone].validMask & AUTO_ZONE_VALID_X) == 0) return false;
  }
  return true;
}

bool ProgramStorageV6::load(uint8_t slot, AutoProgramV6& p) const {
  if (slot >= AUTO_PROGRAM_SLOTS_V6) return false;
  EEPROM.get(slotAddress(slot), p);
  return validate(p);
}

bool ProgramStorageV6::save(uint8_t slot, AutoProgramV6& p) const {
  if (slot >= AUTO_PROGRAM_SLOTS_V6) return false;
  p.magic = AUTO_PROGRAM_MAGIC_V6;
  p.version = AUTO_PROGRAM_VERSION_V6;
  p.size = sizeof(AutoProgramV6);
  p.crc = calcCrc(p);
  if (!validate(p)) return false;
  const int end = slotAddress(slot) + (int)sizeof(AutoProgramV6);
  if (end > EEPROM.length()) return false;
  EEPROM.put(slotAddress(slot), p);
  return true;
}

uint8_t ProgramStorageV6::loadActiveSlot() const {
  AutoMetaV6 m{};
  EEPROM.get(EEPROM_AUTO_META_ADDR_V6, m);
  if (m.magic != AUTO_META_MAGIC_V6 || m.activeSlot >= AUTO_PROGRAM_SLOTS_V6 || m.crc != metaCrc(m)) return 0;
  return m.activeSlot;
}

void ProgramStorageV6::saveActiveSlot(uint8_t slot) const {
  if (slot >= AUTO_PROGRAM_SLOTS_V6) return;
  AutoMetaV6 m{};
  m.magic = AUTO_META_MAGIC_V6;
  m.activeSlot = slot;
  m.crc = metaCrc(m);
  EEPROM.put(EEPROM_AUTO_META_ADDR_V6, m);
}

uint16_t ProgramStorageV6::knownTimedSeconds(const AutoProgramV6& p) const {
  uint32_t total = 0;
  for (uint8_t oi = 0; oi < p.zoneCount; ++oi) {
    const uint8_t zid = p.order[oi];
    if (zid >= p.zoneCount || !p.zones[zid].enabled) continue;
    total += p.zones[zid].stepWaitS;
    total += p.zones[zid].dipTimeS;
    total += p.dripWaitS;
  }
  if (p.dryingEnabled) total += p.dryingTimeS;
  return total > 65535UL ? 65535 : (uint16_t)total;
}
