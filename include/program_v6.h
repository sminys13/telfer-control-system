#pragma once

#include <Arduino.h>
#include <stdint.h>

static constexpr uint8_t AUTO_MAX_ZONES_V6 = 10;
static constexpr uint8_t AUTO_PROGRAM_SLOTS_V6 = 4;

enum AutoZoneValidBitsV6 : uint8_t {
  AUTO_ZONE_VALID_NONE = 0x00,
  AUTO_ZONE_VALID_X    = 0x01,
  AUTO_ZONE_VALID_Z    = 0x02,
  AUTO_ZONE_VALID_ALL  = AUTO_ZONE_VALID_X | AUTO_ZONE_VALID_Z
};

enum AutoProgramValidBitsV6 : uint8_t {
  AUTO_PROGRAM_VALID_NONE   = 0x00,
  AUTO_PROGRAM_VALID_HOME_X = 0x01,
  AUTO_PROGRAM_VALID_TRAVEL_Z = 0x02,
  AUTO_PROGRAM_VALID_DRY_X  = 0x04,
  AUTO_PROGRAM_VALID_DRY_Z  = 0x08
};

struct AutoZoneV6 {
  int32_t xMm[2];                 // X1/X2 target, mm
  int32_t zMm[2];                 // Z1/Z2 target, mm; logical UP increases coordinate
  uint16_t dipTimeS;              // immersion dwell
  uint16_t tiltStepMm;            // requested Z1/Z2 differential during drain tilt, mm (|Z1-Z2|)
  uint16_t stepWaitS;             // wait after tilt
  uint8_t movePercent;            // horizontal cap
  uint8_t verticalPercent;        // vertical cap
  uint8_t enabled;
  uint8_t validMask;              // AutoZoneValidBitsV6
};

struct AutoProgramV6 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char name[12];

  uint8_t zoneCount;
  uint8_t order[AUTO_MAX_ZONES_V6];
  AutoZoneV6 zones[AUTO_MAX_ZONES_V6];

  int32_t homeX[2];
  int32_t travelZ[2];
  uint16_t dripWaitS;
  uint8_t tiltPercent;            // legacy v_tilt_speed_pct, default 35%
  uint8_t lowSide;                // 0=V1 is low side while dipping, 1=V2
  uint8_t validMask;              // AutoProgramValidBitsV6

  uint8_t dryingEnabled;
  uint8_t dryValid;
  uint8_t stagingZone;
  uint8_t reserved0;
  uint16_t dryingTimeS;
  int32_t dryX[2];
  int32_t dryZ[2];

  uint16_t crc;
};

class ProgramStorageV6 {
public:
  void defaults(AutoProgramV6& p, uint8_t slot) const;
  void demo(AutoProgramV6& p) const;
  bool validate(const AutoProgramV6& p) const;
  bool readyForAuto(const AutoProgramV6& p) const;

  bool load(uint8_t slot, AutoProgramV6& p) const;
  bool save(uint8_t slot, AutoProgramV6& p) const;

  uint8_t loadActiveSlot() const;
  void saveActiveSlot(uint8_t slot) const;

  uint16_t knownTimedSeconds(const AutoProgramV6& p) const;
  uint8_t enabledZoneCount(const AutoProgramV6& p) const;

private:
  uint16_t calcCrc(const AutoProgramV6& p) const;
  int slotAddress(uint8_t slot) const;
};
