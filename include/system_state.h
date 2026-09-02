#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config_v6_bringup.h"

// High-level system states restored for the DWIN project.
// These are internal states. DWIN button codes are CMD_* in config_v6_bringup.h.
enum class SystemModeV6 : uint8_t
{
  SERVICE = MODE_SERVICE,
  MANUAL = MODE_MANUAL,
  AUTO = MODE_AUTO,
  HOME = MODE_HOME,
  STOP = MODE_STOP,
  SETTINGS = MODE_SETTINGS,
  CALIBRATION = MODE_CALIBRATION
};

uint16_t systemModeToVp(SystemModeV6 mode);
const __FlashStringHelper* systemModeName(SystemModeV6 mode);
