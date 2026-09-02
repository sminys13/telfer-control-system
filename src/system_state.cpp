#include "system_state.h"

uint16_t systemModeToVp(SystemModeV6 mode)
{
  switch (mode)
  {
  case SystemModeV6::SERVICE:
    return MODE_SERVICE;
  case SystemModeV6::MANUAL:
    return MODE_MANUAL;
  case SystemModeV6::AUTO:
    return MODE_AUTO;
  case SystemModeV6::HOME:
    return MODE_HOME;
  case SystemModeV6::STOP:
    return MODE_STOP;
  case SystemModeV6::SETTINGS:
    return MODE_SETTINGS;
  case SystemModeV6::CALIBRATION:
    return MODE_CALIBRATION;
  }
  return MODE_SERVICE;
}

const __FlashStringHelper* systemModeName(SystemModeV6 mode)
{
  switch (mode)
  {
  case SystemModeV6::SERVICE:
    return F("SERVICE");
  case SystemModeV6::MANUAL:
    return F("MANUAL");
  case SystemModeV6::AUTO:
    return F("AUTO");
  case SystemModeV6::HOME:
    return F("HOME");
  case SystemModeV6::STOP:
    return F("STOP");
  case SystemModeV6::SETTINGS:
    return F("SETTINGS");
  case SystemModeV6::CALIBRATION:
    return F("CALIBRATION");
  }
  return F("?");
}
