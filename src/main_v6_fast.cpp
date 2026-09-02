#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "config_v6_bringup.h"
#include "sc16is752.h"
#include "fast_laser_sensor.h"
#include "dwin_link.h"
#include "settings_v6.h"
#include "system_state.h"
#include "motor_control_v6.h"
#include "safety_v6.h"
#include "program_v6.h"
#include "auto_runner_v6.h"

// =====================================================
// v6 sensor core FAST CONTINUOUS working base.
// Keeps ZERO/SAVE/LOAD/RESET logic from safe sensor core.
// Coordinates are held on stale/lost; DWIN does not receive false zero values.
// =====================================================

static Sc16Is752 g_sc16_1(PIN_SC16_1_CS, SC16_XTAL_HZ);
static Sc16Is752 g_sc16_2(PIN_SC16_2_CS, SC16_XTAL_HZ);

static FastLaserSensor g_laserX1(g_sc16_1, Sc16Is752::Channel::A);
static FastLaserSensor g_laserX2(g_sc16_1, Sc16Is752::Channel::B);
static FastLaserSensor g_laserZ1(g_sc16_2, Sc16Is752::Channel::A);
static FastLaserSensor g_laserZ2(g_sc16_2, Sc16Is752::Channel::B);

static DwinLink g_dwin(Serial2);
static SettingsStorageV6 g_storage;
static SettingsV6 g_settings;
static SettingsV6 g_editSettings;
static MotorControlV6 g_motor;
static SafetyV6 g_safety;
static ProgramStorageV6 g_programStorage;
static AutoProgramV6 g_program;
static AutoRunnerV6 g_auto;
static uint8_t g_programSlot = 0;          // currently loaded/active program slot
static uint8_t g_programSelectedSlot = 0;  // card selected in DWIN program list; LOAD makes it active
static uint8_t g_programSelectedZone = 0;

// Manual numeric coordinate confirmation tracker. A pair is accepted only after
// BOTH fields of that pair were actually entered since load/capture/zone change.
static uint16_t g_programCoordEditMask = 0;
enum ProgramCoordEditBitsV6 : uint16_t {
  PROG_EDIT_ZONE_X1   = 1u << 0,
  PROG_EDIT_ZONE_X2   = 1u << 1,
  PROG_EDIT_ZONE_Z1   = 1u << 2,
  PROG_EDIT_ZONE_Z2   = 1u << 3,
  PROG_EDIT_HOME_X1   = 1u << 4,
  PROG_EDIT_HOME_X2   = 1u << 5,
  PROG_EDIT_TRAVEL_Z1 = 1u << 6,
  PROG_EDIT_TRAVEL_Z2 = 1u << 7,
  PROG_EDIT_DRY_X1    = 1u << 8,
  PROG_EDIT_DRY_X2    = 1u << 9,
  PROG_EDIT_DRY_Z1    = 1u << 10,
  PROG_EDIT_DRY_Z2    = 1u << 11
};

static constexpr uint16_t PROG_EDIT_ZONE_X_PAIR   = PROG_EDIT_ZONE_X1 | PROG_EDIT_ZONE_X2;
static constexpr uint16_t PROG_EDIT_ZONE_Z_PAIR   = PROG_EDIT_ZONE_Z1 | PROG_EDIT_ZONE_Z2;
static constexpr uint16_t PROG_EDIT_HOME_X_PAIR   = PROG_EDIT_HOME_X1 | PROG_EDIT_HOME_X2;
static constexpr uint16_t PROG_EDIT_TRAVEL_Z_PAIR = PROG_EDIT_TRAVEL_Z1 | PROG_EDIT_TRAVEL_Z2;
static constexpr uint16_t PROG_EDIT_DRY_X_PAIR    = PROG_EDIT_DRY_X1 | PROG_EDIT_DRY_X2;
static constexpr uint16_t PROG_EDIT_DRY_Z_PAIR    = PROG_EDIT_DRY_Z1 | PROG_EDIT_DRY_Z2;
static bool g_programDirty = false;
static bool g_programIsDemo = false;
static bool g_autoSimulation = false;
static uint16_t g_programUiState = 0; // 0 idle,1 loaded,2 saved,3 dirty,4 error,5 slot selected

// USB periodic diagnostics. Event/fault messages are always printed.
// NORMAL is intentionally quieter than Step9A so console commands stay readable.
static bool g_consolePeriodicEnabled = true;
static uint16_t g_consolePrintIntervalMs = 2000;

static bool g_settingsFault = false;
static bool g_settingsDirty = false;
static uint16_t g_settingsUiState = SETTINGS_UI_IDLE;
static uint16_t g_settingsUiError = SETTINGS_VALID;

// High-level system mode. Real motor/VFD commands are connected later through MotorControlV6.
static SystemModeV6 g_systemMode = SystemModeV6::SERVICE;

struct SensorRuntimeFast
{
  FastLaserSensor *hw;
  const char *label;
  SensorIndex idx;
  uint16_t vp;
  bool hwOk;
  bool valid;
  int32_t rawMm;
  int32_t valueMm;
  uint32_t lastValidMs;
};

static SensorRuntimeFast g_sensor[SENSOR_COUNT] = {
    {&g_laserX1, "X1", SENSOR_X1, VP_X1, false, false, 0, 0, 0},
    {&g_laserX2, "X2", SENSOR_X2, VP_X2, false, false, 0, 0, 0},
    {&g_laserZ1, "Z1", SENSOR_Z1, VP_Z1, false, false, 0, 0, 0},
    {&g_laserZ2, "Z2", SENSOR_Z2, VP_Z2, false, false, 0, 0, 0},
};

uint16_t displayValue(int32_t v)
{
  if (v < 0)
    return 0;
  if (v > 65535L)
    return 65535;
  return (uint16_t)v;
}

void printBool(const __FlashStringHelper *label, bool value)
{
  Serial.print(label);
  Serial.println(value ? F("OK") : F("FAIL"));
}

void writeModeToDwin()
{
  g_dwin.writeU16(VP_MODE, systemModeToVp(g_systemMode));
}

void writeMotorStateToDwin()
{
  g_dwin.writeU16(VP_MOTOR_STATE, g_motor.stateCode());
  g_dwin.writeU16(VP_VFD_STATUS, g_motor.vfdStatusCode());
}

uint8_t sensorOnlineCount()
{
  const uint32_t now = millis();
  uint8_t n = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    if (!g_settings.sensor[i].enabled) continue;
    const SensorRuntimeFast &s = g_sensor[i];
    if (s.hwOk && s.valid && s.lastValidMs != 0 &&
        (uint32_t)(now - s.lastValidMs) <= g_settings.staleTimeoutMs)
      ++n;
  }
  return n;
}

bool safetyEstopRequestedEnabled(const SettingsV6 &cfg)
{
  return (cfg.vfd.safetyDisableMask & SAFETY_DISABLE_ESTOP) == 0;
}

bool safetyLimitsRequestedEnabled(const SettingsV6 &cfg)
{
  return (cfg.vfd.safetyDisableMask & SAFETY_DISABLE_LIMITS) == 0;
}

void applySafetySettings(const SettingsV6 &cfg)
{
  (void)g_safety.setEstopEnabled(safetyEstopRequestedEnabled(cfg));
  (void)g_safety.setLimitsEnabled(safetyLimitsRequestedEnabled(cfg));
}

AutoSensorsV6 buildAutoSensors()
{
  AutoSensorsV6 out{};
  const uint32_t now = millis();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    out.mm[i] = g_sensor[i].valueMm;
    const SensorRuntimeFast &sr = g_sensor[i];
    if (!g_settings.sensor[i].enabled) continue;
    if (sr.hwOk && sr.valid && sr.lastValidMs != 0 &&
        (uint32_t)(now - sr.lastValidMs) <= g_settings.staleTimeoutMs)
      out.usableMask |= (uint8_t)(1U << i);
  }
  return out;
}

void writeHmsToDwin(uint32_t seconds, uint16_t vpH, uint16_t vpM, uint16_t vpS)
{
  const uint16_t h = seconds > 359999UL ? 99 : (uint16_t)(seconds / 3600UL);
  const uint16_t m = (uint16_t)((seconds / 60UL) % 60UL);
  const uint16_t sec = (uint16_t)(seconds % 60UL);
  g_dwin.writeU16(vpH, h);
  g_dwin.writeU16(vpM, m);
  g_dwin.writeU16(vpS, sec);
}

void writePersistentHeaderToDwin()
{
  const uint32_t now = millis();
  const uint8_t enabledZones = g_programStorage.enabledZoneCount(g_program);
  const bool ready = g_programStorage.readyForAuto(g_program);

  g_dwin.writeU16(VP_PROGRAM_INDEX, (uint16_t)(g_programSlot + 1));
  g_dwin.writeU16(VP_ZONE_CURRENT, (g_auto.running() && g_systemMode == SystemModeV6::AUTO) ? (uint16_t)(g_auto.zoneIndex() + 1) : 0);
  g_dwin.writeU16(VP_ZONE_TOTAL, enabledZones);
  g_dwin.writeU16(VP_STEP_CURRENT, g_auto.currentStep());
  g_dwin.writeU16(VP_STEP_TOTAL, g_auto.totalSteps());
  writeHmsToDwin(g_auto.elapsedSeconds(now), VP_TIME_ELAPSED_H, VP_TIME_ELAPSED_M, VP_TIME_ELAPSED_S);
  writeHmsToDwin(g_programStorage.knownTimedSeconds(g_program), VP_TIME_TOTAL_H, VP_TIME_TOTAL_M, VP_TIME_TOTAL_S);

  g_dwin.writeU16(VP_SENSOR_OK_COUNT, sensorOnlineCount());
  g_dwin.writeU16(VP_SENSOR_TOTAL, SENSOR_COUNT);
  g_dwin.writeU16(VP_VFD_OK_COUNT, g_motor.vfdConnectedCount());
  g_dwin.writeU16(VP_VFD_TOTAL, DRIVE_COUNT_V6);
  g_dwin.writeU16(VP_RS485_STATE, VFD_RS485_ENABLED ? 1 : (VFD_DRY_RUN ? 2 : 0));
  g_dwin.writeU16(VP_NETWORK_STATE, 0);
  g_dwin.writeU16(VP_SAFETY_STATE, g_safety.stateWord());
  g_dwin.writeU16(VP_LIMIT_STATE, g_safety.limitMask());
  g_dwin.writeU16(VP_MODBUS_QUEUE_BUSY, g_motor.vfdHasPendingWork() ? 1 : 0);
  g_dwin.writeU16(VP_ESTOP_ENABLED, g_safety.estopEnabled() ? 1 : 0);
  g_dwin.writeU16(VP_LIMITS_ENABLED, g_safety.limitsEnabled() ? 1 : 0);

  g_dwin.writeU16(VP_AUTO_PHASE, (uint16_t)g_auto.phase());
  g_dwin.writeU16(VP_AUTO_ERROR, g_auto.error());
  g_dwin.writeU16(VP_AUTO_RUNNING, g_auto.running() ? 1 : 0);
  g_dwin.writeU16(VP_AUTO_PAUSED, g_auto.paused() ? 1 : 0);
  g_dwin.writeU16(VP_AUTO_SIMULATION, g_autoSimulation ? 1 : 0);
  g_dwin.writeU16(VP_AUTO_WAIT_OPERATOR, g_auto.waitOperator() ? 1 : 0);
  const uint16_t remain = g_auto.remainingWaitSeconds(now);
  g_dwin.writeU16(VP_AUTO_REMAIN_S, remain == 0xFFFF ? 0 : remain);
  g_dwin.writeU16(VP_PROGRAM_READY, ready ? 1 : 0);
  g_dwin.writeU16(VP_PROGRAM_ACTIVE_SLOT, (uint16_t)(g_programSlot + 1));
  g_dwin.writeU16(VP_PROG_EDIT_LOCKED, g_auto.running() ? 1 : 0);
}

void writeBootScreen()
{
  writeModeToDwin();
  g_dwin.writeU16(VP_ERROR, ERROR_NONE);
  g_dwin.writeU16(VP_X1, 0);
  g_dwin.writeU16(VP_X2, 0);
  g_dwin.writeU16(VP_Z1, 0);
  g_dwin.writeU16(VP_Z2, 0);
  writeMotorStateToDwin();
  writePersistentHeaderToDwin();
  g_dwin.writeU16(VP_JOG_HOLD_BITS, 0);
  g_dwin.clearCommand(VP_CMD);
}

void updateErrorMask()
{
  uint16_t err = ERROR_NONE;
  const uint32_t now = millis();

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    const SensorRuntimeFast &s = g_sensor[i];
    if (!g_settings.sensor[i].enabled)
      continue;

    if (!s.hwOk || !s.valid || s.lastValidMs == 0)
    {
      // LOST/no value: low bits 0..3
      err |= (uint16_t)(1 << i);
      continue;
    }

    const uint32_t age = now - s.lastValidMs;
    if (age > g_settings.staleTimeoutMs)
    {
      // LOST: low bits 0..3
      err |= (uint16_t)(1 << i);
    }
    else if (age > SENSOR_FRESH_TIMEOUT_MS)
    {
      // STALE: bits 4..7, coordinate is still held on screen
      err |= (uint16_t)(1 << (i + 4));
    }
  }

  if (g_settingsFault)
    err |= ERR_SETTINGS;
  if (g_safety.estopActive() || g_safety.estopLatched())
    err |= ERR_ESTOP;
  if (g_safety.limitMask() != 0)
    err |= ERR_LIMIT;
  if (g_auto.error() != AUTO_ERR_NONE)
    err |= ERR_AUTO;

  g_dwin.writeU16(VP_ERROR, err);
}


static const uint16_t DRIVE_PROFILE_BASE_VP[DRIVE_COUNT_V6] = {
    VP_DRIVE_H1_BASE, VP_DRIVE_H2_BASE, VP_DRIVE_V1_BASE, VP_DRIVE_V2_BASE};

void writeSettingsUiStatus()
{
  g_dwin.writeU16(VP_SETTINGS_STATE, g_settingsUiState);
  g_dwin.writeU16(VP_SETTINGS_ERROR, g_settingsUiError);
  g_dwin.writeU16(VP_SETTINGS_DIRTY, g_settingsDirty ? 1 : 0);
  g_dwin.writeU16(VP_SETTINGS_VERSION, g_settings.version);
}

void writeSettingsToDwin(const SettingsV6 &cfg)
{
  g_dwin.writeU16(VP_VFD_BAUD_CODE, cfg.vfd.baudCode);
  g_dwin.writeU16(VP_VFD_PARITY, cfg.vfd.parity);
  g_dwin.writeU16(VP_VFD_STOP_BITS, cfg.vfd.stopBits);
  g_dwin.writeU16(VP_VFD_RESPONSE_TIMEOUT, cfg.vfd.responseTimeoutMs);
  g_dwin.writeU16(VP_VFD_RETRIES, cfg.vfd.retries);
  g_dwin.writeU16(VP_VFD_INTER_REQUEST_MS, cfg.vfd.interRequestMs);
  g_dwin.writeU16(VP_VFD_ONLINE_POLL_MS, cfg.vfd.onlinePollMs);
  g_dwin.writeU16(VP_VFD_OFFLINE_POLL_MS, cfg.vfd.offlinePollMs);

  g_dwin.writeU16(VP_VFD_ADDR_H1, cfg.vfd.address[DRIVE_H1]);
  g_dwin.writeU16(VP_VFD_ADDR_H2, cfg.vfd.address[DRIVE_H2]);
  g_dwin.writeU16(VP_VFD_ADDR_V1, cfg.vfd.address[DRIVE_V1]);
  g_dwin.writeU16(VP_VFD_ADDR_V2, cfg.vfd.address[DRIVE_V2]);
  g_dwin.writeU16(VP_VFD_INVERT_MASK, cfg.vfd.invertDirectionMask);
  g_dwin.writeU16(VP_MANUAL_JOG_TIMEOUT_MS, cfg.vfd.manualJogTimeoutMs);
  g_dwin.writeU16(VP_SAFETY_ESTOP_ENABLE, safetyEstopRequestedEnabled(cfg) ? 1 : 0);
  g_dwin.writeU16(VP_SAFETY_LIMITS_ENABLE, safetyLimitsRequestedEnabled(cfg) ? 1 : 0);

  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i)
  {
    const uint16_t base = DRIVE_PROFILE_BASE_VP[i];
    g_dwin.writeU16(base + VP_DRIVE_MANUAL_OFS, cfg.drive[i].manualPercent);
    g_dwin.writeU16(base + VP_DRIVE_MAX_OFS, cfg.drive[i].maxPercent);
    g_dwin.writeU16(base + VP_DRIVE_SLOW_OFS, cfg.drive[i].slowPercent);
    g_dwin.writeU16(base + VP_DRIVE_SLOWDOWN_OFS, cfg.drive[i].slowdownDistanceMm);
    g_dwin.writeU16(base + VP_DRIVE_TOLERANCE_OFS, cfg.drive[i].stopToleranceMm);
  }

  writeSettingsUiStatus();
}

void writeProgramToDwin()
{
  if (g_programSelectedZone >= g_program.zoneCount) g_programSelectedZone = 0;
  AutoZoneV6 &z = g_program.zones[g_programSelectedZone];
  g_dwin.writeU16(VP_PROG_ZONE_SELECTED, (uint16_t)(g_programSelectedZone + 1));
  g_dwin.writeU16(VP_PROG_ZONE_COUNT, g_program.zoneCount);
  g_dwin.writeU16(VP_PROG_ZONE_ENABLED, z.enabled ? 1 : 0);
  g_dwin.writeU16(VP_PROG_ZONE_X1, displayValue(z.xMm[0]));
  g_dwin.writeU16(VP_PROG_ZONE_X2, displayValue(z.xMm[1]));
  g_dwin.writeU16(VP_PROG_ZONE_Z1, displayValue(z.zMm[0]));
  g_dwin.writeU16(VP_PROG_ZONE_Z2, displayValue(z.zMm[1]));
  g_dwin.writeU16(VP_PROG_ZONE_DIP_S, z.dipTimeS);
  g_dwin.writeU16(VP_PROG_ZONE_TILT_MM, z.tiltStepMm);
  g_dwin.writeU16(VP_PROG_ZONE_WAIT_S, z.stepWaitS);
  g_dwin.writeU16(VP_PROG_ZONE_H_PCT, z.movePercent);
  g_dwin.writeU16(VP_PROG_ZONE_V_PCT, z.verticalPercent);
  g_dwin.writeU16(VP_PROG_HOME_X1, displayValue(g_program.homeX[0]));
  g_dwin.writeU16(VP_PROG_HOME_X2, displayValue(g_program.homeX[1]));
  g_dwin.writeU16(VP_PROG_TRAVEL_Z1, displayValue(g_program.travelZ[0]));
  g_dwin.writeU16(VP_PROG_TRAVEL_Z2, displayValue(g_program.travelZ[1]));
  g_dwin.writeU16(VP_PROG_DRIP_S, g_program.dripWaitS);
  g_dwin.writeU16(VP_PROG_LOW_SIDE, g_program.lowSide);
  g_dwin.writeU16(VP_PROG_TILT_PCT, g_program.tiltPercent);
  g_dwin.writeU16(VP_PROG_DRY_ENABLE, g_program.dryingEnabled ? 1 : 0);
  g_dwin.writeU16(VP_PROG_DRY_TIME_S, g_program.dryingTimeS);
  g_dwin.writeU16(VP_PROG_STAGING_ZONE, (uint16_t)(g_program.stagingZone + 1));
  g_dwin.writeU16(VP_PROG_DRY_X1, displayValue(g_program.dryX[0]));
  g_dwin.writeU16(VP_PROG_DRY_X2, displayValue(g_program.dryX[1]));
  g_dwin.writeU16(VP_PROG_DRY_Z1, displayValue(g_program.dryZ[0]));
  g_dwin.writeU16(VP_PROG_DRY_Z2, displayValue(g_program.dryZ[1]));
  g_dwin.writeU16(VP_PROG_VALID_MASK, g_program.validMask);
  g_dwin.writeU16(VP_PROG_ZONE_VALID_MASK, z.validMask);
  g_dwin.writeU16(VP_PROG_DIRTY, g_programDirty ? 1 : 0);
  g_dwin.writeU16(VP_PROG_SLOT_SELECTED, (uint16_t)(g_programSelectedSlot + 1));
  g_dwin.writeU16(VP_PROG_UI_STATE, g_programUiState);
  g_dwin.writeU16(VP_PROG_EDIT_LOCKED, g_auto.running() ? 1 : 0);
  writePersistentHeaderToDwin();
}

void markProgramDirty()
{
  g_programDirty = true;
  g_programUiState = 3;
  // Keep SIM-DEMO RAM-only even if its fields are edited from DWIN/USB.
  writeProgramToDwin();
}

bool programSelectedSlotIsLoaded()
{
  return g_programSelectedSlot == g_programSlot;
}

bool updateProgramFromVp(uint16_t vp, uint16_t value)
{
  if (vp >= VP_PROG_ZONE_SELECTED && vp <= VP_PROG_TILT_PCT && !programSelectedSlotIsLoaded()) {
    g_programUiState = 4;
    Serial.println(F("Program edit rejected: selected slot is not loaded; press LOAD first"));
    writeProgramToDwin();
    return true;
  }
  if (g_auto.running()) {
    if (vp >= VP_PROG_ZONE_SELECTED && vp <= VP_PROG_TILT_PCT) {
      Serial.println(F("Program edit ignored while AUTO/HOME is running"));
      writeProgramToDwin();
      return true;
    }
    return false;
  }

  if (vp == VP_PROG_ZONE_SELECTED) {
    if (value < 1 || value > AUTO_MAX_ZONES_V6) return true;
    g_programSelectedZone = (uint8_t)(value - 1);
    g_programCoordEditMask &= (uint16_t)~(PROG_EDIT_ZONE_X_PAIR | PROG_EDIT_ZONE_Z_PAIR);
    if (g_programSelectedZone >= g_program.zoneCount) {
      g_program.zoneCount = (uint8_t)(g_programSelectedZone + 1);
      for (uint8_t i = 0; i < g_program.zoneCount; ++i) g_program.order[i] = i;
      g_programDirty = true;
    }
    writeProgramToDwin();
    return true;
  }

  AutoZoneV6 &z = g_program.zones[g_programSelectedZone];
  switch (vp) {
    case VP_PROG_ZONE_COUNT:
      if (value >= 1 && value <= AUTO_MAX_ZONES_V6) {
        g_program.zoneCount = (uint8_t)value;
        if (g_programSelectedZone >= g_program.zoneCount) g_programSelectedZone = (uint8_t)(g_program.zoneCount - 1);
        g_programCoordEditMask &= (uint16_t)~(PROG_EDIT_ZONE_X_PAIR | PROG_EDIT_ZONE_Z_PAIR);
        for (uint8_t i = 0; i < g_program.zoneCount; ++i) g_program.order[i] = i;
      } else return true;
      break;
    case VP_PROG_ZONE_ENABLED: z.enabled = value ? 1 : 0; break;
    case VP_PROG_ZONE_X1: z.xMm[0] = value; z.validMask &= (uint8_t)~AUTO_ZONE_VALID_X; g_programCoordEditMask |= PROG_EDIT_ZONE_X1; break;
    case VP_PROG_ZONE_X2: z.xMm[1] = value; z.validMask &= (uint8_t)~AUTO_ZONE_VALID_X; g_programCoordEditMask |= PROG_EDIT_ZONE_X2; break;
    case VP_PROG_ZONE_Z1: z.zMm[0] = value; z.validMask &= (uint8_t)~AUTO_ZONE_VALID_Z; g_programCoordEditMask |= PROG_EDIT_ZONE_Z1; break;
    case VP_PROG_ZONE_Z2: z.zMm[1] = value; z.validMask &= (uint8_t)~AUTO_ZONE_VALID_Z; g_programCoordEditMask |= PROG_EDIT_ZONE_Z2; break;
    case VP_PROG_ZONE_DIP_S: z.dipTimeS = value; break;
    case VP_PROG_ZONE_TILT_MM: z.tiltStepMm = value; break;
    case VP_PROG_ZONE_WAIT_S: z.stepWaitS = value; break;
    case VP_PROG_ZONE_H_PCT: if (value < 1 || value > 100) return true; z.movePercent = (uint8_t)value; break;
    case VP_PROG_ZONE_V_PCT: if (value < 1 || value > 100) return true; z.verticalPercent = (uint8_t)value; break;
    case VP_PROG_HOME_X1: g_program.homeX[0] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_HOME_X; g_programCoordEditMask |= PROG_EDIT_HOME_X1; break;
    case VP_PROG_HOME_X2: g_program.homeX[1] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_HOME_X; g_programCoordEditMask |= PROG_EDIT_HOME_X2; break;
    case VP_PROG_TRAVEL_Z1: g_program.travelZ[0] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_TRAVEL_Z; g_programCoordEditMask |= PROG_EDIT_TRAVEL_Z1; break;
    case VP_PROG_TRAVEL_Z2: g_program.travelZ[1] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_TRAVEL_Z; g_programCoordEditMask |= PROG_EDIT_TRAVEL_Z2; break;
    case VP_PROG_DRIP_S: g_program.dripWaitS = value; break;
    case VP_PROG_LOW_SIDE: if (value > 1) return true; g_program.lowSide = (uint8_t)value; break;
    case VP_PROG_TILT_PCT: if (value < 1 || value > 100) return true; g_program.tiltPercent = (uint8_t)value; break;
    case VP_PROG_DRY_ENABLE: g_program.dryingEnabled = value ? 1 : 0; break;
    case VP_PROG_DRY_TIME_S: g_program.dryingTimeS = value; break;
    case VP_PROG_STAGING_ZONE:
      if (value < 1 || value > g_program.zoneCount) return true;
      g_program.stagingZone = (uint8_t)(value - 1);
      break;
    case VP_PROG_DRY_X1: g_program.dryX[0] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_DRY_X; g_program.dryValid = 0; g_programCoordEditMask |= PROG_EDIT_DRY_X1; break;
    case VP_PROG_DRY_X2: g_program.dryX[1] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_DRY_X; g_program.dryValid = 0; g_programCoordEditMask |= PROG_EDIT_DRY_X2; break;
    case VP_PROG_DRY_Z1: g_program.dryZ[0] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_DRY_Z; g_program.dryValid = 0; g_programCoordEditMask |= PROG_EDIT_DRY_Z1; break;
    case VP_PROG_DRY_Z2: g_program.dryZ[1] = value; g_program.validMask &= (uint8_t)~AUTO_PROGRAM_VALID_DRY_Z; g_program.dryValid = 0; g_programCoordEditMask |= PROG_EDIT_DRY_Z2; break;
    default: return false;
  }
  if ((g_program.validMask & (AUTO_PROGRAM_VALID_DRY_X | AUTO_PROGRAM_VALID_DRY_Z)) ==
      (AUTO_PROGRAM_VALID_DRY_X | AUTO_PROGRAM_VALID_DRY_Z)) g_program.dryValid = 1;
  markProgramDirty();
  return true;
}

uint8_t valueToU8OrInvalid(uint16_t value)
{
  return value <= 255 ? (uint8_t)value : 255;
}

void markSettingsDirty()
{
  g_settingsDirty = true;
  g_settingsUiState = SETTINGS_UI_DIRTY;
  g_settingsUiError = SETTINGS_VALID;
  writeSettingsUiStatus();
}

bool updatePendingSettingsFromVp(uint16_t vp, uint16_t value)
{
  switch (vp)
  {
  case VP_VFD_BAUD_CODE: g_editSettings.vfd.baudCode = valueToU8OrInvalid(value); break;
  case VP_VFD_PARITY: g_editSettings.vfd.parity = valueToU8OrInvalid(value); break;
  case VP_VFD_STOP_BITS: g_editSettings.vfd.stopBits = valueToU8OrInvalid(value); break;
  case VP_VFD_RESPONSE_TIMEOUT: g_editSettings.vfd.responseTimeoutMs = value; break;
  case VP_VFD_RETRIES: g_editSettings.vfd.retries = valueToU8OrInvalid(value); break;
  case VP_VFD_INTER_REQUEST_MS: g_editSettings.vfd.interRequestMs = value; break;
  case VP_VFD_ONLINE_POLL_MS: g_editSettings.vfd.onlinePollMs = value; break;
  case VP_VFD_OFFLINE_POLL_MS: g_editSettings.vfd.offlinePollMs = value; break;
  case VP_VFD_ADDR_H1: g_editSettings.vfd.address[DRIVE_H1] = valueToU8OrInvalid(value); break;
  case VP_VFD_ADDR_H2: g_editSettings.vfd.address[DRIVE_H2] = valueToU8OrInvalid(value); break;
  case VP_VFD_ADDR_V1: g_editSettings.vfd.address[DRIVE_V1] = valueToU8OrInvalid(value); break;
  case VP_VFD_ADDR_V2: g_editSettings.vfd.address[DRIVE_V2] = valueToU8OrInvalid(value); break;
  case VP_VFD_INVERT_MASK: g_editSettings.vfd.invertDirectionMask = (uint8_t)(value & 0x0F); break;
  case VP_MANUAL_JOG_TIMEOUT_MS: g_editSettings.vfd.manualJogTimeoutMs = value; break;
  case VP_SAFETY_ESTOP_ENABLE:
    if (value > 1 || (!value && !SAFETY_BENCH_MODE)) {
      g_settingsUiState = SETTINGS_UI_REJECTED;
      g_settingsUiError = SETTINGS_ERR_UNSAFE_MODE;
      writeSettingsUiStatus();
      Serial.println(F("Settings safety E-stop change rejected"));
      return true;
    }
    if (value) g_editSettings.vfd.safetyDisableMask &= (uint8_t)~SAFETY_DISABLE_ESTOP;
    else       g_editSettings.vfd.safetyDisableMask |= SAFETY_DISABLE_ESTOP;
    break;
  case VP_SAFETY_LIMITS_ENABLE:
    if (value > 1) {
      g_settingsUiState = SETTINGS_UI_REJECTED;
      g_settingsUiError = SETTINGS_ERR_UNSAFE_MODE;
      writeSettingsUiStatus();
      Serial.println(F("Settings limit change rejected"));
      return true;
    }
    if (value) g_editSettings.vfd.safetyDisableMask &= (uint8_t)~SAFETY_DISABLE_LIMITS;
    else       g_editSettings.vfd.safetyDisableMask |= SAFETY_DISABLE_LIMITS;
    break;
  default:
    for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i)
    {
      const uint16_t base = DRIVE_PROFILE_BASE_VP[i];
      if (vp == base + VP_DRIVE_MANUAL_OFS)
        g_editSettings.drive[i].manualPercent = valueToU8OrInvalid(value);
      else if (vp == base + VP_DRIVE_MAX_OFS)
        g_editSettings.drive[i].maxPercent = valueToU8OrInvalid(value);
      else if (vp == base + VP_DRIVE_SLOW_OFS)
        g_editSettings.drive[i].slowPercent = valueToU8OrInvalid(value);
      else if (vp == base + VP_DRIVE_SLOWDOWN_OFS)
        g_editSettings.drive[i].slowdownDistanceMm = value;
      else if (vp == base + VP_DRIVE_TOLERANCE_OFS)
        g_editSettings.drive[i].stopToleranceMm = value;
      else
        continue;

      markSettingsDirty();
      return true;
    }
    return false;
  }

  markSettingsDirty();
  return true;
}

bool settingsCanBeApplied()
{
  if (g_motor.isMotionActive()) return false;
  return g_systemMode == SystemModeV6::STOP ||
         g_systemMode == SystemModeV6::SETTINGS ||
         g_systemMode == SystemModeV6::SERVICE ||
         g_systemMode == SystemModeV6::CALIBRATION;
}

bool applyPendingSettings(bool saveToEeprom)
{
  if (!settingsCanBeApplied())
  {
    g_settingsUiState = SETTINGS_UI_REJECTED;
    g_settingsUiError = SETTINGS_ERR_UNSAFE_MODE;
    writeSettingsUiStatus();
    Serial.println(F("Settings rejected: system must be stopped / in SETTINGS mode"));
    return false;
  }

  const uint16_t validation = g_storage.validate(g_editSettings);
  if (validation != SETTINGS_VALID)
  {
    g_settingsUiState = SETTINGS_UI_REJECTED;
    g_settingsUiError = validation;
    writeSettingsUiStatus();
    Serial.print(F("Settings rejected, validation mask=0x"));
    Serial.println(validation, HEX);
    return false;
  }

  g_settings = g_editSettings;
  g_motor.applySettings(g_settings);
  g_auto.applySettings(g_settings);
  applySafetySettings(g_settings);
  g_settingsDirty = false;
  g_settingsUiError = SETTINGS_VALID;

  if (saveToEeprom)
  {
    g_storage.save(g_settings);
    g_settingsFault = false;
    g_settingsUiState = SETTINGS_UI_SAVED;
    Serial.println(F("VFD/RS485 settings applied and saved to EEPROM"));
  }
  else
  {
    g_settingsUiState = SETTINGS_UI_APPLIED;
    Serial.println(F("VFD/RS485 settings applied in RAM"));
  }

  writeSettingsToDwin(g_settings);
  updateErrorMask();
  return true;
}

void loadSettingsFromEepromForUi()
{
  if (!settingsCanBeApplied())
  {
    g_settingsUiState = SETTINGS_UI_REJECTED;
    g_settingsUiError = SETTINGS_ERR_UNSAFE_MODE;
    writeSettingsUiStatus();
    return;
  }

  SettingsV6 loaded;
  if (!g_storage.load(loaded))
  {
    g_settingsFault = true;
    g_settingsUiState = SETTINGS_UI_REJECTED;
    g_settingsUiError = SETTINGS_ERR_EEPROM;
    writeSettingsUiStatus();
    updateErrorMask();
    Serial.println(F("Settings LOAD failed: EEPROM record invalid"));
    return;
  }

  g_settings = loaded;
  g_editSettings = loaded;
  g_motor.applySettings(g_settings);
  g_auto.applySettings(g_settings);
  applySafetySettings(g_settings);
  g_settingsFault = false;
  g_settingsDirty = false;
  g_settingsUiState = SETTINGS_UI_LOADED;
  g_settingsUiError = SETTINGS_VALID;
  writeSettingsToDwin(g_settings);
  updateErrorMask();
  Serial.println(F("Settings loaded from EEPROM and applied"));
}

void setVfdDefaultsForUi()
{
  g_editSettings = g_settings;
  g_storage.defaultsVfdSection(g_editSettings);
  g_settingsDirty = true;
  g_settingsUiState = SETTINGS_UI_DEFAULTS;
  g_settingsUiError = SETTINGS_VALID;
  writeSettingsToDwin(g_editSettings);
  Serial.println(F("Safe VFD defaults loaded into editor; press APPLY or SAVE"));
}

bool handleVfdSettingsCommand(uint16_t cmd)
{
  switch (cmd)
  {
  case CMD_VFD_SETTINGS_APPLY:
    applyPendingSettings(false);
    return true;
  case CMD_VFD_SETTINGS_SAVE:
    applyPendingSettings(true);
    return true;
  case CMD_VFD_SETTINGS_LOAD:
    loadSettingsFromEepromForUi();
    return true;
  case CMD_VFD_SETTINGS_DEFAULTS:
    setVfdDefaultsForUi();
    return true;
  case CMD_VFD_TEST_H1:
  case CMD_VFD_TEST_H2:
  case CMD_VFD_TEST_V1:
  case CMD_VFD_TEST_V2:
  case CMD_VFD_TEST_ALL:
    // Test uses the values currently visible in the editor. They are applied
    // to RAM after validation, but are not saved to EEPROM automatically.
    if (!applyPendingSettings(false))
      return true;
    {
      const uint8_t index = (cmd == CMD_VFD_TEST_ALL) ? (uint8_t)DRIVE_COUNT_V6 : (uint8_t)(cmd - CMD_VFD_TEST_H1);
      g_motor.testVfdConnection(index);
      g_settingsUiState = SETTINGS_UI_TEST_DRY_RUN;
      g_settingsUiError = SETTINGS_VALID;
      g_dwin.writeU16(VP_SETTINGS_TEST_DRIVE, (index < DRIVE_COUNT_V6) ? (uint16_t)(index + 1) : 5);
      writeSettingsUiStatus();
    }
    return true;
  default:
    return false;
  }
}

void writeCoordinatesToDwin()
{
  const bool sim = g_auto.running() && g_auto.simulation();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    const int32_t value = sim ? g_auto.simPosition((SensorIndex)i) : g_sensor[i].valueMm;
    const bool haveValue = sim || g_sensor[i].lastValidMs > 0;
    g_dwin.writeU16(g_sensor[i].vp, haveValue ? displayValue(value) : 0);
  }
}

void writeAllValuesToDwin()
{
  writeCoordinatesToDwin();
  writeModeToDwin();
  writeMotorStateToDwin();
  updateErrorMask();
}

void initSensorsFast()
{
  Serial.println(F("=== SC16 SELF TEST FAST ==="));

  // Release both CS lines before any SPI transaction.
  pinMode(MEGA_SPI_SS_PIN, OUTPUT);
  pinMode(PIN_SC16_1_CS, OUTPUT);
  digitalWrite(PIN_SC16_1_CS, HIGH);
  pinMode(PIN_SC16_2_CS, OUTPUT);
  digitalWrite(PIN_SC16_2_CS, HIGH);

  g_sc16_1.begin();
  g_sc16_2.begin();

  const bool s1a = g_sc16_1.selfTest(Sc16Is752::Channel::A);
  const bool s1b = g_sc16_1.selfTest(Sc16Is752::Channel::B);
  const bool s2a = g_sc16_2.selfTest(Sc16Is752::Channel::A);
  const bool s2b = g_sc16_2.selfTest(Sc16Is752::Channel::B);

  printBool(F("SC16 #1 CH_A: "), s1a);
  printBool(F("SC16 #1 CH_B: "), s1b);
  printBool(F("SC16 #2 CH_A: "), s2a);
  printBool(F("SC16 #2 CH_B: "), s2b);

  g_sensor[SENSOR_X1].hwOk = s1a && g_settings.sensor[SENSOR_X1].enabled;
  g_sensor[SENSOR_X2].hwOk = s1b && g_settings.sensor[SENSOR_X2].enabled;
  g_sensor[SENSOR_Z1].hwOk = s2a && g_settings.sensor[SENSOR_Z1].enabled;
  g_sensor[SENSOR_Z2].hwOk = s2b && g_settings.sensor[SENSOR_Z2].enabled;

  const uint32_t now = millis();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    if (!g_sensor[i].hwOk)
      continue;
    const uint16_t phase = (uint16_t)(i * FAST_SENSOR_PHASE_MS);
    g_sensor[i].hw->begin(now, FAST_SENSOR_PERIOD_MS, phase);
    Serial.print(g_sensor[i].label);
    Serial.println(F(" fast init: OK"));
  }
}

void serviceSensorsFast()
{
  const uint32_t now = millis();

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    SensorRuntimeFast &s = g_sensor[i];
    if (!s.hwOk)
      continue;

    s.hw->service(now);

    if (s.hw->consumeUpdated())
    {
      s.rawMm = s.hw->rawMm();
      s.valueMm = g_storage.applyCalibration(g_settings, s.idx, s.rawMm);
      s.valid = true;
      s.lastValidMs = s.hw->lastUpdateMs();
      if (!(g_auto.running() && g_auto.simulation()))
        g_dwin.writeU16(s.vp, displayValue(s.valueMm));
    }

    // Do not clear s.valid on stale. It means "has last value" here.
    // Fresh/stale/lost state is reported through VP_ERROR in updateErrorMask().
  }
}

void zeroSensor(SensorIndex idx)
{
  if (idx >= SENSOR_COUNT)
    return;
  SensorRuntimeFast &s = g_sensor[idx];

  // Calibration must use the same value that the operator sees on DWIN.
  // Therefore we allow ZERO by the last known raw value, even if the sensor is
  // currently STALE. We reject only the case when the channel has never produced
  // a valid measurement after boot/reinit.
  const uint32_t now = millis();
  if (!s.valid || s.rawMm <= 0 || s.lastValidMs == 0)
  {
    Serial.print(F("ZERO ignored: no measured value for "));
    Serial.println(s.label);
    return;
  }

  const uint32_t age = now - s.lastValidMs;
  if (age > g_settings.staleTimeoutMs)
  {
    Serial.print(F("ZERO warning: using held/stale value for "));
    Serial.print(s.label);
    Serial.print(F(" age="));
    Serial.print(age);
    Serial.println(F("ms"));
  }

  g_storage.zeroSensor(g_settings, idx, s.rawMm);
  g_editSettings.sensor[idx] = g_settings.sensor[idx];
  s.valueMm = g_storage.applyCalibration(g_settings, idx, s.rawMm);
  g_dwin.writeU16(s.vp, displayValue(s.valueMm));
  updateErrorMask();

  Serial.print(F("ZERO "));
  Serial.print(s.label);
  Serial.print(F(" raw="));
  Serial.print(s.rawMm);
  Serial.print(F(" value="));
  Serial.print(s.valueMm);
  Serial.print(F(" offset="));
  Serial.println(g_settings.sensor[idx].offsetMm);
}

bool loadProgramSlotV6(uint8_t slot)
{
  if (slot >= AUTO_PROGRAM_SLOTS_V6) return false;
  AutoProgramV6 loaded{};
  if (!g_programStorage.load(slot, loaded)) {
    g_programStorage.defaults(g_program, slot);
    g_programSlot = slot;
    g_programSelectedSlot = slot;
    g_programCoordEditMask = 0;
    g_programStorage.saveActiveSlot(slot);
    g_programDirty = true;
    g_programIsDemo = false;
    g_programSelectedZone = 0;
    g_programUiState = 4;
    Serial.print(F("Program slot "));
    Serial.print(slot + 1);
    Serial.println(F(" invalid/empty: safe uncalibrated defaults loaded in RAM"));
    writeProgramToDwin();
    return false;
  }
  g_program = loaded;
  g_programSlot = slot;
  g_programSelectedSlot = slot;
  g_programCoordEditMask = 0;
  g_programStorage.saveActiveSlot(slot);
  g_programDirty = false;
  g_programIsDemo = false;
  g_programSelectedZone = 0;
  g_programUiState = 1;
  Serial.print(F("Program loaded slot="));
  Serial.print(slot + 1);
  Serial.print(F(" name="));
  Serial.println(g_program.name);
  writeProgramToDwin();
  return true;
}

bool saveProgramSlotV6()
{
  if (g_auto.running()) {
    g_programUiState = 4;
    Serial.println(F("Program SAVE rejected while AUTO/HOME is running"));
    writeProgramToDwin();
    return false;
  }
  if (g_programIsDemo) {
    g_programUiState = 4;
    Serial.println(F("Program SAVE rejected: SIM-DEMO is RAM-only by design"));
    writeProgramToDwin();
    return false;
  }
  if (g_programSelectedSlot != g_programSlot) {
    g_programUiState = 4;
    Serial.print(F("Program SAVE rejected: selected slot "));
    Serial.print(g_programSelectedSlot + 1);
    Serial.print(F(" differs from loaded slot "));
    Serial.print(g_programSlot + 1);
    Serial.println(F("; press LOAD first"));
    writeProgramToDwin();
    return false;
  }
  if (!g_programStorage.save(g_programSlot, g_program)) {
    g_programUiState = 4;
    Serial.println(F("Program SAVE failed validation/EEPROM bounds"));
    writeProgramToDwin();
    return false;
  }
  g_programStorage.saveActiveSlot(g_programSlot);
  g_programDirty = false;
  g_programUiState = 2;
  Serial.print(F("Program saved slot="));
  Serial.println(g_programSlot + 1);
  writeProgramToDwin();
  return true;
}

bool sensorPairUsable(SensorIndex a, SensorIndex b)
{
  const AutoSensorsV6 s = buildAutoSensors();
  const uint8_t mask = (uint8_t)((1U << a) | (1U << b));
  return (s.usableMask & mask) == mask;
}

bool captureProgramCoordinates(uint16_t cmd)
{
  if (!programSelectedSlotIsLoaded()) {
    if (cmd >= CMD_PROGRAM_CAPTURE_HOME && cmd <= CMD_PROGRAM_CAPTURE_DRY_Z) {
      g_programUiState = 4;
      Serial.println(F("Program capture rejected: selected slot is not loaded; press LOAD first"));
      writeProgramToDwin();
      return true;
    }
  }
  if (g_auto.running()) {
    Serial.println(F("Program capture rejected while AUTO/HOME is running"));
    return true;
  }
  AutoZoneV6 &z = g_program.zones[g_programSelectedZone];
  switch (cmd) {
    case CMD_PROGRAM_CAPTURE_HOME:
      if (!sensorPairUsable(SENSOR_X1, SENSOR_X2)) { Serial.println(F("CAP HOME rejected: X1+X2 required")); return true; }
      g_program.homeX[0] = g_sensor[SENSOR_X1].valueMm;
      g_program.homeX[1] = g_sensor[SENSOR_X2].valueMm;
      g_program.validMask |= AUTO_PROGRAM_VALID_HOME_X;
      g_programCoordEditMask &= (uint16_t)~PROG_EDIT_HOME_X_PAIR;
      Serial.println(F("CAP HOME X1/X2 OK"));
      break;
    case CMD_PROGRAM_CAPTURE_TRAVEL:
      if (!sensorPairUsable(SENSOR_Z1, SENSOR_Z2)) { Serial.println(F("CAP TRAVEL rejected: Z1+Z2 required")); return true; }
      g_program.travelZ[0] = g_sensor[SENSOR_Z1].valueMm;
      g_program.travelZ[1] = g_sensor[SENSOR_Z2].valueMm;
      g_program.validMask |= AUTO_PROGRAM_VALID_TRAVEL_Z;
      g_programCoordEditMask &= (uint16_t)~PROG_EDIT_TRAVEL_Z_PAIR;
      Serial.println(F("CAP TRAVEL Z1/Z2 OK"));
      break;
    case CMD_PROGRAM_CAPTURE_ZONE_X:
      if (!sensorPairUsable(SENSOR_X1, SENSOR_X2)) { Serial.println(F("CAP ZONE X rejected: X1+X2 required")); return true; }
      z.xMm[0] = g_sensor[SENSOR_X1].valueMm;
      z.xMm[1] = g_sensor[SENSOR_X2].valueMm;
      z.validMask |= AUTO_ZONE_VALID_X;
      g_programCoordEditMask &= (uint16_t)~PROG_EDIT_ZONE_X_PAIR;
      z.enabled = 1;
      Serial.println(F("CAP ZONE X1/X2 OK"));
      break;
    case CMD_PROGRAM_CAPTURE_ZONE_Z:
      if (!sensorPairUsable(SENSOR_Z1, SENSOR_Z2)) { Serial.println(F("CAP ZONE Z rejected: Z1+Z2 required")); return true; }
      z.zMm[0] = g_sensor[SENSOR_Z1].valueMm;
      z.zMm[1] = g_sensor[SENSOR_Z2].valueMm;
      z.validMask |= AUTO_ZONE_VALID_Z;
      g_programCoordEditMask &= (uint16_t)~PROG_EDIT_ZONE_Z_PAIR;
      z.enabled = 1;
      Serial.println(F("CAP ZONE Z1/Z2 OK"));
      break;
    case CMD_PROGRAM_CAPTURE_DRY_X:
      if (!sensorPairUsable(SENSOR_X1, SENSOR_X2)) { Serial.println(F("CAP DRY X rejected: X1+X2 required")); return true; }
      g_program.dryX[0] = g_sensor[SENSOR_X1].valueMm;
      g_program.dryX[1] = g_sensor[SENSOR_X2].valueMm;
      g_program.validMask |= AUTO_PROGRAM_VALID_DRY_X;
      g_programCoordEditMask &= (uint16_t)~PROG_EDIT_DRY_X_PAIR;
      Serial.println(F("CAP DRY X1/X2 OK"));
      break;
    case CMD_PROGRAM_CAPTURE_DRY_Z:
      if (!sensorPairUsable(SENSOR_Z1, SENSOR_Z2)) { Serial.println(F("CAP DRY Z rejected: Z1+Z2 required")); return true; }
      g_program.dryZ[0] = g_sensor[SENSOR_Z1].valueMm;
      g_program.dryZ[1] = g_sensor[SENSOR_Z2].valueMm;
      g_program.validMask |= AUTO_PROGRAM_VALID_DRY_Z;
      g_programCoordEditMask &= (uint16_t)~PROG_EDIT_DRY_Z_PAIR;
      Serial.println(F("CAP DRY Z1/Z2 OK"));
      break;
    default: return false;
  }
  if ((g_program.validMask & (AUTO_PROGRAM_VALID_DRY_X | AUTO_PROGRAM_VALID_DRY_Z)) ==
      (AUTO_PROGRAM_VALID_DRY_X | AUTO_PROGRAM_VALID_DRY_Z)) g_program.dryValid = 1;
  markProgramDirty();
  return true;
}

bool acceptProgramCoordinates(uint16_t cmd)
{
  if (!programSelectedSlotIsLoaded()) {
    if (cmd >= CMD_PROGRAM_ACCEPT_ZONE_X && cmd <= CMD_PROGRAM_ACCEPT_DRY_Z) {
      g_programUiState = 4;
      Serial.println(F("Program accept rejected: selected slot is not loaded; press LOAD first"));
      writeProgramToDwin();
      return true;
    }
  }
  if (g_auto.running()) return true;
  AutoZoneV6 &z = g_program.zones[g_programSelectedZone];
  uint16_t required = 0;
  switch (cmd) {
    case CMD_PROGRAM_ACCEPT_ZONE_X: required = PROG_EDIT_ZONE_X_PAIR; break;
    case CMD_PROGRAM_ACCEPT_ZONE_Z: required = PROG_EDIT_ZONE_Z_PAIR; break;
    case CMD_PROGRAM_ACCEPT_HOME: required = PROG_EDIT_HOME_X_PAIR; break;
    case CMD_PROGRAM_ACCEPT_TRAVEL: required = PROG_EDIT_TRAVEL_Z_PAIR; break;
    case CMD_PROGRAM_ACCEPT_DRY_X: required = PROG_EDIT_DRY_X_PAIR; break;
    case CMD_PROGRAM_ACCEPT_DRY_Z: required = PROG_EDIT_DRY_Z_PAIR; break;
    default: return false;
  }

  if ((g_programCoordEditMask & required) != required) {
    g_programUiState = 4;
    Serial.println(F("Program numeric pair rejected: enter BOTH values first"));
    writeProgramToDwin();
    return true;
  }

  switch (cmd) {
    case CMD_PROGRAM_ACCEPT_ZONE_X: z.validMask |= AUTO_ZONE_VALID_X; z.enabled = 1; break;
    case CMD_PROGRAM_ACCEPT_ZONE_Z: z.validMask |= AUTO_ZONE_VALID_Z; z.enabled = 1; break;
    case CMD_PROGRAM_ACCEPT_HOME: g_program.validMask |= AUTO_PROGRAM_VALID_HOME_X; break;
    case CMD_PROGRAM_ACCEPT_TRAVEL: g_program.validMask |= AUTO_PROGRAM_VALID_TRAVEL_Z; break;
    case CMD_PROGRAM_ACCEPT_DRY_X: g_program.validMask |= AUTO_PROGRAM_VALID_DRY_X; break;
    case CMD_PROGRAM_ACCEPT_DRY_Z: g_program.validMask |= AUTO_PROGRAM_VALID_DRY_Z; break;
    default: return false;
  }
  g_programCoordEditMask &= (uint16_t)~required;
  g_program.dryValid = ((g_program.validMask & (AUTO_PROGRAM_VALID_DRY_X | AUTO_PROGRAM_VALID_DRY_Z)) ==
                       (AUTO_PROGRAM_VALID_DRY_X | AUTO_PROGRAM_VALID_DRY_Z)) ? 1 : 0;
  markProgramDirty();
  Serial.println(F("Program numeric coordinate pair accepted"));
  return true;
}

void printProgramSummary()
{
  Serial.println(F("--- PROGRAM V6 ---"));
  Serial.print(F("loadedSlot=")); Serial.print(g_programSlot + 1);
  Serial.print(F(" selectedSlot=")); Serial.print(g_programSelectedSlot + 1);
  Serial.print(F(" name=")); Serial.print(g_program.name);
  Serial.print(F(" dirty=")); Serial.print(g_programDirty ? 1 : 0);
  Serial.print(F(" ready=")); Serial.print(g_programStorage.readyForAuto(g_program) ? 1 : 0);
  Serial.print(F(" zones=")); Serial.println(g_program.zoneCount);
  Serial.print(F("HOME X=")); Serial.print(g_program.homeX[0]); Serial.print('/'); Serial.println(g_program.homeX[1]);
  Serial.print(F("TRAVEL Z=")); Serial.print(g_program.travelZ[0]); Serial.print('/'); Serial.println(g_program.travelZ[1]);
  Serial.print(F("validMask=0x")); Serial.print(g_program.validMask, HEX);
  Serial.print(F(" tiltSpeed=")); Serial.print(g_program.tiltPercent); Serial.println('%');
  for (uint8_t oi = 0; oi < g_program.zoneCount; ++oi) {
    const uint8_t zid = g_program.order[oi];
    if (zid >= g_program.zoneCount) continue;
    const AutoZoneV6 &z = g_program.zones[zid];
    Serial.print(F("Z")); Serial.print(zid + 1);
    Serial.print(F(" en=")); Serial.print(z.enabled);
    Serial.print(F(" valid=0x")); Serial.print(z.validMask, HEX);
    Serial.print(F(" X=")); Serial.print(z.xMm[0]); Serial.print('/'); Serial.print(z.xMm[1]);
    Serial.print(F(" Z=")); Serial.print(z.zMm[0]); Serial.print('/'); Serial.print(z.zMm[1]);
    Serial.print(F(" dip=")); Serial.print(z.dipTimeS);
    Serial.print(F(" tiltDiffMm=")); Serial.print(z.tiltStepMm);
    Serial.print(F(" wait=")); Serial.print(z.stepWaitS);
    Serial.print(F(" H%=")); Serial.print(z.movePercent);
    Serial.print(F(" V%=")); Serial.println(z.verticalPercent);
  }
  Serial.print(F("dry=")); Serial.print(g_program.dryingEnabled);
  Serial.print(F(" time=")); Serial.print(g_program.dryingTimeS);
  Serial.print(F(" staging=")); Serial.println(g_program.stagingZone + 1);
}

void setSystemMode(SystemModeV6 mode)
{
  // A released E-stop remains latched until the operator explicitly clears it.
  // Do not allow MANUAL/AUTO/HOME while the safety chain is active or latched.
  if (mode != SystemModeV6::STOP &&
      (g_safety.estopActive() || g_safety.estopLatched()))
  {
    Serial.print(F("MODE BLOCKED by E-STOP safety latch; requested="));
    Serial.println(systemModeName(mode));
    mode = SystemModeV6::STOP;
  }

  if (g_auto.running() && mode != SystemModeV6::AUTO && mode != SystemModeV6::HOME)
    g_auto.stop(F("mode change"));

  // STOP has priority and always goes through MotorControlV6::stopAll().
  g_systemMode = mode;
  g_motor.onModeChanged(g_systemMode);
  // Clear the screen-side hold state whenever mode changes. If a page switch
  // occurs while a finger is down, the existing watchdog remains a second safety net.
  g_dwin.writeU16(VP_JOG_HOLD_BITS, 0);

  writeModeToDwin();
  writeMotorStateToDwin();
  updateErrorMask();

  Serial.print(F("System mode: "));
  Serial.println(systemModeName(g_systemMode));
}

void reinitFastSensors()
{
  Serial.println(F("Manual sensor reinit requested"));
  initSensorsFast();
  writeAllValuesToDwin();
}

uint16_t motorStateForJogCommand(uint16_t cmd)
{
  switch (cmd)
  {
  case CMD_JOG_H1_FWD: return MOTOR_STATE_H1_FWD;
  case CMD_JOG_H1_BWD: return MOTOR_STATE_H1_BWD;
  case CMD_JOG_H2_FWD: return MOTOR_STATE_H2_FWD;
  case CMD_JOG_H2_BWD: return MOTOR_STATE_H2_BWD;
  case CMD_JOG_H_BOTH_FWD: return MOTOR_STATE_H_BOTH_FWD;
  case CMD_JOG_H_BOTH_BWD: return MOTOR_STATE_H_BOTH_BWD;
  case CMD_JOG_V1_UP: return MOTOR_STATE_V1_UP;
  case CMD_JOG_V1_DOWN: return MOTOR_STATE_V1_DOWN;
  case CMD_JOG_V2_UP: return MOTOR_STATE_V2_UP;
  case CMD_JOG_V2_DOWN: return MOTOR_STATE_V2_DOWN;
  case CMD_JOG_V_BOTH_UP: return MOTOR_STATE_V_BOTH_UP;
  case CMD_JOG_V_BOTH_DOWN: return MOTOR_STATE_V_BOTH_DOWN;
  default: return MOTOR_STATE_IDLE;
  }
}

bool handleManualJogCommand(uint16_t cmd)
{
  const uint16_t requestedState = motorStateForJogCommand(cmd);
  if (requestedState != MOTOR_STATE_IDLE && g_safety.blocksMotorState(requestedState))
  {
    Serial.print(F("MANUAL JOG BLOCKED by safety: "));
    Serial.println(g_safety.blockReason(requestedState));
    g_motor.manualStop(F("safety interlock"));
    writeMotorStateToDwin();
    updateErrorMask();
    return true;
  }
  bool isManualCmd = true;

  switch (cmd)
  {
  case CMD_JOG_H1_FWD:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::H1, MotorDirV6::POSITIVE);
    break;
  case CMD_JOG_H1_BWD:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::H1, MotorDirV6::NEGATIVE);
    break;
  case CMD_JOG_H2_FWD:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::H2, MotorDirV6::POSITIVE);
    break;
  case CMD_JOG_H2_BWD:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::H2, MotorDirV6::NEGATIVE);
    break;
  case CMD_JOG_H_BOTH_FWD:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::H_BOTH, MotorDirV6::POSITIVE);
    break;
  case CMD_JOG_H_BOTH_BWD:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::H_BOTH, MotorDirV6::NEGATIVE);
    break;

  case CMD_JOG_V1_UP:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::V1, MotorDirV6::POSITIVE);
    break;
  case CMD_JOG_V1_DOWN:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::V1, MotorDirV6::NEGATIVE);
    break;
  case CMD_JOG_V2_UP:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::V2, MotorDirV6::POSITIVE);
    break;
  case CMD_JOG_V2_DOWN:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::V2, MotorDirV6::NEGATIVE);
    break;
  case CMD_JOG_V_BOTH_UP:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::V_BOTH, MotorDirV6::POSITIVE);
    break;
  case CMD_JOG_V_BOTH_DOWN:
    g_motor.requestManualMove(g_systemMode, MotorAxisV6::V_BOTH, MotorDirV6::NEGATIVE);
    break;

  case CMD_JOG_STOP:
    g_motor.manualStop(F("DWIN jog stop"));
    break;

  default:
    isManualCmd = false;
    break;
  }

  if (isManualCmd)
  {
    writeMotorStateToDwin();
  }

  return isManualCmd;
}

bool handleJogHoldBits(uint16_t value)
{
  value &= JOG_HOLD_VALID_MASK;
  if (value == 0) {
    if (g_systemMode == SystemModeV6::MANUAL || g_motor.isMotionActive()) {
      g_motor.manualStop(F("DWIN jog release"));
      writeMotorStateToDwin();
    }
    return true;
  }

  // Exactly one hold bit is allowed. Multi-touch/multiple active bits is stopped
  // deliberately instead of inventing a combined direction.
  if ((value & (uint16_t)(value - 1u)) != 0) {
    Serial.print(F("DWIN jog hold rejected multi-bit=0x"));
    Serial.println(value, HEX);
    g_motor.manualStop(F("DWIN jog multi-touch"));
    writeMotorStateToDwin();
    return true;
  }

  uint8_t bit = 0;
  while (((uint16_t)1u << bit) != value && bit < 12) ++bit;
  static const uint16_t HOLD_TO_CMD[12] = {
    CMD_JOG_H1_FWD, CMD_JOG_H1_BWD,
    CMD_JOG_H2_FWD, CMD_JOG_H2_BWD,
    CMD_JOG_H_BOTH_FWD, CMD_JOG_H_BOTH_BWD,
    CMD_JOG_V1_UP, CMD_JOG_V1_DOWN,
    CMD_JOG_V2_UP, CMD_JOG_V2_DOWN,
    CMD_JOG_V_BOTH_UP, CMD_JOG_V_BOTH_DOWN
  };
  if (bit >= 12) return true;
  return handleManualJogCommand(HOLD_TO_CMD[bit]);
}

void handleCommand(uint16_t cmd)
{
  if (cmd == CMD_NONE)
    return;

  Serial.print(F("DWIN CMD=0x"));
  Serial.println(cmd, HEX);

  if (handleManualJogCommand(cmd))
  {
    g_dwin.clearCommand(VP_CMD);
    return;
  }

  if (handleVfdSettingsCommand(cmd))
  {
    writeMotorStateToDwin();
    g_dwin.clearCommand(VP_CMD);
    return;
  }

  if (captureProgramCoordinates(cmd))
  {
    g_dwin.clearCommand(VP_CMD);
    return;
  }

  if (acceptProgramCoordinates(cmd))
  {
    g_dwin.clearCommand(VP_CMD);
    return;
  }

  switch (cmd)
  {
  case CMD_ZERO_X1:
    zeroSensor(SENSOR_X1);
    break;
  case CMD_ZERO_X2:
    zeroSensor(SENSOR_X2);
    break;
  case CMD_ZERO_Z1:
    zeroSensor(SENSOR_Z1);
    break;
  case CMD_ZERO_Z2:
    zeroSensor(SENSOR_Z2);
    break;

  case CMD_SAVE:
    g_storage.save(g_settings);
    g_settingsFault = false;
    g_editSettings = g_settings;
    g_settingsDirty = false;
    g_settingsUiState = SETTINGS_UI_SAVED;
    g_settingsUiError = SETTINGS_VALID;
    writeSettingsToDwin(g_settings);
    updateErrorMask();
    Serial.println(F("All settings saved"));
    break;

  case CMD_LOAD:
    if (!g_storage.load(g_settings))
    {
      g_storage.defaults(g_settings);
      g_settingsFault = true;
      Serial.println(F("Settings load failed, defaults restored in RAM"));
    }
    else
    {
      g_settingsFault = false;
      Serial.println(F("Settings loaded"));
    }
    g_editSettings = g_settings;
    g_settingsDirty = false;
    g_settingsUiState = SETTINGS_UI_LOADED;
    g_settingsUiError = SETTINGS_VALID;
    g_motor.applySettings(g_settings);
    g_auto.applySettings(g_settings);
    applySafetySettings(g_settings);
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
      if (g_sensor[i].valid)
      {
        g_sensor[i].valueMm = g_storage.applyCalibration(g_settings, g_sensor[i].idx, g_sensor[i].rawMm);
      }
    }
    writeSettingsToDwin(g_settings);
    writeAllValuesToDwin();
    break;

  case CMD_RESET_CAL:
    g_storage.resetOffsets(g_settings);
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) g_editSettings.sensor[i] = g_settings.sensor[i];
    Serial.println(F("Calibration reset in RAM. Press SAVE to store it."));
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
      if (g_sensor[i].valid)
      {
        g_sensor[i].valueMm = g_storage.applyCalibration(g_settings, g_sensor[i].idx, g_sensor[i].rawMm);
      }
    }
    writeAllValuesToDwin();
    break;

  case CMD_MANUAL:
    g_auto.stop(F("MANUAL selected"));
    setSystemMode(SystemModeV6::MANUAL);
    break;

  case CMD_AUTO: {
    if (g_auto.running()) {
      Serial.println(F("AUTO already running; use PAUSE/RESUME/STOP"));
      break;
    }
    setSystemMode(SystemModeV6::AUTO);
    if (g_systemMode == SystemModeV6::AUTO) {
      const AutoSensorsV6 ss = buildAutoSensors();
      if (!g_auto.start(g_program, ss, g_autoSimulation, millis()))
        setSystemMode(SystemModeV6::STOP);
    }
    writePersistentHeaderToDwin();
  } break;

  case CMD_HOME: {
    g_auto.stop(F("new HOME request"));
    setSystemMode(SystemModeV6::HOME);
    if (g_systemMode == SystemModeV6::HOME) {
      const AutoSensorsV6 ss = buildAutoSensors();
      if (!g_auto.startHome(g_program, ss, g_autoSimulation, millis()))
        setSystemMode(SystemModeV6::STOP);
    }
    writePersistentHeaderToDwin();
  } break;

  case CMD_STOP:
    g_auto.stop(F("operator STOP"));
    setSystemMode(SystemModeV6::STOP);
    break;

  case CMD_AUTO_PAUSE:
    g_auto.pause();
    writePersistentHeaderToDwin();
    break;

  case CMD_AUTO_RESUME:
    g_auto.resume(millis());
    writePersistentHeaderToDwin();
    break;

  case CMD_AUTO_OPERATOR_NEXT:
    g_auto.operatorNext();
    break;

  case CMD_AUTO_SIM_TOGGLE:
    if (!SAFETY_BENCH_MODE || (VFD_RS485_ENABLED && VFD_WRITE_COMMANDS_ENABLED)) {
      Serial.println(F("AUTO SIM toggle rejected in FIELD/WRITE build"));
    } else if (!g_auto.running()) {
      g_autoSimulation = !g_autoSimulation;
      Serial.print(F("AUTO SIMULATION "));
      Serial.println(g_autoSimulation ? F("ON") : F("OFF"));
      writePersistentHeaderToDwin();
    }
    break;

  case CMD_PROGRAM_LOAD:
    if (!g_auto.running()) (void)loadProgramSlotV6(g_programSelectedSlot);
    break;

  case CMD_PROGRAM_SAVE:
    (void)saveProgramSlotV6();
    break;

  case CMD_PROGRAM_DEFAULTS:
    if (!g_auto.running()) {
      if (g_programSelectedSlot != g_programSlot) {
        g_programUiState = 4;
        Serial.println(F("Program DEFAULTS rejected: press LOAD for selected slot first"));
        writeProgramToDwin();
        break;
      }
      g_programStorage.defaults(g_program, g_programSlot);
      g_programDirty = true;
      g_programIsDemo = false;
      g_programUiState = 3;
      g_programSelectedZone = 0;
      g_programCoordEditMask = 0;
      Serial.println(F("Program safe defaults loaded in RAM; calibration required"));
      writeProgramToDwin();
    }
    break;

  case CMD_PROGRAM_SLOT_1:
  case CMD_PROGRAM_SLOT_2:
  case CMD_PROGRAM_SLOT_3:
  case CMD_PROGRAM_SLOT_4:
    if (g_auto.running()) {
      g_programUiState = 4;
      Serial.println(F("Program slot selection rejected while AUTO/HOME is running"));
      writeProgramToDwin();
    } else {
      g_programSelectedSlot = (uint8_t)(cmd - CMD_PROGRAM_SLOT_1);
      g_programUiState = 5;
      Serial.print(F("Program slot selected="));
      Serial.print(g_programSelectedSlot + 1);
      Serial.print(F("; loaded slot remains="));
      Serial.print(g_programSlot + 1);
      Serial.println(F(". Press LOAD to open selected slot."));
      writeProgramToDwin();
    }
    break;

  case CMD_PROGRAM_ZONE_PREV:
    if (!programSelectedSlotIsLoaded()) {
      g_programUiState = 4;
      Serial.println(F("Zone navigation rejected: press LOAD for selected slot first"));
      writeProgramToDwin();
    } else if (!g_auto.running()) {
      if (g_programSelectedZone == 0)
        g_programSelectedZone = (uint8_t)(g_program.zoneCount - 1);
      else
        --g_programSelectedZone;
      g_programCoordEditMask &= (uint16_t)~(PROG_EDIT_ZONE_X_PAIR | PROG_EDIT_ZONE_Z_PAIR);
      writeProgramToDwin();
    }
    break;

  case CMD_PROGRAM_ZONE_NEXT:
    if (!programSelectedSlotIsLoaded()) {
      g_programUiState = 4;
      Serial.println(F("Zone navigation rejected: press LOAD for selected slot first"));
      writeProgramToDwin();
    } else if (!g_auto.running()) {
      g_programSelectedZone = (uint8_t)((g_programSelectedZone + 1) % g_program.zoneCount);
      g_programCoordEditMask &= (uint16_t)~(PROG_EDIT_ZONE_X_PAIR | PROG_EDIT_ZONE_Z_PAIR);
      writeProgramToDwin();
    }
    break;

  case CMD_PROGRAM_ZONE_TOGGLE:
    if (!programSelectedSlotIsLoaded()) {
      g_programUiState = 4;
      Serial.println(F("Zone toggle rejected: press LOAD for selected slot first"));
      writeProgramToDwin();
    } else if (!g_auto.running()) {
      AutoZoneV6 &z = g_program.zones[g_programSelectedZone];
      z.enabled = z.enabled ? 0 : 1;
      markProgramDirty();
      Serial.print(F("Program zone "));
      Serial.print(g_programSelectedZone + 1);
      Serial.print(F(" -> "));
      Serial.println(z.enabled ? F("ON") : F("OFF"));
    }
    break;

  case CMD_SETTINGS:
    setSystemMode(SystemModeV6::SETTINGS);
    break;

  case CMD_CALIBRATION:
    setSystemMode(SystemModeV6::CALIBRATION);
    break;

  case CMD_CLEAR_STATUS:
    Serial.println(F("Status clear requested"));
    (void)g_safety.clearEstopLatch();
    if (!g_auto.running() && g_auto.error() != AUTO_ERR_NONE)
      g_auto.stop(F("status clear"));
    updateErrorMask();
    writePersistentHeaderToDwin();
    break;

  case CMD_SENSOR_REINIT:
    reinitFastSensors();
    break;

  case CMD_DIAG_SNAPSHOT:
    Serial.print(F("DIAG sensors="));
    Serial.print(sensorOnlineCount());
    Serial.print(F("/4 vfd="));
    Serial.print(g_motor.vfdConnectedCount());
    Serial.print(F("/4 rs485="));
    Serial.print(VFD_RS485_ENABLED ? F("PHYSICAL") : F("DRY/DISABLED"));
    Serial.print(F(" estop="));
    Serial.print(g_safety.estopActive() ? F("ACTIVE") : F("OK"));
    Serial.print(F(" latched="));
    Serial.print(g_safety.estopLatched() ? F("YES") : F("NO"));
    Serial.print(F(" limits=0x"));
    Serial.print(g_safety.limitMask(), HEX);
    Serial.print(F(" estopMon="));
    Serial.print(g_safety.estopEnabled() ? F("ON") : F("OFF"));
    Serial.print(F(" limitMon="));
    Serial.print(g_safety.limitsEnabled() ? F("ON") : F("OFF"));
    Serial.print(F(" writes="));
    Serial.print(VFD_WRITE_COMMANDS_ENABLED ? F("ON") : F("BLOCKED"));
    Serial.print(F(" queue="));
    Serial.println(g_motor.vfdHasPendingWork() ? F("BUSY") : F("IDLE"));
    writePersistentHeaderToDwin();
    break;

  case CMD_SAFETY_CLEAR:
    (void)g_safety.clearEstopLatch();
    updateErrorMask();
    writePersistentHeaderToDwin();
    break;

  case CMD_SAFETY_TOGGLE_ESTOP:
    if (!SAFETY_BENCH_MODE) {
      Serial.println(F("SAFETY E-stop toggle rejected outside BENCH mode"));
      break;
    }
    if (g_safety.estopEnabled()) g_settings.vfd.safetyDisableMask |= SAFETY_DISABLE_ESTOP;
    else                         g_settings.vfd.safetyDisableMask &= (uint8_t)~SAFETY_DISABLE_ESTOP;
    g_editSettings = g_settings;
    applySafetySettings(g_settings);
    g_settingsDirty = true;
    Serial.println(F("Safety setting changed in RAM; SAVE to persist"));
    writeSettingsToDwin(g_settings);
    updateErrorMask();
    writePersistentHeaderToDwin();
    break;

  case CMD_SAFETY_TOGGLE_LIMITS:
    // Controller-side limits may be disabled when the cabinet implements them
    // as an external hardwired safety function. Actual state is always shown.
    if (g_safety.limitsEnabled()) g_settings.vfd.safetyDisableMask |= SAFETY_DISABLE_LIMITS;
    else                          g_settings.vfd.safetyDisableMask &= (uint8_t)~SAFETY_DISABLE_LIMITS;
    g_editSettings = g_settings;
    applySafetySettings(g_settings);
    g_settingsDirty = true;
    Serial.println(F("Safety setting changed in RAM; SAVE to persist"));
    writeSettingsToDwin(g_settings);
    updateErrorMask();
    writePersistentHeaderToDwin();
    break;

  default:
    Serial.println(F("Unknown command"));
    break;
  }

  g_dwin.clearCommand(VP_CMD);
}

void printBenchConsoleHelp()
{
  Serial.println(F("USB console:"));
  Serial.println(F("  help | clear | diag | estop on/off | limits on/off | save"));
  Serial.println(F("  log quiet | log normal | log verbose"));
  Serial.println(F("  test h1/h2/v1/v2/all | he200 comm"));
  Serial.println(F("  sim on | sim off | sim demo | auto | home | stop | pause | resume | next"));
  Serial.println(F("  prog show | prog slot 1..4 | prog load | prog save | prog defaults"));
  Serial.println(F("  set home X1 X2 | set travel Z1 Z2"));
  Serial.println(F("  set zone N x X1 X2 | set zone N z Z1 Z2"));
  Serial.println(F("  set dry x X1 X2 | set dry z Z1 Z2"));
  Serial.println(F("  zone N on/off | zone N dip S | wait S | tilt MM | speed H V"));
  Serial.println(F("  tilt speed P   (legacy global tilt speed, 1..100%)"));
  Serial.println(F("  cap home | cap travel | cap zone N x/z | cap dry x/z"));
}

void setBenchSafetyFromConsole(bool estop, bool enabled)
{
  if (estop && !SAFETY_BENCH_MODE && !enabled)
  {
    Serial.println(F("Console E-stop disable rejected outside BENCH mode"));
    return;
  }

  const uint8_t bit = estop ? SAFETY_DISABLE_ESTOP : SAFETY_DISABLE_LIMITS;
  if (enabled) g_settings.vfd.safetyDisableMask &= (uint8_t)~bit;
  else         g_settings.vfd.safetyDisableMask |= bit;
  g_editSettings = g_settings;
  applySafetySettings(g_settings);
  g_settingsDirty = true;
  writeSettingsToDwin(g_settings);
  updateErrorMask();
  writePersistentHeaderToDwin();
  Serial.println(F("Safety setting changed in RAM; type 'save' to persist"));
}

bool setSimulationFromConsole(bool enabled)
{
  if (g_auto.running()) {
    Serial.println(F("SIM change rejected while AUTO/HOME is running"));
    return false;
  }
  if (enabled && (!SAFETY_BENCH_MODE || (VFD_RS485_ENABLED && VFD_WRITE_COMMANDS_ENABLED))) {
    Serial.println(F("SIM rejected in FIELD/physical WRITE build"));
    return false;
  }
  g_autoSimulation = enabled;
  Serial.print(F("AUTO SIMULATION "));
  Serial.println(enabled ? F("ON") : F("OFF"));
  writePersistentHeaderToDwin();
  return true;
}

bool consoleCoordinate(int value)
{
  return value >= 0 && value <= (int)LASER_MAX_MM;
}

bool consoleSetPair(int32_t out[2], int a, int b)
{
  if (!consoleCoordinate(a) || !consoleCoordinate(b)) {
    Serial.print(F("Coordinate rejected; allowed 0.."));
    Serial.print(LASER_MAX_MM);
    Serial.println(F(" mm"));
    return false;
  }
  out[0] = a;
  out[1] = b;
  return true;
}

void serviceUsbConsole()
{
  static char line[96];
  static uint8_t len = 0;

  while (Serial.available())
  {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n')
    {
      if ((size_t)len + 1U < sizeof(line)) line[len++] = c;
      continue;
    }

    line[len] = '\0';
    len = 0;
    if (line[0] == '\0') continue;

    if (!strcmp(line, "help")) printBenchConsoleHelp();
    else if (!strcmp(line, "log quiet")) {
      g_consolePeriodicEnabled = false;
      Serial.println(F("Periodic status log QUIET; events/faults remain visible"));
    }
    else if (!strcmp(line, "log normal")) {
      g_consolePeriodicEnabled = true;
      g_consolePrintIntervalMs = 2000;
      Serial.println(F("Periodic status log NORMAL: 2 s"));
    }
    else if (!strcmp(line, "log verbose")) {
      g_consolePeriodicEnabled = true;
      g_consolePrintIntervalMs = 500;
      Serial.println(F("Periodic status log VERBOSE: 0.5 s"));
    }
    else if (!strcmp(line, "clear")) handleCommand(CMD_SAFETY_CLEAR);
    else if (!strcmp(line, "diag")) handleCommand(CMD_DIAG_SNAPSHOT);
    else if (!strcmp(line, "save")) handleCommand(CMD_SAVE);
    else if (!strcmp(line, "estop on")) setBenchSafetyFromConsole(true, true);
    else if (!strcmp(line, "estop off")) setBenchSafetyFromConsole(true, false);
    else if (!strcmp(line, "limits on")) setBenchSafetyFromConsole(false, true);
    else if (!strcmp(line, "limits off")) setBenchSafetyFromConsole(false, false);
    else if (!strcmp(line, "he200 comm")) {
      if (!HE200_COMMISSIONING) {
        Serial.println(F("he200 comm is intended for the HE200 commissioning build"));
      } else {
        g_settings.vfd.baudCode = VFD_BAUD_9600;
        g_settings.vfd.parity = VFD_PARITY_NONE;
        g_settings.vfd.stopBits = 1;
        g_settings.vfd.retries = 1;
        g_settings.vfd.responseTimeoutMs = 250;
        g_settings.vfd.interRequestMs = 20;
        g_settings.vfd.address[DRIVE_H1] = 1;
        g_settings.vfd.address[DRIVE_H2] = 2;
        g_settings.vfd.address[DRIVE_V1] = 3;
        g_settings.vfd.address[DRIVE_V2] = 4;
        g_editSettings = g_settings;
        g_motor.applySettings(g_settings);
        g_settingsDirty = true;
        writeSettingsToDwin(g_settings);
        Serial.println(F("HE200 COMM applied in RAM: 9600 8-N-1, addr H1/H2/V1/V2=1/2/3/4, timeout=250ms"));
        Serial.println(F("Not saved to EEPROM; use DWIN/console SAVE only after communication is verified"));
      }
    }
    else if (!strcmp(line, "test h1")) handleCommand(CMD_VFD_TEST_H1);
    else if (!strcmp(line, "test h2")) handleCommand(CMD_VFD_TEST_H2);
    else if (!strcmp(line, "test v1")) handleCommand(CMD_VFD_TEST_V1);
    else if (!strcmp(line, "test v2")) handleCommand(CMD_VFD_TEST_V2);
    else if (!strcmp(line, "test all")) handleCommand(CMD_VFD_TEST_ALL);
    else if (!strcmp(line, "auto")) handleCommand(CMD_AUTO);
    else if (!strcmp(line, "home")) handleCommand(CMD_HOME);
    else if (!strcmp(line, "stop")) handleCommand(CMD_STOP);
    else if (!strcmp(line, "pause")) handleCommand(CMD_AUTO_PAUSE);
    else if (!strcmp(line, "resume")) handleCommand(CMD_AUTO_RESUME);
    else if (!strcmp(line, "next")) handleCommand(CMD_AUTO_OPERATOR_NEXT);
    else if (!strcmp(line, "sim on")) (void)setSimulationFromConsole(true);
    else if (!strcmp(line, "sim off")) (void)setSimulationFromConsole(false);
    else if (!strcmp(line, "sim demo")) {
      if (setSimulationFromConsole(true)) {
        g_programStorage.demo(g_program);
        g_programSlot = 0;
        g_programSelectedZone = 0;
        g_programDirty = true;
        g_programIsDemo = true;
        Serial.println(F("RAM-only SIM-DEMO loaded; it cannot be saved to EEPROM"));
        writeProgramToDwin();
        printProgramSummary();
      }
    }
    else if (!strcmp(line, "prog show")) printProgramSummary();
    else if (!strcmp(line, "prog load")) (void)loadProgramSlotV6(g_programSlot);
    else if (!strcmp(line, "prog save")) (void)saveProgramSlotV6();
    else if (!strcmp(line, "prog defaults")) handleCommand(CMD_PROGRAM_DEFAULTS);
    else {
      int n=0, a=0, b=0;
      char word[12] = {0};
      char word2[12] = {0};

      if (sscanf(line, "prog slot %d", &n) == 1) {
        if (n >= 1 && n <= AUTO_PROGRAM_SLOTS_V6 && !g_auto.running())
          (void)loadProgramSlotV6((uint8_t)(n - 1));
        else Serial.println(F("Program slot rejected; use 1..4 and STOP first"));
      }
      else if (sscanf(line, "set home %d %d", &a, &b) == 2) {
        if (consoleSetPair(g_program.homeX, a, b)) {
          g_program.validMask |= AUTO_PROGRAM_VALID_HOME_X; markProgramDirty();
        }
      }
      else if (sscanf(line, "set travel %d %d", &a, &b) == 2) {
        if (consoleSetPair(g_program.travelZ, a, b)) {
          g_program.validMask |= AUTO_PROGRAM_VALID_TRAVEL_Z; markProgramDirty();
        }
      }
      else if (sscanf(line, "set zone %d %11s %d %d", &n, word, &a, &b) == 4) {
        if (n < 1 || n > AUTO_MAX_ZONES_V6 || g_auto.running()) {
          Serial.println(F("Zone SET rejected"));
        } else {
          const uint8_t zi = (uint8_t)(n - 1);
          if (zi >= g_program.zoneCount) {
            g_program.zoneCount = zi + 1;
            for (uint8_t i=0; i<g_program.zoneCount; ++i) g_program.order[i]=i;
          }
          AutoZoneV6 &z = g_program.zones[zi];
          bool ok=false;
          if (!strcmp(word,"x")) { ok=consoleSetPair(z.xMm,a,b); if(ok) z.validMask|=AUTO_ZONE_VALID_X; }
          else if (!strcmp(word,"z")) { ok=consoleSetPair(z.zMm,a,b); if(ok) z.validMask|=AUTO_ZONE_VALID_Z; }
          if (ok) { z.enabled=1; g_programSelectedZone=zi; markProgramDirty(); }
          else if (strcmp(word,"x") && strcmp(word,"z")) Serial.println(F("Use x or z"));
        }
      }
      else if (sscanf(line, "set dry %11s %d %d", word, &a, &b) == 3) {
        bool ok=false;
        if (!strcmp(word,"x")) { ok=consoleSetPair(g_program.dryX,a,b); if(ok) g_program.validMask|=AUTO_PROGRAM_VALID_DRY_X; }
        else if (!strcmp(word,"z")) { ok=consoleSetPair(g_program.dryZ,a,b); if(ok) g_program.validMask|=AUTO_PROGRAM_VALID_DRY_Z; }
        if (ok) {
          g_program.dryValid = ((g_program.validMask & (AUTO_PROGRAM_VALID_DRY_X|AUTO_PROGRAM_VALID_DRY_Z)) ==
                               (AUTO_PROGRAM_VALID_DRY_X|AUTO_PROGRAM_VALID_DRY_Z));
          markProgramDirty();
        }
      }
      else if (sscanf(line, "tilt speed %d", &a) == 1) {
        if (a>=1 && a<=100 && !g_auto.running()) { g_program.tiltPercent=(uint8_t)a; markProgramDirty(); }
        else Serial.println(F("Use: tilt speed 1..100 while stopped"));
      }
      else if (sscanf(line, "zone %d %11s %d", &n, word, &a) == 3) {
        if (n < 1 || n > g_program.zoneCount || g_auto.running()) Serial.println(F("Zone command rejected"));
        else {
          AutoZoneV6 &z=g_program.zones[n-1]; bool ok=true;
          if (!strcmp(word,"dip") && a>=0 && a<=36000) z.dipTimeS=(uint16_t)a;
          else if (!strcmp(word,"wait") && a>=0 && a<=36000) z.stepWaitS=(uint16_t)a;
          else if (!strcmp(word,"tilt") && a>=0 && a<=5000) z.tiltStepMm=(uint16_t)a;
          else ok=false;
          if(ok) { g_programSelectedZone=n-1; markProgramDirty(); }
          else Serial.println(F("Use: zone N dip/wait/tilt VALUE"));
        }
      }
      else if (sscanf(line, "zone %d speed %d %d", &n, &a, &b) == 3) {
        if (n>=1 && n<=g_program.zoneCount && a>=1 && a<=100 && b>=1 && b<=100 && !g_auto.running()) {
          AutoZoneV6 &z=g_program.zones[n-1]; z.movePercent=(uint8_t)a; z.verticalPercent=(uint8_t)b;
          g_programSelectedZone=n-1; markProgramDirty();
        } else Serial.println(F("Use: zone N speed H V, 1..100"));
      }
      else if (sscanf(line, "zone %d %11s", &n, word) == 2 && (!strcmp(word,"on") || !strcmp(word,"off"))) {
        if (n>=1 && n<=g_program.zoneCount && !g_auto.running()) {
          g_program.zones[n-1].enabled = !strcmp(word,"on") ? 1 : 0;
          g_programSelectedZone=n-1; markProgramDirty();
        } else Serial.println(F("Zone enable rejected"));
      }
      else if (sscanf(line, "cap zone %d %11s", &n, word) == 2) {
        if (n>=1 && n<=AUTO_MAX_ZONES_V6 && !g_auto.running()) {
          const uint8_t zi=(uint8_t)(n-1);
          if (zi>=g_program.zoneCount) { g_program.zoneCount=zi+1; for(uint8_t i=0;i<g_program.zoneCount;++i)g_program.order[i]=i; }
          g_programSelectedZone=zi;
          if (!strcmp(word,"x")) handleCommand(CMD_PROGRAM_CAPTURE_ZONE_X);
          else if (!strcmp(word,"z")) handleCommand(CMD_PROGRAM_CAPTURE_ZONE_Z);
          else Serial.println(F("Use x or z"));
        }
      }
      else if (sscanf(line, "cap %11s %11s", word, word2) == 2 && !strcmp(word,"dry")) {
        if (!strcmp(word2,"x")) handleCommand(CMD_PROGRAM_CAPTURE_DRY_X);
        else if (!strcmp(word2,"z")) handleCommand(CMD_PROGRAM_CAPTURE_DRY_Z);
        else Serial.println(F("Use: cap dry x/z"));
      }
      else if (!strcmp(line,"cap home")) handleCommand(CMD_PROGRAM_CAPTURE_HOME);
      else if (!strcmp(line,"cap travel")) handleCommand(CMD_PROGRAM_CAPTURE_TRAVEL);
      else {
        Serial.print(F("Unknown USB console command: "));
        Serial.println(line);
        printBenchConsoleHelp();
      }
    }
  }
}

void setup()
{
  Serial.begin(DBG_BAUD);
  delay(300);
  Serial.println();
  Serial.println(F("========== V6 SYSTEM STEP9E HE200 READ-ONLY COMMISSIONING START =========="));
  Serial.print(F("FW: ")); Serial.println(F(FW_VERSION_V6_BRINGUP));

  g_dwin.begin(DWIN_BAUD);
  g_safety.begin();

  if (!g_storage.load(g_settings))
  {
    g_storage.defaults(g_settings);
    g_settingsFault = true;
    Serial.println(F("Settings: invalid/missing EEPROM record, defaults in RAM"));
  }
  else
  {
    g_settingsFault = false;
    Serial.println(g_storage.lastLoadMigrated()
                       ? F("Settings: EEPROM v1 migrated to v2; calibration preserved")
                       : F("Settings: loaded from EEPROM v2"));
  }

  g_editSettings = g_settings;
  g_settingsDirty = false;
  g_settingsUiState = SETTINGS_UI_IDLE;
  g_settingsUiError = SETTINGS_VALID;

  applySafetySettings(g_settings);
  g_motor.begin(g_settings);
  g_auto.begin(g_motor, g_settings);

  // Programs live in a separate EEPROM area. Invalid/empty memory is never
  // auto-filled with runnable coordinates: safe uncalibrated defaults stay in RAM.
  g_programSlot = g_programStorage.loadActiveSlot();
  g_programSelectedSlot = g_programSlot;
  (void)loadProgramSlotV6(g_programSlot);

  if (g_safety.estopActive() || g_safety.estopLatched())
  {
    g_motor.stopAll(F("E-STOP at boot"));
    g_systemMode = SystemModeV6::STOP;
  }

  initSensorsFast();
  writeBootScreen();
  writeSettingsToDwin(g_settings);
  writeProgramToDwin();
  writeAllValuesToDwin();

  Serial.println(F("Commands: 0x0001 MANUAL, 0x0002 AUTO START, 0x0003 HOME, 0x0004 STOP, 0x0005 SETTINGS, 0x0006 CALIBRATION"));
  Serial.println(F("Calibration: 0x0021-0x0024 ZERO, 0x0030 SAVE, 0x0031 LOAD, 0x0032 RESET CAL"));
  Serial.println(F("Service: 0x0044 CLEAR STATUS, 0x0045 SENSOR REINIT, 0x0046 DIAG, 0x0047 SAFETY CLEAR"));
  Serial.println(F("Bench safety: 0x0048 TOGGLE E-STOP monitor, 0x0049 TOGGLE LIMIT monitor; SAVE persists"));
  Serial.println(F("Manual jog legacy: 0x0101..0x010C, 0x010F JOG STOP"));
  Serial.println(F("Manual hold-to-run: VP 0x1110 bit0..11, DGUS Bit Button/Inching; release=STOP"));
  Serial.println(F("AUTO: 0x0300 PAUSE, 0x0301 RESUME, 0x0302 OPERATOR NEXT, 0x0303 SIM TOGGLE"));
  Serial.println(F("Program: 0x0314..0x0317 SELECT SLOT, 0x0310 LOAD SELECTED, 0x0311 SAVE LOADED, 0x0312 DEFAULTS, 0x0318/19 PREV/NEXT, 0x031A TOGGLE, 0x0320..0x032B capture/accept"));

  if (!VFD_RS485_ENABLED)
    Serial.println(F("VFD layer: DRY-RUN; MAX485 disabled, exact RTU frames available for manual tests"));
  else if (!VFD_WRITE_COMMANDS_ENABLED)
    Serial.println(HE200_COMMISSIONING
                       ? F("VFD layer: HE200 PHYSICAL RS485 READ-ONLY; monitoring only; ALL writes/motion blocked")
                       : F("VFD layer: PHYSICAL RS485 READ-ONLY; all writes/motion blocked"));
  else if (!AUTO_PHYSICAL_ENABLED)
    Serial.println(F("VFD layer: FIELD MANUAL WRITE; physical AUTO/HOME compile-time BLOCKED"));
  else
    Serial.println(F("VFD layer: FIELD AUTO WRITE ENABLED"));

  Serial.print(F("Physical AUTO/HOME interlock: "));
  Serial.println(AUTO_PHYSICAL_ENABLED ? F("ENABLED") : F("BLOCKED"));
  Serial.println(F("VFD settings: 0x0200 APPLY, 0x0201 SAVE, 0x0202 LOAD, 0x0203 DEFAULTS, 0x0204..0x0208 TEST"));
  printBenchConsoleHelp();
  printProgramSummary();

  Serial.print(F("Safety inputs: "));
  Serial.print(SAFETY_BENCH_MODE ? F("BENCH active-low") : F("FIELD NC active-high"));
  Serial.print(F(" E-stop="));
  Serial.print(g_safety.estopActive() ? F("ACTIVE") : F("OK"));
  Serial.print(F(" limits=0x"));
  Serial.println(g_safety.limitMask(), HEX);

  Serial.print(F("Manual jog watchdog from EEPROM: "));
  Serial.print(MANUAL_JOG_TIMEOUT_ENABLED ? F("ON ") : F("OFF "));
  Serial.print(g_motor.manualJogTimeoutMs());
  Serial.println(F("ms"));
}

void loop()
{
  serviceSensorsFast();
  serviceUsbConsole();

  // IMPORTANT: take the loop timestamp AFTER the USB console. A console
  // command may start/resume AUTO using millis(); reusing a timestamp captured
  // before that command makes unsigned timeout arithmetic look like a 49-day
  // wrap and can cause an immediate movement timeout.
  const uint32_t now = millis();

  const bool safetyChanged = g_safety.service();
  const bool estopBlocked = g_safety.estopActive() || g_safety.estopLatched();
  if (estopBlocked && (g_auto.running() || g_motor.isMotionActive() || g_systemMode != SystemModeV6::STOP))
  {
    if (g_auto.running()) g_auto.stop(g_safety.estopActive() ? F("E-STOP") : F("E-STOP latched"));
    g_motor.stopAll(g_safety.estopActive() ? F("E-STOP") : F("E-STOP latched"));
    g_systemMode = SystemModeV6::STOP;
    writeModeToDwin();
    writeMotorStateToDwin();
  }
  else if (g_systemMode == SystemModeV6::MANUAL &&
           g_motor.isMotionActive() && g_safety.blocksMotorState(g_motor.stateCode()))
  {
    // In AUTO the runner knows the target direction and owns directional-limit
    // handling. This generic check is intentionally MANUAL-only.
    g_motor.stopAll(F("limit switch"));
    g_systemMode = SystemModeV6::STOP;
    writeModeToDwin();
    writeMotorStateToDwin();
  }
  if (safetyChanged)
  {
    updateErrorMask();
    writePersistentHeaderToDwin();
  }

  // Automatic runner continuously generates logical H(+right) / V(+up) targets.
  // In SIM no motor command is emitted. In physical builds start() itself enforces
  // RS485 write + FIELD safety + explicit AUTO_PHYSICAL_ENABLED.
  if ((g_systemMode == SystemModeV6::AUTO || g_systemMode == SystemModeV6::HOME) && g_auto.running())
  {
    const bool autoChanged = g_auto.service(now, buildAutoSensors(), estopBlocked, g_safety.limitMask());
    if (autoChanged) {
      writeMotorStateToDwin();
      writePersistentHeaderToDwin();
    }
  }

  // DONE or FAULT returns the high-level system to STOP while preserving the
  // runner's terminal phase/error for DWIN diagnostics.
  if ((g_systemMode == SystemModeV6::AUTO || g_systemMode == SystemModeV6::HOME) &&
      !g_auto.running() &&
      (g_auto.phase() == AutoRunnerV6::Phase::DONE || g_auto.phase() == AutoRunnerV6::Phase::FAULT))
  {
    const bool wasSimulation = g_auto.simulation();
    g_systemMode = SystemModeV6::STOP;
    if (wasSimulation) g_motor.autoStop(F("simulation terminal state"));
    else g_motor.onModeChanged(SystemModeV6::STOP);
    writeModeToDwin();
    writeMotorStateToDwin();
    writePersistentHeaderToDwin();
  }

  if (g_motor.service(now))
  {
    writeMotorStateToDwin();
  }

  uint16_t rxVp = 0;
  uint16_t rxValue = 0;
  uint8_t dwinFrames = 0;
  while (dwinFrames < 8 && g_dwin.pollWriteU16(rxVp, rxValue))
  {
    ++dwinFrames;
    if (rxVp == VP_CMD)
      handleCommand(rxValue);
    else if (rxVp == VP_JOG_HOLD_BITS)
      (void)handleJogHoldBits(rxValue);
    else if (!updatePendingSettingsFromVp(rxVp, rxValue) && !updateProgramFromVp(rxVp, rxValue))
    {
      Serial.print(F("DWIN write ignored VP=0x"));
      Serial.print(rxVp, HEX);
      Serial.print(F(" value="));
      Serial.println(rxValue);
    }
  }

  static uint32_t lastUiMs = 0;
  if (now - lastUiMs >= FAST_DWIN_UPDATE_MS)
  {
    lastUiMs = now;
    writeModeToDwin();
    writeMotorStateToDwin();
    writeCoordinatesToDwin();
    writePersistentHeaderToDwin();
    updateErrorMask();
  }

  static uint32_t lastPrintMs = 0;
  if (g_consolePeriodicEnabled && now - lastPrintMs >= g_consolePrintIntervalMs)
  {
    lastPrintMs = now;
    Serial.print(F("mode="));
    Serial.print((uint8_t)g_systemMode);
    Serial.print(F(" motor="));
    Serial.print(g_motor.stateCode());
    Serial.print(' ');
    Serial.print(g_motor.stateName());
    Serial.print(F(" vfd="));
    Serial.print(g_motor.vfdStatusCode());
    Serial.print(F(" saf="));
    Serial.print(g_safety.stateWord());
    Serial.print(F(" lim=0x"));
    Serial.print(g_safety.limitMask(), HEX);
    Serial.print(F(" auto="));
    Serial.print((uint8_t)g_auto.phase());
    Serial.print('/');
    Serial.print(g_auto.currentStep());
    Serial.print('/');
    Serial.print(g_auto.totalSteps());
    if (g_auto.simulation()) Serial.print(F(" SIM"));
    Serial.print(F(" X1="));
    Serial.print(g_auto.simulation() ? g_auto.simPosition(SENSOR_X1) : g_sensor[SENSOR_X1].valueMm);
    Serial.print(F(" a1="));
    Serial.print(g_sensor[SENSOR_X1].lastValidMs ? (now - g_sensor[SENSOR_X1].lastValidMs) : 9999);
    Serial.print(F(" X2="));
    Serial.print(g_auto.simulation() ? g_auto.simPosition(SENSOR_X2) : g_sensor[SENSOR_X2].valueMm);
    Serial.print(F(" a2="));
    Serial.print(g_sensor[SENSOR_X2].lastValidMs ? (now - g_sensor[SENSOR_X2].lastValidMs) : 9999);
    Serial.print(F(" Z1="));
    Serial.print(g_auto.simulation() ? g_auto.simPosition(SENSOR_Z1) : g_sensor[SENSOR_Z1].valueMm);
    Serial.print(F(" Z2="));
    Serial.print(g_auto.simulation() ? g_auto.simPosition(SENSOR_Z2) : g_sensor[SENSOR_Z2].valueMm);
    if (g_auto.error()) { Serial.print(F(" autoErr=")); Serial.print(g_auto.error()); }
    Serial.println();
  }
}
