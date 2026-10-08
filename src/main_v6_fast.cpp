#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "config_v6_bringup.h"
#include "sc16is752.h"
#include "fast_laser_sensor.h"
#include "sensor_guard_v6.h"
#include "he200_service_v6.h"
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

static DwinLink g_dwin(DESKTOP_SIMULATION_ENABLED?Serial:Serial2);
static SettingsStorageV6 g_storage;
static SettingsV6 g_settings;
static SettingsV6 g_editSettings;
static MotorControlV6 g_motor;
static SafetyV6 g_safety;
static ProgramStorageV6 g_programStorage;
static AutoProgramV6 g_program;
static AutoRunnerV6 g_auto;
static He200ServiceV6 g_he200Service;
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

// Step9G/9H diagnostic acquisition mode. Continuous mode remains the production
// baseline; alternating single-shot is a controlled experiment for X1/X2.
enum class LaserServiceModeV6 : uint8_t {
  Continuous = 0,
  AlternatingX = 1
};

struct LaserAlternatingXStateV6 {
  LaserServiceModeV6 mode = LaserServiceModeV6::Continuous;
  bool preparing = false;
  uint8_t targetHzPerSensor = 5;
  uint8_t resolutionCode = (uint8_t)FastLaserResolution::Mm1;
  uint8_t nextSensor = SENSOR_X1;
  int8_t activeSensor = -1;
  uint32_t lastRequestMs = 0;
  uint32_t nextRequestMs = 0;
  uint16_t requestSpacingMs = 100;
};

static LaserAlternatingXStateV6 g_laserAlternatingX;

static bool g_settingsFault = false;
static bool g_settingsDirty = false;
static uint16_t g_settingsUiState = SETTINGS_UI_IDLE;
static uint16_t g_settingsUiError = SETTINGS_VALID;

void handleCommand(uint16_t cmd);

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
#if V6_DESKTOP_SIMULATION_ENABLED
#include "simulation_plant_v6.h"
#include "direction_calibration_v6.h"
static SimulationPlantV6 g_simPlant;
static DirectionCalibrationV6 g_simDirections;
#endif

static SensorGuardV6 g_sensorGuard[SENSOR_COUNT];
static bool g_sensorGuardEnabled = true;
static bool g_guardMotionX = false;
static bool g_guardMotionZ = false;

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

SensorGuardConfigV6 sensorGuardConfigV6()
{
  SensorGuardConfigV6 cfg;
  cfg.warningAgeMs = SENSOR_GUARD_WARNING_MS;
  cfg.shadowStopAgeMs = SENSOR_GUARD_SHADOW_STOP_MS;
  cfg.faultAgeMs = SENSOR_GUARD_FAULT_MS;
  cfg.nominalPeriodMs = 1000U / FAST_LASER_DEFAULT_FREQ_HZ;
  cfg.diagnosticMaxVelocityMmS = SENSOR_GUARD_REFERENCE_VELOCITY_MM_S;
  cfg.diagnosticJumpMarginMm = SENSOR_GUARD_JUMP_MARGIN_MM;
  cfg.recoveryFrames = SENSOR_GUARD_RECOVERY_FRAMES;
  return cfg;
}

void configureSensorGuards(uint32_t now, bool resetStatistics)
{
  const SensorGuardConfigV6 cfg = sensorGuardConfigV6();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    g_sensorGuard[i].configure(cfg);
    if (resetStatistics) g_sensorGuard[i].reset(now);
    g_sensorGuard[i].setEnabled(g_sensorGuardEnabled && g_settings.sensor[i].enabled, now);
    const bool motion = (i == SENSOR_X1 || i == SENSOR_X2) ? g_guardMotionX : g_guardMotionZ;
    (void)g_sensorGuard[i].setMotionMarked(motion, now);
  }
}

uint8_t sensorGuardStateRank(SensorGuardStateV6 state)
{
  switch (state)
  {
    case SensorGuardStateV6::Fault: return 6;
    case SensorGuardStateV6::NoData: return 5;
    case SensorGuardStateV6::Lost: return 4;
    case SensorGuardStateV6::Recovering: return 3;
    case SensorGuardStateV6::Warning: return 2;
    case SensorGuardStateV6::Healthy: return 1;
    default: return 0;
  }
}

void printSensorGuardEvent(uint8_t index, const SensorGuardEventV6 &event,
                           uint32_t now)
{
  if (index >= SENSOR_COUNT) return;
  if (event.stateChanged)
  {
    Serial.print(F("@SENSOR_EVENT name=")); Serial.print(g_sensor[index].label);
    Serial.print(F(" type=STATE from=")); Serial.print(SensorGuardV6::stateName(event.previousState));
    Serial.print(F(" to=")); Serial.print(SensorGuardV6::stateName(event.currentState));
    Serial.print(F(" age="));
    const uint32_t age = g_sensorGuard[index].ageMs(now);
    if (age == UINT32_MAX) Serial.print(F("NA")); else Serial.print(age);
    Serial.print(F(" motion=")); Serial.print(g_sensorGuard[index].motionMarked() ? 1 : 0);
    Serial.print(F(" shadowStop=")); Serial.println(g_sensorGuard[index].shadowStopActive() ? 1 : 0);
  }
  if (event.gapClosed && event.gapMs > SENSOR_GUARD_WARNING_MS)
  {
    Serial.print(F("@SENSOR_EVENT name=")); Serial.print(g_sensor[index].label);
    Serial.print(F(" type=GAP_CLOSE gapMs=")); Serial.print(event.gapMs);
    Serial.print(F(" fromMm=")); Serial.print(event.gapStartMm);
    Serial.print(F(" toMm=")); Serial.print(event.gapEndMm);
    Serial.print(F(" blindRefMm="));
    Serial.println((uint32_t)(((uint32_t)event.gapMs * SENSOR_GUARD_REFERENCE_VELOCITY_MM_S + 999UL) / 1000UL));
  }
  if (event.shadowStopChanged)
  {
    Serial.print(F("@SENSOR_EVENT name=")); Serial.print(g_sensor[index].label);
    Serial.print(F(" type=SHADOW_STOP active=")); Serial.print(event.shadowStopActive ? 1 : 0);
    Serial.print(F(" state=")); Serial.print(SensorGuardV6::stateName(g_sensorGuard[index].state()));
    Serial.print(F(" NOTE="));
    Serial.println(event.shadowStopActive ? F("WOULD_STOP; Step9H does not command motors") : F("CLEARED_AFTER_RECOVERY_OR_MOTION_END"));
  }
}

void printSensorGuardStatusLine(uint8_t index)
{
  if (index >= SENSOR_COUNT) return;
  const uint32_t now = millis();
  const SensorGuardV6 &guard = g_sensorGuard[index];
  const SensorGuardStatsV6 &st = guard.stats();
  Serial.print(F("@SENSOR_GUARD name=")); Serial.print(g_sensor[index].label);
  Serial.print(F(" state=")); Serial.print(SensorGuardV6::stateName(guard.state()));
  Serial.print(F(" enabled=")); Serial.print(guard.enabled() ? 1 : 0);
  Serial.print(F(" motion=")); Serial.print(guard.motionMarked() ? 1 : 0);
  Serial.print(F(" shadowStop=")); Serial.print(guard.shadowStopActive() ? 1 : 0);
  Serial.print(F(" age="));
  const uint32_t age = guard.ageMs(now);
  if (age == UINT32_MAX) Serial.print(F("NA")); else Serial.print(age);
  Serial.print(F(" recovery=")); Serial.print(guard.recoveryStreak());
  Serial.print(F(" frames=")); Serial.print(st.acceptedFrames);
  Serial.print(F(" dtAvg=")); Serial.print(guard.averageIntervalMs());
  Serial.print(F(" dtMin=")); Serial.print(st.minIntervalMs);
  Serial.print(F(" dtMax=")); Serial.print(st.maxIntervalMs);
  Serial.print(F(" bin250=")); Serial.print(st.intervalBins[0]);
  Serial.print(F(" bin400=")); Serial.print(st.intervalBins[1]);
  Serial.print(F(" bin600=")); Serial.print(st.intervalBins[2]);
  Serial.print(F(" bin800=")); Serial.print(st.intervalBins[3]);
  Serial.print(F(" bin1200=")); Serial.print(st.intervalBins[4]);
  Serial.print(F(" bin1500=")); Serial.print(st.intervalBins[5]);
  Serial.print(F(" binOver1500=")); Serial.print(st.intervalBins[6]);
  Serial.print(F(" warnGaps=")); Serial.print(st.warningGapCount);
  Serial.print(F(" stopGaps=")); Serial.print(st.shadowStopGapCount);
  Serial.print(F(" faultGaps=")); Serial.print(st.faultGapCount);
  Serial.print(F(" maxGap=")); Serial.print(st.maxGapMs);
  Serial.print(F(" maxGapFrom=")); Serial.print(st.maxGapStartMm);
  Serial.print(F(" maxGapTo=")); Serial.print(st.maxGapEndMm);
  Serial.print(F(" blindRefMm=")); Serial.print(guard.estimatedBlindTravelMm());
  Serial.print(F(" jumpDiag=")); Serial.print(st.suspiciousJumpCount);
  Serial.print(F(" maxJump=")); Serial.print(st.largestJumpMm);
  Serial.print(F(" maxSpeed=")); Serial.println(st.maxObservedSpeedMmS);
}

void printAxisGuardStatus(const char *axis, uint8_t first, uint8_t second)
{
  SensorGuardStateV6 state = g_sensorGuard[first].state();
  if (sensorGuardStateRank(g_sensorGuard[second].state()) > sensorGuardStateRank(state))
    state = g_sensorGuard[second].state();
  const bool motion = g_sensorGuard[first].motionMarked() || g_sensorGuard[second].motionMarked();
  const bool stop = g_sensorGuard[first].shadowStopActive() || g_sensorGuard[second].shadowStopActive();
  Serial.print(F("@AXIS_GUARD axis=")); Serial.print(axis);
  Serial.print(F(" state=")); Serial.print(SensorGuardV6::stateName(state));
  Serial.print(F(" motion=")); Serial.print(motion ? 1 : 0);
  Serial.print(F(" shadowStop=")); Serial.println(stop ? 1 : 0);
}

void printAllSensorGuardStatus()
{
  Serial.print(F("SENSOR GUARD thresholds warning/stop/fault="));
  Serial.print(SENSOR_GUARD_WARNING_MS); Serial.print('/');
  Serial.print(SENSOR_GUARD_SHADOW_STOP_MS); Serial.print('/');
  Serial.print(SENSOR_GUARD_FAULT_MS); Serial.print(F("ms recoveryFrames="));
  Serial.print(SENSOR_GUARD_RECOVERY_FRAMES);
  Serial.println(F(" SHADOW ONLY; no motor command"));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) printSensorGuardStatusLine(i);
  printAxisGuardStatus("X", SENSOR_X1, SENSOR_X2);
  printAxisGuardStatus("Z", SENSOR_Z1, SENSOR_Z2);
}

void resetAllSensorGuardDiagnostics()
{
  const uint32_t now = millis();
  configureSensorGuards(now, true);
  Serial.println(F("SENSOR GUARD DIAGNOSTICS RESET"));
}

void setSensorGuardMotion(bool xAxis, bool moving)
{
  const uint32_t now = millis();
  if (xAxis) g_guardMotionX = moving; else g_guardMotionZ = moving;
  const uint8_t first = xAxis ? SENSOR_X1 : SENSOR_Z1;
  const uint8_t second = xAxis ? SENSOR_X2 : SENSOR_Z2;
  printSensorGuardEvent(first, g_sensorGuard[first].setMotionMarked(moving, now), now);
  printSensorGuardEvent(second, g_sensorGuard[second].setMotionMarked(moving, now), now);
  Serial.print(F("SENSOR GUARD motion axis=")); Serial.print(xAxis ? F("X") : F("Z"));
  Serial.print(F(" state=")); Serial.println(moving ? F("ON") : F("OFF"));
  printAxisGuardStatus(xAxis ? "X" : "Z", first, second);
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

// HE200 commissioning builds must always boot with the protocol confirmed on
// the real drives: 9600 8-N-1, addresses 1..4.  This is applied only in RAM
// and is deliberately not written to EEPROM automatically.  It prevents an
// old NE200/8-E-1 record from causing confusing timeouts after every reset.
void applyHe200CommissioningProfile(SettingsV6 &cfg)
{
  cfg.vfd.baudCode = VFD_BAUD_9600;
  cfg.vfd.parity = VFD_PARITY_NONE;
  cfg.vfd.stopBits = 1;
  cfg.vfd.retries = 1;
  cfg.vfd.responseTimeoutMs = 250;
  cfg.vfd.interRequestMs = 20;
  cfg.vfd.address[DRIVE_H1] = 1;
  cfg.vfd.address[DRIVE_H2] = 2;
  cfg.vfd.address[DRIVE_V1] = 3;
  cfg.vfd.address[DRIVE_V2] = 4;
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
#if V6_DESKTOP_SIMULATION_ENABLED
  for(uint8_t i=0;i<4;i++){g_sensor[i].hwOk=true;g_sensor[i].valid=true;g_sensor[i].rawMm=g_simPlant.raw(i);g_sensor[i].valueMm=g_storage.applyCalibration(g_settings,(SensorIndex)i,g_sensor[i].rawMm);g_sensor[i].lastValidMs=millis();}
  Serial.println(F("SIM: virtual 5Hz sensors active; SC16IS752/SEN0366 not initialized"));
  return;
#endif
  Serial.println(F("=== SC16 SELF TEST FAST ==="));
  configureSensorGuards(millis(), true);

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
    g_sensor[i].hw->begin(now, FAST_SENSOR_PERIOD_MS, phase, FAST_LASER_DEFAULT_FREQ_HZ);
    Serial.print(g_sensor[i].label);
    Serial.println(F(" fast init: OK"));
  }
}

bool laserServiceChangeAllowed()
{
  if (g_auto.running() || g_systemMode == SystemModeV6::AUTO ||
      g_systemMode == SystemModeV6::HOME)
  {
    Serial.println(F("LASER mode change rejected while AUTO/HOME is active"));
    return false;
  }
  return true;
}

bool startAlternatingX(uint8_t targetHzPerSensor,
                       FastLaserResolution resolution)
{
  if (!laserServiceChangeAllowed()) return false;
  if (targetHzPerSensor != 1 && targetHzPerSensor != 2 &&
      targetHzPerSensor != 5 && targetHzPerSensor != 10)
  {
    Serial.println(F("LASER ALT rejected: rate must be 1, 2, 5 or 10 Hz per sensor"));
    return false;
  }
  if (!g_sensor[SENSOR_X1].hwOk || !g_sensor[SENSOR_X2].hwOk)
  {
    Serial.println(F("LASER ALT rejected: X1 and X2 SC16 channels must be available"));
    return false;
  }

  FastLaserSensor &x1Sensor = *g_sensor[SENSOR_X1].hw;
  FastLaserSensor &x2Sensor = *g_sensor[SENSOR_X2].hw;
  if (x1Sensor.configurationBusy() || x2Sensor.configurationBusy())
  {
    Serial.println(F("LASER ALT rejected: X1/X2 configuration busy"));
    return false;
  }

  const uint32_t now = millis();
  const bool x1 = x1Sensor.requestConfigure(
      now, 5, resolution, FastLaserAcquisitionMode::SingleShot);
  const bool x2 = x2Sensor.requestConfigure(
      now, 5, resolution, FastLaserAcquisitionMode::SingleShot);
  if (!x1 || !x2)
  {
    Serial.println(F("LASER ALT rejected: X1/X2 configuration busy"));
    return false;
  }

  g_laserAlternatingX = LaserAlternatingXStateV6{};
  g_laserAlternatingX.mode = LaserServiceModeV6::AlternatingX;
  g_laserAlternatingX.preparing = true;
  g_laserAlternatingX.targetHzPerSensor = targetHzPerSensor;
  g_laserAlternatingX.resolutionCode = (uint8_t)resolution;
  g_laserAlternatingX.nextSensor = SENSOR_X1;
  g_laserAlternatingX.activeSensor = -1;
  g_laserAlternatingX.requestSpacingMs =
      (uint16_t)(1000U / ((uint16_t)targetHzPerSensor * 2U));
  if (g_laserAlternatingX.requestSpacingMs < 20)
    g_laserAlternatingX.requestSpacingMs = 20;
  g_laserAlternatingX.nextRequestMs = now;

  Serial.print(F("LASER ALT PREPARE X1/X2 target="));
  Serial.print(targetHzPerSensor);
  Serial.print(F("Hz/sensor resolution="));
  Serial.print(resolution == FastLaserResolution::TenthMm ? F("0.1mm") : F("1mm"));
  Serial.println(F(" response-driven sequence"));
  return true;
}

bool stopAlternatingXAndRestore(uint8_t frequencyHz,
                                FastLaserResolution resolution)
{
  if (!laserServiceChangeAllowed()) return false;
  if (frequencyHz != 5 && frequencyHz != 10 && frequencyHz != 20)
  {
    Serial.println(F("LASER CONT rejected: frequency must be 5, 10 or 20 Hz"));
    return false;
  }

  g_laserAlternatingX.mode = LaserServiceModeV6::Continuous;
  g_laserAlternatingX.preparing = false;
  g_laserAlternatingX.activeSensor = -1;

  const uint32_t now = millis();
  const bool x1 = g_sensor[SENSOR_X1].hwOk &&
      g_sensor[SENSOR_X1].hw->requestConfigure(
          now, frequencyHz, resolution, FastLaserAcquisitionMode::Continuous);
  const bool x2 = g_sensor[SENSOR_X2].hwOk &&
      g_sensor[SENSOR_X2].hw->requestConfigure(
          now, frequencyHz, resolution, FastLaserAcquisitionMode::Continuous);

  Serial.print(F("LASER CONT RESTORE X1/X2 freq="));
  Serial.print(frequencyHz);
  Serial.print(F(" resolution="));
  Serial.print(resolution == FastLaserResolution::TenthMm ? F("0.1mm") : F("1mm"));
  Serial.println((x1 && x2) ? F(" REQUESTED") : F(" PARTIAL/BUSY"));
  return x1 && x2;
}

void serviceAlternatingX(uint32_t now)
{
  if (g_laserAlternatingX.mode != LaserServiceModeV6::AlternatingX) return;

  FastLaserSensor &x1 = *g_sensor[SENSOR_X1].hw;
  FastLaserSensor &x2 = *g_sensor[SENSOR_X2].hw;

  if (g_laserAlternatingX.preparing)
  {
    if (x1.configurationBusy() || x2.configurationBusy()) return;
    g_laserAlternatingX.preparing = false;
    g_laserAlternatingX.activeSensor = -1;
    g_laserAlternatingX.nextSensor = SENSOR_X1;
    g_laserAlternatingX.nextRequestMs = now;
    Serial.print(F("LASER ALT RUNNING X1/X2 target="));
    Serial.print(g_laserAlternatingX.targetHzPerSensor);
    Serial.print(F("Hz/sensor resolutionCode="));
    Serial.println(g_laserAlternatingX.resolutionCode);
  }

  if (g_laserAlternatingX.activeSensor >= 0)
  {
    FastLaserSensor &active = *g_sensor[(uint8_t)g_laserAlternatingX.activeSensor].hw;
    if (active.singleRequestPending()) return;

    g_laserAlternatingX.nextSensor =
        ((uint8_t)g_laserAlternatingX.activeSensor == SENSOR_X1)
            ? SENSOR_X2 : SENSOR_X1;
    g_laserAlternatingX.activeSensor = -1;

    const uint32_t scheduled = g_laserAlternatingX.lastRequestMs +
                               g_laserAlternatingX.requestSpacingMs;
    g_laserAlternatingX.nextRequestMs =
        ((int32_t)(now - scheduled) >= 0) ? now : scheduled;
  }

  if ((int32_t)(now - g_laserAlternatingX.nextRequestMs) < 0) return;

  const uint8_t index = g_laserAlternatingX.nextSensor;
  if (g_sensor[index].hw->requestSingleMeasurement(now))
  {
    g_laserAlternatingX.activeSensor = (int8_t)index;
    g_laserAlternatingX.lastRequestMs = now;
  }
  else
  {
    g_laserAlternatingX.nextRequestMs = now + 20;
  }
}

void serviceSensorsFast()
{
  const uint32_t now = millis();
#if V6_DESKTOP_SIMULATION_ENABLED
  g_simPlant.tick(now);
  if(g_simPlant.sampleDue(now))for(uint8_t i=0;i<4;i++){
    auto& sensor=g_sensor[i];sensor.hwOk=true;sensor.valid=!(g_simPlant.sensorLostMask&(1u<<i));
    if(sensor.valid){sensor.rawMm=g_simPlant.raw(i);sensor.valueMm=g_storage.applyCalibration(g_settings,(SensorIndex)i,sensor.rawMm);sensor.lastValidMs=now;g_simDirections.samples[i].add(sensor.rawMm,now);}
    else g_simDirections.samples[i].add(-1,now);
  }
  return;
#endif

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    SensorRuntimeFast &s = g_sensor[i];
    SensorGuardEventV6 guardEvent;

    if (!s.hwOk)
    {
      guardEvent = g_sensorGuard[i].service(now);
      printSensorGuardEvent(i, guardEvent, now);
      continue;
    }

    s.hw->service(now);

    if (s.hw->consumeUpdated())
    {
      s.rawMm = s.hw->rawMm();
      s.valueMm = g_storage.applyCalibration(g_settings, s.idx, s.rawMm);
      s.valid = true;
      s.lastValidMs = s.hw->lastUpdateMs();
      guardEvent = g_sensorGuard[i].observe(s.lastValidMs, s.valueMm);
      printSensorGuardEvent(i, guardEvent, now);
      if (!(g_auto.running() && g_auto.simulation()))
        g_dwin.writeU16(s.vp, displayValue(s.valueMm));
    }
    else
    {
      guardEvent = g_sensorGuard[i].service(now);
      printSensorGuardEvent(i, guardEvent, now);
    }

    // Do not clear s.valid on stale. It means "has last value" here.
    // Step9H reports freshness through both VP_ERROR and the shadow guard.
  }

  serviceAlternatingX(now);
}


void printLaserStatusLine(uint8_t i)
{
  if (i >= SENSOR_COUNT) return;
  const uint32_t now = millis();
  const SensorRuntimeFast &s = g_sensor[i];
  const FastLaserDiagnostics &d = s.hw->diagnostics();

  Serial.print(F("@LASER name=")); Serial.print(s.label);
  Serial.print(F(" hw=")); Serial.print(s.hwOk ? 1 : 0);
  Serial.print(F(" valid=")); Serial.print(s.valid ? 1 : 0);
  Serial.print(F(" mm=")); Serial.print(s.valueMm);
  Serial.print(F(" age="));
  if (s.lastValidMs) Serial.print(now - s.lastValidMs); else Serial.print(F("NA"));
  Serial.print(F(" byteAge="));
  if (d.lastByteMs) Serial.print(now - d.lastByteMs); else Serial.print(F("NA"));
  Serial.print(F(" freq=")); Serial.print(d.configuredFrequencyHz);
  Serial.print(F(" res=")); Serial.print(d.configuredResolutionCode);
  Serial.print(F(" mode=")); Serial.print(d.acquisitionMode == (uint8_t)FastLaserAcquisitionMode::SingleShot ? F("single") : F("cont"));
  Serial.print(F(" cfg=")); Serial.print(d.configuring ? 1 : 0);
  Serial.print(F(" ack=0x")); Serial.print(d.ackMask, HEX);
  Serial.print(F(" missing=0x")); Serial.print(d.missingAckMask, HEX);
  Serial.print(F(" nack=0x")); Serial.print(d.nackMask, HEX);
  Serial.print(F(" rate10=")); Serial.print(d.frameRateX10);
  Serial.print(F(" good=")); Serial.print(d.goodFrames);
  Serial.print(F(" stream=")); Serial.print(d.streamFrames);
  Serial.print(F(" sensorErr=")); Serial.print(d.sensorErrorFrames);
  Serial.print(F(" errCode=")); Serial.print(d.lastSensorError);
  Serial.print(F(" err15=")); Serial.print(d.errorCode15Frames);
  Serial.print(F(" err16=")); Serial.print(d.errorCode16Frames);
  Serial.print(F(" errOther=")); Serial.print(d.otherSensorErrorFrames);
  Serial.print(F(" streak=")); Serial.print(d.currentErrorStreak);
  Serial.print(F(" maxStreak=")); Serial.print(d.maxErrorStreak);
  Serial.print(F(" maxAge=")); Serial.print(d.maxAgeMs);
  Serial.print(F(" singleReq=")); Serial.print(d.singleRequests);
  Serial.print(F(" singleRsp=")); Serial.print(d.singleResponses);
  Serial.print(F(" singleTO=")); Serial.print(d.singleTimeouts);
  Serial.print(F(" singlePending=")); Serial.print(d.singlePending ? 1 : 0);
  Serial.print(F(" crc=")); Serial.print(d.checksumErrors);
  Serial.print(F(" malformed=")); Serial.print(d.malformedFrames);
  Serial.print(F(" range=")); Serial.print(d.rangeRejects);
  Serial.print(F(" discard=")); Serial.print(d.discardedBytes);
  Serial.print(F(" swOv=")); Serial.print(d.softwareOverruns);
  Serial.print(F(" uartOE=")); Serial.print(d.uartOverrunErrors);
  Serial.print(F(" uartPE=")); Serial.print(d.uartParityErrors);
  Serial.print(F(" uartFE=")); Serial.print(d.uartFramingErrors);
  Serial.print(F(" uartBI=")); Serial.print(d.uartBreakErrors);
  Serial.print(F(" uartFIFO=")); Serial.println(d.uartFifoErrors);
}

void printAllLaserStatus()
{
  Serial.print(F("LASER ACQUISITION mode="));
  Serial.print(g_laserAlternatingX.mode == LaserServiceModeV6::AlternatingX ? F("alternating-x") : F("continuous"));
  if (g_laserAlternatingX.mode == LaserServiceModeV6::AlternatingX) {
    Serial.print(F(" targetHz=")); Serial.print(g_laserAlternatingX.targetHzPerSensor);
    Serial.print(F(" resolutionCode=")); Serial.print(g_laserAlternatingX.resolutionCode);
    Serial.print(F(" preparing=")); Serial.print(g_laserAlternatingX.preparing ? 1 : 0);
    Serial.print(F(" active="));
    if (g_laserAlternatingX.activeSensor >= 0) Serial.print(g_sensor[(uint8_t)g_laserAlternatingX.activeSensor].label);
    else Serial.print(F("none"));
  }
  Serial.println();
  Serial.println(F("LASER STATUS BEGIN ack bits: 01 shutdown,02 laser,04 range,08 resolution,10 frequency,20 data-seen"));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) printLaserStatusLine(i);
  Serial.println(F("LASER STATUS END"));
}

void resetAllLaserDiagnostics()
{
  const uint32_t now = millis();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    g_sensor[i].hw->resetDiagnostics(now);
  }
  resetAllSensorGuardDiagnostics();
  Serial.println(F("LASER DIAGNOSTICS RESET"));
  printAllLaserStatus();
  printAllSensorGuardStatus();
}

int8_t laserIndexFromName(const char *name)
{
  if (!name) return -2;
  if (!strcmp(name, "all")) return -1;
  if (!strcmp(name, "x") || !strcmp(name, "xpair")) return -3;
  if (!strcmp(name, "z") || !strcmp(name, "zpair")) return -4;
  if (!strcmp(name, "x1")) return SENSOR_X1;
  if (!strcmp(name, "x2")) return SENSOR_X2;
  if (!strcmp(name, "z1")) return SENSOR_Z1;
  if (!strcmp(name, "z2")) return SENSOR_Z2;
  return -2;
}

bool configureLaserSensorsAdvanced(int8_t index, uint8_t frequencyHz,
                                   FastLaserResolution resolution,
                                   FastLaserAcquisitionMode mode)
{
  if (frequencyHz != 5 && frequencyHz != 10 && frequencyHz != 20) {
    Serial.println(F("LASER CONFIG rejected: frequency must be 5, 10 or 20 Hz"));
    return false;
  }
  if (!laserServiceChangeAllowed()) return false;

  if (mode == FastLaserAcquisitionMode::Continuous &&
      (index < 0 || index == SENSOR_X1 || index == SENSOR_X2)) {
    g_laserAlternatingX.mode = LaserServiceModeV6::Continuous;
    g_laserAlternatingX.preparing = false;
    g_laserAlternatingX.activeSensor = -1;
  }

  const uint32_t now = millis();
  bool any = false;
  bool allRequested = true;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (index == -3 && i != SENSOR_X1 && i != SENSOR_X2) continue;
    if (index == -4 && i != SENSOR_Z1 && i != SENSOR_Z2) continue;
    if (index >= 0 && i != (uint8_t)index) continue;
    if (!g_sensor[i].hwOk) {
      Serial.print(F("LASER CONFIG skipped ")); Serial.print(g_sensor[i].label);
      Serial.println(F(": SC16 channel unavailable/disabled"));
      allRequested = false;
      continue;
    }
    any = true;
    const bool requested = g_sensor[i].hw->requestConfigure(
        now, frequencyHz, resolution, mode);
    Serial.print(F("LASER CONFIG ")); Serial.print(g_sensor[i].label);
    Serial.print(F(" freq=")); Serial.print(frequencyHz);
    Serial.print(F(" resolution="));
    Serial.print(resolution == FastLaserResolution::TenthMm ? F("0.1mm") : F("1mm"));
    Serial.print(F(" mode="));
    Serial.print(mode == FastLaserAcquisitionMode::SingleShot ? F("single") : F("continuous"));
    Serial.println(requested ? F(" REQUESTED") : F(" BUSY/REJECTED"));
    if (!requested) allRequested = false;
  }
  if (!any) {
    Serial.println(F("LASER CONFIG rejected: no matching active channel"));
    return false;
  }
  return allRequested;
}

bool configureLaserSensors(int8_t index, uint8_t frequencyHz)
{
  return configureLaserSensorsAdvanced(index, frequencyHz,
                                       FastLaserResolution::Mm1,
                                       FastLaserAcquisitionMode::Continuous);
}

void printFieldReport()
{
  Serial.println(F("========== FIELD REPORT BEGIN =========="));
  handleCommand(CMD_DIAG_SNAPSHOT);
  printAllLaserStatus();
  printAllSensorGuardStatus();
  if (HE200_COMMISSIONING) {
    Serial.println(F("FIELD REPORT: HE200 READ-ONLY test queued"));
    handleCommand(CMD_VFD_TEST_ALL);
  } else {
    Serial.println(F("FIELD REPORT: HE200 test skipped (not commissioning build)"));
  }
  Serial.println(F("========== FIELD REPORT REQUESTED =========="));
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
  if (slot >= AUTO_PROGRAM_SLOTS_V6 || g_auto.running()) return false;
  // No concurrent execution: load directly into the idle program instead of
  // reserving another 336-byte AVR program on the command stack.
  if (!g_programStorage.load(slot, g_program)) {
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

  Serial.print(F("@PROGRAM slot=")); Serial.print(g_programSlot + 1);
  Serial.print(F(" selected=")); Serial.print(g_programSelectedSlot + 1);
  Serial.print(F(" dirty=")); Serial.print(g_programDirty ? 1 : 0);
  Serial.print(F(" ready=")); Serial.print(g_programStorage.readyForAuto(g_program) ? 1 : 0);
  Serial.print(F(" zones=")); Serial.print(g_program.zoneCount);
  Serial.print(F(" home1=")); Serial.print(g_program.homeX[0]);
  Serial.print(F(" home2=")); Serial.print(g_program.homeX[1]);
  Serial.print(F(" travel1=")); Serial.print(g_program.travelZ[0]);
  Serial.print(F(" travel2=")); Serial.print(g_program.travelZ[1]);
  Serial.print(F(" drip=")); Serial.print(g_program.dripWaitS);
  Serial.print(F(" tiltPct=")); Serial.print(g_program.tiltPercent);
  Serial.print(F(" lowSide=")); Serial.print(g_program.lowSide);
  Serial.print(F(" dry=")); Serial.print(g_program.dryingEnabled);
  Serial.print(F(" dryTime=")); Serial.print(g_program.dryingTimeS);
  Serial.print(F(" staging=")); Serial.println(g_program.stagingZone + 1);
  for (uint8_t zi = 0; zi < g_program.zoneCount; ++zi) {
    const AutoZoneV6& mz = g_program.zones[zi];
    Serial.print(F("@ZONE n=")); Serial.print(zi + 1);
    Serial.print(F(" en=")); Serial.print(mz.enabled);
    Serial.print(F(" valid=0x")); Serial.print(mz.validMask, HEX);
    Serial.print(F(" x1=")); Serial.print(mz.xMm[0]);
    Serial.print(F(" x2=")); Serial.print(mz.xMm[1]);
    Serial.print(F(" z1=")); Serial.print(mz.zMm[0]);
    Serial.print(F(" z2=")); Serial.print(mz.zMm[1]);
    Serial.print(F(" dip=")); Serial.print(mz.dipTimeS);
    Serial.print(F(" wait=")); Serial.print(mz.stepWaitS);
    Serial.print(F(" tilt=")); Serial.print(mz.tiltStepMm);
    Serial.print(F(" hpct=")); Serial.print(mz.movePercent);
    Serial.print(F(" vpct=")); Serial.println(mz.verticalPercent);
  }
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
  printAllSensorGuardStatus();
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
  if (!DWIN_MOTION_ENABLED && cmd != CMD_JOG_STOP) {
    const uint16_t candidate = motorStateForJogCommand(cmd);
    if (candidate != MOTOR_STATE_IDLE) {
      Serial.println(F("DWIN JOG BLOCKED in Step9I FIELD SERVICE; use browser service cockpit"));
      return true;
    }
  }
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
  if (!DWIN_MOTION_ENABLED && value != 0) {
    Serial.println(F("DWIN HOLD JOG BLOCKED in Step9I FIELD SERVICE"));
    return true;
  }
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
    if (HE200_FIELD_SERVICE) g_he200Service.stopAll(F("operator STOP"));
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

void printStep9IServiceHelp();
void printStep9IServiceInfo();

void printBenchConsoleHelp()
{
  Serial.println(F("USB console:"));
  printStep9IServiceHelp();
  Serial.println(F("  help | clear | diag | estop on/off | limits on/off | save"));
  Serial.println(F("  log quiet | log normal | log verbose"));
  Serial.println(F("  test h1/h2/v1/v2/all | he200 comm | report"));
  Serial.println(F("  laser profile [all/x/z]   (field profile: continuous 5Hz, 1mm, 10m)"));
  Serial.println(F("  laser status | laser reset | laser reinit"));
  Serial.println(F("  guard status | guard reset | guard on/off"));
  Serial.println(F("  guard motion x/z on/off   (SHADOW stop evaluation only)"));
  Serial.println(F("  laser config all/x/z/x1/x2/z1/z2 5/10/20   (experimental)"));
  Serial.println(F("  laser continuous all/x/z/x1/x2/z1/z2 5/10/20 1/01   (experimental)"));
  Serial.println(F("  laser alternate start 1/2/5/10 1/01 | laser alternate stop"));
  Serial.println(F("  laser single x1/x2   (manual request while in single mode)"));
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

bool parseLaserResolutionToken(const char *token,
                               FastLaserResolution &resolution)
{
  if (!token) return false;
  if (!strcmp(token, "1") || !strcmp(token, "1mm")) {
    resolution = FastLaserResolution::Mm1;
    return true;
  }
  if (!strcmp(token, "01") || !strcmp(token, "0.1") ||
      !strcmp(token, "0.1mm") || !strcmp(token, "2")) {
    resolution = FastLaserResolution::TenthMm;
    return true;
  }
  return false;
}

int8_t serviceDriveIndex(const char* token)
{
  if (!token) return -1;
  if (!strcmp(token, "h1")) return DRIVE_H1;
  if (!strcmp(token, "h2")) return DRIVE_H2;
  if (!strcmp(token, "v1")) return DRIVE_V1;
  if (!strcmp(token, "v2")) return DRIVE_V2;
  return -1;
}

uint8_t serviceDriveMask(const char* token)
{
  const int8_t drive = serviceDriveIndex(token);
  if (drive >= 0) return (uint8_t)(1u << drive);
  if (!strcmp(token, "h") || !strcmp(token, "x")) return He200ServiceV6::MASK_H;
  if (!strcmp(token, "v") || !strcmp(token, "z")) return He200ServiceV6::MASK_V;
  return 0;
}

bool serviceDirectionPositive(const char* token, bool& positive)
{
  if (!token) return false;
  if (!strcmp(token, "pos")) {
    positive = true; return true;
  }
  if (!strcmp(token, "neg")) {
    positive = false; return true;
  }
  return false;
}

bool runServicePreflight()
{
  if (!HE200_FIELD_SERVICE) {
    g_he200Service.setPreflight(false, F("FIELD_SERVICE_BUILD_REQUIRED"));
    return false;
  }
  if (g_safety.estopActive() || g_safety.estopLatched()) {
    g_he200Service.setPreflight(false, F("ESTOP"));
    return false;
  }
  if (g_safety.limitsEnabled() && g_safety.limitMask() != 0) {
    g_he200Service.setPreflight(false, F("LIMIT_INPUT_ACTIVE"));
    return false;
  }
  const uint32_t now = millis();
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    const DriveTelemetry& t = g_motor.vfdTelemetry(i);
    if (!t.connected || t.lastOkMs == 0 || (uint32_t)(now - t.lastOkMs) > 5000) {
      g_he200Service.setPreflight(false, F("DRIVE_STATUS_NOT_FRESH")); return false;
    }
    if (t.faultCode != 0) { g_he200Service.setPreflight(false, F("DRIVE_FAULT")); return false; }
    if (t.runningFreq001Hz > 20 || t.statusWord != 0) {
      g_he200Service.setPreflight(false, F("DRIVE_NOT_STOPPED")); return false;
    }
  }
  g_he200Service.setPreflight(true, F("FOUR_DRIVES_STOPPED_SAFETY_OK"));
  return true;
}

void printStep9IServiceInfo()
{
  Serial.print(F("@SERVICE_INFO fw=")); Serial.print(F(FW_VERSION_V6_BRINGUP));
  Serial.print(F(" build="));
  if (DESKTOP_SIMULATION_ENABLED) Serial.print(F("SIM"));
  else if (HE200_FIELD_SERVICE) Serial.print(F("FIELD"));
  else if (HE200_COMMISSIONING && VFD_RS485_ENABLED && !VFD_WRITE_COMMANDS_ENABLED) Serial.print(F("READONLY"));
  else Serial.print(F("BENCH"));
  Serial.print(F(" audit=1 diagnosticLock=")); Serial.print(HE200_DIAGNOSTIC_LOCK ? 1 : 0);
  Serial.print(F(" native=")); Serial.print(HE200_NATIVE_PROTOCOL ? 1 : 0);
  Serial.print(F(" auto=")); Serial.print(AUTO_PHYSICAL_ENABLED ? 1 : 0);
  Serial.print(F(" dwinMotion=")); Serial.print(DWIN_MOTION_ENABLED ? 1 : 0);
  Serial.print(F(" desktopSim="));Serial.print(DESKTOP_SIMULATION_ENABLED);
  Serial.print(F(" physicalTx="));Serial.print(VFD_RS485_ENABLED&&VFD_WRITE_COMMANDS_ENABLED);
  Serial.print(F(" waveshare=")); Serial.println(RS485_AUTO_DIRECTION ? 1 : 0);
}

void printStep9IServiceHelp()
{
  Serial.println(F("Step9I service cockpit commands:"));
  Serial.println(F("  service info"));
  Serial.println(F("  service preflight | service gate status | service gate reset | service stop  (all 4 drives)"));
  Serial.println(F("  he200 audit all | he200 audit cancel (FC03 only, all four drives)"));
  Serial.println(F("  he200 probe h1/h2/v1/v2/all (BLOCKED in diagnostic release)"));
  Serial.println(F("  service pulse h1/h2/v1/v2/h/z pos/neg [pct 5..20] [ms 500..3000]"));
  Serial.println(F("  service confirm h1/h2/v1/v2 pos/neg"));
  Serial.println(F("  assist x/z pos/neg [basePct] | assist stop  (active pair) | assist clear"));
}

#include "web_control_v6.inc"

void serviceUsbConsole()
{
  static char line[160];
  static uint8_t len = 0;
  static bool overflow=false;

  while (Serial.available())
  {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n')
    {
      if ((size_t)len + 1U < sizeof(line)) line[len++] = c;
      else overflow=true;
      continue;
    }

    line[len] = '\0';
    len = 0;
    if(overflow){overflow=false;Serial.println(F("@WEB_ACK ok=0 reason=LINE_TOO_LONG"));continue;}
    if (line[0] == '\0') continue;

    if(webCommand(line))continue;
    if(WEB_CONTROL_ENABLED && strcmp(line,"service info") && strcmp(line,"prog show") &&
       strcmp(line,"laser status") && strcmp(line,"service gate status") &&
       strcmp(line,"he200 audit all") && strcmp(line,"he200 audit cancel") &&
       strcmp(line,"log quiet") && strcmp(line,"log normal") && strcmp(line,"log verbose")) {
      Serial.println(F("@WEB_ACK ok=0 reason=USE_WEB_PROTOCOL"));continue;
    }

    if (g_motor.auditActive() && strcmp(line,"he200 audit cancel") &&
        strcmp(line,"service info") && strcmp(line,"service gate status") &&
        strcmp(line,"service stop")) {
      Serial.println(F("@AUDIT state=BUSY reason=READ_SCAN_ACTIVE"));
      continue;
    }
    if (!strcmp(line,"he200 audit all")) {
      if (g_he200Service.pulseActive() || g_he200Service.assistActive() || !g_motor.startHe200Audit())
        Serial.println(F("@AUDIT state=REJECTED reason=BUSY_OR_BUILD_OR_ADDRESS"));
    }
    else if (!strcmp(line,"he200 audit cancel")) g_motor.cancelHe200Audit();
    else if (!strcmp(line, "help")) printBenchConsoleHelp();
    else if (!strcmp(line, "service info")) printStep9IServiceInfo();
    else if (!strcmp(line, "service gate status")) g_he200Service.printGateStatus();
    else if (!strcmp(line, "service gate reset")) g_he200Service.resetGates();
    else if (!strcmp(line, "service preflight")) (void)runServicePreflight();
    else if (!strcmp(line, "service stop")) {
      g_motor.cancelHe200Audit();
      g_he200Service.stopAll(F("browser global STOP"));
      g_systemMode = SystemModeV6::STOP;
      writeMotorStateToDwin();
    }
    else if (!strcmp(line, "assist stop")) {
      g_he200Service.stopAssist(F("browser assist STOP"));
      writeMotorStateToDwin();
    }
    else if (!strcmp(line, "assist clear")) g_he200Service.clearAssistFault();
    else if (!strncmp(line, "he200 probe ", 12)) {
      if (HE200_DIAGNOSTIC_LOCK || !g_he200Service.preflightPassed()) {
        Serial.println(F("@GATE name=PROTOCOL result=FAIL reason=PREFLIGHT_REQUIRED"));
        continue;
      }
      char target[8] = {0};
      if (sscanf(line, "he200 probe %7s", target) == 1) {
        if (!strcmp(target, "all")) {
          for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
            He200ProtocolSnapshotV6 after{};
            const bool ok = g_motor.probeHe200Protocol(i, 500, after);
            g_he200Service.markProtocol(i, ok);
            delay(60);
          }
          g_he200Service.printGateStatus();
        } else {
          const int8_t di = serviceDriveIndex(target);
          if (di < 0) Serial.println(F("Use: he200 probe h1/h2/v1/v2/all"));
          else {
            He200ProtocolSnapshotV6 after{};
            const bool ok = g_motor.probeHe200Protocol((uint8_t)di, 500, after);
            g_he200Service.markProtocol((uint8_t)di, ok);
          }
        }
      }
    }
    else if (!strncmp(line, "service confirm ", 16)) {
      char target[8] = {0}, dir[8] = {0}; bool positive = true;
      if (sscanf(line, "service confirm %7s %7s", target, dir) == 2 && serviceDirectionPositive(dir, positive)) {
        const int8_t di = serviceDriveIndex(target);
        if (di >= 0) g_he200Service.confirmDirection((uint8_t)di, positive, true);
        else Serial.println(F("Confirm only individual h1/h2/v1/v2"));
      } else Serial.println(F("Use: service confirm h1/h2/v1/v2 pos/neg"));
    }
    else if (!strncmp(line,"service calibrate ",18)) {
      char target[8]={0}, dir[8]={0}, extra[8]={0};
      if (sscanf(line,"service calibrate %7s %7s %7s",target,dir,extra)==2 &&
          (!strcmp(dir,"fwd") || !strcmp(dir,"rev"))) {
        const int8_t di=serviceDriveIndex(target);
        if (di>=0) (void)g_he200Service.startPulse((uint8_t)(1U<<di),!strcmp(dir,"fwd"),10,1200,millis(),true);
      } else Serial.println(F("Use: service calibrate h1/h2/v1/v2 fwd/rev (locked)"));
    }
    else if (!strncmp(line, "service pulse ", 14)) {
      char target[8] = {0}, dir[8] = {0}, extra[8] = {0}; int pct = 10, ms = 1200; bool positive = true;
      const int fields = sscanf(line, "service pulse %7s %7s %d %d %7s", target, dir, &pct, &ms, extra);
      if (fields == 4 && pct>=5 && pct<=20 && ms>=500 && ms<=3000 && serviceDirectionPositive(dir, positive)) {
        const uint8_t mask = serviceDriveMask(target);
        if (!mask) Serial.println(F("Use target h1/h2/v1/v2/h/z"));
        else (void)g_he200Service.startPulse(mask, positive, (uint8_t)pct, (uint16_t)ms, millis());
      } else Serial.println(F("Use: service pulse TARGET pos/neg [pct] [ms]"));
    }
    else if (!strncmp(line, "assist ", 7)) {
      char axis[8] = {0}, dir[8] = {0}; int pct = 0; bool positive = true;
      if (sscanf(line, "assist %7s %7s %d", axis, dir, &pct) >= 2 && serviceDirectionPositive(dir, positive)) {
        He200ServiceV6::AssistAxis ax = He200ServiceV6::AssistAxis::X;
        bool axisOk = true;
        if (!strcmp(axis,"x") || !strcmp(axis,"h")) { ax = He200ServiceV6::AssistAxis::X; if (pct <= 0) pct = 40; }
        else if (!strcmp(axis,"z") || !strcmp(axis,"v")) { ax = He200ServiceV6::AssistAxis::Z; if (pct <= 0) pct = 68; }
        else axisOk = false;
        if (!axisOk) Serial.println(F("Use assist x/z ..."));
        else {
          uint32_t ages[4], lastSamples[4]; const uint32_t t = millis();
          for (uint8_t i=0;i<4;++i) { ages[i]=g_sensor[i].lastValidMs ? (uint32_t)(t-g_sensor[i].lastValidMs) : UINT32_MAX; lastSamples[i]=g_sensor[i].lastValidMs; }
          (void)g_he200Service.startAssist(ax, positive, (uint8_t)pct, t, ages, lastSamples);
        }
      } else Serial.println(F("Use: assist x/z pos/neg [basePct]"));
    }
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
    else if (!strcmp(line, "laser status")) { printAllLaserStatus(); printAllSensorGuardStatus(); }
    else if (!strcmp(line, "laser reset")) resetAllLaserDiagnostics();
    else if (!strcmp(line, "laser reinit")) reinitFastSensors();
    else if (!strcmp(line, "laser profile") || !strcmp(line, "laser profile all"))
      (void)configureLaserSensorsAdvanced(-1, FAST_LASER_DEFAULT_FREQ_HZ, FastLaserResolution::Mm1, FastLaserAcquisitionMode::Continuous);
    else if (!strcmp(line, "laser profile x"))
      (void)configureLaserSensorsAdvanced(-3, FAST_LASER_DEFAULT_FREQ_HZ, FastLaserResolution::Mm1, FastLaserAcquisitionMode::Continuous);
    else if (!strcmp(line, "laser profile z"))
      (void)configureLaserSensorsAdvanced(-4, FAST_LASER_DEFAULT_FREQ_HZ, FastLaserResolution::Mm1, FastLaserAcquisitionMode::Continuous);
    else if (!strcmp(line, "guard status")) printAllSensorGuardStatus();
    else if (!strcmp(line, "guard reset")) { resetAllSensorGuardDiagnostics(); printAllSensorGuardStatus(); }
    else if (!strcmp(line, "guard on")) { g_sensorGuardEnabled = true; configureSensorGuards(millis(), false); Serial.println(F("SENSOR GUARD ON (shadow only)")); }
    else if (!strcmp(line, "guard off")) { g_sensorGuardEnabled = false; configureSensorGuards(millis(), false); Serial.println(F("SENSOR GUARD OFF")); }
    else if (!strcmp(line, "guard motion x on")) setSensorGuardMotion(true, true);
    else if (!strcmp(line, "guard motion x off")) setSensorGuardMotion(true, false);
    else if (!strcmp(line, "guard motion z on")) setSensorGuardMotion(false, true);
    else if (!strcmp(line, "guard motion z off")) setSensorGuardMotion(false, false);
    else if (!strcmp(line, "report")) printFieldReport();
    else if (!strncmp(line, "laser continuous ", 17)) {
      char target[8] = {0};
      char resolutionToken[8] = {0};
      int frequency = 0;
      FastLaserResolution resolution = FastLaserResolution::Mm1;
      if (sscanf(line, "laser continuous %7s %d %7s", target, &frequency,
                 resolutionToken) == 3 &&
          parseLaserResolutionToken(resolutionToken, resolution)) {
        const int8_t index = laserIndexFromName(target);
        if (index == -2) {
          Serial.println(F("Use: laser continuous all/x/z/x1/x2/z1/z2 5/10/20 1/01"));
        } else {
          (void)configureLaserSensorsAdvanced(index, (uint8_t)frequency,
                                              resolution,
                                              FastLaserAcquisitionMode::Continuous);
        }
      } else {
        Serial.println(F("Use: laser continuous all/x/z/x1/x2/z1/z2 5/10/20 1/01"));
      }
    }
    else if (!strncmp(line, "laser alternate start ", 22)) {
      char resolutionToken[8] = {0};
      int rate = 0;
      FastLaserResolution resolution = FastLaserResolution::Mm1;
      if (sscanf(line, "laser alternate start %d %7s", &rate,
                 resolutionToken) == 2 &&
          parseLaserResolutionToken(resolutionToken, resolution)) {
        (void)startAlternatingX((uint8_t)rate, resolution);
      } else {
        Serial.println(F("Use: laser alternate start 1/2/5/10 1/01"));
      }
    }
    else if (!strcmp(line, "laser alternate stop")) {
      (void)stopAlternatingXAndRestore(FAST_LASER_DEFAULT_FREQ_HZ,
                                      FastLaserResolution::Mm1);
    }
    else if (!strncmp(line, "laser single ", 13)) {
      char target[8] = {0};
      if (sscanf(line, "laser single %7s", target) == 1) {
        const int8_t index = laserIndexFromName(target);
        if (index < 0 || (index != SENSOR_X1 && index != SENSOR_X2)) {
          Serial.println(F("Use: laser single x1/x2"));
        } else {
          const bool requested = g_sensor[(uint8_t)index].hw->requestSingleMeasurement(millis());
          Serial.print(F("LASER SINGLE ")); Serial.print(g_sensor[(uint8_t)index].label);
          Serial.println(requested ? F(" REQUESTED") : F(" BUSY/WRONG MODE"));
        }
      } else {
        Serial.println(F("Use: laser single x1/x2"));
      }
    }
    else if (!strncmp(line, "laser config ", 13)) {
      char target[8] = {0};
      int frequency = 0;
      if (sscanf(line, "laser config %7s %d", target, &frequency) == 2) {
        const int8_t index = laserIndexFromName(target);
        if (index == -2) Serial.println(F("Use: laser config all/x/z/x1/x2/z1/z2 5/10/20"));
        else (void)configureLaserSensors(index, (uint8_t)frequency);
      } else {
        Serial.println(F("Use: laser config all/x/z/x1/x2/z1/z2 5/10/20"));
      }
    }
    else if (!strcmp(line, "he200 comm")) {
      if (!HE200_COMMISSIONING) {
        Serial.println(F("he200 comm is intended for the HE200 commissioning build"));
      } else {
        applyHe200CommissioningProfile(g_settings);
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
  Serial.println(F("========== V6 SYSTEM STEP9I HE200 SERVICE COCKPIT START =========="));
  Serial.print(F("FW: ")); Serial.println(F(FW_VERSION_V6_BRINGUP));

  g_dwin.begin(DWIN_BAUD);
  g_safety.begin();

  if (!g_storage.load(g_settings))
  {
    g_storage.defaults(g_settings);
    if(DESKTOP_SIMULATION_ENABLED)for(auto& profile:g_settings.drive)profile={10,40,10,0,100,8};
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

  if (HE200_COMMISSIONING)
  {
    applyHe200CommissioningProfile(g_settings);
    Serial.println(F("HE200 commissioning profile enforced in RAM: 9600 8-N-1, addresses 1/2/3/4"));
    Serial.println(F("EEPROM is not modified automatically"));
  }

  g_editSettings = g_settings;
  g_settingsDirty = false;
  g_settingsUiState = SETTINGS_UI_IDLE;
  g_settingsUiError = SETTINGS_VALID;

  applySafetySettings(g_settings);
  if(!DESKTOP_SIMULATION_ENABLED)g_motor.begin(g_settings);
  g_auto.begin(g_motor, g_settings);
#if V6_DESKTOP_SIMULATION_ENABLED
  g_auto.externalSimulation(true);g_simPlant.begin(millis());webModelDirections();
#endif
  g_he200Service.begin(g_motor);

  // Programs live in a separate EEPROM area. Invalid/empty memory is never
  // auto-filled with runnable coordinates: safe uncalibrated defaults stay in RAM.
  g_programSlot = g_programStorage.loadActiveSlot();
  g_programSelectedSlot = g_programSlot;
  (void)loadProgramSlotV6(g_programSlot);

  if (!DESKTOP_SIMULATION_ENABLED&&(g_safety.estopActive() || g_safety.estopLatched()))
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
  Serial.println(F("Laser profile: SEN0366-compatible continuous 5Hz / 1mm / 10m"));
  if (WEB_CONTROL_ENABLED) Serial.println(F("Step9L web control: boots DISARMED; measured directions required; program starts by operator button"));
  else if (HE200_FIELD_SERVICE) Serial.println(F("Step9I service cockpit: native HE200 0x1000/0x2000/0x3000 writes runtime-gated; physical AUTO/HOME BLOCKED"));
  printStep9IServiceInfo();
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

  Serial.print(F("RS485 transceiver direction: "));
  Serial.println(RS485_AUTO_DIRECTION ? F("AUTO/Waveshare (Mega pin 6 unused)") : F("MANUAL/MAX485 (Mega pin 6 DE/RE)"));
  Serial.print(F("Physical AUTO/HOME interlock: "));
  Serial.println(AUTO_PHYSICAL_ENABLED ? F("ENABLED") : F("BLOCKED"));
  Serial.println(F("VFD settings: 0x0200 APPLY, 0x0201 SAVE, 0x0202 LOAD, 0x0203 DEFAULTS, 0x0204..0x0208 TEST"));
  printBenchConsoleHelp();
  printProgramSummary();
  printAllSensorGuardStatus();
  webBegin();

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
  webService(millis());
  const bool estopBlocked = DESKTOP_SIMULATION_ENABLED?webSafetyBlocked():(g_safety.estopActive() || g_safety.estopLatched());
  if (!DESKTOP_SIMULATION_ENABLED&&estopBlocked && (g_auto.running() || g_motor.isMotionActive() || g_systemMode != SystemModeV6::STOP))
  {
    if (g_auto.running()) g_auto.stop(g_safety.estopActive() ? F("E-STOP") : F("E-STOP latched"));
    g_motor.stopAll(g_safety.estopActive() ? F("E-STOP") : F("E-STOP latched"));
    g_systemMode = SystemModeV6::STOP;
    writeModeToDwin();
    writeMotorStateToDwin();
  }
  else if (!DESKTOP_SIMULATION_ENABLED&&g_systemMode == SystemModeV6::MANUAL &&
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
    const bool autoChanged = g_auto.service(webAutoClock(now), buildAutoSensors(), estopBlocked, DESKTOP_SIMULATION_ENABLED?0:g_safety.limitMask());
#if V6_DESKTOP_SIMULATION_ENABLED
    int16_t targets[4];for(uint8_t i=0;i<4;i++)targets[i]=g_auto.simulationTarget(i);
    (void)webOutputTargets(targets);
#endif
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
    if (DESKTOP_SIMULATION_ENABLED) webReleaseOutput();
    else if (wasSimulation) g_motor.autoStop(F("simulation terminal state"));
    else g_motor.onModeChanged(SystemModeV6::STOP);
    writeModeToDwin();
    writeMotorStateToDwin();
    writePersistentHeaderToDwin();
  }

  if(!DESKTOP_SIMULATION_ENABLED){
    uint32_t ages[4];
    uint32_t lastSamples[4];
    for (uint8_t i = 0; i < 4; ++i) {
      ages[i] = g_sensor[i].lastValidMs ? (uint32_t)(now - g_sensor[i].lastValidMs) : UINT32_MAX;
      lastSamples[i] = g_sensor[i].lastValidMs;
    }
    int32_t rawMm[4];
    for (uint8_t i=0;i<4;++i) rawMm[i]=g_sensor[i].valid ? g_sensor[i].rawMm : -1;
    g_he200Service.observeSensors(rawMm,lastSamples);
    const bool serviceSafetyBlocked = g_safety.estopActive() || g_safety.estopLatched() ||
        (g_safety.limitsEnabled() && g_safety.limitMask()!=0);
    g_he200Service.service(now, serviceSafetyBlocked, ages, lastSamples);
  }

  if (!DESKTOP_SIMULATION_ENABLED&&g_motor.service(now))
  {
    writeMotorStateToDwin();
  }

  uint16_t rxVp = 0;
  uint16_t rxValue = 0;
  uint8_t dwinFrames = 0;
  while (dwinFrames < 8 && g_dwin.pollWriteU16(rxVp, rxValue))
  {
    ++dwinFrames;
    if(WEB_CONTROL_ENABLED)continue; // Web owns mutation; DWIN remains a status mirror.
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
