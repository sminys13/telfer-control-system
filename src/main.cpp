#include <Arduino.h>
#include "config_v6_bringup.h"
#include "sc16is752.h"
#include "laser_sensor.h"
#include "dwin_link.h"
#include "settings_v6.h"

// =====================================================
// v6 sensor core: X1/X2/Z1/Z2 + zero/save/load/reset
// =====================================================

static Sc16Is752 g_sc16_1(PIN_SC16_1_CS, SC16_XTAL_HZ);
static Sc16Is752 g_sc16_2(PIN_SC16_2_CS, SC16_XTAL_HZ);

static LaserSensor g_laserX1(g_sc16_1, Sc16Is752::Channel::A);
static LaserSensor g_laserX2(g_sc16_1, Sc16Is752::Channel::B);
static LaserSensor g_laserZ1(g_sc16_2, Sc16Is752::Channel::A);
static LaserSensor g_laserZ2(g_sc16_2, Sc16Is752::Channel::B);

static DwinLink g_dwin(Serial2);
static SettingsStorageV6 g_storage;
static SettingsV6 g_settings;

struct SensorRuntime
{
  LaserSensor *hw;
  const char *label;
  SensorIndex idx;
  uint16_t vp;
  bool hwOk;
  bool valid;
  int32_t rawMm;
  int32_t valueMm;
  uint32_t lastValidMs;
};

static SensorRuntime g_sensor[SENSOR_COUNT] = {
    {&g_laserX1, "X1", SENSOR_X1, VP_X1, false, false, 0, 0, 0},
    {&g_laserX2, "X2", SENSOR_X2, VP_X2, false, false, 0, 0, 0},
    {&g_laserZ1, "Z1", SENSOR_Z1, VP_Z1, false, false, 0, 0, 0},
    {&g_laserZ2, "Z2", SENSOR_Z2, VP_Z2, false, false, 0, 0, 0},
};

static uint8_t g_nextSensorToPoll = 0;

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

void writeBootScreen()
{
  g_dwin.writeU16(VP_MODE, MODE_SENSOR_CORE);
  g_dwin.writeU16(VP_ERROR, ERROR_NONE);
  g_dwin.writeU16(VP_X1, 0);
  g_dwin.writeU16(VP_X2, 0);
  g_dwin.writeU16(VP_Z1, 0);
  g_dwin.writeU16(VP_Z2, 0);
  g_dwin.clearCommand(VP_CMD);
}

void updateErrorMask()
{
  uint16_t err = ERROR_NONE;
  const uint32_t now = millis();

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    const SensorRuntime &s = g_sensor[i];

    if (!g_settings.sensor[i].enabled)
      continue;

    const bool stale = s.valid && (now - s.lastValidMs > g_settings.staleTimeoutMs);
    if (!s.hwOk || !s.valid || stale)
    {
      err |= (uint16_t)(1 << i);
    }
  }

  g_dwin.writeU16(VP_ERROR, err);
}

void writeAllValuesToDwin()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    if (g_sensor[i].valid)
    {
      g_dwin.writeU16(g_sensor[i].vp, displayValue(g_sensor[i].valueMm));
    }
    else
    {
      g_dwin.writeU16(g_sensor[i].vp, 0);
    }
  }
  g_dwin.writeU16(VP_MODE, MODE_SENSOR_CORE);
  updateErrorMask();
}

void initSensors()
{
  Serial.println(F("=== SC16 SELF TEST ==="));

  g_sc16_1.begin();
  // Release second SC16 too before using the SPI bus.
  pinMode(PIN_SC16_2_CS, OUTPUT);
  digitalWrite(PIN_SC16_2_CS, HIGH);
  g_sc16_2.begin();

  printBool(F("SC16 #1 CH_A: "), g_sc16_1.selfTest(Sc16Is752::Channel::A));
  printBool(F("SC16 #1 CH_B: "), g_sc16_1.selfTest(Sc16Is752::Channel::B));
  printBool(F("SC16 #2 CH_A: "), g_sc16_2.selfTest(Sc16Is752::Channel::A));
  printBool(F("SC16 #2 CH_B: "), g_sc16_2.selfTest(Sc16Is752::Channel::B));

  Serial.println(F("=== LASER INIT ==="));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
  {
    if (!g_settings.sensor[i].enabled)
    {
      g_sensor[i].hwOk = false;
      continue;
    }

    g_sensor[i].hwOk = g_sensor[i].hw->begin();

    Serial.print(g_sensor[i].label);
    Serial.print(F(" init: "));
    Serial.println(g_sensor[i].hwOk ? F("OK") : F("FAIL"));
  }
}

void pollOneSensor(SensorRuntime &s)
{
  if (!g_settings.sensor[s.idx].enabled || !s.hwOk)
    return;

  int32_t raw = 0;
  const bool ok = s.hw->readFilteredMm(raw);

  if (!ok || raw <= 0)
    return;

  s.rawMm = raw;
  s.valueMm = g_storage.applyCalibration(g_settings, s.idx, s.rawMm);
  s.valid = true;
  s.lastValidMs = millis();

  g_dwin.writeU16(s.vp, displayValue(s.valueMm));
}

void pollSensorsRoundRobin()
{
  const uint8_t start = g_nextSensorToPoll;

  for (uint8_t tries = 0; tries < SENSOR_COUNT; ++tries)
  {
    const uint8_t i = g_nextSensorToPoll;
    g_nextSensorToPoll = (uint8_t)((g_nextSensorToPoll + 1) % SENSOR_COUNT);

    if (g_settings.sensor[i].enabled && g_sensor[i].hwOk)
    {
      pollOneSensor(g_sensor[i]);
      return;
    }
  }

  (void)start;
}

void zeroSensor(SensorIndex idx)
{
  if (idx >= SENSOR_COUNT)
    return;
  SensorRuntime &s = g_sensor[idx];

  if (!s.valid || s.rawMm <= 0)
  {
    Serial.println(F("ZERO ignored: sensor has no valid raw value"));
    return;
  }

  g_storage.zeroSensor(g_settings, idx, s.rawMm);

  s.valueMm = g_storage.applyCalibration(g_settings, idx, s.rawMm);
  g_dwin.writeU16(s.vp, displayValue(s.valueMm));

  Serial.print(F("ZERO "));
  Serial.print(s.label);
  Serial.print(F(" raw="));
  Serial.print(s.rawMm);
  Serial.print(F(" offset="));
  Serial.println(g_settings.sensor[idx].offsetMm);
}

void handleCommand(uint16_t cmd)
{
  if (cmd == CMD_NONE)
    return;

  Serial.print(F("DWIN CMD="));
  Serial.println(cmd);

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
    Serial.println(F("Settings saved"));
    break;

  case CMD_LOAD:
    if (!g_storage.load(g_settings))
    {
      g_storage.defaults(g_settings);
      Serial.println(F("Settings load failed, defaults restored in RAM"));
    }
    else
    {
      Serial.println(F("Settings loaded"));
    }
    writeAllValuesToDwin();
    break;

  case CMD_RESET_CAL:
    g_storage.resetOffsets(g_settings);
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

  default:
    Serial.println(F("Unknown command"));
    break;
  }

  g_dwin.clearCommand(VP_CMD);
}

void setup()
{
  Serial.begin(DBG_BAUD);
  delay(300);
  Serial.println();
  Serial.println(F("========== V6 SENSOR CORE START =========="));

  g_dwin.begin(DWIN_BAUD);
  writeBootScreen();

  if (!g_storage.load(g_settings))
  {
    g_storage.defaults(g_settings);
    Serial.println(F("Settings: defaults"));
  }
  else
  {
    Serial.println(F("Settings: loaded from EEPROM"));
  }

  initSensors();
  writeAllValuesToDwin();

  Serial.println(F("Commands: 21-24 ZERO, 30 SAVE, 31 LOAD, 32 RESET CAL"));
}

void loop()
{
  const uint32_t now = millis();

  static uint32_t lastPollMs = 0;
  if (now - lastPollMs >= g_settings.samplePeriodMs)
  {
    lastPollMs = now;
    pollSensorsRoundRobin();
  }

  uint16_t cmd = 0;
  if (g_dwin.pollCommand(VP_CMD, cmd))
  {
    handleCommand(cmd);
  }

  static uint32_t lastUiMs = 0;
  if (now - lastUiMs >= 250)
  {
    lastUiMs = now;
    writeAllValuesToDwin();
  }

  static uint32_t lastPrintMs = 0;
  if (now - lastPrintMs >= 1000)
  {
    lastPrintMs = now;

    Serial.print(F("X1="));
    Serial.print(g_sensor[SENSOR_X1].valueMm);
    Serial.print(F(" X2="));
    Serial.print(g_sensor[SENSOR_X2].valueMm);
    Serial.print(F(" Z1="));
    Serial.print(g_sensor[SENSOR_Z1].valueMm);
    Serial.print(F(" Z2="));
    Serial.print(g_sensor[SENSOR_Z2].valueMm);
    Serial.println();
  }
}
