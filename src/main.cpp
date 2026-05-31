#include <Arduino.h>
#include "config_v6_bringup.h"
#include "sc16is752.h"
#include "laser_sensor.h"
#include "dwin_link.h"

static Sc16Is752 g_sc16(PIN_SC16_1_CS, SC16_XTAL_HZ);
static LaserSensor g_laserX1(g_sc16, Sc16Is752::Channel::A);
static LaserSensor g_laserX2(g_sc16, Sc16Is752::Channel::B);
static DwinLink g_dwin(Serial2);

static bool g_x1Enabled = false;
static bool g_x2Enabled = false;

static int32_t g_x1LastMm = 0;
static int32_t g_x2LastMm = 0;

static bool g_x1Valid = false;
static bool g_x2Valid = false;

void setup()
{
  Serial.begin(DBG_BAUD);
  delay(300);
  Serial.println(F("v6 step2 bring-up start"));

  g_dwin.begin(DWIN_BAUD);
  g_dwin.writeU16(VP_MODE, MODE_BRINGUP);
  g_dwin.writeU16(VP_ERROR, ERROR_NONE);
  g_dwin.writeU16(VP_X1, 0);
  g_dwin.writeU16(VP_X2, 0);
  g_dwin.writeU16(VP_Z1, 0);
  g_dwin.writeU16(VP_Z2, 0);

  g_sc16.begin();

  const bool selfA = g_sc16.selfTest(Sc16Is752::Channel::A);
  Serial.print(F("SC16 self A: "));
  Serial.println(selfA ? F("OK") : F("FAIL"));

  const bool selfB = g_sc16.selfTest(Sc16Is752::Channel::B);
  Serial.print(F("SC16 self B: "));
  Serial.println(selfB ? F("OK") : F("FAIL"));

  g_x1Enabled = g_laserX1.begin();
  Serial.print(F("Laser X1 init: "));
  Serial.println(g_x1Enabled ? F("OK") : F("FAIL"));

  g_x2Enabled = g_laserX2.begin();
  Serial.print(F("Laser X2 init: "));
  Serial.println(g_x2Enabled ? F("OK") : F("FAIL"));

  uint16_t err = ERROR_NONE;
  if (!g_x1Enabled)
    err = ERROR_LASER1;
  if (!g_x2Enabled)
    err = ERROR_LASER2;
  g_dwin.writeU16(VP_ERROR, err);
}

void loop()
{
  const uint32_t now = millis();

  static uint32_t lastX1Ms = 0;
  static uint32_t lastX2Ms = LASER_SAMPLE_PERIOD_MS / 2;
  static uint32_t lastUiMs = 0;

  if (g_x1Enabled && (now - lastX1Ms >= LASER_SAMPLE_PERIOD_MS))
  {
    lastX1Ms = now;

    int32_t mm = 0;
    const bool ok = g_laserX1.readFilteredMm(mm);
    g_x1Valid = ok;

    if (ok)
    {
      g_x1LastMm = mm;
      Serial.print(F("X1 = "));
      Serial.print(mm);
      Serial.println(F(" mm"));
    }
    else
    {
      Serial.println(F("X1 = FAIL"));
    }
  }

  if (g_x2Enabled && (now - lastX2Ms >= LASER_SAMPLE_PERIOD_MS))
  {
    lastX2Ms = now;

    int32_t mm = 0;
    const bool ok = g_laserX2.readFilteredMm(mm);
    g_x2Valid = ok;

    if (ok)
    {
      g_x2LastMm = mm;
      Serial.print(F("X2 = "));
      Serial.print(mm);
      Serial.println(F(" mm"));
    }
    else
    {
      Serial.println(F("X2 = FAIL"));
    }
  }

  if (now - lastUiMs >= 150)
  {
    lastUiMs = now;

    g_dwin.writeU16(VP_X1, g_x1Valid ? (uint16_t)g_x1LastMm : 0);
    g_dwin.writeU16(VP_X2, g_x2Valid ? (uint16_t)g_x2LastMm : 0);
    g_dwin.writeU16(VP_MODE, MODE_BRINGUP);

    uint16_t err = ERROR_NONE;
    if (!g_x1Enabled)
      err = ERROR_LASER1;
    else if (!g_x1Valid)
      err = ERROR_LASER1;

    if (!g_x2Enabled)
      err = ERROR_LASER2;
    else if (!g_x2Valid)
      err = ERROR_LASER2;

    g_dwin.writeU16(VP_ERROR, err);
  }
}