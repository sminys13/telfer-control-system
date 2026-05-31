#include <Arduino.h>
#include "config_v6_bringup.h"
#include "sc16is752.h"
#include "laser_sensor.h"
#include "dwin_link.h"

static Sc16Is752 g_sc16(PIN_SC16_1_CS, SC16_XTAL_HZ);
static LaserSensor g_laserX1(g_sc16, Sc16Is752::Channel::A);
static DwinLink g_dwin(Serial2);

void setup() {
  Serial.begin(DBG_BAUD);
  delay(300);
  Serial.println(F("v6 step1 bring-up start"));

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

  const bool okLaser = g_laserX1.begin();
  Serial.print(F("Laser X1 init: "));
  Serial.println(okLaser ? F("OK") : F("FAIL"));
  if (!okLaser) {
    g_dwin.writeU16(VP_ERROR, ERROR_LASER1);
  }
}

void loop() {
  static uint32_t lastSampleMs = 0;
  static uint32_t lastUiMs = 0;
  static int32_t lastMm = 0;
  static bool lastValid = false;

  const uint32_t now = millis();

  if (now - lastSampleMs >= LASER_SAMPLE_PERIOD_MS) {
    lastSampleMs = now;

    int32_t mm = 0;
    const bool ok = g_laserX1.readFilteredMm(mm);
    lastValid = ok;
    if (ok) {
      lastMm = mm;
      Serial.print(F("X1 = "));
      Serial.print(mm);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F("X1 = FAIL"));
    }
  }

  if (now - lastUiMs >= 150) {
    lastUiMs = now;
    g_dwin.writeU16(VP_X1, lastValid ? (uint16_t)lastMm : 0);
    g_dwin.writeU16(VP_MODE, MODE_BRINGUP);
    g_dwin.writeU16(VP_ERROR, lastValid ? ERROR_NONE : ERROR_LASER1);
  }
}

