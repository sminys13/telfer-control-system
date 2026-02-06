\
/**
 * @file sensors.h
 * @brief Датчики:
 *   - 2 лазерных дальномера (UART 9600), измеряют горизонтальное положение каждого тельфера.
 *   - 2 HC-SR04 (5V), измеряют вертикальную "высоту" груза под каждым тельфером.
 *
 * Требования проекта:
 *  - Показания в миллиметрах.
 *  - Надёжность важнее "умных" фильтров → используем лёгкое скользящее среднее на 3 измерениях.
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "config.h"

struct SensorValue {
  int32_t mm;
  bool valid;
  uint32_t lastUpdateMs;
};

struct SensorsSnapshot {
  SensorValue laser[TELFER_COUNT];
  SensorValue us[TELFER_COUNT];
};

class Sensors {
public:
  void begin();
  void tick(uint32_t nowMs);

  const SensorsSnapshot& get() const { return _snap; }

  // Быстрые helpers
  bool lasersValid() const { return _snap.laser[0].valid && _snap.laser[1].valid; }
  bool usValid() const     { return _snap.us[0].valid && _snap.us[1].valid; }

private:
  SensorsSnapshot _snap{};

  // маленькие буферы для скользящего среднего
  int32_t _laserHist[2][3] = {{0}};
  uint8_t _laserHistN[2] = {0};

  uint32_t _laserReinitMs[2] = {0, 0};

  int32_t _usHist[2][3] = {{0}};
  uint8_t _usHistN[2] = {0};

  void initLaserPort(HardwareSerial& s);
  void initUltrasonicPins();

  bool readLaserFrame(HardwareSerial& s, int32_t& outMm);
  int32_t readUltrasonicMm(uint8_t trigPin, uint8_t echoPin);

  int32_t pushAvg3(int32_t hist[3], uint8_t& n, int32_t v);
};

