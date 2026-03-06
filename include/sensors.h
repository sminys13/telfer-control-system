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
  void applySettings(const GlobalSettings& settings);
  // Отправить конфигурационные команды (частота/диапазон/разрешение/нулевая точка/автостарт).
  // Команды не отправляются сами по себе (если только laser_apply_on_boot не включен).
  void applyLaserDeviceConfig(const GlobalSettings& settings);
  // Мягко перезапустить поток измерений (laser ON + continuous) без конфигурации 0x04.
  // Полезно для восстановления после неудачного APPLY без перезагрузки датчика.
  void restartLaserStreaming();
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
  uint32_t _laserLastRxMs[2] = {0, 0};

  uint16_t _laserTimeoutMs = 2000;
  bool     _laserReinitEnabled = true;
  uint8_t  _laserAddr[2] = {0x80, 0x80};
  int16_t  _laserOffsetMm[2] = {0,0};
  int16_t  _usOffsetMm[2] = {0,0};

  // Парсер потока UART (ASCII и "0x80 0x06 0x82")
  char     _laserAscii[2][20] = {{0}};
  uint8_t  _laserAsciiN[2] = {0};
  uint8_t  _laserBinState[2] = {0};
  char     _laserBinDigits[2][8] = {{0}};
  uint8_t  _laserBinN[2] = {0};

  int32_t _usHist[2][3] = {{0}};
  uint8_t _usHistN[2] = {0};

  void initLaserPort(HardwareSerial& s);
  void reopenLaserPort(HardwareSerial& s);
  void sendLaserRuntimeStart(HardwareSerial& s, uint8_t addr);
  void sendLaserCmd80(HardwareSerial& s, const uint8_t* payload, uint8_t n);
  void sendLaserCmd04(HardwareSerial& s, const uint8_t* payload, uint8_t n);
  void initUltrasonicPins();

  bool readLaserStream(uint8_t idx, HardwareSerial& s, int32_t& outMm, bool& hadBytes);
  int32_t readUltrasonicMm(uint8_t trigPin, uint8_t echoPin);

  int32_t pushAvg3(int32_t hist[3], uint8_t& n, int32_t v);
};

