\
/**
 * @file sensors.cpp
 * @brief Реализация датчиков.
 */
#include "sensors.h"
#include "utils.h"
#include <Arduino.h>

static constexpr uint16_t US_TIMEOUT_US = 30000; // 30 ms (~5m), но мы всё равно ограничим расстояния

void Sensors::begin() {
  // Лазеры на Serial2 (TX=16) и Serial3 (TX=14)
  // Держим TX в "1" до инициализации UART, чтобы избежать мусорных старт-битов при ресете.
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  delay(20);

  // Лазеры на Serial2 и Serial3 (UART на Mega)
  initLaserPort(Serial2);
  initLaserPort(Serial3);

  // Ультразвук
  initUltrasonicPins();

  // Сброс snapshot
  for (uint8_t i=0;i<TELFER_COUNT;i++) {
    _snap.laser[i] = {0,false,0};
    _snap.us[i]    = {0,false,0};
    _laserHistN[i] = 0;
    _usHistN[i] = 0;
  }
}


void Sensors::applySettings(const GlobalSettings& settings) {
  uint32_t t = settings.laser_timeout_ms;
  if (t < 500) t = 500;
  if (t > 20000) t = 20000;
  _laserTimeoutMs = (uint16_t)t;
  _laserReinitEnabled = settings.laser_reinit_enabled;

  _laserAddr[0] = settings.laser_addr[0] ? settings.laser_addr[0] : 0x80;
  _laserAddr[1] = settings.laser_addr[1] ? settings.laser_addr[1] : 0x80;

  _laserOffsetMm[0] = settings.laser_offset_mm[0];
  _laserOffsetMm[1] = settings.laser_offset_mm[1];
  _usOffsetMm[0]    = settings.us_offset_mm[0];
  _usOffsetMm[1]    = settings.us_offset_mm[1];

  // Очистим приёмные буферы, чтобы не ловить «хвост» после смены настроек.
  reopenLaserPort(Serial2);
  reopenLaserPort(Serial3);

  // Если включено — применим конфигурацию устройства при старте/смене настроек.
  if (settings.laser_apply_on_boot) {
    applyLaserDeviceConfig(settings);
  } else {
    // Минимально убедимся, что идёт непрерывное измерение.
    sendLaserRuntimeStart(Serial2, _laserAddr[0]);
    sendLaserRuntimeStart(Serial3, _laserAddr[1]);
  }
}


void Sensors::initLaserPort(HardwareSerial& s) {
  s.begin(BAUD_LASER);
  while (s.available()) (void)s.read();
  delay(50);

  // ВНИМАНИЕ: не шлём "тяжёлые" 0x04-команды на каждом старте,
  // чтобы не провоцировать перезапуск/мигание датчика.
  // Здесь делаем только старт непрерывного измерения.
  sendLaserRuntimeStart(s, 0x80);
}


void Sensors::reopenLaserPort(HardwareSerial& s) {
  // Мягкая переинициализация UART без отправки команд в датчик.
  // Это НЕ «перезапуск» датчика, а только очистка UART на Mega.
  s.end();
  delay(10);
  s.begin(BAUD_LASER);
  while (s.available()) (void)s.read();
  delay(10);
}

// ---------------- Laser command helpers ----------------

static uint8_t checksum80(const uint8_t* data, uint8_t n) {
  uint16_t sum = 0;
  for (uint8_t i=0;i<n;i++) sum += data[i];
  return (uint8_t)(0x100u - (sum & 0xFFu));
}

// По вашим примерам для команд, начинающихся с 0x04, чек = (0x100 - sum + 0x06) & 0xFF
static uint8_t checksum04(const uint8_t* data, uint8_t n) {
  uint16_t sum = 0;
  for (uint8_t i=0;i<n;i++) sum += data[i];
  return (uint8_t)((0x100u - (sum & 0xFFu) + 0x06u) & 0xFFu);
}

void Sensors::sendLaserCmd80(HardwareSerial& s, const uint8_t* payload, uint8_t n) {
  // payload already contains address and bytes WITHOUT checksum
  uint8_t buf[12];
  if (n + 1 > sizeof(buf)) return;
  memcpy(buf, payload, n);
  buf[n] = checksum80(buf, n);
  s.write(buf, (size_t)(n + 1));
  s.flush();
}

void Sensors::sendLaserCmd04(HardwareSerial& s, const uint8_t* payload, uint8_t n) {
  uint8_t buf[12];
  if (n + 1 > sizeof(buf)) return;
  memcpy(buf, payload, n);
  buf[n] = checksum04(buf, n);
  s.write(buf, (size_t)(n + 1));
  s.flush();
}

void Sensors::sendLaserRuntimeStart(HardwareSerial& s, uint8_t addr) {
  // Laser beam ON: 80 06 05 01 74
  const uint8_t cmdOn[]   = {addr, 0x06, 0x05, 0x01};
  // Continuous:    80 06 03 77
  const uint8_t cmdCont[] = {addr, 0x06, 0x03};
  sendLaserCmd80(s, cmdOn, (uint8_t)sizeof(cmdOn));
  delay(10);
  sendLaserCmd80(s, cmdCont, (uint8_t)sizeof(cmdCont));
  delay(10);
}

void Sensors::restartLaserStreaming() {
  // Очистим приёмные буферы и перезапустим поток измерений.
  // ВАЖНО: без 0x04 команд.
  reopenLaserPort(Serial2);
  reopenLaserPort(Serial3);
  sendLaserRuntimeStart(Serial2, _laserAddr[0]);
  delay(50);
  sendLaserRuntimeStart(Serial3, _laserAddr[1]);
}

void Sensors::applyLaserDeviceConfig(const GlobalSettings& settings) {
  // Применяем ко всем лазерам одинаковый профиль (частота/диапазон/разрешение/ноль/автостарт)
  // но адрес можем задать индивидуально.

  auto rangeCode = [&](uint8_t m)->uint8_t{
    switch (m) {
      case 5:  return 0x05;
      case 10: return 0x0A;
      case 30: return 0x1E;
      case 50: return 0x32;
      case 80: return 0x50;
      default: return 0x0A;
    }
  };
  auto freqCode = [&](uint8_t hz)->uint8_t{
    switch (hz) {
      case 0:  return 0x00;
      case 5:  return 0x05;
      case 10: return 0x0A;
      case 20: return 0x14;
      default: return 0x0A;
    }
  };

  for (uint8_t i=0;i<2;i++) {
    HardwareSerial& s = (i==0) ? Serial2 : Serial3;
    const uint8_t addr = settings.laser_addr[i] ? settings.laser_addr[i] : 0x80;
    // NOTE: некоторые экземпляры датчиков после записи параметров могут
    // кратковременно «подвисать/перезапускаться». Поэтому:
    //  - не шлём лишний раз команду установки адреса, если он уже такой;
    //  - увеличиваем паузы между командами;
    //  - после APPLY несколько раз запускаем continuous.
    const bool needSetAddr = (_laserAddr[i] != addr);
    _laserAddr[i] = addr;

    // Очистим UART перед конфигурацией (убираем хвосты continuous потока)
    reopenLaserPort(s);
    delay(30);

    // Set address (04 01 addr)
    if (needSetAddr) {
      const uint8_t cAddr[] = {0x04, 0x01, addr};
      sendLaserCmd04(s, cAddr, (uint8_t)sizeof(cAddr));
      delay(200);
    }

    // Set origin (04 08 01 top / 00 back)
    const uint8_t cOrg[] = {0x04, 0x08, (uint8_t)(settings.laser_origin[i] ? 0x01 : 0x00)};
    sendLaserCmd04(s, cOrg, (uint8_t)sizeof(cOrg));
    delay(120);

    // Set range (04 09 xx)
    const uint8_t cRange[] = {0x04, 0x09, rangeCode(settings.laser_range_m)};
    sendLaserCmd04(s, cRange, (uint8_t)sizeof(cRange));
    delay(120);

    // Set frequency (04 0A xx)
    const uint8_t cFreq[] = {0x04, 0x0A, freqCode(settings.laser_freq_hz)};
    sendLaserCmd04(s, cFreq, (uint8_t)sizeof(cFreq));
    delay(120);

    // Set resolution (04 0C 01/02)
    const uint8_t cRes[] = {0x04, 0x0C, (uint8_t)(settings.laser_resolution == 2 ? 0x02 : 0x01)};
    sendLaserCmd04(s, cRes, (uint8_t)sizeof(cRes));
    delay(120);

    // Autostart measurement at power on (04 0D)
    const uint8_t cAuto[] = {0x04, 0x0D, (uint8_t)(settings.laser_autostart ? 0x01 : 0x00)};
    sendLaserCmd04(s, cAuto, (uint8_t)sizeof(cAuto));
    delay(250);

    // Некоторые датчики после 0x04 команд могут не принять следующий 0x80 запуск сразу.
    // Поэтому сделаем несколько попыток с паузой.
    reopenLaserPort(s);
    delay(80);
    for (uint8_t k=0;k<3;k++) {
      sendLaserRuntimeStart(s, addr);
      delay(200);
    }
  }
}


void Sensors::initUltrasonicPins() {
  pinMode(PIN_US1_TRIG, OUTPUT);
  pinMode(PIN_US2_TRIG, OUTPUT);
  pinMode(PIN_US1_ECHO, INPUT);
  pinMode(PIN_US2_ECHO, INPUT);
  digitalWrite(PIN_US1_TRIG, LOW);
  digitalWrite(PIN_US2_TRIG, LOW);
}

int32_t Sensors::pushAvg3(int32_t hist[3], uint8_t& n, int32_t v) {
  if (n < 3) {
    hist[n++] = v;
  } else {
    hist[0] = hist[1];
    hist[1] = hist[2];
    hist[2] = v;
  }
  int64_t sum = 0;
  for (uint8_t i=0;i<n;i++) sum += hist[i];
  return (int32_t)(sum / (int32_t)n);
}

bool Sensors::readLaserStream(uint8_t idx, HardwareSerial& s, int32_t& outMm, bool& hadBytes) {
  hadBytes = false;
  bool got = false;

  // Чтобы не зависать в Serial.available() при большом потоке — ограничим чтение за тик.
  uint8_t guard = 0;
  while (s.available() && guard < 64) {
    guard++;
    const uint8_t b = (uint8_t)s.read();
    hadBytes = true;

    // --- Binary header mode: [ADDR][06][82][7 ascii chars][checksum] ---
    if (_laserBinState[idx] == 0) {
      if (b == _laserAddr[idx]) {
        _laserBinState[idx] = 1;
        _laserBinN[idx] = 0;
      }
    } else if (_laserBinState[idx] == 1) {
      if (b == 0x06) _laserBinState[idx] = 2;
      else _laserBinState[idx] = 0;
    } else if (_laserBinState[idx] == 2) {
      if (b == 0x82) {
        _laserBinState[idx] = 3;
        _laserBinN[idx] = 0;
      } else {
        _laserBinState[idx] = 0;
      }
    } else if (_laserBinState[idx] == 3) {
      if (_laserBinN[idx] < 7) {
        _laserBinDigits[idx][_laserBinN[idx]++] = (char)b;
        if (_laserBinN[idx] == 7) {
          _laserBinDigits[idx][7] = '\0';
          float meters = atof(_laserBinDigits[idx]);
          if (meters >= 0.0f) {
            const int32_t mm = (int32_t)(meters * 1000.0f);
            if (mm >= 0 && mm <= LASER_MAX_MM) {
              outMm = mm;
              got = true;
            }
          }
          _laserBinState[idx] = 0;
        }
      } else {
        _laserBinState[idx] = 0;
      }
      if (got) continue;
    }

    // --- ASCII mode ---
    if ((b >= '0' && b <= '9') || b == '.' || b == '-') {
      if (_laserAsciiN[idx] < sizeof(_laserAscii[idx]) - 1) {
        _laserAscii[idx][_laserAsciiN[idx]++] = (char)b;
        _laserAscii[idx][_laserAsciiN[idx]] = '\0';
      } else {
        _laserAsciiN[idx] = 0;
        _laserAscii[idx][0] = 0;
      }

      // Если датчик шлёт фиксированную длину 7 символов — парсим сразу.
      if (_laserAsciiN[idx] >= 7) {
        float meters = atof(_laserAscii[idx]);
        if (meters >= 0.0f) {
          const int32_t mm = (int32_t)(meters * 1000.0f);
          if (mm >= 0 && mm <= LASER_MAX_MM) {
            outMm = mm;
            got = true;
          }
        }
        _laserAsciiN[idx] = 0;
        _laserAscii[idx][0] = 0;
      }
    } else {
      // Любой разделитель после числа → парсим накопленное
      if (_laserAsciiN[idx] >= 3) {
        float meters = atof(_laserAscii[idx]);
        if (meters >= 0.0f) {
          const int32_t mm = (int32_t)(meters * 1000.0f);
          if (mm >= 0 && mm <= LASER_MAX_MM) {
            outMm = mm;
            got = true;
          }
        }
      }
      _laserAsciiN[idx] = 0;
      _laserAscii[idx][0] = 0;
    }
  }

  return got;
}

int32_t Sensors::readUltrasonicMm(uint8_t trigPin, uint8_t echoPin) {
  // HC-SR04:
  //  - импульс TRIG 10us
  //  - длительность ECHO пропорциональна расстоянию
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  uint32_t dur = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (dur == 0) return -1;

  // Скорость звука ~343 m/s → 29.1 us/cm туда-обратно ~58.2 us/cm, 5.82 us/mm
  // mm = dur / 5.82 / 2? Нет, dur уже туда-обратно, формула: cm = dur / 58.2
  // mm = dur * 10 / 58.2
  int32_t mm = (int32_t)((dur * 10UL) / 58UL); // достаточно точно для наших целей

  // Ограничим разумный диапазон (например 20..4000 мм)
  if (mm < 20 || mm > 4000) return -1;
  return mm;
}

void Sensors::tick(uint32_t nowMs) {
  // Лазеры
  int32_t mm = 0;

  bool had0 = false;
  if (readLaserStream(0, Serial2, mm, had0)) {
        const int32_t avg = pushAvg3(_laserHist[0], _laserHistN[0], mm);
    int32_t corr = avg + (int32_t)_laserOffsetMm[0];
    if (corr < 0) corr = 0;
    _snap.laser[0].mm = corr;
    _snap.laser[0].valid = true;
    _snap.laser[0].lastUpdateMs = nowMs;
  }
  if (had0) _laserLastRxMs[0] = nowMs;
  if ((uint32_t)(nowMs - _laserLastRxMs[0]) > _laserTimeoutMs) {
    _snap.laser[0].valid = false;
    if (_laserReinitEnabled && (uint32_t)(nowMs - _laserReinitMs[0]) > 8000) {
      reopenLaserPort(Serial2);
      sendLaserRuntimeStart(Serial2, _laserAddr[0]);
      _laserReinitMs[0] = nowMs;
    }
  }

  bool had1 = false;
  if (readLaserStream(1, Serial3, mm, had1)) {
        const int32_t avg = pushAvg3(_laserHist[1], _laserHistN[1], mm);
    int32_t corr = avg + (int32_t)_laserOffsetMm[1];
    if (corr < 0) corr = 0;
    _snap.laser[1].mm = corr;
    _snap.laser[1].valid = true;
    _snap.laser[1].lastUpdateMs = nowMs;
  }
  if (had1) _laserLastRxMs[1] = nowMs;
  if ((uint32_t)(nowMs - _laserLastRxMs[1]) > _laserTimeoutMs) {
    _snap.laser[1].valid = false;
    if (_laserReinitEnabled && (uint32_t)(nowMs - _laserReinitMs[1]) > 8000) {
      reopenLaserPort(Serial3);
      sendLaserRuntimeStart(Serial3, _laserAddr[1]);
      _laserReinitMs[1] = nowMs;
    }
  }

  // Ультразвук// Ультразвук
  int32_t us1 = readUltrasonicMm(PIN_US1_TRIG, PIN_US1_ECHO);
  if (us1 > 0) {
        const int32_t avg = pushAvg3(_usHist[0], _usHistN[0], us1);
    int32_t corr = avg + (int32_t)_usOffsetMm[0];
    if (corr < 0) corr = 0;
    _snap.us[0].mm = corr;
    _snap.us[0].valid = true;
    _snap.us[0].lastUpdateMs = nowMs;
  } else if ((nowMs - _snap.us[0].lastUpdateMs) > 1500) {
    _snap.us[0].valid = false;
  }

  int32_t us2 = readUltrasonicMm(PIN_US2_TRIG, PIN_US2_ECHO);
  if (us2 > 0) {
        const int32_t avg = pushAvg3(_usHist[1], _usHistN[1], us2);
    int32_t corr = avg + (int32_t)_usOffsetMm[1];
    if (corr < 0) corr = 0;
    _snap.us[1].mm = corr;
    _snap.us[1].valid = true;
    _snap.us[1].lastUpdateMs = nowMs;
  } else if ((nowMs - _snap.us[1].lastUpdateMs) > 1500) {
    _snap.us[1].valid = false;
  }
}

