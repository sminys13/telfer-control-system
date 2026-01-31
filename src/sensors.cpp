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

void Sensors::initLaserPort(HardwareSerial& s) {
  s.begin(BAUD_LASER);
  while (s.available()) (void)s.read();
  delay(50);

  // Для многих лазерных дальномеров (по вашему описанию команд):
  //  - луч может быть выключен по умолчанию,
  //  - измерение может не стартовать без команды.
  // Поэтому на старте пробуем:
  //  1) включить лазерный луч
  //  2) включить непрерывные измерения
  // Если ваш датчик настроен иначе — эти команды просто будут проигнорированы.
  // Команды приведены для адреса 0x80 (заводской). Поскольку у вас 2 лазера на разных UART,
  // одинаковый адрес допустим.
  const uint8_t CMD_SET_RANGE_10M[] = {0x04, 0x09, 0x0A, 0xEF}; // set range to 10m
  s.write(CMD_SET_RANGE_10M, sizeof(CMD_SET_RANGE_10M));
  s.flush();
  delay(20);

  const uint8_t CMD_LASER_ON[]   = {0x80, 0x06, 0x05, 0x01, 0x74}; // открыть луч
  const uint8_t CMD_CONTINUOUS[] = {0x80, 0x06, 0x03, 0x77};       // непрерывный режим
  s.write(CMD_LASER_ON, sizeof(CMD_LASER_ON));
  s.flush();
  delay(20);
  s.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
  s.flush();
  delay(20);
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

bool Sensors::readLaserFrame(HardwareSerial& s, int32_t& outMm) {
  // Поддерживаем два варианта:
  //  A) Пакет 11 байт: 0x80 0x06 0x82 + 7 ASCII символов расстояния в метрах + checksum
  //  B) ASCII строка вида "123.456" (в метрах) без жёсткого протокола
  // На практике встречается A. Поэтому сначала ищем заголовок.

  // Соберём небольшой буфер
  uint8_t buf[32];
  uint8_t n = 0;
  while (s.available() && n < sizeof(buf)) {
    buf[n++] = (uint8_t)s.read();
    delayMicroseconds(120);
  }
  if (n < 6) return false;

  // Поиск протокола A
  for (uint8_t i=0; i + 11 <= n; i++) {
    if (buf[i] == 0x80 && buf[i+1] == 0x06 && buf[i+2] == 0x82) {
      char d[8];
      for (uint8_t k=0;k<7;k++) d[k] = (char)buf[i+3+k];
      d[7] = '\0';
      float meters = atof(d);
      if (meters < 0.0f) return false;
      outMm = (int32_t)(meters * 1000.0f);
      if (outMm > LASER_MAX_MM) return false;
      return true;
    }
  }

  // Фоллбек: вытащим первое "число с точкой" из ASCII
  char txt[24];
  uint8_t tn = 0;
  for (uint8_t i=0;i<n && tn<sizeof(txt)-1;i++) {
    uint8_t c = buf[i];
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') txt[tn++] = (char)c;
  }
  txt[tn] = '\0';
  if (tn < 3) return false;

  float meters = atof(txt);
  if (meters < 0.0f) return false;
  outMm = (int32_t)(meters * 1000.0f);
  if (outMm > LASER_MAX_MM) return false;
  return true;
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
  if (readLaserFrame(Serial2, mm)) {
    _snap.laser[0].mm = pushAvg3(_laserHist[0], _laserHistN[0], mm);
    _snap.laser[0].valid = true;
    _snap.laser[0].lastUpdateMs = nowMs;
  } else if ((nowMs - _snap.laser[0].lastUpdateMs) > 2000) {
    _snap.laser[0].valid = false;
  }

  if (readLaserFrame(Serial3, mm)) {
    _snap.laser[1].mm = pushAvg3(_laserHist[1], _laserHistN[1], mm);
    _snap.laser[1].valid = true;
    _snap.laser[1].lastUpdateMs = nowMs;
  } else if ((nowMs - _snap.laser[1].lastUpdateMs) > 2000) {
    _snap.laser[1].valid = false;
  }

  // Ультразвук
  int32_t us1 = readUltrasonicMm(PIN_US1_TRIG, PIN_US1_ECHO);
  if (us1 > 0) {
    _snap.us[0].mm = pushAvg3(_usHist[0], _usHistN[0], us1);
    _snap.us[0].valid = true;
    _snap.us[0].lastUpdateMs = nowMs;
  } else if ((nowMs - _snap.us[0].lastUpdateMs) > 1500) {
    _snap.us[0].valid = false;
  }

  int32_t us2 = readUltrasonicMm(PIN_US2_TRIG, PIN_US2_ECHO);
  if (us2 > 0) {
    _snap.us[1].mm = pushAvg3(_usHist[1], _usHistN[1], us2);
    _snap.us[1].valid = true;
    _snap.us[1].lastUpdateMs = nowMs;
  } else if ((nowMs - _snap.us[1].lastUpdateMs) > 1500) {
    _snap.us[1].valid = false;
  }
}

