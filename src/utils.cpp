/**
 * @file utils.cpp
 * @brief Реализация утилит.
 */
#include "utils.h"
#include <Arduino.h>

uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  // CRC16 (Modbus): poly 0xA001, init 0xFFFF
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

int16_t pctToSetpoint(int16_t pct) {
  // pct: -100..100 → -10000..10000
  pct = clampT<int16_t>(pct, -100, 100);
  return (int16_t)(pct * 100);
}

int16_t speedFromErrorMm(int32_t err_mm, int16_t minPct, int16_t maxPct, int32_t slowBand_mm) {
  if (err_mm == 0) return 0;
  const int32_t aerr = (err_mm < 0) ? -err_mm : err_mm;

  // Вне slowBand — максимальная скорость.
  if (aerr >= slowBand_mm) {
    return (err_mm > 0) ? maxPct : -maxPct;
  }

  // Линейное замедление к minPct
  // speed = minPct + (aerr/slowBand)*(maxPct-minPct)
  int32_t sp = minPct + (aerr * (maxPct - minPct)) / (slowBand_mm ? slowBand_mm : 1);
  sp = clampT<int32_t>(sp, minPct, maxPct);
  return (err_mm > 0) ? (int16_t)sp : (int16_t)-sp;
}

bool readActiveLow(uint8_t pin, bool activeLow) {
  const bool level = (digitalRead(pin) != LOW); // true=HIGH
  // Если activeLow — нажатие/сработка = LOW → возвращаем true
  return activeLow ? (!level) : level;
}

uint8_t i32toa(int32_t v, char* out, uint8_t outSize) {
  if (!out || outSize < 2) return 0;
  // простой itoa без sprintf (экономия RAM)
  char tmp[12];
  bool neg = (v < 0);
  uint32_t x = neg ? (uint32_t)(-v) : (uint32_t)v;
  uint8_t i = 0;
  do {
    tmp[i++] = char('0' + (x % 10));
    x /= 10;
  } while (x && i < sizeof(tmp)-1);

  uint8_t pos = 0;
  if (neg && pos < outSize-1) out[pos++] = '-';
  while (i && pos < outSize-1) out[pos++] = tmp[--i];
  out[pos] = '\0';
  return pos;
}

