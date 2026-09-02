/**
 * @file utils.h
 * @brief Небольшие утилиты без динамической памяти.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

template<typename T>
static inline T clampT(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

uint16_t crc16_modbus(const uint8_t* data, size_t len);

/**
 * @brief Преобразование скорости в процентах (-100..100) в Modbus setpoint (-10000..10000).
 */
int16_t pctToSetpoint(int16_t pct);

/**
 * @brief Простая "мягкая" функция скорости: чем ближе к цели — тем меньше скорость.
 * @param err_mm  ошибка позиционирования (может быть отрицательной)
 * @param minPct  минимальная скорость (%), чтобы привод не "залипал"
 * @param maxPct  максимальная скорость (%)
 * @param slowBand_mm зона, в которой начинаем замедляться (мм)
 */
int16_t speedFromErrorMm(int32_t err_mm, int16_t minPct, int16_t maxPct, int32_t slowBand_mm);

/**
 * @brief Быстрое чтение кнопки с учётом "active low".
 */
bool readActiveLow(uint8_t pin, bool activeLow);

/**
 * @brief Ограниченное форматирование int32 в буфер (без String).
 * @return длина строки.
 */
uint8_t i32toa(int32_t v, char* out, uint8_t outSize);

