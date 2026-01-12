/**
 * @file utils.h
 * @brief Вспомогательные функции: математика, звук, таймеры
 * @version 4.0
 */

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "../include/config.h"

// ========== КОНСТАНТЫ УТИЛИТ ==========

// Музыкальные ноты (частота в Гц)
#define NOTE_C3 131
#define NOTE_G3 196

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988

#define NOTE_C6 1047

// Мелодии
#define MELODY_STARTUP {NOTE_C5, NOTE_E5, NOTE_G5}
#define MELODY_SUCCESS {NOTE_E5, NOTE_G5, NOTE_C6}
#define MELODY_ERROR {NOTE_C4, NOTE_G3, NOTE_C3}
#define MELODY_WARNING {NOTE_A4, NOTE_A4, NOTE_A4}
#define MELODY_EMERGENCY {NOTE_C5, NOTE_C5, NOTE_C5}

// Таймеры
#define TIMER_ONESHOT 0x01
#define TIMER_REPEAT 0x02
#define TIMER_PAUSED 0x04
#define TIMER_EXPIRED 0x08

// Математические константы
#define PI_ 3.14159265358979323846
// #define DEG_TO_RAD (PI_ / 180.0)
// #define RAD_TO_DEG (180.0 / PI_)

// ========== СТРУКТУРЫ ДАННЫХ ==========

// Структура таймера
typedef struct
{
    uint32_t startTime; // Время старта
    uint32_t duration;  // Длительность (мс)
    uint32_t elapsed;   // Прошедшее время
    uint32_t remaining; // Оставшееся время
    uint8_t flags;      // Флаги таймера
    void (*callback)(); // Функция обратного вызова
    void *context;      // Контекст для callback
} Timer;

// Структура мелодии
typedef struct
{
    const uint16_t *notes;     // Массив нот
    const uint16_t *durations; // Массив длительностей
    uint8_t length;            // Количество нот
    uint8_t tempo;             // Темп (BPM)
    bool playing;              // Играется ли сейчас
    uint8_t currentNote;       // Текущая нота
    uint32_t noteStartTime;    // Время начала ноты
} Melody;

// Структура статистики выполнения
typedef struct
{
    uint32_t minLoopTime;  // Минимальное время цикла
    uint32_t maxLoopTime;  // Максимальное время цикла
    uint32_t avgLoopTime;  // Среднее время цикла
    uint32_t lastLoopTime; // Время последнего цикла
    uint32_t loopCount;    // Счетчик циклов
    uint32_t errorCount;   // Счетчик ошибок
    float cpuLoad;         // Загрузка ЦП (%)
} PerformanceStats;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация
void utilsInit(void);
void utilsResetStats(void);

// Звук
void beep(uint16_t frequency, uint16_t duration);
void beepSequence(uint8_t count, uint16_t frequency, uint16_t duration);
void playMelody(const uint16_t *notes, const uint16_t *durations, uint8_t length, uint8_t tempo);
void stopMelody(void);
bool isMelodyPlaying(void);
void updateMelody(void);
void playStartupMelody(void);
void playSuccessMelody(void);
void playErrorMelody(void);
void playWarningMelody(void);
void playEmergencyMelody(void);

// Таймеры
Timer *timerCreate(uint32_t duration, uint8_t flags, void (*callback)(), void *context);
bool timerStart(Timer *timer);
bool timerStop(Timer *timer);
bool timerPause(Timer *timer);
bool timerResume(Timer *timer);
bool timerReset(Timer *timer);
bool timerIsRunning(Timer *timer);
bool timerIsExpired(Timer *timer);
uint32_t timerGetRemaining(Timer *timer);
uint32_t timerGetElapsed(Timer *timer);
void timerUpdateAll(void);

// Математические функции
int32_t constrainValue(int32_t value, int32_t minVal, int32_t maxVal);
float constrainValueF(float value, float minVal, float maxVal);
int32_t mapValue(int32_t value, int32_t fromMin, int32_t fromMax, int32_t toMin, int32_t toMax);
float mapValueF(float value, float fromMin, float fromMax, float toMin, float toMax);
int32_t clamp(int32_t value, int32_t minVal, int32_t maxVal);
float clampF(float value, float minVal, float maxVal);
float lerp(float a, float b, float t);
float smoothstep(float edge0, float edge1, float x);
int32_t roundToNearest(int32_t value, int32_t nearest);
float calculatePID(float setpoint, float actual, float *integral, float previousError,
                   float Kp, float Ki, float Kd, float dt, float integralLimit);
float calculateLowPass(float input, float previous, float alpha);
float calculateEMA(float input, float previous, float alpha);

// Фильтрация
int32_t medianFilter(int32_t *buffer, uint8_t size, int32_t newValue);
float movingAverage(float *buffer, uint8_t size, float newValue);
void initCircularBuffer(int32_t *buffer, uint8_t size);
int32_t getCircularBufferAverage(int32_t *buffer, uint8_t size);

// Преобразования
int32_t metersToMillimeters(float meters);
float millimetersToMeters(int32_t millimeters);
int32_t centimetersToMillimeters(float centimeters);
float millimetersToCentimeters(int32_t millimeters);
int32_t rpmToSpeed(float rpm, float wheelDiameter);
float speedToRpm(int32_t speed, float wheelDiameter);

// Работа со строками
char *floatToString(float value, char *buffer, uint8_t decimalPlaces);
char *intToString(int32_t value, char *buffer, uint8_t base);
char *formatDistance(int32_t millimeters, char *buffer, bool includeUnits);
char *formatTime(uint32_t milliseconds, char *buffer, bool includeSeconds);
void truncateString(char *str, uint8_t maxLength);
bool stringStartsWith(const char *str, const char *prefix);
bool stringEndsWith(const char *str, const char *suffix);
void replaceChar(char *str, char find, char replace);

// Битовые операции
uint8_t setBit(uint8_t byte, uint8_t bit);
uint8_t clearBit(uint8_t byte, uint8_t bit);
uint8_t toggleBit(uint8_t byte, uint8_t bit);
bool checkBit(uint8_t byte, uint8_t bit);
uint16_t mergeBytes(uint8_t high, uint8_t low);
void splitBytes(uint16_t value, uint8_t *high, uint8_t *low);
uint16_t reverseBits(uint16_t value);
uint32_t calculateParity(uint32_t value);

// CRC и проверки
uint16_t crc16(const uint8_t *data, uint16_t length);
uint32_t crc32(const uint8_t *data, uint16_t length);
uint8_t calculateChecksum(const uint8_t *data, uint16_t length);
bool verifyChecksum(const uint8_t *data, uint16_t length, uint8_t checksum);

// Производительность
void updatePerformanceStats(uint32_t loopTime);
PerformanceStats *getPerformanceStats(void);
void printPerformanceStats(void);
float getCPULoad(void);
uint32_t getFreeMemory(void);

// Задержки и синхронизация
void delayMs(uint32_t milliseconds);
void delayUs(uint32_t microseconds);
void preciseDelay(uint32_t microseconds);
uint32_t getSystemTick(void);
bool isTimeout(uint32_t startTime, uint32_t timeout);
void syncToInterval(uint32_t intervalUs, uint32_t *lastTime);

// Безопасность
bool validateRange(int32_t value, int32_t minVal, int32_t maxVal, const char *name);
bool validateRangeF(float value, float minVal, float maxVal, const char *name);
bool validatePointer(void *ptr, const char *name);
void safeMemoryCopy(void *dest, const void *src, size_t size);
void safeMemorySet(void *dest, uint8_t value, size_t size);

// Отладка
#ifdef DEBUG_UTILS
void printBufferHex(const uint8_t *buffer, uint16_t length, uint8_t bytesPerLine);
void printBufferAscii(const uint8_t *buffer, uint16_t length);
void dumpMemory(void *address, uint16_t length);
void benchmarkFunction(void (*func)(), uint32_t iterations, const char *name);
#endif

#endif // UTILS_H