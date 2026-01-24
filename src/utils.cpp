/**
 * @file utils.cpp
 * @brief Реализация вспомогательных функций
 * @version 4.0
 */

#include "../include/utils.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

// ========== ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Статистика производительности
static PerformanceStats perfStats = {
    .minLoopTime = 0xFFFFFFFF,
    .maxLoopTime = 0,
    .avgLoopTime = 0,
    .lastLoopTime = 0,
    .loopCount = 0,
    .errorCount = 0,
    .cpuLoad = 0.0f};

// Таймеры
static Timer timers[10];
static uint8_t timerCount = 0;

// Мелодии
static Melody currentMelody = {0};
static bool soundEnabled = true;

// Буферы
// static char formatBuffer[64];

// ========== ИНИЦИАЛИЗАЦИЯ ==========

/**
 * @brief Инициализация утилит
 */
void utilsInit(void)
{
    Serial.println(F("Инициализация утилит..."));

    // Инициализация пина звукового сигнала
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Инициализация таймеров
    memset(timers, 0, sizeof(timers));
    timerCount = 0;

    // Сброс статистики
    utilsResetStats();

    // Инициализация системного времени
    // (Arduino автоматически инициализирует millis() и micros())

    Serial.println(F("Утилиты инициализированы"));
}

/**
 * @brief Сброс статистики производительности
 */
void utilsResetStats(void)
{
    memset(&perfStats, 0, sizeof(PerformanceStats));
    perfStats.minLoopTime = 0xFFFFFFFF;
}

// ========== ЗВУК ==========

/**
 * @brief Звуковой сигнал
 * @param frequency Частота (Гц)
 * @param duration Длительность (мс)
 */
void beep(uint16_t frequency, uint16_t duration)
{
    if (!soundEnabled)
    {
        return;
    }

    if (frequency == 0)
    {
        // Просто пауза
        delay(duration);
        return;
    }

    // Расчет периода в микросекундах
    uint32_t period = 1000000L / frequency;
    uint32_t halfPeriod = period / 2;

    // Расчет количества циклов
    uint32_t cycles = (duration * 1000L) / period;

    // Генерация звука
    for (uint32_t i = 0; i < cycles; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(halfPeriod);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(halfPeriod);
    }
}

/**
 * @brief Последовательность звуковых сигналов
 * @param count Количество сигналов
 * @param frequency Частота (Гц)
 * @param duration Длительность каждого сигнала (мс)
 */
void beepSequence(uint8_t count, uint16_t frequency, uint16_t duration)
{
    for (uint8_t i = 0; i < count; i++)
    {
        beep(frequency, duration);
        if (i < count - 1)
        {
            delay(duration / 2);
        }
    }
}

/**
 * @brief Воспроизведение мелодии
 * @param notes Массив нот (частот)
 * @param durations Массив длительностей
 * @param length Количество нот
 * @param tempo Темп (BPM)
 */
void playMelody(const uint16_t *notes, const uint16_t *durations, uint8_t length, uint8_t tempo)
{
    if (!soundEnabled || length == 0)
    {
        return;
    }

    currentMelody.notes = notes;
    currentMelody.durations = durations;
    currentMelody.length = length;
    currentMelody.tempo = tempo;
    currentMelody.playing = true;
    currentMelody.currentNote = 0;
    currentMelody.noteStartTime = millis();

    // Начать первую ноту
    if (notes[0] > 0)
    {
        beep(notes[0], durations[0]);
    }
}

/**
 * @brief Обновление воспроизведения мелодии (вызывается в основном цикле)
 */
void updateMelody(void)
{
    if (!currentMelody.playing || currentMelody.currentNote >= currentMelody.length)
    {
        return;
    }

    uint32_t currentTime = millis();
    uint32_t noteDuration = currentMelody.durations[currentMelody.currentNote];

    // Проверка завершения текущей ноты
    if (currentTime - currentMelody.noteStartTime >= noteDuration)
    {
        currentMelody.currentNote++;

        if (currentMelody.currentNote >= currentMelody.length)
        {
            currentMelody.playing = false;
            return;
        }

        // Начать следующую ноту
        currentMelody.noteStartTime = currentTime;

        if (currentMelody.notes[currentMelody.currentNote] > 0)
        {
            beep(currentMelody.notes[currentMelody.currentNote],
                 currentMelody.durations[currentMelody.currentNote]);
        }
    }
}

/**
 * @brief Мелодия запуска системы
 */
void playStartupMelody(void)
{
    static const uint16_t startupNotes[] = {NOTE_C5, NOTE_E5, NOTE_G5};
    static const uint16_t startupDurations[] = {200, 200, 400};

    playMelody(startupNotes, startupDurations, 3, 120);
}

/**
 * @brief Мелодия ошибки
 */
void playErrorMelody(void)
{
    static const uint16_t errorNotes[] = {NOTE_C4, NOTE_G3, NOTE_C3};
    static const uint16_t errorDurations[] = {300, 300, 500};
    playMelody(errorNotes, errorDurations, 3, 120);
}

/**
 * @brief Мелодия предупреждения
 */
void playWarningMelody(void)
{
    static const uint16_t warningNotes[] = {NOTE_A4, NOTE_A4, NOTE_A4};
    static const uint16_t warningDurations[] = {200, 200, 200};
    playMelody(warningNotes, warningDurations, 3, 120);
}

/**
 * @brief Мелодия аварийной остановки
 */
void playEmergencyMelody(void)
{
    static const uint16_t emergencyNotes[] = {NOTE_C5, NOTE_C5, NOTE_C5};
    static const uint16_t emergencyDurations[] = {100, 100, 100};
    playMelody(emergencyNotes, emergencyDurations, 3, 240);
}

/**
 * @brief Мелодия успеха
 */
void playSuccessMelody(void)
{
    static const uint16_t successNotes[] = {NOTE_E5, NOTE_G5, NOTE_C6};
    static const uint16_t successDurations[] = {200, 200, 400};
    playMelody(successNotes, successDurations, 3, 120);
}

// ========== ТАЙМЕРЫ ==========

/**
 * @brief Создание таймера
 * @param duration Длительность (мс)
 * @param flags Флаги таймера
 * @param callback Функция обратного вызова
 * @param context Контекст
 * @return Указатель на таймер или NULL при ошибке
 */
Timer *timerCreate(uint32_t duration, uint8_t flags, void (*callback)(), void *context)
{
    if (timerCount >= 10)
    {
        Serial.println(F("ОШИБКА: Достигнут лимит таймеров"));
        return NULL;
    }

    Timer *timer = &timers[timerCount];
    memset(timer, 0, sizeof(Timer));

    timer->duration = duration;
    timer->flags = flags;
    timer->callback = callback;
    timer->context = context;
    timer->remaining = duration;

    timerCount++;

    return timer;
}

/**
 * @brief Запуск таймера
 * @param timer Указатель на таймер
 * @return true если успешно
 */
bool timerStart(Timer *timer)
{
    if (!timer)
    {
        return false;
    }

    timer->startTime = millis();
    timer->elapsed = 0;
    timer->remaining = timer->duration;
    timer->flags &= ~TIMER_EXPIRED;

    return true;
}

/**
 * @brief Проверка и обновление всех таймеров
 */
void timerUpdateAll(void)
{
    uint32_t currentTime = millis();

    for (uint8_t i = 0; i < timerCount; i++)
    {
        Timer *timer = &timers[i];

        if (!(timer->flags & TIMER_PAUSED) && timer->startTime > 0)
        {
            timer->elapsed = currentTime - timer->startTime;

            if (timer->elapsed >= timer->duration)
            {
                timer->elapsed = timer->duration;
                timer->remaining = 0;
                timer->flags |= TIMER_EXPIRED;

                // Вызов callback
                if (timer->callback)
                {
                    timer->callback();
                }

                // Обработка повторяющегося таймера
                if (timer->flags & TIMER_REPEAT)
                {
                    timer->startTime = currentTime;
                    timer->elapsed = 0;
                    timer->remaining = timer->duration;
                    timer->flags &= ~TIMER_EXPIRED;
                }
            }
            else
            {
                timer->remaining = timer->duration - timer->elapsed;
            }
        }
    }
}

// ========== МАТЕМАТИЧЕСКИЕ ФУНКЦИИ ==========

/**
 * @brief Ограничение значения в диапазоне
 * @param value Значение
 * @param minVal Минимальное значение
 * @param maxVal Максимальное значение
 * @return Ограниченное значение
 */
int32_t constrainValue(int32_t value, int32_t minVal, int32_t maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

/**
 * @brief Преобразование значения из одного диапазона в другой
 * @param value Значение
 * @param fromMin Минимум исходного диапазона
 * @param fromMax Максимум исходного диапазона
 * @param toMin Минимум целевого диапазона
 * @param toMax Максимум целевого диапазона
 * @return Преобразованное значение
 */
int32_t mapValue(int32_t value, int32_t fromMin, int32_t fromMax, int32_t toMin, int32_t toMax)
{
    if (fromMax == fromMin)
    {
        return toMin;
    }

    int64_t result = (int64_t)(value - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;

    // Ограничение результата
    if (result < toMin)
        return toMin;
    if (result > toMax)
        return toMax;

    return (int32_t)result;
}

/**
 * @brief ПИД-регулятор
 * @param setpoint Заданное значение
 * @param actual Текущее значение
 * @param integral Интегральная сумма
 * @param previousError Предыдущая ошибка
 * @param Kp Пропорциональный коэффициент
 * @param Ki Интегральный коэффициент
 * @param Kd Дифференциальный коэффициент
 * @param dt Временной шаг
 * @param integralLimit Лимит интегральной суммы
 * @return Выход ПИД-регулятора
 */
float calculatePID(float setpoint, float actual, float *integral, float previousError,
                   float Kp, float Ki, float Kd, float dt, float integralLimit)
{
    float error = setpoint - actual;

    // Пропорциональная составляющая
    float proportional = Kp * error;

    // Интегральная составляющая
    *integral += error * dt;

    // Ограничение интегральной суммы
    if (integralLimit > 0)
    {
        if (*integral > integralLimit)
            *integral = integralLimit;
        if (*integral < -integralLimit)
            *integral = -integralLimit;
    }

    float integralTerm = Ki * (*integral);

    // Дифференциальная составляющая
    float derivative = (error - previousError) / dt;
    float derivativeTerm = Kd * derivative;

    // Суммирование составляющих
    float output = proportional + integralTerm + derivativeTerm;

    return output;
}

// ========== ФИЛЬТРАЦИЯ ==========

/**
 * @brief Медианный фильтр
 * @param buffer Буфер значений
 * @param size Размер буфера
 * @param newValue Новое значение
 * @return Медианное значение
 */
int32_t medianFilter(int32_t *buffer, uint8_t size, int32_t newValue)
{
    if (size == 0)
    {
        return newValue;
    }

    // Сдвиг значений в буфере
    for (uint8_t i = 0; i < size - 1; i++)
    {
        buffer[i] = buffer[i + 1];
    }
    buffer[size - 1] = newValue;

    // Копирование для сортировки
    int32_t sorted[10]; // Максимальный размер
    if (size > 10)
        size = 10;

    memcpy(sorted, buffer, size * sizeof(int32_t));

    // Сортировка пузырьком
    for (uint8_t i = 0; i < size - 1; i++)
    {
        for (uint8_t j = i + 1; j < size; j++)
        {
            if (sorted[i] > sorted[j])
            {
                int32_t temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }

    // Возврат медианы
    return sorted[size / 2];
}

// ========== ПРЕОБРАЗОВАНИЯ ==========

/**
 * @brief Преобразование метров в миллиметры
 * @param meters Метры
 * @return Миллиметры
 */
int32_t metersToMillimeters(float meters)
{
    return (int32_t)(meters * 1000.0f);
}

/**
 * @brief Преобразование миллиметров в метры
 * @param millimeters Миллиметры
 * @return Метры
 */
float millimetersToMeters(int32_t millimeters)
{
    return millimeters / 1000.0f;
}

// ========== РАБОТА СО СТРОКАМИ ==========

/**
 * @brief Преобразование числа с плавающей точкой в строку
 * @param value Значение
 * @param buffer Буфер для строки
 * @param decimalPlaces Количество знаков после запятой
 * @return Указатель на буфер
 */
char *floatToString(float value, char *buffer, uint8_t decimalPlaces)
{
    if (!buffer)
    {
        return NULL;
    }

    // Обработка отрицательных значений
    if (value < 0)
    {
        *buffer++ = '-';
        value = -value;
    }

    // Целая часть
    int32_t integerPart = (int32_t)value;
    char intBuffer[16];
    itoa(integerPart, intBuffer, 10);
    strcpy(buffer, intBuffer);
    buffer += strlen(intBuffer);

    if (decimalPlaces > 0)
    {
        *buffer++ = '.';

        // Дробная часть
        float fractional = value - integerPart;
        for (uint8_t i = 0; i < decimalPlaces; i++)
        {
            fractional *= 10;
            int digit = (int)fractional;
            *buffer++ = '0' + digit;
            fractional -= digit;
        }
        *buffer = '\0';
    }

    return buffer;
}

/**
 * @brief Форматирование расстояния
 * @param millimeters Миллиметры
 * @param buffer Буфер для строки
 * @param includeUnits Включать ли единицы измерения
 * @return Указатель на буфер
 */
char *formatDistance(int32_t millimeters, char *buffer, bool includeUnits)
{
    if (!buffer)
    {
        return NULL;
    }

    if (millimeters >= 1000)
    {
        float meters = millimeters / 1000.0f;
        floatToString(meters, buffer, 2);
        if (includeUnits)
        {
            strcat(buffer, " м");
        }
    }
    else
    {
        itoa(millimeters, buffer, 10);
        if (includeUnits)
        {
            strcat(buffer, " мм");
        }
    }

    return buffer;
}

// ========== CRC И ПРОВЕРКИ ==========

/**
 * @brief Расчет CRC16 (Modbus)
 * @param data Указатель на данные
 * @param length Длина данных
 * @return CRC16
 */
uint16_t crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// ========== ПРОИЗВОДИТЕЛЬНОСТЬ ==========

/**
 * @brief Обновление статистики производительности
 * @param loopTime Время выполнения цикла
 */
void updatePerformanceStats(uint32_t loopTime)
{
    perfStats.loopCount++;
    perfStats.lastLoopTime = loopTime;

    if (loopTime < perfStats.minLoopTime)
    {
        perfStats.minLoopTime = loopTime;
    }

    if (loopTime > perfStats.maxLoopTime)
    {
        perfStats.maxLoopTime = loopTime;
    }

    // Скользящее среднее
    perfStats.avgLoopTime = (perfStats.avgLoopTime * 9 + loopTime) / 10;

    // Расчет загрузки ЦП (предполагаем целевое время цикла 10 мс)
    if (loopTime > 0)
    {
        perfStats.cpuLoad = (100.0f * loopTime) / TARGET_LOOP_TIME_US;
        if (perfStats.cpuLoad > 100.0f)
        {
            perfStats.cpuLoad = 100.0f;
        }
    }
}

/**
 * @brief Вывод статистики производительности
 */
void printPerformanceStats(void)
{
    Serial.println(F("\n=== СТАТИСТИКА ПРОИЗВОДИТЕЛЬНОСТИ ==="));
    Serial.print(F("Циклов: "));
    Serial.println(perfStats.loopCount);

    Serial.print(F("Время цикла: мин="));
    Serial.print(perfStats.minLoopTime);
    Serial.print(F("мкс, макс="));
    Serial.print(perfStats.maxLoopTime);
    Serial.print(F("мкс, ср="));
    Serial.print(perfStats.avgLoopTime);
    Serial.println(F("мкс"));

    Serial.print(F("Загрузка ЦП: "));
    Serial.print(perfStats.cpuLoad, 1);
    Serial.println(F("%"));

    Serial.print(F("Свободная память: "));
    Serial.print(getFreeMemory());
    Serial.println(F(" байт"));

    Serial.println(F("====================================\n"));
}

/**
 * @brief Получение свободной памяти
 * @return Количество свободных байт
 */
uint32_t getFreeMemory(void)
{
    extern int __heap_start, *__brkval;
    int v;

    return (uint32_t)&v - (__brkval == 0 ? (uint32_t)&__heap_start : (uint32_t)__brkval);
}

// ========== БЕЗОПАСНОСТЬ ==========

/**
 * @brief Проверка диапазона значения
 * @param value Значение
 * @param minVal Минимум
 * @param maxVal Максимум
 * @param name Имя параметра (для сообщения об ошибке)
 * @return true если значение в допустимом диапазоне
 */
bool validateRange(int32_t value, int32_t minVal, int32_t maxVal, const char *name)
{
    if (value < minVal || value > maxVal)
    {
        Serial.print(F("ОШИБКА: Параметр "));
        Serial.print(name);
        Serial.print(F(" вне диапазона: "));
        Serial.print(value);
        Serial.print(F(" не в ["));
        Serial.print(minVal);
        Serial.print(F(", "));
        Serial.print(maxVal);
        Serial.println(F("]"));
        return false;
    }

    return true;
}

// ========== ОТЛАДКА ==========

#ifdef DEBUG_UTILS
/**
 * @brief Вывод буфера в шестнадцатеричном виде
 * @param buffer Буфер
 * @param length Длина
 * @param bytesPerLine Количество байт на строку
 */
void printBufferHex(const uint8_t *buffer, uint16_t length, uint8_t bytesPerLine)
{
    for (uint16_t i = 0; i < length; i++)
    {
        if (buffer[i] < 0x10)
            Serial.print('0');
        Serial.print(buffer[i], HEX);
        Serial.print(' ');

        if ((i + 1) % bytesPerLine == 0)
        {
            Serial.println();
        }
    }

    if (length % bytesPerLine != 0)
    {
        Serial.println();
    }
}
#endif