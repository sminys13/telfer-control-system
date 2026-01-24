/**
 * @file sensors.cpp
 * @brief Реализация функций управления датчиками системы
 * @version 4.0
 */

#include "../include/sensors.h"
#include <Arduino.h>
#include <HardwareSerial.h>

// ========== ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Серийные порты для лазерных дальномеров
static HardwareSerial *laserSerial1 = NULL;
static HardwareSerial *laserSerial2 = NULL;

// Данные датчиков
static SensorData laserSensor1 = {0};
static SensorData laserSensor2 = {0};
static SensorData ultrasonicSensor1 = {0};
static SensorData ultrasonicSensor2 = {0};

// Фильтры Калмана
static KalmanFilter kalmanLaser1 = {0};
static KalmanFilter kalmanLaser2 = {0};
static KalmanFilter kalmanUS1 = {0};
static KalmanFilter kalmanUS2 = {0};

// Буферы для данных
// static uint8_t laserBuffer[32] = {0};
// static uint8_t laserBufferIndex = 0;

// Статистика
static uint32_t sensorReadCount = 0;
static uint32_t sensorErrorCount = 0;
static uint32_t lastDiagnosticTime = 0;

// ========== ИНИЦИАЛИЗАЦИЯ ==========

/**
 * @brief Инициализация всех датчиков системы
 */
void sensorsInit(void)
{
    Serial.println(F("Инициализация датчиков..."));

    // Инициализация лазерных дальномеров (Serial1 и Serial2)
    laserSerial1 = &Serial1;
    laserSerial2 = &Serial2;
    laserSensorsInit(laserSerial1, laserSerial2);

    // Инициализация ультразвуковых датчиков
    ultrasonicSensorsInit();

    // Инициализация фильтров Калмана
    initKalmanFilter(&kalmanLaser1, 0.0f, 0.1f, 1.0f);
    initKalmanFilter(&kalmanLaser2, 0.0f, 0.1f, 1.0f);
    initKalmanFilter(&kalmanUS1, 0.0f, 0.5f, 10.0f);
    initKalmanFilter(&kalmanUS2, 0.0f, 0.5f, 10.0f);

    // Инициализация структур данных датчиков
    memset(&laserSensor1, 0, sizeof(SensorData));
    memset(&laserSensor2, 0, sizeof(SensorData));
    memset(&ultrasonicSensor1, 0, sizeof(SensorData));
    memset(&ultrasonicSensor2, 0, sizeof(SensorData));

    laserSensor1.isValid = false;
    laserSensor2.isValid = false;
    ultrasonicSensor1.isValid = false;
    ultrasonicSensor2.isValid = false;

    laserSensor1.minValue = INT32_MAX;
    laserSensor1.maxValue = INT32_MIN;
    laserSensor2.minValue = INT32_MAX;
    laserSensor2.maxValue = INT32_MIN;
    ultrasonicSensor1.minValue = INT32_MAX;
    ultrasonicSensor1.maxValue = INT32_MIN;
    ultrasonicSensor2.minValue = INT32_MAX;
    ultrasonicSensor2.maxValue = INT32_MIN;

    Serial.println(F("Датчики инициализированы"));
}

/**
 * @brief Инициализация лазерных дальномеров
 * @param serial1 Указатель на Serial1 (левый дальномер)
 * @param serial2 Указатель на Serial2 (правый дальномер)
 */
void laserSensorsInit(HardwareSerial *serial1, HardwareSerial *serial2)
{
    if (!serial1 || !serial2)
    {
        Serial.println(F("ОШИБКА: Неверные указатели на последовательные порты"));
        return;
    }

    // Настройка последовательных портов
    serial1->begin(SERIAL_LASER1_BAUD);
    serial2->begin(SERIAL_LASER2_BAUD);

    // Очистка буферов
    while (serial1->available())
        serial1->read();
    while (serial2->available())
        serial2->read();

    delay(100);

    // Настройка левого дальномера
    Serial.println(F("Настройка левого лазерного дальномера..."));
    uint8_t initSequence1[][5] = {
        {LASER_CMD_LASER_ON},
        {LASER_CMD_SET_FREQ_10HZ},
        {LASER_CMD_SET_RES_1MM},
        {LASER_CMD_CONT_MEAS}};

    for (uint8_t i = 0; i < 4; i++)
    {
        sendLaserCommand(serial1, initSequence1[i], 5);
        delay(50);
    }

    // Настройка правого дальномера
    Serial.println(F("Настройка правого лазерного дальномера..."));
    uint8_t initSequence2[][5] = {
        {LASER_CMD_LASER_ON},
        {LASER_CMD_SET_FREQ_10HZ},
        {LASER_CMD_SET_RES_1MM},
        {LASER_CMD_CONT_MEAS}};

    for (uint8_t i = 0; i < 4; i++)
    {
        sendLaserCommand(serial2, initSequence2[i], 5);
        delay(50);
    }

    // Ожидание стабилизации
    delay(200);

    Serial.println(F("Лазерные дальномеры настроены"));
}

/**
 * @brief Инициализация ультразвуковых датчиков
 */
void ultrasonicSensorsInit(void)
{
    Serial.println(F("Инициализация ультразвуковых датчиков..."));

    // Настройка пинов
    pinMode(US1_TRIG_PIN, OUTPUT);
    pinMode(US1_ECHO_PIN, INPUT);
    pinMode(US2_TRIG_PIN, OUTPUT);
    pinMode(US2_ECHO_PIN, INPUT);

    // Установка начального состояния
    digitalWrite(US1_TRIG_PIN, LOW);
    digitalWrite(US2_TRIG_PIN, LOW);

    // Задержка для стабилизации
    delay(100);

    // Калибровка (измерение смещения)
    int32_t offset1 = 0, offset2 = 0;
    ultrasonicCalibration(US1_TRIG_PIN, US1_ECHO_PIN, &offset1);
    ultrasonicCalibration(US2_TRIG_PIN, US2_ECHO_PIN, &offset2);

    Serial.print(F("Смещение УЗ1: "));
    Serial.print(offset1);
    Serial.println(F(" мм"));

    Serial.print(F("Смещение УЗ2: "));
    Serial.print(offset2);
    Serial.println(F(" мм"));

    Serial.println(F("Ультразвуковые датчики инициализированы"));
}

/**
 * @brief Самодиагностика датчиков
 * @return true если все датчики работают корректно, false в противном случае
 */
bool performSensorSelfTest(void)
{
    Serial.println(F("\n--- САМОДИАГНОСТИКА ДАТЧИКОВ ---"));

    bool allSensorsOK = true;

    // Тест лазерных дальномеров
    Serial.println(F("Тест лазерных дальномеров..."));

    float distance1 = readLaserDistance(laserSerial1);
    float distance2 = readLaserDistance(laserSerial2);

    if (distance1 < 0)
    {
        Serial.println(F("ОШИБКА: Левый лазерный дальномер не отвечает"));
        allSensorsOK = false;
    }
    else
    {
        Serial.print(F("Левый лазер: "));
        Serial.print(distance1, 3);
        Serial.println(F(" м"));
    }

    if (distance2 < 0)
    {
        Serial.println(F("ОШИБКА: Правый лазерный дальномер не отвечает"));
        allSensorsOK = false;
    }
    else
    {
        Serial.print(F("Правый лазер: "));
        Serial.print(distance2, 3);
        Serial.println(F(" м"));
    }

    // Тест ультразвуковых датчиков
    Serial.println(F("Тест ультразвуковых датчиков..."));

    int32_t usDistance1 = readUltrasonicDistance(US1_TRIG_PIN, US1_ECHO_PIN);
    int32_t usDistance2 = readUltrasonicDistance(US2_TRIG_PIN, US2_ECHO_PIN);

    if (usDistance1 < 10 || usDistance1 > 4000)
    {
        Serial.println(F("ОШИБКА: Левый УЗ датчик выдает неверные данные"));
        allSensorsOK = false;
    }
    else
    {
        Serial.print(F("Левый УЗ: "));
        Serial.print(usDistance1);
        Serial.println(F(" мм"));
    }

    if (usDistance2 < 10 || usDistance2 > 4000)
    {
        Serial.println(F("ОШИБКА: Правый УЗ датчик выдает неверные данные"));
        allSensorsOK = false;
    }
    else
    {
        Serial.print(F("Правый УЗ: "));
        Serial.print(usDistance2);
        Serial.println(F(" мм"));
    }

    if (allSensorsOK)
    {
        Serial.println(F("Самодиагностика датчиков: УСПЕХ"));
    }
    else
    {
        Serial.println(F("Самодиагностика датчиков: ОШИБКА"));
    }

    Serial.println(F("--- САМОДИАГНОСТИКА ЗАВЕРШЕНА ---\n"));

    return allSensorsOK;
}

// ========== ЛАЗЕРНЫЕ ДАЛЬНОМЕРЫ ==========

/**
 * @brief Отправка команды лазерному дальномеру
 * @param serial Указатель на последовательный порт
 * @param command Массив с командой
 * @param length Длина команды
 * @return true если команда отправлена успешно
 */
bool sendLaserCommand(HardwareSerial *serial, const uint8_t *command, uint8_t length)
{
    if (!serial || !command || length == 0)
    {
        return false;
    }

    // Очистка входного буфера
    while (serial->available())
    {
        serial->read();
    }

    // Отправка команды
    size_t bytesWritten = serial->write(command, length);

    if (bytesWritten != length)
    {
        Serial.println(F("ОШИБКА: Не удалось отправить команду лазеру"));
        return false;
    }

    // Ожидание выполнения команды
    delay(10);

    return true;
}

/**
 * @brief Чтение расстояния с лазерного дальномера
 * @param serial Указатель на последовательный порт
 * @return Расстояние в метрах или -1 при ошибке
 */
float readLaserDistance(HardwareSerial *serial)
{
    if (!serial)
    {
        return -1.0f;
    }

    static uint8_t buffer[32];
    uint8_t bufferIndex = 0;

    // Чтение доступных данных
    while (serial->available() && bufferIndex < sizeof(buffer))
    {
        buffer[bufferIndex] = serial->read();
        bufferIndex++;

        // Небольшая задержка между байтами
        delayMicroseconds(100);
    }

    // Если получено недостаточно данных
    if (bufferIndex < 11)
    {
        return -1.0f;
    }

    // Поиск корректного пакета в буфере
    for (uint8_t start = 0; start <= bufferIndex - 11; start++)
    {
        if (buffer[start] == 0x80 && buffer[start + 1] == 0x06 && buffer[start + 2] == 0x82)
        {
            // Пакет найден, парсим данные
            float distance = 0.0f;
            if (parseLaserData(&buffer[start], 11, &distance))
            {
                return distance;
            }
        }
    }

    return -1.0f;
}

/**
 * @brief Парсинг данных лазерного дальномера
 * @param buffer Буфер с данными
 * @param length Длина данных
 * @param distance Указатель для сохранения расстояния
 * @return true если данные корректны
 */
bool parseLaserData(const uint8_t *buffer, uint8_t length, float *distance)
{
    if (!buffer || length < 11 || !distance)
    {
        return false;
    }

    // Проверка заголовка пакета
    if (buffer[0] != 0x80 || buffer[1] != 0x06 || buffer[2] != 0x82)
    {
        return false;
    }

    // Преобразование ASCII в строку
    char distanceStr[8] = {0};
    for (uint8_t i = 0; i < 7; i++)
    {
        distanceStr[i] = buffer[3 + i];
    }
    distanceStr[7] = '\0';

    // Проверка контрольной суммы (простейшая)
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        checksum += buffer[i];
    }

    // Преобразование строки в число
    *distance = atof(distanceStr);

    // Дополнительная проверка на корректность значения
    if (*distance < 0.0f || *distance > 80.0f)
    {
        return false;
    }

    return true;
}

/**
 * @brief Установка частоты измерений лазерного дальномера
 * @param serial Указатель на последовательный порт
 * @param frequency Частота: 5, 10 или 20 Гц
 */
void setLaserFrequency(HardwareSerial *serial, uint8_t frequency)
{
    uint8_t command[5] = {0x04, 0x0A, 0x00, 0x00, 0x00};

    switch (frequency)
    {
    case 5:
        command[2] = 0x05;
        command[3] = 0xF3;
        break;
    case 10:
        command[2] = 0x0A;
        command[3] = 0xEE;
        break;
    case 20:
        command[2] = 0x14;
        command[3] = 0xE4;
        break;
    default:
        // По умолчанию 10 Гц
        command[2] = 0x0A;
        command[3] = 0xEE;
        break;
    }

    sendLaserCommand(serial, command, 4);
}

/**
 * @brief Управление питанием лазерного дальномера
 * @param serial Указатель на последовательный порт
 * @param powerOn true - включить, false - выключить
 */
void laserPowerControl(HardwareSerial *serial, bool powerOn)
{
    if (powerOn)
    {
        uint8_t command[] = {LASER_CMD_LASER_ON};
        sendLaserCommand(serial, command, 5);
    }
    else
    {
        uint8_t command[] = {LASER_CMD_LASER_OFF};
        sendLaserCommand(serial, command, 5);
    }
}

// ========== УЛЬТРАЗВУКОВЫЕ ДАТЧИКИ ==========

/**
 * @brief Чтение расстояния с ультразвукового датчика
 * @param trigPin Пин Trig
 * @param echoPin Пин Echo
 * @return Расстояние в миллиметрах
 */
int32_t readUltrasonicDistance(uint8_t trigPin, uint8_t echoPin)
{
    // Измерение длительности импульса
    int32_t duration = measureUltrasonicPulse(trigPin, echoPin);

    if (duration <= 0)
    {
        return -1; // Ошибка измерения
    }

    // Преобразование в расстояние (мм)
    // Скорость звука: 340 м/с = 0.34 мм/мкс
    // Расстояние = (время * скорость) / 2
    int32_t distance = (duration * 0.34f) / 2.0f;

    return distance;
}

/**
 * @brief Измерение длительности эхо-импульса
 * @param trigPin Пин Trig
 * @param echoPin Пин Echo
 * @param timeout Таймаут в микросекундах
 * @return Длительность импульса в микросекундах
 */
int32_t measureUltrasonicPulse(uint8_t trigPin, uint8_t echoPin, uint32_t timeout)
{
    // Генерация импульса 10 мкс
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Измерение длительности ответного импульса
    uint32_t duration = pulseIn(echoPin, HIGH, timeout);

    // Проверка таймаута
    if (duration == 0)
    {
        return -1; // Таймаут
    }

    return duration;
}

/**
 * @brief Калибровка ультразвукового датчика
 * @param trigPin Пин Trig
 * @param echoPin Пин Echo
 * @param offset Указатель для сохранения смещения
 */
void ultrasonicCalibration(uint8_t trigPin, uint8_t echoPin, int32_t *offset)
{
    if (!offset)
    {
        return;
    }

    const uint8_t samples = 10;
    int32_t measurements[samples];

    // Выполнение нескольких измерений
    for (uint8_t i = 0; i < samples; i++)
    {
        measurements[i] = readUltrasonicDistance(trigPin, echoPin);
        delay(50);
    }

    // Сортировка для медианного фильтра
    for (uint8_t i = 0; i < samples - 1; i++)
    {
        for (uint8_t j = i + 1; j < samples; j++)
        {
            if (measurements[i] > measurements[j])
            {
                int32_t temp = measurements[i];
                measurements[i] = measurements[j];
                measurements[j] = temp;
            }
        }
    }

    // Использование медианы
    *offset = measurements[samples / 2];
}

// ========== ФИЛЬТРАЦИЯ И ОБРАБОТКА ДАННЫХ ==========

/**
 * @brief Инициализация фильтра Калмана
 * @param filter Указатель на структуру фильтра
 * @param initialEstimate Начальная оценка
 * @param processNoise Шум процесса
 * @param measureNoise Шум измерения
 */
void initKalmanFilter(KalmanFilter *filter, float initialEstimate,
                      float processNoise, float measureNoise)
{
    if (!filter)
    {
        return;
    }

    filter->estimate = initialEstimate;
    filter->estimateError = 1.0f;
    filter->processNoise = processNoise;
    filter->measureNoise = measureNoise;
    filter->kalmanGain = 0.0f;
}

/**
 * @brief Обновление фильтра Калмана
 * @param filter Указатель на структуру фильтра
 * @param measurement Новое измерение
 * @return Отфильтрованное значение
 */
float updateKalmanFilter(KalmanFilter *filter, float measurement)
{
    if (!filter)
    {
        return measurement;
    }

    // Предсказание
    filter->estimateError += filter->processNoise;

    // Обновление
    filter->kalmanGain = filter->estimateError /
                         (filter->estimateError + filter->measureNoise);
    filter->estimate += filter->kalmanGain * (measurement - filter->estimate);
    filter->estimateError *= (1.0f - filter->kalmanGain);

    return filter->estimate;
}

/**
 * @brief Применение скользящего среднего
 * @param sensor Указатель на данные датчика
 * @param newValue Новое значение
 * @param windowSize Размер окна
 * @return Отфильтрованное значение
 */
int32_t applyMovingAverage(SensorData *sensor, int32_t newValue, uint8_t windowSize)
{
    if (!sensor || windowSize == 0)
    {
        return newValue;
    }

    // Простая реализация скользящего среднего
    sensor->averageValue = (sensor->averageValue * (windowSize - 1) + newValue) / windowSize;

    return sensor->averageValue;
}

/**
 * @brief Применение медианного фильтра
 * @param values Массив значений
 * @param size Размер массива
 * @param newValue Новое значение
 * @return Медианное значение
 */
int32_t applyMedianFilter(int32_t *values, uint8_t size, int32_t newValue)
{
    if (!values || size == 0)
    {
        return newValue;
    }

    // Сдвиг значений в массиве
    for (uint8_t i = 0; i < size - 1; i++)
    {
        values[i] = values[i + 1];
    }
    values[size - 1] = newValue;

    // Копирование для сортировки
    int32_t sortedValues[size];
    memcpy(sortedValues, values, sizeof(sortedValues));

    // Сортировка пузырьком
    for (uint8_t i = 0; i < size - 1; i++)
    {
        for (uint8_t j = i + 1; j < size; j++)
        {
            if (sortedValues[i] > sortedValues[j])
            {
                int32_t temp = sortedValues[i];
                sortedValues[i] = sortedValues[j];
                sortedValues[j] = temp;
            }
        }
    }

    // Возврат медианы
    return sortedValues[size / 2];
}

// ========== ОБНОВЛЕНИЕ ДАННЫХ ==========

/**
 * @brief Обновление всех датчиков системы
 * @param status Указатель на статус системы
 */
void updateAllSensors(SystemStatus *status)
{
    if (!status)
    {
        return;
    }

    uint32_t currentTime = millis();

    // Обновление лазерных датчиков
    updateLaserSensors(status);

    // Обновление ультразвуковых датчиков
    updateUltrasonicSensors(status);

    // Расчет производных значений
    calculateDerivedValues(status);

    // Обновление статистики
    sensorReadCount++;

    // Периодическая диагностика
    if (currentTime - lastDiagnosticTime > 5000)
    { // Каждые 5 секунд
        diagnoseSensorIssues(status);
        lastDiagnosticTime = currentTime;
    }
}

/**
 * @brief Обновление лазерных датчиков
 * @param status Указатель на статус системы
 */
void updateLaserSensors(SystemStatus *status)
{
    // Чтение левого лазерного дальномера
    float distance1 = readLaserDistance(laserSerial1);
    if (distance1 >= 0)
    {
        int32_t mmDistance = convertMetersToMillimeters(distance1);

        // Фильтрация
        float filtered1 = updateKalmanFilter(&kalmanLaser1, mmDistance);
        mmDistance = (int32_t)filtered1;

        // Обновление данных
        laserSensor1.rawValue = mmDistance;
        laserSensor1.filteredValue = applyMovingAverage(&laserSensor1, mmDistance, 5);
        laserSensor1.lastUpdate = millis();
        laserSensor1.isValid = true;
        laserSensor1.errorCount = 0;

        // Обновление min/max
        if (mmDistance < laserSensor1.minValue)
            laserSensor1.minValue = mmDistance;
        if (mmDistance > laserSensor1.maxValue)
            laserSensor1.maxValue = mmDistance;

        status->telfer1Pos = laserSensor1.filteredValue;
    }
    else
    {
        laserSensor1.errorCount++;
        if (laserSensor1.errorCount > 10)
        {
            laserSensor1.isValid = false;
        }
    }

    // Чтение правого лазерного дальномера
    float distance2 = readLaserDistance(laserSerial2);
    if (distance2 >= 0)
    {
        int32_t mmDistance = convertMetersToMillimeters(distance2);

        // Фильтрация
        float filtered2 = updateKalmanFilter(&kalmanLaser2, mmDistance);
        mmDistance = (int32_t)filtered2;

        // Обновление данных
        laserSensor2.rawValue = mmDistance;
        laserSensor2.filteredValue = applyMovingAverage(&laserSensor2, mmDistance, 5);
        laserSensor2.lastUpdate = millis();
        laserSensor2.isValid = true;
        laserSensor2.errorCount = 0;

        // Обновление min/max
        if (mmDistance < laserSensor2.minValue)
            laserSensor2.minValue = mmDistance;
        if (mmDistance > laserSensor2.maxValue)
            laserSensor2.maxValue = mmDistance;

        status->telfer2Pos = laserSensor2.filteredValue;
    }
    else
    {
        laserSensor2.errorCount++;
        if (laserSensor2.errorCount > 10)
        {
            laserSensor2.isValid = false;
        }
    }
}

/**
 * @brief Обновление ультразвуковых датчиков
 * @param status Указатель на статус системы
 */
void updateUltrasonicSensors(SystemStatus *status)
{
    // Чтение левого ультразвукового датчика
    int32_t usDistance1 = readUltrasonicDistance(US1_TRIG_PIN, US1_ECHO_PIN);
    if (usDistance1 > 10 && usDistance1 < 4000)
    {
        // Фильтрация
        float filtered1 = updateKalmanFilter(&kalmanUS1, usDistance1);
        usDistance1 = (int32_t)filtered1;

        // Обновление данных
        ultrasonicSensor1.rawValue = usDistance1;
        ultrasonicSensor1.filteredValue = applyMovingAverage(&ultrasonicSensor1, usDistance1, 3);
        ultrasonicSensor1.lastUpdate = millis();
        ultrasonicSensor1.isValid = true;
        ultrasonicSensor1.errorCount = 0;

        // Обновление min/max
        if (usDistance1 < ultrasonicSensor1.minValue)
            ultrasonicSensor1.minValue = usDistance1;
        if (usDistance1 > ultrasonicSensor1.maxValue)
            ultrasonicSensor1.maxValue = usDistance1;

        status->cargoHeight1 = ultrasonicSensor1.filteredValue;
    }
    else
    {
        ultrasonicSensor1.errorCount++;
        if (ultrasonicSensor1.errorCount > 5)
        {
            ultrasonicSensor1.isValid = false;
        }
    }

    // Чтение правого ультразвукового датчика
    int32_t usDistance2 = readUltrasonicDistance(US2_TRIG_PIN, US2_ECHO_PIN);
    if (usDistance2 > 10 && usDistance2 < 4000)
    {
        // Фильтрация
        float filtered2 = updateKalmanFilter(&kalmanUS2, usDistance2);
        usDistance2 = (int32_t)filtered2;

        // Обновление данных
        ultrasonicSensor2.rawValue = usDistance2;
        ultrasonicSensor2.filteredValue = applyMovingAverage(&ultrasonicSensor2, usDistance2, 3);
        ultrasonicSensor2.lastUpdate = millis();
        ultrasonicSensor2.isValid = true;
        ultrasonicSensor2.errorCount = 0;

        // Обновление min/max
        if (usDistance2 < ultrasonicSensor2.minValue)
            ultrasonicSensor2.minValue = usDistance2;
        if (usDistance2 > ultrasonicSensor2.maxValue)
            ultrasonicSensor2.maxValue = usDistance2;

        status->cargoHeight2 = ultrasonicSensor2.filteredValue;
    }
    else
    {
        ultrasonicSensor2.errorCount++;
        if (ultrasonicSensor2.errorCount > 5)
        {
            ultrasonicSensor2.isValid = false;
        }
    }
}

/**
 * @brief Расчет производных значений
 * @param status Указатель на статус системы
 */
void calculateDerivedValues(SystemStatus *status)
{
    // Средняя горизонтальная позиция
    status->avgHorizontalPos = (status->telfer1Pos + status->telfer2Pos) / 2;

    // Средняя высота
    status->avgHeight = (status->cargoHeight1 + status->cargoHeight2) / 2;

    // Разница высот для наклона
    status->tiltDifference = abs(status->cargoHeight1 - status->cargoHeight2);

    // Общая валидность данных датчиков
    status->sensorsValid = laserSensor1.isValid && laserSensor2.isValid &&
                           ultrasonicSensor1.isValid && ultrasonicSensor2.isValid;

    // Расчет скорости изменения
    uint32_t currentTime = millis();
    laserSensor1.velocity = calculateVelocity(&laserSensor1, status->telfer1Pos, currentTime);
    laserSensor2.velocity = calculateVelocity(&laserSensor2, status->telfer2Pos, currentTime);
}

// ========== ВАЛИДАЦИЯ И ДИАГНОСТИКА ==========

/**
 * @brief Проверка согласованности данных датчиков
 * @param status Указатель на статус системы
 * @return true если данные согласованы
 */
bool checkSensorConsistency(const SystemStatus *status)
{
    extern SystemFlags systemFlags;

    if (!status)
    {
        return false;
    }

    // Проверка расхождения между тельферами (не должно быть больше 100 мм)
    int32_t horizontalDiff = abs(status->telfer1Pos - status->telfer2Pos);
    if (horizontalDiff > 100)
    {
        Serial.print(F("ПРЕДУПРЕЖДЕНИЕ: Большое расхождение тельферов: "));
        Serial.print(horizontalDiff);
        Serial.println(F(" мм"));
        return false;
    }

    // Проверка разницы высот (не должно быть больше 500 мм при нормальной работе)
    int32_t heightDiff = abs(status->cargoHeight1 - status->cargoHeight2);
    if (heightDiff > 500 && !systemFlags.manualMode)
    {
        Serial.print(F("ПРЕДУПРЕЖДЕНИЕ: Большая разница высот: "));
        Serial.print(heightDiff);
        Serial.println(F(" мм"));
        return false;
    }

    return true;
}

/**
 * @brief Диагностика проблем с датчиками
 * @param status Указатель на статус системы
 */
void diagnoseSensorIssues(SystemStatus *status)
{
    if (!status)
    {
        return;
    }

    Serial.println(F("\n--- ДИАГНОСТИКА ДАТЧИКОВ ---"));

    // Проверка валидности датчиков
    if (!laserSensor1.isValid)
    {
        Serial.println(F("ОШИБКА: Левый лазерный дальномер неисправен"));
    }

    if (!laserSensor2.isValid)
    {
        Serial.println(F("ОШИБКА: Правый лазерный дальномер неисправен"));
    }

    if (!ultrasonicSensor1.isValid)
    {
        Serial.println(F("ОШИБКА: Левый УЗ датчик неисправен"));
    }

    if (!ultrasonicSensor2.isValid)
    {
        Serial.println(F("ОШИБКА: Правый УЗ датчик неисправен"));
    }

    // Вывод статистики
    Serial.print(F("Счетчик чтений: "));
    Serial.println(sensorReadCount);

    Serial.print(F("Счетчик ошибок: "));
    Serial.println(sensorErrorCount);

    float errorRate = (sensorReadCount > 0) ? (100.0f * sensorErrorCount / sensorReadCount) : 0.0f;

    Serial.print(F("Процент ошибок: "));
    Serial.print(errorRate, 2);
    Serial.println(F(" %"));

    Serial.println(F("--- ДИАГНОСТИКА ЗАВЕРШЕНА ---\n"));
}

// ========== УТИЛИТЫ ==========

/**
 * @brief Конвертация метров в миллиметры
 * @param meters Расстояние в метрах
 * @return Расстояние в миллиметрах
 */
int32_t convertMetersToMillimeters(float meters)
{
    return (int32_t)(meters * 1000.0f);
}

/**
 * @brief Расчет скорости изменения значения датчика
 * @param sensor Указатель на данные датчика
 * @param newValue Новое значение
 * @param currentTime Текущее время
 * @return Скорость изменения (мм/с)
 */
float calculateVelocity(SensorData *sensor, int32_t newValue, uint32_t currentTime)
{
    if (!sensor || sensor->lastUpdate == 0)
    {
        return 0.0f;
    }

    uint32_t timeDiff = currentTime - sensor->lastUpdate;
    if (timeDiff == 0)
    {
        return 0.0f;
    }

    int32_t valueDiff = newValue - sensor->filteredValue;
    float velocity = (1000.0f * valueDiff) / timeDiff; // мм/с

    return velocity;
}

// ========== ОТЛАДКА ==========

#ifdef DEBUG_SENSORS
/**
 * @brief Вывод данных датчика в последовательный порт
 * @param sensor Указатель на данные датчика
 * @param name Имя датчика
 */
void printSensorData(const SensorData *sensor, const char *name)
{
    if (!sensor || !name)
    {
        return;
    }

    Serial.print(name);
    Serial.print(F(": Raw="));
    Serial.print(sensor->rawValue);
    Serial.print(F(" mm, Filtered="));
    Serial.print(sensor->filteredValue);
    Serial.print(F(" mm, Valid="));
    Serial.print(sensor->isValid ? "Yes" : "No");
    Serial.print(F(", Errors="));
    Serial.println(sensor->errorCount);
}

/**
 * @brief Вывод пакета данных лазерного дальномера
 * @param data Буфер с данными
 * @param length Длина данных
 */
void printLaserPacket(const uint8_t *data, uint8_t length)
{
    if (!data || length == 0)
    {
        return;
    }

    Serial.print(F("Laser packet: "));
    for (uint8_t i = 0; i < length; i++)
    {
        if (data[i] < 0x10)
            Serial.print('0');
        Serial.print(data[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}
#endif