/**
 * @file storage.cpp
 * @brief Реализация модуля хранения данных в EEPROM
 * @version 4.0
 */

#include "../include/storage.h"
#include "../include/common_definitions.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <stdarg.h>

// ========== ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Текущий статус хранилища
static StorageStatus storageStatus = {
    .initialized = false,
    .valid = false,
    .flags = 0,
    .writeCount = 0,
    .errorCount = 0,
    .freeSpace = 0,
    .lastBackup = 0};

// Буфер для временного хранения
// static uint8_t storageBuffer[256];
// static uint16_t bufferIndex = 0;

// Текущий уровень логгирования
static LogLevel currentLogLevel = LOG_LEVEL_INFO;

// Индекс следующей записи лога
static uint16_t logIndex = 0;
static uint16_t logEntryCount = 0;

// Статистика использования EEPROM
static uint32_t sectorWear[16] = {0}; // 16 секторов по 256 байт

// ========== ИНИЦИАЛИЗАЦИЯ ==========

/**
 * @brief Инициализация системы хранения данных
 * @return true если инициализация успешна
 */
bool storageInit(void)
{
    Serial.println(F("Инициализация системы хранения данных..."));

    // Инициализация EEPROM
    EEPROM.begin();

    // Проверка размера EEPROM
    if (EEPROM.length() < EEPROM_SIZE)
    {
        Serial.print(F("ОШИБКА: Недостаточно EEPROM. Требуется: "));
        Serial.print(EEPROM_SIZE);
        Serial.print(F(", доступно: "));
        Serial.println(EEPROM.length());
        return false;
    }

    // Проверка целостности данных
    bool integrityOk = storageCheckIntegrity();

    if (!integrityOk)
    {
        Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: Данные в EEPROM повреждены или отсутствуют"));

        // Попытка восстановления
        if (!storageRepair())
        {
            Serial.println(F("Восстановление не удалось, требуется форматирование"));

            // Запрос на форматирование (в реальной системе может быть кнопка или меню)
            storageFormat(false);
        }
    }

    // Загрузка количества записей лога
    logEntryCount = 0;
    for (uint16_t i = 0; i < MAX_LOG_SIZE / sizeof(LogEntry); i++)
    {
        LogEntry entry;
        if (storageRead(ADDR_SYSTEM_LOG + i * sizeof(LogEntry), &entry, sizeof(LogEntry)))
        {
            if (entry.timestamp != 0xFFFFFFFF)
            {
                logEntryCount++;
            }
        }
    }

    logIndex = logEntryCount % (MAX_LOG_SIZE / sizeof(LogEntry));

    // Расчет свободного места
    storageStatus.freeSpace = storageGetFreeSpace();

    // Установка флагов
    storageStatus.initialized = true;
    storageStatus.valid = integrityOk;
    storageStatus.flags = FLAG_VALID;

    // Запись в лог
    logMessage(LOG_INFO, "Система хранения данных инициализирована", 0);

    Serial.println(F("Система хранения данных готова к работе"));

#ifdef DEBUG_STORAGE
    printStorageStats();
#endif

    return true;
}

/**
 * @brief Проверка целостности данных в EEPROM
 * @return true если данные целы
 */
bool storageCheckIntegrity(void)
{
    uint32_t signature = 0;
    uint32_t version = 0;
    uint32_t magic = 0;
    uint32_t storedCRC = 0;
    uint32_t calculatedCRC = 0;

    // Чтение сигнатуры
    storageRead(ADDR_SIGNATURE, &signature, sizeof(signature));
    storageRead(ADDR_VERSION, &version, sizeof(version));
    storageRead(ADDR_MAGIC, &magic, sizeof(magic));

    // Проверка сигнатур
    if (signature != STORAGE_SIGNATURE ||
        version != STORAGE_VERSION ||
        magic != STORAGE_MAGIC)
    {

        Serial.println(F("Проверка сигнатур: НЕ ПРОЙДЕНА"));
        Serial.print(F("Ожидалось: S=0x"));
        Serial.print(STORAGE_SIGNATURE, HEX);
        Serial.print(F(", V=0x"));
        Serial.print(STORAGE_VERSION, HEX);
        Serial.print(F(", M=0x"));
        Serial.println(STORAGE_MAGIC, HEX);

        Serial.print(F("Получено:  S=0x"));
        Serial.print(signature, HEX);
        Serial.print(F(", V=0x"));
        Serial.print(version, HEX);
        Serial.print(F(", M=0x"));
        Serial.println(magic, HEX);

        return false;
    }

    // Проверка CRC
    storageRead(ADDR_CRC, &storedCRC, sizeof(storedCRC));
    calculatedCRC = calculateCRC(NULL, ADDR_CRC);

    if (storedCRC != calculatedCRC)
    {
        Serial.println(F("Проверка CRC: НЕ ПРОЙДЕНА"));
        Serial.print(F("Ожидалось: 0x"));
        Serial.println(calculatedCRC, HEX);
        Serial.print(F("Получено:  0x"));
        Serial.println(storedCRC, HEX);
        return false;
    }

    Serial.println(F("Проверка целостности данных: ПРОЙДЕНА"));
    return true;
}

/**
 * @brief Форматирование EEPROM
 * @param force Принудительное форматирование без проверок
 * @return true если форматирование успешно
 */
bool storageFormat(bool force)
{
    Serial.println(F("Форматирование EEPROM..."));

    if (!force)
    {
        // Запрос подтверждения (в реальной системе через интерфейс)
        Serial.println(F("ВНИМАНИЕ: Все данные будут удалены!"));
        // Здесь может быть ожидание подтверждения от пользователя
    }

    // Очистка всей EEPROM
    for (uint16_t i = 0; i < EEPROM_SIZE; i++)
    {
        EEPROM.write(i, 0xFF);
    }

    // Запись сигнатур
    uint32_t signature = STORAGE_SIGNATURE;
    uint32_t version = STORAGE_VERSION;
    uint32_t magic = STORAGE_MAGIC;

    storageWrite(ADDR_SIGNATURE, &signature, sizeof(signature));
    storageWrite(ADDR_VERSION, &version, sizeof(version));
    storageWrite(ADDR_MAGIC, &magic, sizeof(magic));

    // Инициализация структур по умолчанию
    saveDefaultCalibration();

    // Сброс счетчика программ
    uint8_t programCount = 0;
    storageWrite(ADDR_PROGRAM_COUNT, &programCount, sizeof(programCount));

    // Сброс статистики
    SystemStatistics stats = {0};
    saveStatistics(&stats);

    // Сброс пользовательских настроек
    UserSettings userSettings = {
        .displayContrast = 150,
        .displayTimeout = 10,
        .soundVolume = 80,
        .soundEnabled = true,
        .beepOnAction = true,
        .autoSave = true,
        .language = 0, // Русский
        .units = 0,    // мм
        .logRetention = 30,
        .logLevel = LOG_LEVEL_INFO,
        .brightness = 100,
        .showHelp = true,
        .confirmActions = true};
    saveUserSettings(&userSettings);

    // Очистка лога
    clearLog();

    // Расчет и сохранение CRC
    updateCRC();

    // Обновление статуса
    storageStatus.valid = true;
    storageStatus.flags = FLAG_VALID;
    storageStatus.writeCount++;
    storageStatus.freeSpace = storageGetFreeSpace();

    Serial.println(F("Форматирование завершено успешно"));

    // Запись в лог
    logMessage(LOG_INFO, "EEPROM отформатирована", 0);

    return true;
}

// ========== РАБОТА С КАЛИБРОВКОЙ ==========

/**
 * @brief Сохранение калибровочных данных
 * @param cal Указатель на структуру калибровки
 * @return true если сохранение успешно
 */
bool saveCalibration(const SystemCalibration *cal)
{
    if (!cal)
    {
        return false;
    }

    Serial.println(F("Сохранение калибровочных данных..."));

    // Проверка валидности данных
    if (cal->maxHorizontalTravel <= 0 || cal->maxVerticalTravel <= 0)
    {
        Serial.println(F("ОШИБКА: Некорректные калибровочные данные"));
        return false;
    }

    // Сохранение в EEPROM
    bool result = storageWrite(ADDR_CALIBRATION, cal, sizeof(SystemCalibration));

    if (result)
    {
        // Обновление CRC
        updateCRC();

        // Запись в лог
        logMessage(LOG_CALIBRATION, "Калибровка сохранена", cal->maxHorizontalTravel);

        storageStatus.writeCount++;
        Serial.println(F("Калибровка сохранена успешно"));
    }
    else
    {
        storageStatus.errorCount++;
        Serial.println(F("ОШИБКА сохранения калибровки"));
    }

    return result;
}

/**
 * @brief Загрузка калибровочных данных
 * @param cal Указатель на структуру для загрузки
 * @return true если загрузка успешна
 */
bool loadCalibration(SystemCalibration *cal)
{
    if (!cal)
    {
        return false;
    }

    // Чтение из EEPROM
    bool result = storageRead(ADDR_CALIBRATION, cal, sizeof(SystemCalibration));

    if (result)
    {
        // Проверка валидности данных
        if (cal->maxHorizontalTravel <= 0 || cal->maxHorizontalTravel > MAX_HORIZONTAL_TRAVEL ||
            cal->maxVerticalTravel <= 0 || cal->maxVerticalTravel > MAX_VERTICAL_TRAVEL)
        {

            Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: Некорректные калибровочные данные в EEPROM"));

            // Загрузка значений по умолчанию
            saveDefaultCalibration();
            return loadCalibration(cal);
        }

        Serial.println(F("Калибровка загружена успешно"));
        return true;
    }

    Serial.println(F("ОШИБКА загрузки калибровки"));
    return false;
}

/**
 * @brief Сохранение калибровки по умолчанию
 * @return true если сохранение успешно
 */
bool saveDefaultCalibration(void)
{
    SystemCalibration defaultCal = {
        .homePosition = 0,
        .maxHorizontalTravel = MAX_HORIZONTAL_TRAVEL,
        .maxVerticalTravel = MAX_VERTICAL_TRAVEL,
        .tiltSpeed = DEFAULT_TILT_SPEED,
        .levelingSpeed = 20,
        .accelerationTime = 1000,
        .decelerationTime = 1000,
        .safetyMargin = 100,
        .manualOverrideAllowed = true,
        .displayContrast = 150,
        .sensorFilterTime = 100};

    return saveCalibration(&defaultCal);
}

// ========== РАБОТА С ПРОГРАММАМИ ==========

/**
 * @brief Сохранение программы
 * @param program Указатель на программу
 * @param index Индекс программы (0-MAX_PROGRAMS-1)
 * @return true если сохранение успешно
 */
bool saveProgram(const ProgramSettings *program, uint8_t index)
{
    if (!program || index >= MAX_PROGRAMS)
    {
        return false;
    }

    // Проверка валидности программы
    if (program->zoneCount == 0 || program->zoneCount > MAX_ZONES_PER_PROGRAM)
    {
        Serial.println(F("ОШИБКА: Некорректное количество зон в программе"));
        return false;
    }

    // Проверка имени программы
    if (strlen(program->name) == 0 || strlen(program->name) > MAX_PROGRAM_NAME_LENGTH - 1)
    {
        Serial.println(F("ОШИБКА: Некорректное имя программы"));
        return false;
    }

    // Расчет адреса программы в EEPROM
    uint16_t address = ADDR_PROGRAMS + index * sizeof(ProgramSettings);

    Serial.print(F("Сохранение программы #"));
    Serial.print(index);
    Serial.print(F(" '"));
    Serial.print(program->name);
    Serial.println(F("'..."));

    // Сохранение
    bool result = storageWrite(address, program, sizeof(ProgramSettings));

    if (result)
    {
        // Обновление CRC
        updateCRC();

        // Запись в лог
        char logMsg[64];
        snprintf(logMsg, sizeof(logMsg), "Программа сохранена: %s", program->name);
        logMessage(LOG_INFO, logMsg, index);

        storageStatus.writeCount++;
        Serial.println(F("Программа сохранена успешно"));
    }
    else
    {
        storageStatus.errorCount++;
        Serial.println(F("ОШИБКА сохранения программы"));
    }

    return result;
}

/**
 * @brief Загрузка программы
 * @param program Указатель на структуру для загрузки
 * @param index Индекс программы
 * @return true если загрузка успешна
 */
bool loadProgram(ProgramSettings *program, uint8_t index)
{
    if (!program || index >= MAX_PROGRAMS)
    {
        return false;
    }

    // Расчет адреса
    uint16_t address = ADDR_PROGRAMS + index * sizeof(ProgramSettings);

    // Чтение из EEPROM
    bool result = storageRead(address, program, sizeof(ProgramSettings));

    if (result)
    {
        // Проверка валидности данных
        if (program->zoneCount == 0 || program->zoneCount > MAX_ZONES_PER_PROGRAM)
        {
            Serial.print(F("ПРЕДУПРЕЖДЕНИЕ: Программа #"));
            Serial.print(index);
            Serial.println(F(" содержит некорректные данные"));
            return false;
        }

#ifdef DEBUG_STORAGE
        Serial.print(F("Программа #"));
        Serial.print(index);
        Serial.print(F(" '"));
        Serial.print(program->name);
        Serial.println(F("' загружена"));
#endif

        return true;
    }

    Serial.print(F("ОШИБКА загрузки программы #"));
    Serial.println(index);
    return false;
}

/**
 * @brief Получение количества сохраненных программ
 * @return Количество программ
 */
uint8_t getProgramCount(void)
{
    uint8_t count = 0;
    storageRead(ADDR_PROGRAM_COUNT, &count, sizeof(count));
    return count;
}

/**
 * @brief Установка количества программ
 * @param count Новое количество
 * @return true если успешно
 */
bool setProgramCount(uint8_t count)
{
    if (count > MAX_PROGRAMS)
    {
        return false;
    }

    bool result = storageWrite(ADDR_PROGRAM_COUNT, &count, sizeof(count));

    if (result)
    {
        updateCRC();
        storageStatus.writeCount++;
    }

    return result;
}

/**
 * @brief Сохранение всех программ
 * @param programs Массив программ
 * @param count Количество программ
 * @return true если сохранение успешно
 */
bool saveAllPrograms(const ProgramSettings *programs, uint8_t count)
{
    if (!programs || count > MAX_PROGRAMS)
    {
        return false;
    }

    Serial.print(F("Сохранение "));
    Serial.print(count);
    Serial.println(F(" программ..."));

    // Сохранение каждой программы
    for (uint8_t i = 0; i < count; i++)
    {
        if (!saveProgram(&programs[i], i))
        {
            return false;
        }
    }

    // Обновление счетчика
    if (!setProgramCount(count))
    {
        return false;
    }

    Serial.println(F("Все программы сохранены успешно"));
    return true;
}

/**
 * @brief Загрузка настроек по умолчанию
 */
bool loadDefaultSettings(SystemCalibration* cal, ProgramSettings* progs, unsigned char* count)
{
    if (!cal || !progs || !count)
    {
        return false;
    }

    // Калибровка по умолчанию
    cal->homePosition = 0;
    cal->maxHorizontalTravel = MAX_HORIZONTAL_TRAVEL;
    cal->maxVerticalTravel = MAX_VERTICAL_TRAVEL;
    cal->tiltSpeed = DEFAULT_TILT_SPEED;
    cal->levelingSpeed = 50;
    cal->accelerationTime = 1000;
    cal->decelerationTime = 1000;
    cal->safetyMargin = 50;
    cal->manualOverrideAllowed = true;
    cal->displayContrast = 180;
    cal->sensorFilterTime = 100;

    // Программы по умолчанию
    *count = 2; // 2 программы по умолчанию

    // Программа 1
    strncpy(progs[0].name, "Программа 1", MAX_PROGRAM_NAME_LENGTH - 1);
    progs[0].zoneCount = 3;
    progs[0].repeatEnabled = true;
    progs[0].repeatCount = 0; // Бесконечно
    progs[0].currentRepeat = 0;
    progs[0].totalRuntime = 180000; // 3 минуты

    // Зоны для программы 1
    strncpy(progs[0].zones[0].name, "Зона 1", MAX_ZONE_NAME_LENGTH - 1);
    progs[0].zones[0].position = 1000;
    progs[0].zones[0].targetHeight = 500;
    progs[0].zones[0].dipTime = 5000;
    progs[0].zones[0].tiltAngle = 0;
    progs[0].zones[0].waitTime = 1000;
    progs[0].zones[0].enabled = true;
    progs[0].zones[0].motorSpeed = DEFAULT_HORIZONTAL_SPEED;

    strncpy(progs[0].zones[1].name, "Зона 2", MAX_ZONE_NAME_LENGTH - 1);
    progs[0].zones[1].position = 2000;
    progs[0].zones[1].targetHeight = 600;
    progs[0].zones[1].dipTime = 6000;
    progs[0].zones[1].tiltAngle = 10;
    progs[0].zones[1].waitTime = 1500;
    progs[0].zones[1].enabled = true;
    progs[0].zones[1].motorSpeed = DEFAULT_HORIZONTAL_SPEED;

    strncpy(progs[0].zones[2].name, "Зона 3", MAX_ZONE_NAME_LENGTH - 1);
    progs[0].zones[2].position = 3000;
    progs[0].zones[2].targetHeight = 700;
    progs[0].zones[2].dipTime = 7000;
    progs[0].zones[2].tiltAngle = 20;
    progs[0].zones[2].waitTime = 2000;
    progs[0].zones[2].enabled = true;
    progs[0].zones[2].motorSpeed = DEFAULT_HORIZONTAL_SPEED;

    // Порядок зон
    for (int i = 0; i < progs[0].zoneCount; i++)
    {
        progs[0].zoneOrder[i] = i;
    }

    // Программа 2
    strncpy(progs[1].name, "Программа 2", MAX_PROGRAM_NAME_LENGTH - 1);
    progs[1].zoneCount = 2;
    progs[1].repeatEnabled = false;
    progs[1].repeatCount = 1;
    progs[1].currentRepeat = 0;
    progs[1].totalRuntime = 120000; // 2 минуты

    // Зоны для программы 2
    strncpy(progs[1].zones[0].name, "Точка А", MAX_ZONE_NAME_LENGTH - 1);
    progs[1].zones[0].position = 1500;
    progs[1].zones[0].targetHeight = 400;
    progs[1].zones[0].dipTime = 4000;
    progs[1].zones[0].tiltAngle = 5;
    progs[1].zones[0].waitTime = 800;
    progs[1].zones[0].enabled = true;
    progs[1].zones[0].motorSpeed = 40;

    strncpy(progs[1].zones[1].name, "Точка Б", MAX_ZONE_NAME_LENGTH - 1);
    progs[1].zones[1].position = 2500;
    progs[1].zones[1].targetHeight = 450;
    progs[1].zones[1].dipTime = 4500;
    progs[1].zones[1].tiltAngle = 15;
    progs[1].zones[1].waitTime = 1200;
    progs[1].zones[1].enabled = true;
    progs[1].zones[1].motorSpeed = 60;

    // Порядок зон для программы 2
    for (int i = 0; i < progs[1].zoneCount; i++)
    {
        progs[1].zoneOrder[i] = i;
    }

    Serial.println(F("Загружены настройки по умолчанию"));
    return true;
}


/**
 * @brief Загрузка всех программ
 * @param programs Массив для загрузки программ
 * @param count Указатель для сохранения количества
 * @return true если загрузка успешна
 */
bool loadAllPrograms(ProgramSettings *programs, uint8_t *count)
{
    if (!programs || !count)
    {
        return false;
    }

    // Получение количества программ
    *count = getProgramCount();

    if (*count == 0 || *count > MAX_PROGRAMS)
    {
        Serial.println(F("Нет сохраненных программ"));
        return false;
    }

    Serial.print(F("Загрузка "));
    Serial.print(*count);
    Serial.println(F(" программ..."));

    // Загрузка каждой программы
    for (uint8_t i = 0; i < *count; i++)
    {
        if (!loadProgram(&programs[i], i))
        {
            // Если не удалось загрузить, уменьшаем счетчик
            *count = i;
            break;
        }
    }

    Serial.println(F("Все программы загружены успешно"));
    return true;
}

// ========== СИСТЕМНЫЙ ЛОГ ==========

/**
 * @brief Запись сообщения в лог
 * @param type Тип сообщения
 * @param message Текст сообщения
 * @param data Дополнительные данные
 * @return true если запись успешна
 */
bool logMessage(LogType type, const char *message, uint16_t data)
{
    // Проверка уровня логгирования
    if (currentLogLevel == LOG_LEVEL_NONE)
    {
        return true;
    }

    if (currentLogLevel == LOG_LEVEL_ERROR && type != LOG_ERROR && type != LOG_EMERGENCY)
    {
        return true;
    }

    if (currentLogLevel == LOG_LEVEL_WARNING &&
        type != LOG_ERROR && type != LOG_EMERGENCY && type != LOG_WARNING)
    {
        return true;
    }

    // Создание записи лога
    LogEntry entry;
    entry.timestamp = millis() / 1000; // В секундах
    entry.type = type;
    entry.level = (type == LOG_ERROR || type == LOG_EMERGENCY) ? 1 : 0;
    entry.data = data;

    // Копирование сообщения с ограничением длины
    strncpy(entry.message, message, sizeof(entry.message) - 1);
    entry.message[sizeof(entry.message) - 1] = '\0';

    // Расчет адреса
    uint16_t address = ADDR_SYSTEM_LOG + logIndex * sizeof(LogEntry);

    // Запись в EEPROM
    bool result = storageWrite(address, &entry, sizeof(LogEntry));

    if (result)
    {
        // Обновление индекса
        logIndex = (logIndex + 1) % (MAX_LOG_SIZE / sizeof(LogEntry));

        // Обновление счетчика
        if (logEntryCount < (MAX_LOG_SIZE / sizeof(LogEntry)))
        {
            logEntryCount++;
        }

        // Запись индекса в отдельную область для сохранения при перезагрузке
        storageWrite(ADDR_SYSTEM_LOG + MAX_LOG_SIZE, &logIndex, sizeof(logIndex));
        storageWrite(ADDR_SYSTEM_LOG + MAX_LOG_SIZE + 2, &logEntryCount, sizeof(logEntryCount));

        // Обновление CRC
        updateCRC();

        storageStatus.writeCount++;

#ifdef DEBUG_STORAGE
        Serial.print(F("LOG["));
        Serial.print(entry.timestamp);
        Serial.print(F("]: "));
        Serial.println(message);
#endif
    }
    else
    {
        storageStatus.errorCount++;
    }

    return result;
}

/**
 * @brief Запись форматированного сообщения в лог
 * @param type Тип сообщения
 * @param format Форматная строка
 * @param ... Аргументы
 * @return true если запись успешна
 */
bool logMessageF(LogType type, const char *format, ...)
{
    char buffer[128];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    return logMessage(type, buffer, 0);
}

/**
 * @brief Очистка лога
 * @return true если очистка успешна
 */
bool clearLog(void)
{
    Serial.println(F("Очистка системного лога..."));

    // Заполнение области лога значениями 0xFF
    for (uint16_t i = 0; i < MAX_LOG_SIZE; i++)
    {
        EEPROM.write(ADDR_SYSTEM_LOG + i, 0xFF);
    }

    // Сброс индексов
    logIndex = 0;
    logEntryCount = 0;

    // Сохранение индексов
    storageWrite(ADDR_SYSTEM_LOG + MAX_LOG_SIZE, &logIndex, sizeof(logIndex));
    storageWrite(ADDR_SYSTEM_LOG + MAX_LOG_SIZE + 2, &logEntryCount, sizeof(logEntryCount));

    // Обновление CRC
    updateCRC();

    storageStatus.writeCount++;

    // Запись в лог о очистке
    logMessage(LOG_INFO, "Системный лог очищен", 0);

    Serial.println(F("Лог очищен успешно"));
    return true;
}

// ========== СТАТИСТИКА ==========

/**
 * @brief Сохранение статистики
 * @param stats Указатель на статистику
 * @return true если сохранение успешно
 */
bool saveStatistics(const SystemStatistics *stats)
{
    if (!stats)
    {
        return false;
    }

    bool result = storageWrite(ADDR_STATISTICS, stats, sizeof(SystemStatistics));

    if (result)
    {
        updateCRC();
        storageStatus.writeCount++;
    }

    return result;
}

/**
 * @brief Загрузка статистики
 * @param stats Указатель на структуру для загрузки
 * @return true если загрузка успешна
 */
bool loadStatistics(SystemStatistics *stats)
{
    if (!stats)
    {
        return false;
    }

    return storageRead(ADDR_STATISTICS, stats, sizeof(SystemStatistics));
}

/**
 * @brief Инкремент счетчика выполненных программ
 * @return true если успешно
 */
bool incrementProgramExecutions(void)
{
    SystemStatistics stats;

    if (!loadStatistics(&stats))
    {
        return false;
    }

    stats.programExecutions++;
    stats.totalUptime = millis() / 1000;

    return saveStatistics(&stats);
}

// ========== НИЗКОУРОВНЕВЫЕ ФУНКЦИИ ==========

/**
 * @brief Чтение данных из EEPROM
 * @param address Адрес начала
 * @param data Указатель на буфер
 * @param size Размер данных
 * @return true если чтение успешно
 */
bool storageRead(uint16_t address, void *data, uint16_t size)
{
    if (address + size > EEPROM_SIZE || !data)
    {
        return false;
    }

    uint8_t *ptr = (uint8_t *)data;

    for (uint16_t i = 0; i < size; i++)
    {
        ptr[i] = EEPROM.read(address + i);
    }

    // Обновление статистики износа сектора
    uint8_t sector = address / 256;
    if (sector < 16)
    {
        sectorWear[sector]++;
    }

    return true;
}

/**
 * @brief Запись данных в EEPROM
 * @param address Адрес начала
 * @param data Указатель на данные
 * @param size Размер данных
 * @return true если запись успешна
 */
bool storageWrite(uint16_t address, const void *data, uint16_t size)
{
    if (address + size > EEPROM_SIZE || !data)
    {
        return false;
    }

    const uint8_t *ptr = (const uint8_t *)data;

    // Проверка, отличаются ли данные
    bool needsWrite = false;
    for (uint16_t i = 0; i < size; i++)
    {
        if (EEPROM.read(address + i) != ptr[i])
        {
            needsWrite = true;
            break;
        }
    }

    if (!needsWrite)
    {
        return true; // Данные уже совпадают
    }

    // Запись данных
    for (uint16_t i = 0; i < size; i++)
    {
        EEPROM.write(address + i, ptr[i]);
    }

    // Обновление статистики износа
    uint8_t sector = address / 256;
    if (sector < 16)
    {
        sectorWear[sector]++;
    }

    // Установка флага "грязных" данных
    storageStatus.flags |= FLAG_DIRTY;

    return true;
}

/**
 * @brief Расчет контрольной суммы CRC32
 * @param data Указатель на данные (NULL для расчета по всей EEPROM)
 * @param size Размер данных
 * @return CRC32
 */
uint32_t calculateCRC(const void *data, uint16_t size)
{
    static const uint32_t crc_table[256] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
        0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
        0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
        0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
        0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
        0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
        0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
        0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
        0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
        0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
        0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
        0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
        0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
        0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
        0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
        0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
        0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
        0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
        0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
        0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
        0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
        0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
        0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
        0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
        0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
        0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
        0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
        0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
        0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
        0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
        0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
        0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
        0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
        0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
        0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
        0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
        0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
        0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
        0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
        0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
        0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
        0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *bytes;
    uint16_t length;

    if (data)
    {
        bytes = (const uint8_t *)data;
        length = size;
    }
    else
    {
        // Расчет по всей EEPROM до адреса CRC
        length = ADDR_CRC;
        bytes = NULL;
    }

    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t byte;

        if (data)
        {
            byte = bytes[i];
        }
        else
        {
            byte = EEPROM.read(i);
        }

        crc = (crc >> 8) ^ crc_table[(crc ^ byte) & 0xFF];
    }

    return crc ^ 0xFFFFFFFF;
}

/**
 * @brief Обновление контрольной суммы в EEPROM
 * @return true если успешно
 */
bool updateCRC(void)
{
    uint32_t crc = calculateCRC(NULL, ADDR_CRC);
    return storageWrite(ADDR_CRC, &crc, sizeof(crc));
}

// ========== УТИЛИТЫ ==========

/**
 * @brief Получение свободного места в EEPROM
 * @return Свободное место в байтах
 */
uint16_t storageGetFreeSpace(void)
{
    // Простой расчет свободного места
    // В реальной системе нужно отслеживать используемые блоки
    uint16_t used = ADDR_CRC + 4; // До CRC включительно

    // Добавляем размер лога (используется циклически)
    used += MAX_LOG_SIZE;

    // Добавляем размер программ
    uint8_t programCount = getProgramCount();
    used += programCount * sizeof(ProgramSettings);

    return EEPROM_SIZE - used;
}

// ========== ОТЛАДКА ==========

#ifdef DEBUG_STORAGE
/**
 * @brief Вывод статистики хранилища
 */
void printStorageStats(void)
{
    Serial.println(F("\n=== СТАТИСТИКА ХРАНИЛИЩА ==="));

    Serial.print(F("Инициализировано: "));
    Serial.println(storageStatus.initialized ? "Да" : "Нет");

    Serial.print(F("Валидность данных: "));
    Serial.println(storageStatus.valid ? "Да" : "Нет");

    Serial.print(F("Флаги: 0x"));
    Serial.println(storageStatus.flags, HEX);

    Serial.print(F("Количество записей: "));
    Serial.println(storageStatus.writeCount);

    Serial.print(F("Количество ошибок: "));
    Serial.println(storageStatus.errorCount);

    Serial.print(F("Свободное место: "));
    Serial.print(storageStatus.freeSpace);
    Serial.println(F(" байт"));

    Serial.print(F("Количество программ: "));
    Serial.println(getProgramCount());

    Serial.print(F("Записей в логе: "));
    Serial.println(logEntryCount);

    Serial.println(F("Износ секторов EEPROM:"));
    for (uint8_t i = 0; i < 16; i++)
    {
        if (sectorWear[i] > 0)
        {
            Serial.print(F("  Сектор "));
            Serial.print(i);
            Serial.print(F(": "));
            Serial.print(sectorWear[i]);
            Serial.println(F(" записей"));
        }
    }

    Serial.println(F("=============================\n"));
}

/**
 * @brief Дамп области EEPROM
 * @param start Начальный адрес
 * @param length Длина дампа
 */
void dumpStorage(uint16_t start, uint16_t length)
{
    if (start + length > EEPROM_SIZE)
    {
        length = EEPROM_SIZE - start;
    }

    Serial.println(F("\n=== ДАМП EEPROM ==="));
    Serial.print(F("Адрес: 0x"));
    Serial.print(start, HEX);
    Serial.print(F(" - 0x"));
    Serial.println(start + length - 1, HEX);

    for (uint16_t i = 0; i < length; i += 16)
    {
        // Адрес
        Serial.print(F("0x"));
        if (start + i < 0x1000)
            Serial.print('0');
        if (start + i < 0x0100)
            Serial.print('0');
        if (start + i < 0x0010)
            Serial.print('0');
        Serial.print(start + i, HEX);
        Serial.print(F(": "));

        // Данные в hex
        for (uint8_t j = 0; j < 16; j++)
        {
            if (i + j < length)
            {
                uint8_t value = EEPROM.read(start + i + j);
                if (value < 0x10)
                    Serial.print('0');
                Serial.print(value, HEX);
                Serial.print(' ');
            }
            else
            {
                Serial.print(F("   "));
            }

            if (j == 7)
                Serial.print(F(" "));
        }

        Serial.print(F(" "));

        // Данные в ASCII
        for (uint8_t j = 0; j < 16; j++)
        {
            if (i + j < length)
            {
                uint8_t value = EEPROM.read(start + i + j);
                if (value >= 32 && value <= 126)
                {
                    Serial.write(value);
                }
                else
                {
                    Serial.print('.');
                }
            }
        }

        Serial.println();
    }

    Serial.println(F("===================\n"));
}
#endif