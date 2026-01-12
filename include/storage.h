/**
 * @file storage.h
 * @brief Модуль хранения данных в EEPROM: программы, настройки, калибровка, логи
 * @version 4.0
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
// #include <EEPROM.h>
#include "../include/config.h"

// ========== КОНСТАНТЫ ХРАНИЛИЩА ==========

// Структура EEPROM
#define EEPROM_SIZE 4096 // Размер EEPROM Arduino Mega

// Сигнатуры для проверки целостности данных
#define STORAGE_SIGNATURE 0xAA55AA55UL
#define STORAGE_VERSION 0x00040000UL // Версия 4.0.0
#define STORAGE_MAGIC 0x54454C46UL   // "TELF" в hex

// Адреса в EEPROM
#define ADDR_SIGNATURE 0           // 4 байта: сигнатура
#define ADDR_VERSION 4             // 4 байта: версия структуры
#define ADDR_MAGIC 8               // 4 байта: магическое число
#define ADDR_CALIBRATION 12        // Размер: sizeof(SystemCalibration)
#define ADDR_PROGRAM_COUNT 100     // 1 байт: количество программ
#define ADDR_PROGRAMS 101          // Программы: MAX_PROGRAMS * sizeof(ProgramSettings)
#define ADDR_ZONE_CALIBRATION 1500 // Калибровка зон: MAX_ZONES * sizeof(ZoneCalibration)
#define ADDR_SYSTEM_LOG 2000       // Лог системы: 1024 байта
#define ADDR_STATISTICS 3024       // Статистика: sizeof(SystemStatistics)
#define ADDR_USER_SETTINGS 3100    // Пользовательские настройки
#define ADDR_CRC 4092              // 4 байта: CRC32 всех данных

// Максимальные размеры
#define MAX_PROGRAM_SIZE (MAX_PROGRAMS * sizeof(ProgramSettings))
#define MAX_ZONE_CALIB_SIZE (MAX_ZONES * sizeof(ZoneCalibration))
#define MAX_LOG_SIZE 1024

// Флаги валидности
#define FLAG_VALID 0x01
#define FLAG_DIRTY 0x02
#define FLAG_CORRUPTED 0x04
#define FLAG_BACKUP 0x08

// Типы записей лога
typedef enum
{
    LOG_INFO,            // Информационное сообщение
    LOG_WARNING,         // Предупреждение
    LOG_ERROR,           // Ошибка
    LOG_EMERGENCY,       // Аварийная остановка
    LOG_PROGRAM_START,   // Запуск программы
    LOG_PROGRAM_END,     // Завершение программы
    LOG_CALIBRATION,     // Калибровка
    LOG_SETTINGS_CHANGE, // Изменение настроек
    LOG_DIAGNOSTICS      // Диагностика неисправностей
} LogType;

// Уровни логгирования
typedef enum
{
    LOG_LEVEL_NONE,    // Без логов
    LOG_LEVEL_ERROR,   // Только ошибки
    LOG_LEVEL_WARNING, // Ошибки и предупреждения
    LOG_LEVEL_INFO,    // Вся информация
    LOG_LEVEL_DEBUG    // Отладочная информация
} LogLevel;

// Структура записи лога
typedef struct
{
    uint32_t timestamp; // Временная метка (секунды с запуска)
    LogType type;       // Тип записи
    uint8_t level;      // Уровень важности
    char message[48];   // Сообщение (ограничено для экономии места)
    uint16_t data;      // Дополнительные данные
} LogEntry;

// Структура калибровки зоны
typedef struct
{
    uint8_t zoneId;        // ID зоны
    int32_t position;      // Калиброванная позиция
    int32_t height;        // Калиброванная высота
    uint8_t accuracy;      // Точность калибровки (1-100)
    uint32_t timestamp;    // Время калибровки
    bool valid;            // Валидны ли данные
    char operatorName[16]; // Имя оператора
} ZoneCalibration;

// Структура статистики системы
typedef struct
{
    uint32_t totalUptime;                       // Общее время работы (секунды)
    uint32_t programExecutions;                 // Количество выполненных программ
    uint32_t emergencyStops;                    // Количество аварийных остановок
    uint32_t motorStarts;                       // Количество запусков двигателей
    uint32_t sensorReads;                       // Количество чтений датчиков
    uint32_t eepromWrites;                      // Количество записей в EEPROM
    uint32_t lastMaintenance;                   // Время последнего обслуживания
    uint32_t errorCount;                        // Общее количество ошибок
    uint16_t zoneVisits[MAX_ZONES_PER_PROGRAM]; // Посещения зон
    float totalDistance;                        // Общий пройденный путь (метры)
    float totalHeight;                          // Общая высота подъема (метры)
} SystemStatistics;

// Структура пользовательских настроек
typedef struct
{
    uint8_t displayContrast; // Контраст дисплея
    uint8_t displayTimeout;  // Таймаут отключения дисплея (минуты)
    uint8_t soundVolume;     // Громкость звука (0-100)
    bool soundEnabled;       // Включен ли звук
    bool beepOnAction;       // Звук при действиях
    bool autoSave;           // Автосохранение при изменении
    uint8_t language;        // Язык интерфейса (0-русский, 1-английский)
    uint8_t units;           // Единицы измерения (0-мм, 1-см, 2-м)
    uint16_t logRetention;   // Время хранения логов (дней)
    LogLevel logLevel;       // Уровень логгирования
    uint8_t brightness;      // Яркость подсветки
    bool showHelp;           // Показывать подсказки
    bool confirmActions;     // Запрашивать подтверждение действий
} UserSettings;
// extern UserSettings userSettings;

// Структура состояния хранилища
typedef struct
{
    bool initialized;    // Инициализировано ли хранилище
    bool valid;          // Валидны ли данные
    uint8_t flags;       // Флаги состояния
    uint32_t writeCount; // Счетчик записей
    uint32_t errorCount; // Счетчик ошибок
    uint16_t freeSpace;  // Свободное место (байт)
    uint32_t lastBackup; // Время последней резервной копии
} StorageStatus;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация и управление
bool storageInit(void);
bool storageFormat(bool force);
bool storageCheckIntegrity(void);
bool storageRepair(void);
StorageStatus storageGetStatus(void);
void storagePrintStatus(void);

// Работа с калибровкой
bool saveCalibration(const SystemCalibration *cal);
bool loadCalibration(SystemCalibration *cal);
bool saveDefaultCalibration(void);
bool resetCalibration(void);

// Работа с программами
bool saveProgram(const ProgramSettings *program, uint8_t index);
bool loadProgram(ProgramSettings *program, uint8_t index);
bool deleteProgram(uint8_t index);
bool copyProgram(uint8_t srcIndex, uint8_t dstIndex);
bool moveProgram(uint8_t fromIndex, uint8_t toIndex);
uint8_t getProgramCount(void);
bool setProgramCount(uint8_t count);
bool saveAllPrograms(const ProgramSettings *programs, uint8_t count);
bool loadAllPrograms(ProgramSettings *programs, uint8_t *count);

// Калибровка зон
bool saveZoneCalibration(const ZoneCalibration *zoneCal);
bool loadZoneCalibration(ZoneCalibration *zoneCal, uint8_t zoneId);
bool deleteZoneCalibration(uint8_t zoneId);
bool saveAllZoneCalibrations(const ZoneCalibration *zones, uint8_t count);
bool loadAllZoneCalibrations(ZoneCalibration *zones, uint8_t *count);

// Системный лог
bool logMessage(LogType type, const char *message, uint16_t data);
bool logMessageF(LogType type, const char *format, ...);
bool clearLog(void);
bool getLogEntry(LogEntry *entry, uint16_t index);
uint16_t getLogEntryCount(void);
bool exportLog(char *buffer, uint16_t bufferSize);
bool setLogLevel(LogLevel level);
LogLevel getLogLevel(void);

// Статистика
bool saveStatistics(const SystemStatistics *stats);
bool loadStatistics(SystemStatistics *stats);
bool updateStatistics(void (*updateFunction)(SystemStatistics *));
bool resetStatistics(bool keepUptime);
bool incrementProgramExecutions(void);
bool incrementEmergencyStops(void);
bool incrementMotorStarts(void);
bool incrementSensorReads(void);

// Пользовательские настройки
bool saveUserSettings(const UserSettings *settings);
bool loadUserSettings(UserSettings *settings);
bool resetUserSettings(void);

// Резервное копирование и восстановление
bool createBackup(uint8_t *buffer, uint16_t *size);
bool restoreBackup(const uint8_t *buffer, uint16_t size);
bool exportSettings(char *jsonBuffer, uint16_t bufferSize);
bool importSettings(const char *jsonBuffer);

// Низкоуровневые функции
bool storageRead(uint16_t address, void *data, uint16_t size);
bool storageWrite(uint16_t address, const void *data, uint16_t size);
bool storageUpdate(uint16_t address, const void *data, uint16_t size);
bool storageClear(uint16_t address, uint16_t size);
uint32_t calculateCRC(const void *data, uint16_t size);
bool verifyCRC(void);
bool updateCRC(void);

// Утилиты
void storageDefragment(void);
uint16_t storageGetFreeSpace(void);
bool storageIsFull(void);
void storagePrintMap(void);

// Отладка
#ifdef DEBUG_STORAGE
void printStorageStats(void);
void dumpStorage(uint16_t start, uint16_t length);
void printProgramInfo(uint8_t index);
void printCalibrationInfo(void);
#endif

// Объявление внешних переменных
extern SystemCalibration calibration;
extern ProgramSettings programs[];

extern uint8_t programCount;

#endif // STORAGE_H