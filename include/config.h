/**
 * @file config.h
 * @brief Конфигурация системы управления тельферами
 * @version 4.0
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// ========== ВЕРСИЯ СИСТЕМЫ ==========
#define SYSTEM_VERSION_MAJOR 4
#define SYSTEM_VERSION_MINOR 0
#define SYSTEM_VERSION_PATCH 0

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define SYSTEM_VERSION_STRING TOSTRING(SYSTEM_VERSION_MAJOR) "." TOSTRING(SYSTEM_VERSION_MINOR) "." TOSTRING(SYSTEM_VERSION_PATCH)

// ========== РЕЖИМЫ ОТЛАДКИ ==========
// Раскомментировать для включения отладочного вывода
// #define DEBUG_MODE
// #define DEBUG_SENSORS
// #define DEBUG_MOTORS
// #define DEBUG_DISPLAY

// ========== КОНСТАНТЫ СИСТЕМЫ ==========

// Временные константы (в миллисекундах)
#define SENSOR_UPDATE_INTERVAL 100 // Обновление датчиков каждые 100 мс
#define DISPLAY_UPDATE_INTERVAL 50 // Обновление дисплея каждые 50 мс (20 FPS)
#define ENCODER_DEBOUNCE_TIME 50   // Время антидребезга энкодера
#define MOTOR_COMMAND_INTERVAL 20  // Интервал команд двигателям
#define SAFETY_CHECK_INTERVAL 100  // Проверка безопасности каждые 100 мс
#define TARGET_LOOP_TIME_US 10000  // Целевое время цикла 10 мс (100 Гц)

// Лимиты системы
#define MAX_PROGRAMS 1             // Максимальное количество программ
#define MAX_ZONES_PER_PROGRAM 10   // Максимальное количество зон в программе
#define MAX_ZONE_NAME_LENGTH 8    // Длина имени зоны
#define MAX_PROGRAM_NAME_LENGTH 10 // Длина имени программы

// Геометрические ограничения (в миллиметрах)
#define MAX_HORIZONTAL_TRAVEL 10000 // Максимальное горизонтальное перемещение
#define MAX_VERTICAL_TRAVEL 5000    // Максимальное вертикальное перемещение
#define MIN_SAFE_HEIGHT 100         // Минимальная безопасная высота
#define MAX_TILT_ANGLE 30           // Максимальный угол наклона (%)

// -------------------- НАКЛОН ГРУЗА (ступенчатый) --------------------
// Низкая сторона: 0 = V1 (левый), 1 = V2 (правый)  <-- Поменять тут, если перепутали подключение
#define TILT_LOW_SIDE_IS_V2   0

// Разница высот между сторонами (мм)
#define TILT_DIFF_MM          100

// Ступени погружения/подъема (мм относительно предыдущего уровня)
#define DIP_STEP1_MM          60
#define DIP_STEP2_MM          60
#define LIFT_STEP1_MM         60
#define LIFT_STEP2_MM         60

// Паузы (мс)
#define DIP_FILL_PAUSE_MS     15000   // пауза для заполнения труб
#define LIFT_DRAIN_PAUSE_MS   45000   // пауза для стекания жидкости

// Сколько ступеней использовать (2 достаточно )
#define TILT_STEPS_COUNT      2

// Допуски позиционирования (в миллиметрах)
#define HORIZONTAL_TOLERANCE 10 // Допуск по горизонтали
#define VERTICAL_TOLERANCE 5    // Допуск по вертикали
#define TILT_TOLERANCE 2        // Допуск по наклону

// Скорости по умолчанию (% от максимальной)
#define DEFAULT_HORIZONTAL_SPEED 50
#define DEFAULT_VERTICAL_SPEED 40
#define DEFAULT_TILT_SPEED 30
#define MAX_MOTOR_SPEED 100
#define MIN_MOTOR_SPEED 10 // Минимальная скорость для предотвращения залипания

// Количество диагностических тестов
#define MAX_DIAGNOSTIC_TESTS 3

// ========== КОНФИГУРАЦИЯ ОБОРУДОВАНИЯ ==========

// Пины дисплея GMG12864-06D
#define DISPLAY_SCL_PIN 52  // SPI Clock
#define DISPLAY_SDA_PIN 51  // SPI Data
#define DISPLAY_CS_PIN 10   // Chip Select
#define DISPLAY_DC_PIN 8    // Data/Command
#define DISPLAY_RESET_PIN 9 // Reset

// Пины энкодера (KY-040 или аналогичный)
#define ENCODER_CLK_PIN 3 // CLK (Channel A)
#define ENCODER_DT_PIN 4  // DT (Channel B)
#define ENCODER_SW_PIN 5  // SW (Кнопка)

// Пины безопасности
#define EMERGENCY_STOP_PIN 2 // Нормально-разомкнутая аварийная кнопка
#define BUZZER_PIN 12        // Пьезоизлучатель
#define LED_STATUS_PIN 13    // Светодиод статуса

// Пины ультразвуковых датчиков HC-SR04
#define US1_TRIG_PIN 22
#define US1_ECHO_PIN 23
#define US2_TRIG_PIN 24
#define US2_ECHO_PIN 25

// Пины управления RS-485
#define RS485_RE_DE_PIN 6 // Управление направлением (RE/DE)
#define RS485_TX_ENABLE 7 // Разрешение передачи (опционально)

// Пины дополнительных кнопок управления
#define BTN_STOP_PIN 26
#define BTN_START_PIN 27
#define BTN_FORWARD_PIN 28
#define BTN_BACKWARD_PIN 29
#define BTN_UP_PIN 30
#define BTN_DOWN_PIN 31

// Пины реле/дополнительных выходов
#define RELAY_1_PIN 32
#define RELAY_2_PIN 33
#define OUTPUT_1_PIN 34
#define OUTPUT_2_PIN 35

// Настройки последовательных портов
#define SERIAL_DEBUG_BAUD 115200 // Отладочный порт
#define SERIAL_LASER1_BAUD 9600  // Левый лазерный дальномер
#define SERIAL_LASER2_BAUD 9600  // Правый лазерный дальномер
#define SERIAL_RS485_BAUD 9600   // Частотные преобразователи

// ========== СТРУКТУРЫ ДАННЫХ ==========

/**
 * @brief Параметры зоны обработки
 */
typedef struct __attribute__((packed))
{
    char name[MAX_ZONE_NAME_LENGTH]; // Название зоны
    int16_t position;                // Горизонтальная позиция (мм)
    int16_t targetHeight;            // Целевая высота (мм)
    uint16_t dipTime;                // Время погружения (сек) !(мс)
    uint8_t tiltAngle;               // Угол наклона (0-100%)
    uint8_t waitTime;                // Время ожидания после подъема (сек) !(мс)
    uint8_t motorSpeed;              // Скорость движения к зоне (%)
    bool enabled;                    // Зона включена
} ZoneSettings;

/**
 * @brief Настройки программы
 */
typedef struct __attribute__((packed))
{
    char name[MAX_PROGRAM_NAME_LENGTH];        // Название программы
    ZoneSettings zones[MAX_ZONES_PER_PROGRAM]; // Массив зон
    uint8_t zoneCount;                         // Количество зон
    uint8_t zoneOrder[MAX_ZONES_PER_PROGRAM];  // Порядок прохождения зон
    bool repeatEnabled;                        // Повтор программы
    uint8_t repeatCount;                       // Количество повторений (0 = бесконечно)
    uint8_t currentRepeat;                     // Текущее повторение
    uint32_t totalRuntime;                     // Общее время выполнения (мс)
} ProgramSettings;

/**
 * @brief Калибровочные параметры системы
 */
typedef struct
{
    int32_t homePosition;        // Начальная позиция (мм)
    int32_t maxHorizontalTravel; // Максимальное горизонтальное перемещение (мм)
    int32_t maxVerticalTravel;   // Максимальное вертикальное перемещение (мм)
    uint8_t tiltSpeed;           // Скорость наклона (%)
    uint8_t levelingSpeed;       // Скорость выравнивания (%)
    uint16_t accelerationTime;   // Время разгона (мс)
    uint16_t decelerationTime;   // Время торможения (мс)
    int16_t safetyMargin;        // Запас безопасности (мм)
    bool manualOverrideAllowed;  // Разрешение ручного управления
    uint8_t displayContrast;     // Контраст дисплея (0-255)
    uint16_t sensorFilterTime;   // Время фильтрации датчиков (мс)
} SystemCalibration;

/**
 * @brief Состояние системы
 */
typedef struct
{
    int32_t telfer1Pos;       // Позиция тельфера 1 (мм)
    int32_t telfer2Pos;       // Позиция тельфера 2 (мм)
    int32_t cargoHeight1;     // Высота груза 1 (мм)
    int32_t cargoHeight2;     // Высота груза 2 (мм)
    int32_t avgHorizontalPos; // Средняя горизонтальная позиция
    int32_t avgHeight;        // Средняя высота
    int32_t tiltDifference;   // Разница высот (для наклона)
    uint32_t uptime;          // Время работы системы (мс)
    float batteryVoltage;     // Напряжение питания (В)
    int8_t temperature;       // Температура (°C)
    bool sensorsValid;        // Данные датчиков валидны
    bool motorsEnabled;       // Двигатели включены
    bool emergencyActive;     // Аварийная остановка активна
} SystemStatus;

/**
 * @brief Флаги состояния системы
 */
typedef struct
{
    bool isPaused;           // Программа на паузе
    bool isEmergency;        // Аварийный режим
    bool systemInitialized;  // Система инициализирована
    bool displayInitialized; // Дисплей инициализирован
    bool motorsEnabled;      // Двигатели разрешены
    bool sensorsActive;      // Датчики активны
    bool programRunning;     // Программа выполняется
    bool manualMode;         // Ручной режим
    bool errorAutoReset;     // Атоматический сброс ошибок
} SystemFlags;

/**
 * @brief Типы ошибок системы
 */
typedef enum
{
    ERROR_NONE = 0,           // Нет ошибок
    ERROR_SENSOR_LASER1,      // Ошибка лазерного датчика 1
    ERROR_SENSOR_LASER2,      // Ошибка лазерного датчика 2
    ERROR_SENSOR_US1,         // Ошибка УЗ датчика 1
    ERROR_SENSOR_US2,         // Ошибка УЗ датчика 2
    ERROR_MOTOR_H1,           // Ошибка горизонтального двигателя 1
    ERROR_MOTOR_H2,           // Ошибка горизонтального двигателя 2
    ERROR_MOTOR_V1,           // Ошибка вертикального двигателя 1
    ERROR_MOTOR_V2,           // Ошибка вертикального двигателя 2
    ERROR_RS485_COMM,         // Ошибка связи RS-485
    ERROR_OVERLOAD,           // Перегрузка
    ERROR_LIMIT_SWITCH,       // Концевой выключатель
    ERROR_POSITION_DEVIATION, // Отклонение позиции
    ERROR_EMERGENCY_STOP,     // Аварийная остановка
    ERROR_MEMORY,             // Ошибка памяти
    ERROR_DISPLAY,            // Ошибка дисплея
    ERROR_COUNT               // Количество ошибок
} ErrorType;

/**
 * @brief Данные системы
 */
typedef struct
{
    uint8_t currentZone;    // Текущая зона
    uint8_t currentProgram; // Текущая программа
    uint8_t programCount;   // Количество программ
    uint8_t menuIndex;      // Индекс в меню
    uint8_t menuScroll;     // Смещение прокрутки меню
    ErrorType activeError;    // Активная ошибка
    char errorMessage[32];  // Сообщение об ошибке
} SystemData;

/**
 * @brief Временные метки системы
 */
typedef struct
{
    uint32_t startupTime;       // Время запуска системы
    uint32_t stateStartTime;    // Время начала текущего состояния
    uint32_t dipStartTime;      // Время начала погружения
    uint32_t pauseStartTime;    // Время начала паузы
    uint32_t errorTime;         // Время возникновения ошибки
    uint32_t lastSensorUpdate;  // Последнее обновление датчиков
    uint32_t lastDisplayUpdate; // Последнее обновление дисплея
    uint32_t lastEncoderCheck;  // Последняя проверка энкодера
    uint32_t lastMotorCommand;  // Последняя команда двигателям
    uint32_t lastSafetyCheck;   // Последняя проверка безопасности
} SystemTiming;

// ========== ПЕРЕЧИСЛЕНИЯ ==========

/**
 * @brief Состояния системы
 */
typedef enum
{
    STATE_BOOT,              // Загрузка системы
    STATE_IDLE,              // Ожидание (главный экран)
    STATE_MENU_NAVIGATION,   // Навигация по меню
    STATE_PROGRAM_SELECTION, // Выбор программы
    STATE_PROGRAM_EDIT,      // Редактирование программы
    STATE_ZONE_EDIT,         // Редактирование зоны
    STATE_AUTO_RUNNING,      // Автоматическое выполнение
    STATE_MANUAL_CONTROL,    // Ручное управление
    STATE_CALIBRATION,       // Калибровка
    STATE_SETTINGS,          // Настройки системы
    STATE_MONITOR,           // Мониторинг системы
    STATE_DIAGNOSTICS,       // Диагностика
    STATE_ERROR,             // Ошибка системы
    STATE_EMERGENCY,         // Аварийная остановка
    STATE_COUNT              // Количество состояний
} SystemState;


/**
 * @brief Уровни меню
 */
typedef enum
{
    MENU_MAIN,        // Главное меню
    MENU_AUTO_MODE,   // Автоматический режим
    MENU_MANUAL_MODE, // Ручной режим
    MENU_PROGRAMS,    // Программы
    MENU_CALIBRATION, // Калибровка
    MENU_SETTINGS,    // Настройки
    MENU_INFO,        // Информация
    MENU_DIAGNOSTICS, // Диагностика
    MENU_LEVEL_COUNT  // Количество уровней меню
} MenuLevel;

/**
 * @brief Типы двигателей
 */
typedef enum
{
    MOTOR_HORIZONTAL_LEFT,  // Горизонтальный левый
    MOTOR_HORIZONTAL_RIGHT, // Горизонтальный правый
    MOTOR_VERTICAL_LEFT,    // Вертикальный левый
    MOTOR_VERTICAL_RIGHT,   // Вертикальный правый
    MOTOR_COUNT             // Количество двигателей
} MotorType;

/**
 * @brief Направления движения
 */
typedef enum
{
    DIRECTION_FORWARD,  // Вперед/Вверх
    DIRECTION_BACKWARD, // Назад/Вниз
    DIRECTION_STOP      // Стоп
} Direction;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация
void initPins(void);
bool initDisplay(void *display);
void initSensors(void);
void initMotors(void);
void initUI(void);
bool performSelfTest(void);

// Обработка ошибок
void setError(ErrorType error, const char *message);
void clearError(void);
bool hasError(void);
const char *getErrorMessage(ErrorType error);

// Безопасность
void emergencyStop(void);
bool resetEmergency(void);
bool checkSafetyLimits(const SystemStatus *status, const SystemCalibration *cal);

// Утилиты
void beep(uint16_t frequency, uint16_t duration);
void beepSequence(uint8_t count, uint16_t frequency, uint16_t duration);
int32_t constrainValue(int32_t value, int32_t minVal, int32_t maxVal);
float constrainValueF(float value, float minVal, float maxVal);
int32_t mapValue(int32_t value, int32_t fromMin, int32_t fromMax, int32_t toMin, int32_t toMax);

/**
 * @brief Результат диагностического теста
 */
typedef struct {
    uint8_t testId;        // ID теста
    bool passed;           // Результат (успех/неудача)
    char message[32];      // Сообщение
    uint32_t diagnosticsTime;    // Время проведения теста
} DiagnosticsResult;


// /**
//  * @brief Настройки пользователя
//  */
// typedef struct {
//     uint8_t displayContrast;    // Контраст дисплея (0-255)
//     bool soundEnabled;          // Звуковые сигналы
//     uint8_t language;           // Язык интерфейса
//     bool autoStart;             // Автозапуск программы
//     uint16_t screenTimeout;     // Таймаут экрана (мс)
//     uint8_t brightness;         // Яркость подсветки
// } UserSettings;

// 1 = при опускании измеряемое расстояние УЗ уменьшается (частый случай)
#define US_DISTANCE_DECREASES_WHEN_LOWERING  1

static inline void computeTiltTargets(int32_t baseMm, int32_t &v1Target, int32_t &v2Target)
{
#if TILT_LOW_SIDE_IS_V2
    // V2 ниже => V2 цель "глубже" (или "ниже") на TILT_DIFF_MM
    v2Target = baseMm;
    v1Target = baseMm + TILT_DIFF_MM;
#else
    // V1 ниже
    v1Target = baseMm;
    v2Target = baseMm + TILT_DIFF_MM;
#endif
}


// Отладка
#ifdef DEBUG_MODE
void debugPrint(const char *format, ...);
void debugPrintSystemStatus(void);
#else
#define debugPrint(...)
#define debugPrintSystemStatus()
#endif

#endif // CONFIG_H