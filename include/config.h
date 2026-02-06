/**
 * @file config.h
 * @brief Аппаратная конфигурация и основные типы данных проекта telfer-control-system.
 *
 * ВАЖНО:
 *  - Все значения длин/позиций в проекте хранятся в миллиметрах (mm).
 *  - Arduino Mega 2560 имеет всего 8 KB RAM → избегаем больших буферов/структур.
 *  - Все строки для UI стараемся держать во Flash через F("...").
 *
 * Подключение (кратко):
 *  - Дисплей GMG12864-06D (ST7565R, SPI): SCK=52, MOSI=51, CS=10, DC=8, RST=9, питание 3.3V.
 *  - Энкодер: CLK=3, DT=4, SW=5 (INPUT_PULLUP).
 *  - RS-485 (MAX485): Serial1 (TX1=18, RX1=19), DE/RE=6.
 *  - Лазеры (UART 3.3V): Laser1=Serial2 (RX2=17, TX2=16), Laser2=Serial3 (RX3=15, TX3=14).
 *  - УЗ датчики HC-SR04 (5V): (22/23) и (24/25).
 *  - Аварийная кнопка E-Stop: pin 2 (лучше NC → INPUT_PULLUP).
 *  - Концевики горизонтали (NC): 4 входа (36..39).
 *
 * Примечание по уровню сигналов:
 *  - Mega = 5V логика. Лазеры/дисплей = 3.3V.
 *  - Для линий Mega->3.3V устройств нужен понижающий уровень (делитель/level shifter).
 */

#pragma once

#include <Arduino.h>   // IMPORTANT: provides A0..A15 and pin macros in Arduino core

#include <stdint.h>
#include <stdbool.h>

// ----------------------------- Общие параметры -----------------------------

#define FW_VERSION "5.0.0"
static constexpr uint8_t TELFER_COUNT = 2;
static constexpr uint8_t MAX_ZONES    = 10;
static constexpr uint8_t PROGRAM_SLOTS = 4;     // количество слотов программ в EEPROM

// Интервалы (мс)
// UI опрашивает энкодер и рисует дисплей. Для механического EC11 50 мс часто слишком редко
// (при повороте импульсы теряются и кажется, что "не листает").
// Для макетки/стола рекомендуем 10..20 мс.
static constexpr uint16_t UI_TICK_MS       = 10;
static constexpr uint16_t SENSORS_TICK_MS  = 100;
static constexpr uint16_t SAFETY_TICK_MS   = 50;
static constexpr uint16_t MOTORS_TICK_MS   = 50;

// ----------------------------- Пины дисплея -------------------------------
// GMG12864-06D / ST7565R (SPI). U8g2 использует аппаратный SPI.
static constexpr uint8_t PIN_LCD_CS   = 10;
static constexpr uint8_t PIN_LCD_DC   = 8;
static constexpr uint8_t PIN_LCD_RST  = 9;

// Контраст дисплея (0..255). Для многих ST7565R "читаемое" значение лежит в диапазоне 5..30.
static constexpr uint8_t LCD_CONTRAST = 10;

// ----------------------------- Энкодер ------------------------------------
static constexpr uint8_t PIN_ENC_CLK  = 3;
static constexpr uint8_t PIN_ENC_DT   = 4;
static constexpr uint8_t PIN_ENC_SW   = 5;

// У разных EC11/KY-040 модулей количество "тиков" Encoder-библиотеки на один щелчок бывает разным
// (обычно 4, но встречается 2 и даже 1). Если меню "не листает" или листает слишком медленно —
// попробуй ENCODER_DIV=2 или ENCODER_DIV=1.
static constexpr uint8_t ENCODER_DIV  = 4;


// ----------------------------- Input devices ------------------------------
// В этой версии для макетки мы уходим от энкодера (из‑за проблем со стабильностью на пинах без прерываний)
// и используем матричную клавиатуру 4x4. Энкодер можно вернуть, поставив USE_ENCODER=true.
#define USE_KEYPAD 1
#define USE_ENCODER 0

// ----------------------------- Матричная клавиатура 4x4 -------------------
// Пины клавиатуры: C1..C4 (колонки), R1..R4 (ряды).
// Рекомендуем подключать к аналоговым пинам Mega (A0..A7) — они свободны и имеют удобную разводку.
static constexpr uint8_t PIN_KP_C1 = A0;
static constexpr uint8_t PIN_KP_C2 = A1;
static constexpr uint8_t PIN_KP_C3 = A2;
static constexpr uint8_t PIN_KP_C4 = A3;
static constexpr uint8_t PIN_KP_R1 = A4;
static constexpr uint8_t PIN_KP_R2 = A5;
static constexpr uint8_t PIN_KP_R3 = A6;
static constexpr uint8_t PIN_KP_R4 = A7;

// ----------------------------- Режим макетки (стол) ------------------------
// В реальной машине E-STOP и концевики рекомендуется делать NC (разрыв = авария).
// На столе часто стоят NO-кнопки или входы вообще не подключены.
// BENCH_MODE=true включает "удобную" логику: активное состояние = LOW.
// Перед монтажом в шкаф ОБЯЗАТЕЛЬНО поставь BENCH_MODE=false.
static constexpr bool BENCH_MODE = true;

// ----------------------------- Safety --------------------------------------
// Рекомендуется NC на GND + INPUT_PULLUP (обрыв = авария)
static constexpr uint8_t PIN_ESTOP    = 2;
static constexpr bool    ESTOP_ACTIVE_LOW = BENCH_MODE ? true : false;
static constexpr bool    ENABLE_ESTOP = true;

// Концевики горизонтали (NC). Если не подключены — можно временно отключить в коде.
static constexpr uint8_t PIN_LIM_H1_LEFT  = 36;
static constexpr uint8_t PIN_LIM_H1_RIGHT = 37;
static constexpr uint8_t PIN_LIM_H2_LEFT  = 38;
static constexpr uint8_t PIN_LIM_H2_RIGHT = 39;
static constexpr bool    LIMIT_ACTIVE_LOW = BENCH_MODE ? true : false;
static constexpr bool    ENABLE_LIMIT_SWITCHES = true;

// ----------------------------- УЗ датчики ----------------------------------
static constexpr uint8_t PIN_US1_TRIG = 22;
static constexpr uint8_t PIN_US1_ECHO = 23;
static constexpr uint8_t PIN_US2_TRIG = 24;
static constexpr uint8_t PIN_US2_ECHO = 25;

// ----------------------------- RS-485 / Modbus ------------------------------
static constexpr uint8_t PIN_RS485_DE_RE = 6;
// TX_ENABLE (pin 7) не обязателен: обычно хватает DE/RE. Оставляем как резерв.
static constexpr uint8_t PIN_RS485_TX_EN = 7;

static constexpr uint32_t BAUD_RS485  = 9600;

// Формат UART для Modbus RTU.
// Вы в инструкции нашли режим "1-8-N-1" → это SERIAL_8N1.
// Если на частотнике выставите 8E1 (even parity), поменяйте на SERIAL_8E1.
static constexpr uint16_t RS485_SERIAL_CONFIG = SERIAL_8N1;

// Таймаут Modbus (мс).
// ВАЖНО: старые версии ждали таймаут даже после получения ответа → UI "зависал".
// Сейчас Modbus-разбор сделан так, чтобы выходить сразу после получения кадра,
// но таймаут всё равно нужен на случай отсутствия ответа.
static constexpr uint16_t RS485_TIMEOUT_MS = 80;

// ----------------------------- Лазеры --------------------------------------
// Лазеры питаются 3.3V, UART 9600 8N1.
// ВНИМАНИЕ: TX Mega (5V) -> RX Лазера (3.3V) через понижение уровня.
static constexpr uint32_t BAUD_LASER  = 9600;


// На текущем этапе дальность измерения нам нужна до 10 м.
// Это ограничение используется в парсере, чтобы \"мусор\" не попадал в расчёты.
static constexpr int32_t LASER_MAX_MM = 10000;

// ----------------------------- Кнопки --------------------------------------
// Общие
static constexpr uint8_t PIN_BTN_STOP  = 26;
static constexpr uint8_t PIN_BTN_START = 27;

// Горизонталь "оба вместе" (опционально, как резерв)
static constexpr uint8_t PIN_BTN_H_BOTH_FWD = 28;
static constexpr uint8_t PIN_BTN_H_BOTH_BWD = 29;

// Вертикаль Т1 (оставили на старых пинах)
static constexpr uint8_t PIN_BTN_V1_UP   = 30;
static constexpr uint8_t PIN_BTN_V1_DOWN = 31;

// Добавленные независимые кнопки (можно изменить под вашу панель)
static constexpr uint8_t PIN_BTN_H1_FWD = 40;
static constexpr uint8_t PIN_BTN_H1_BWD = 41;
static constexpr uint8_t PIN_BTN_H2_FWD = 42;
static constexpr uint8_t PIN_BTN_H2_BWD = 43;
static constexpr uint8_t PIN_BTN_V2_UP  = 44;
static constexpr uint8_t PIN_BTN_V2_DOWN= 45;

// Все кнопки предполагаем как NC/NO? Для удобства делаем активным LOW через INPUT_PULLUP.
static constexpr bool BUTTON_ACTIVE_LOW = true;

// ----------------------------- Modbus: регистры и команды ------------------
// Управление (команды/уставка) и мониторинг у разных приводов могут отличаться.
// Для HE200 мониторинг (D0.xx) начинается с 0x7000.
//
// Команды (как было в проекте): 0001H — команда, 0002H — уставка (%).
// Если для вашей конфигурации HE200 карта управления отличается — меняйте тут.
static constexpr uint16_t MB_REG_CMD      = 0x0001;
static constexpr uint16_t MB_REG_SETPOINT = 0x0002;

// HE200: Monitoring parameters (Holding regs)
static constexpr uint16_t MB_REG_MON_RUN_FREQ   = 0x7000; // D0.00 Running frequency, 0.01Hz
static constexpr uint16_t MB_REG_MON_SET_FREQ   = 0x7001; // D0.01 Set frequency,     0.01Hz
static constexpr uint16_t MB_REG_MON_FAULT_INFO = 0x702D; // D0.45 Fault information
static constexpr uint16_t MB_REG_MON_RUN_STATE  = 0x703D; // D0.61 AC drive running state

// Значения регистра команды 0001H
static constexpr uint16_t MB_CMD_FWD        = 0x0001;
static constexpr uint16_t MB_CMD_REV        = 0x0002;
static constexpr uint16_t MB_CMD_STOP       = 0x0003;
static constexpr uint16_t MB_CMD_COAST_STOP = 0x0004;
static constexpr uint16_t MB_CMD_RESET_FAULT= 0x0005;

// Задание 0002H: -10000..10000 (=-100.00..100.00%)
static constexpr int16_t MB_SETPOINT_MIN = -10000;
static constexpr int16_t MB_SETPOINT_MAX =  10000;

// ----------------------------- Логика наклона ------------------------------
// Какая сторона всегда "ниже" при погружении. 0 = Т1, 1 = Т2.
// Если монтажник перепутает провода — поменять здесь.
#define LOW_SIDE_TELFER_INDEX 0

// ----------------------------- Направления приводов ------------------------
// По вашему уточнению:
//   H Forward = вправо
//   V Forward = вниз
//
// Это означает:
//   - Для горизонтальных приводов: +скорость (forward) → движение вправо.
//   - Для вертикальных приводов:  +скорость (forward) → движение вниз (опускание).
//
// Все функции управления в коде опираются на это правило.
static constexpr bool H_FORWARD_IS_RIGHT = true;
static constexpr bool V_FORWARD_IS_DOWN  = true;

// ----------------------------- Скорости по умолчанию ------------------------
static constexpr uint8_t DEFAULT_H_SPEED_PCT = 55;   // движение по горизонтали
static constexpr uint8_t DEFAULT_V_SPEED_PCT = 45;   // подъём/опускание
static constexpr uint8_t DEFAULT_V_TILT_PCT  = 35;   // наклонный шаг (медленнее для точности)

// ----------------------------- Допуски --------------------------------------
static constexpr int16_t DEFAULT_H_TOL_MM = 10;
static constexpr int16_t DEFAULT_V_TOL_MM = 8;

// ----------------------------- Структуры данных -----------------------------

/**
 * @brief Настройки одной зоны.
 *
 * x_mm[0], x_mm[1] — целевые горизонтальные позиции Т1 и Т2 (калибровка по лазерам).
 * us_target_mm[*] — целевая "высота" по HC-SR04 (калибровка в ручном режиме).
 *
 * Для HC-SR04 мы используем "расстояние до груза" (mm).
 *  - Если груз опускается, расстояние уменьшается.
 *  - Если груз поднимается, расстояние увеличивается.
 */
struct ZoneConfig {
  int32_t  x_mm[TELFER_COUNT];         // горизонтальная позиция зоны
  int32_t  us_target_mm[TELFER_COUNT]; // целевой уровень по УЗ
  uint16_t dip_time_s;                 // выдержка в жидкости (1..600+)
  uint16_t tilt_step_mm;               // шаг наклона (mm) для ступени
  uint16_t step_wait_s;                // пауза после ступени (сек)
  uint8_t  move_speed_pct;             // скорость движения к зоне
  uint8_t  v_speed_pct;                // скорость вертикали в зоне
  bool     enabled;
};

/**
 * @brief Программа: до 10 зон + порядок обхода.
 */
struct ProgramConfig {
  char     name[12];                   // короткое имя (ASCII/CP1251 не принципиально)
  uint8_t  zone_count;                 // активное количество зон (1..10)
  uint8_t  order[MAX_ZONES];           // порядок индексов зон (0..zone_count-1)
  ZoneConfig zones[MAX_ZONES];
};

/**
 * @brief Общие настройки системы, отдельно от программы.
 */
struct GlobalSettings {
  int32_t  home_x_mm[TELFER_COUNT];     // HOME позиция (по лазерам)
  int32_t  travel_us_mm[TELFER_COUNT];  // транспортная "безопасная" высота по УЗ (mm)
  int16_t  h_tol_mm;                   // допуск горизонтали
  int16_t  v_tol_mm;                   // допуск вертикали
  uint16_t drip_wait_s;                // пауза стекания (сек) - можно использовать как step_wait
  uint8_t  h_speed_pct;                // скорость горизонтали (по умолчанию)
  uint8_t  v_speed_pct;                // скорость вертикали (по умолчанию)
  uint8_t  v_tilt_speed_pct;           // скорость наклонного шага
  bool     manual_h_sync_default;      // по умолчанию: горизонталь синхронизирована в ручном режиме?
  bool     reserved[3];                // выравнивание/резерв
};

// ----------------------------- Ошибки --------------------------------------

/**
 * @brief Коды ошибок (для UI и логики безопасности).
 *
 * В проекте встречаются разные источники ошибок:
 *  - аппаратные цепи безопасности (E-Stop, концевики)
 *  - связь с приводами (Modbus)
 *  - датчики (лазеры, УЗ)
 *  - некорректная программа (пустая/невалидная)
 */
enum class ErrorCode : uint8_t {
  NONE = 0,
  ESTOP,
  LIMIT_SWITCH,
  MODBUS_COMM,

  LASER1_FAIL,
  LASER2_FAIL,
  US1_FAIL,
  US2_FAIL,

  SENSOR_TIMEOUT,   // датчики долго не обновлялись/невалидны в момент старта авто
  INVALID_PROGRAM,  // программа зон некорректна (нет активных зон/ошибка порядка)

  DRIVE_FAULT,      // привод сообщил аварию (если читаем регистры)
};