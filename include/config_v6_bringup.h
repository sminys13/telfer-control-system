#pragma once

#include <Arduino.h>
#include <stdint.h>

#define FW_VERSION_V6_BRINGUP "v6-system-step9e-he200-readonly"

// =====================================================
// Debug / UART
// =====================================================
static constexpr uint32_t DBG_BAUD  = 115200;
static constexpr uint32_t DWIN_BAUD = 115200;

// =====================================================
// FIELD SAFETY INPUTS (restored from the proven legacy project)
// =====================================================
// Bench mode keeps the old convenient NO-to-GND logic. Physical VFD TX is
// forbidden while bench mode is active. For the cabinet/field build the NC
// safety chain must be verified first, then SAFETY_BENCH_MODE is set false.
#ifndef V6_SAFETY_BENCH_MODE
  #define V6_SAFETY_BENCH_MODE 1
#endif
static constexpr bool SAFETY_BENCH_MODE = (V6_SAFETY_BENCH_MODE != 0);
static constexpr uint8_t PIN_ESTOP = 2;
static constexpr uint8_t PIN_LIM_H1_LEFT  = 36;
static constexpr uint8_t PIN_LIM_H1_RIGHT = 37;
static constexpr uint8_t PIN_LIM_H2_LEFT  = 38;
static constexpr uint8_t PIN_LIM_H2_RIGHT = 39;
static constexpr bool ESTOP_ACTIVE_LOW = SAFETY_BENCH_MODE ? true : false;
static constexpr bool LIMIT_ACTIVE_LOW = SAFETY_BENCH_MODE ? true : false;
static constexpr bool ENABLE_ESTOP = true;
static constexpr bool ENABLE_LIMIT_SWITCHES = true;

// =====================================================
// SPI / SC16IS752
// =====================================================
// SC16IS752 #1 -> X1/X2
static constexpr uint8_t PIN_SC16_1_CS = 10;
static constexpr uint8_t PIN_SC16_1_IRQ = 40; // reserved

// SC16IS752 #2 -> Z1/Z2
static constexpr uint8_t PIN_SC16_2_CS = 8;
static constexpr uint8_t PIN_SC16_2_IRQ = 41; // reserved

// Keep Mega in SPI master mode.
static constexpr uint8_t MEGA_SPI_SS_PIN = 53;

// Confirmed on the user's CJMCU-752 module.
static constexpr uint32_t SC16_XTAL_HZ = 1843200UL;
static constexpr uint16_t SC16_DIV_9600 = 12;

// =====================================================
// Laser / sensor common limits
// =====================================================
static constexpr uint32_t LASER_BAUD = 9600;
static constexpr uint16_t LASER_MAX_MM = 10000;

// Safe blocking driver settings, used by old/safe sensor-core environment.
static constexpr uint16_t LASER_SAMPLE_PERIOD_MS = 120;
static constexpr uint8_t LASER_RETRY_COUNT = 3;

// Fast non-blocking sensor core timings.
static constexpr uint16_t FAST_SENSOR_PERIOD_MS = 120;
static constexpr uint16_t FAST_SENSOR_PHASE_MS = 30;
static constexpr uint16_t FAST_SENSOR_RESPONSE_TIMEOUT_MS = 260;
static constexpr uint16_t FAST_DWIN_UPDATE_MS = 80;

// True continuous stream mode.
// Working base after field diagnostics:
// LASER ON -> RANGE 10m -> RES 1mm -> FREQ 20Hz -> CONTINUOUS.
// No READ_CACHE, no auto-restart, no SINGLE in motion.
static constexpr uint16_t FAST_CONTINUOUS_INIT_GAP_MS = 160;
static constexpr uint16_t FAST_CONTINUOUS_RESTART_GAP_MS = 160;
static constexpr uint16_t FAST_CONTINUOUS_RESTART_MS = 900; // reserved; auto-restart disabled

// Sensor freshness.
// FRESH: age <= SENSOR_FRESH_TIMEOUT_MS.
// STALE: age > SENSOR_FRESH_TIMEOUT_MS and age <= staleTimeoutMs from EEPROM.
// LOST:  age > staleTimeoutMs from EEPROM, or no value/hardware fault.
static constexpr uint16_t SENSOR_FRESH_TIMEOUT_MS = 350;
static constexpr uint16_t SENSOR_FRESH_MS = SENSOR_FRESH_TIMEOUT_MS; // readable alias
static constexpr uint16_t SENSOR_LOST_MS = 1200; // default value; actual value is stored in SettingsV6.staleTimeoutMs

// Laser setup commands, sent once at start before CONTINUOUS.
static constexpr bool FAST_LASER_SET_RANGE_10M = true;
static constexpr bool FAST_LASER_SET_RESOLUTION_1MM = true;
static constexpr bool FAST_LASER_SET_FREQ_10HZ = false;
static constexpr bool FAST_LASER_SET_FREQ_20HZ = true;

static constexpr bool LASER_USE_CONTINUOUS = true;
static constexpr bool LASER_USE_READ_CACHE = false;
static constexpr bool LASER_AUTO_RESTART   = false;

// Optional readable command constants for documentation/future code.
static const uint8_t LASER_CMD_ON[]         = {0x80, 0x06, 0x05, 0x01, 0x74};
static const uint8_t LASER_CMD_CONTINUOUS[] = {0x80, 0x06, 0x03, 0x77};
static const uint8_t LASER_CMD_RANGE_10M[]  = {0x04, 0x09, 0x0A, 0xEF};
static const uint8_t LASER_CMD_RES_1MM[]    = {0x04, 0x0C, 0x01, 0xF5};
static const uint8_t LASER_CMD_FREQ_20HZ[]  = {0x04, 0x0A, 0x14, 0xE4};

// =====================================================
// DWIN VP MAP
// =====================================================
// Рабочие координаты
static constexpr uint16_t VP_X1    = 0x1000;
static constexpr uint16_t VP_X2    = 0x1002;
static constexpr uint16_t VP_Z1    = 0x1004;
static constexpr uint16_t VP_Z2    = 0x1006;

// Состояние системы
static constexpr uint16_t VP_MODE        = 0x1010;
static constexpr uint16_t VP_ERROR       = 0x1012;
static constexpr uint16_t VP_MOTOR_STATE = 0x1014; // состояние ручного/будущего движения
static constexpr uint16_t VP_VFD_STATUS   = 0x1016; // состояние слоя RS485/VFD

// Persistent header/status row for the new 1024x600 DWIN pages.
// These VPs are safe to add even before all DGUS objects are placed: writing
// an unused VP does not change the background image.
static constexpr uint16_t VP_PROGRAM_INDEX    = 0x1020; // 0=no active program, otherwise 1..4
static constexpr uint16_t VP_ZONE_CURRENT     = 0x1022; // 0 when idle
static constexpr uint16_t VP_ZONE_TOTAL       = 0x1024;
static constexpr uint16_t VP_STEP_CURRENT     = 0x1026;
static constexpr uint16_t VP_STEP_TOTAL       = 0x1028;

static constexpr uint16_t VP_TIME_ELAPSED_H   = 0x1030;
static constexpr uint16_t VP_TIME_ELAPSED_M   = 0x1032;
static constexpr uint16_t VP_TIME_ELAPSED_S   = 0x1034;
static constexpr uint16_t VP_TIME_TOTAL_H     = 0x1036;
static constexpr uint16_t VP_TIME_TOTAL_M     = 0x1038;
static constexpr uint16_t VP_TIME_TOTAL_S     = 0x103A;

static constexpr uint16_t VP_SENSOR_OK_COUNT  = 0x1040;
static constexpr uint16_t VP_SENSOR_TOTAL     = 0x1042;
static constexpr uint16_t VP_VFD_OK_COUNT     = 0x1044;
static constexpr uint16_t VP_VFD_TOTAL        = 0x1046;
static constexpr uint16_t VP_RS485_STATE      = 0x1048; // 0=disabled,1=ready/physical,2=dry-run
static constexpr uint16_t VP_NETWORK_STATE    = 0x104A; // reserved; network is planned
static constexpr uint16_t VP_SAFETY_STATE     = 0x104C; // bit0=E-stop active, bit1=E-stop latched
static constexpr uint16_t VP_LIMIT_STATE      = 0x104E; // H1L,H1R,H2L,H2R bits 0..3
static constexpr uint16_t VP_MODBUS_QUEUE_BUSY= 0x1050;
static constexpr uint16_t VP_ESTOP_ENABLED     = 0x1052; // effective runtime state
static constexpr uint16_t VP_LIMITS_ENABLED    = 0x1054; // effective runtime state

// AUTO/HOME runtime status. 0x1060..0x107E are reserved for the new runner.
static constexpr uint16_t VP_AUTO_PHASE         = 0x1060;
static constexpr uint16_t VP_AUTO_ERROR         = 0x1062;
static constexpr uint16_t VP_AUTO_RUNNING       = 0x1064;
static constexpr uint16_t VP_AUTO_PAUSED        = 0x1066;
static constexpr uint16_t VP_AUTO_SIMULATION    = 0x1068;
static constexpr uint16_t VP_AUTO_WAIT_OPERATOR = 0x106A;
static constexpr uint16_t VP_AUTO_REMAIN_S      = 0x106C;
static constexpr uint16_t VP_PROGRAM_READY      = 0x106E;
static constexpr uint16_t VP_PROGRAM_ACTIVE_SLOT= 0x1070; // 1..4

// Program editor/status VPs. These are intentionally independent from background images;
// DGUS objects can be added progressively without changing firmware addresses.
static constexpr uint16_t VP_PROG_ZONE_SELECTED = 0x1300; // 1..10
static constexpr uint16_t VP_PROG_ZONE_COUNT     = 0x1302;
static constexpr uint16_t VP_PROG_ZONE_ENABLED   = 0x1304;
static constexpr uint16_t VP_PROG_ZONE_X1        = 0x1306;
static constexpr uint16_t VP_PROG_ZONE_X2        = 0x1308;
static constexpr uint16_t VP_PROG_ZONE_Z1        = 0x130A;
static constexpr uint16_t VP_PROG_ZONE_Z2        = 0x130C;
static constexpr uint16_t VP_PROG_ZONE_DIP_S     = 0x130E;
static constexpr uint16_t VP_PROG_ZONE_TILT_MM   = 0x1310;
static constexpr uint16_t VP_PROG_ZONE_WAIT_S    = 0x1312;
static constexpr uint16_t VP_PROG_ZONE_H_PCT     = 0x1314;
static constexpr uint16_t VP_PROG_ZONE_V_PCT     = 0x1316;
static constexpr uint16_t VP_PROG_HOME_X1        = 0x1320;
static constexpr uint16_t VP_PROG_HOME_X2        = 0x1322;
static constexpr uint16_t VP_PROG_TRAVEL_Z1      = 0x1324;
static constexpr uint16_t VP_PROG_TRAVEL_Z2      = 0x1326;
static constexpr uint16_t VP_PROG_DRIP_S         = 0x1328;
static constexpr uint16_t VP_PROG_LOW_SIDE       = 0x132A; // 0=V1 low,1=V2 low
static constexpr uint16_t VP_PROG_DRY_ENABLE     = 0x1330;
static constexpr uint16_t VP_PROG_DRY_TIME_S     = 0x1332;
static constexpr uint16_t VP_PROG_STAGING_ZONE   = 0x1334; // 1..N
static constexpr uint16_t VP_PROG_DRY_X1         = 0x1336;
static constexpr uint16_t VP_PROG_DRY_X2         = 0x1338;
static constexpr uint16_t VP_PROG_DRY_Z1         = 0x133A;
static constexpr uint16_t VP_PROG_DRY_Z2         = 0x133C;
static constexpr uint16_t VP_PROG_VALID_MASK     = 0x1340;
static constexpr uint16_t VP_PROG_ZONE_VALID_MASK= 0x1342;
static constexpr uint16_t VP_PROG_DIRTY          = 0x1344;
static constexpr uint16_t VP_PROG_TILT_PCT        = 0x1346; // legacy global v_tilt_speed_pct
// Step9D editor/navigation status.
static constexpr uint16_t VP_PROG_SLOT_SELECTED    = 0x1348; // 1..4
static constexpr uint16_t VP_PROG_UI_STATE         = 0x134A; // 0=idle,1=loaded,2=saved,3=dirty,4=error,5=slot selected
static constexpr uint16_t VP_PROG_EDIT_LOCKED      = 0x134C; // AUTO/HOME running

// Команды от кнопок DWIN.
// Все кнопки должны писать сюда.
static constexpr uint16_t VP_CMD   = 0x1100;

// Step9D: hold-to-run manual control. Configure every manual arrow in DGUS as
// Touch Variable -> Bit Button, all buttons use this same VP and unique bit.
// Adj_Mode=0x03 (Inching): press writes bit=1, release writes bit=0.
// This gives a deterministic STOP on finger release without relying on the watchdog.
static constexpr uint16_t VP_JOG_HOLD_BITS = 0x1110;
static constexpr uint16_t JOG_HOLD_VALID_MASK = 0x0FFF; // bits 0..11

// =====================================================
// DWIN SETTINGS VP MAP (Step7A)
// 0x1200..0x127F — редактируемые настройки.
// Поля DWIN должны отправлять своё значение через Data auto-uploading.
// =====================================================
static constexpr uint16_t VP_VFD_BAUD_CODE          = 0x1200; // 0=4800,1=9600,2=19200,3=38400,4=57600,5=115200
static constexpr uint16_t VP_VFD_PARITY             = 0x1202; // 0=None,1=Even,2=Odd
static constexpr uint16_t VP_VFD_STOP_BITS          = 0x1204; // 1 or 2
static constexpr uint16_t VP_VFD_RESPONSE_TIMEOUT   = 0x1206; // ms
static constexpr uint16_t VP_VFD_RETRIES            = 0x1208; // 0..5
static constexpr uint16_t VP_VFD_INTER_REQUEST_MS   = 0x120A;
static constexpr uint16_t VP_VFD_ONLINE_POLL_MS     = 0x120C;
static constexpr uint16_t VP_VFD_OFFLINE_POLL_MS    = 0x120E;

static constexpr uint16_t VP_VFD_ADDR_H1            = 0x1210;
static constexpr uint16_t VP_VFD_ADDR_H2            = 0x1212;
static constexpr uint16_t VP_VFD_ADDR_V1            = 0x1214;
static constexpr uint16_t VP_VFD_ADDR_V2            = 0x1216;
static constexpr uint16_t VP_VFD_INVERT_MASK        = 0x1218;
static constexpr uint16_t VP_MANUAL_JOG_TIMEOUT_MS  = 0x1220;
// Bench-only safety editor. Values 0/1. In FIELD mode disabling is rejected.
static constexpr uint16_t VP_SAFETY_ESTOP_ENABLE    = 0x1222;
static constexpr uint16_t VP_SAFETY_LIMITS_ENABLE   = 0x1224;

// 5 values per drive: manual%, max%, slow%, slowdown distance mm, stop tolerance mm.
static constexpr uint16_t VP_DRIVE_H1_BASE           = 0x1230;
static constexpr uint16_t VP_DRIVE_H2_BASE           = 0x1240;
static constexpr uint16_t VP_DRIVE_V1_BASE           = 0x1250;
static constexpr uint16_t VP_DRIVE_V2_BASE           = 0x1260;
static constexpr uint16_t VP_DRIVE_MANUAL_OFS        = 0x0000;
static constexpr uint16_t VP_DRIVE_MAX_OFS           = 0x0002;
static constexpr uint16_t VP_DRIVE_SLOW_OFS          = 0x0004;
static constexpr uint16_t VP_DRIVE_SLOWDOWN_OFS      = 0x0006;
static constexpr uint16_t VP_DRIVE_TOLERANCE_OFS     = 0x0008;

// 0x1280.. — status/diagnostics of the settings editor.
static constexpr uint16_t VP_SETTINGS_STATE          = 0x1280;
static constexpr uint16_t VP_SETTINGS_ERROR          = 0x1282;
static constexpr uint16_t VP_SETTINGS_DIRTY          = 0x1284;
static constexpr uint16_t VP_SETTINGS_VERSION        = 0x1286;
static constexpr uint16_t VP_SETTINGS_TEST_DRIVE     = 0x1288;

static constexpr uint16_t SETTINGS_UI_IDLE            = 0;
static constexpr uint16_t SETTINGS_UI_DIRTY           = 1;
static constexpr uint16_t SETTINGS_UI_APPLIED         = 2;
static constexpr uint16_t SETTINGS_UI_SAVED           = 3;
static constexpr uint16_t SETTINGS_UI_LOADED          = 4;
static constexpr uint16_t SETTINGS_UI_DEFAULTS        = 5;
static constexpr uint16_t SETTINGS_UI_REJECTED        = 6;
static constexpr uint16_t SETTINGS_UI_TEST_DRY_RUN    = 7;

// =====================================================
// DWIN COMMANDS
// Button type = Return Key Code
// VP = 0x1100
// Data auto-uploading = ON
// key value(0x) = код ниже
// =====================================================
static constexpr uint16_t CMD_NONE        = 0x0000;

// Основные кнопки / страницы, уже сделанные на DWIN.
static constexpr uint16_t CMD_MANUAL      = 0x0001; // Ручное управление
static constexpr uint16_t CMD_AUTO        = 0x0002; // Авто
static constexpr uint16_t CMD_HOME        = 0x0003; // Домой
static constexpr uint16_t CMD_STOP        = 0x0004; // Стоп
static constexpr uint16_t CMD_SETTINGS    = 0x0005; // Настройки
static constexpr uint16_t CMD_CALIBRATION = 0x0006; // Калибровка

// Калибровка датчиков
static constexpr uint16_t CMD_ZERO_X1     = 0x0021;
static constexpr uint16_t CMD_ZERO_X2     = 0x0022;
static constexpr uint16_t CMD_ZERO_Z1     = 0x0023;
static constexpr uint16_t CMD_ZERO_Z2     = 0x0024;

// Сохранение / загрузка / сброс калибровки
static constexpr uint16_t CMD_SAVE        = 0x0030;
static constexpr uint16_t CMD_LOAD        = 0x0031;
static constexpr uint16_t CMD_RESET_CAL   = 0x0032;

// Сервис
static constexpr uint16_t CMD_CLEAR_STATUS  = 0x0044;
static constexpr uint16_t CMD_SENSOR_REINIT = 0x0045;
static constexpr uint16_t CMD_DIAG_SNAPSHOT = 0x0046;
static constexpr uint16_t CMD_SAFETY_CLEAR        = 0x0047;
static constexpr uint16_t CMD_SAFETY_TOGGLE_ESTOP = 0x0048; // BENCH only
static constexpr uint16_t CMD_SAFETY_TOGGLE_LIMITS= 0x0049; // BENCH only

// AUTO / program control. Existing CMD_AUTO starts the selected program; CMD_HOME runs HOME.
static constexpr uint16_t CMD_AUTO_PAUSE          = 0x0300;
static constexpr uint16_t CMD_AUTO_RESUME         = 0x0301;
static constexpr uint16_t CMD_AUTO_OPERATOR_NEXT  = 0x0302;
static constexpr uint16_t CMD_AUTO_SIM_TOGGLE     = 0x0303; // BENCH + non-write builds only
static constexpr uint16_t CMD_PROGRAM_LOAD        = 0x0310;
static constexpr uint16_t CMD_PROGRAM_SAVE        = 0x0311;
static constexpr uint16_t CMD_PROGRAM_DEFAULTS    = 0x0312;
// Step9C direct touch navigation for Program/Zone screen.
static constexpr uint16_t CMD_PROGRAM_SLOT_1      = 0x0314;
static constexpr uint16_t CMD_PROGRAM_SLOT_2      = 0x0315;
static constexpr uint16_t CMD_PROGRAM_SLOT_3      = 0x0316;
static constexpr uint16_t CMD_PROGRAM_SLOT_4      = 0x0317;
static constexpr uint16_t CMD_PROGRAM_ZONE_PREV   = 0x0318;
static constexpr uint16_t CMD_PROGRAM_ZONE_NEXT   = 0x0319;
static constexpr uint16_t CMD_PROGRAM_ZONE_TOGGLE = 0x031A;
static constexpr uint16_t CMD_PROGRAM_CAPTURE_HOME= 0x0320;
static constexpr uint16_t CMD_PROGRAM_CAPTURE_TRAVEL=0x0321;
static constexpr uint16_t CMD_PROGRAM_CAPTURE_ZONE_X=0x0322;
static constexpr uint16_t CMD_PROGRAM_CAPTURE_ZONE_Z=0x0323;
static constexpr uint16_t CMD_PROGRAM_CAPTURE_DRY_X=0x0324;
static constexpr uint16_t CMD_PROGRAM_CAPTURE_DRY_Z=0x0325;
// Explicit acknowledgement after numeric DWIN editing. A single coordinate field
// never makes a previously uncalibrated pair valid by itself.
static constexpr uint16_t CMD_PROGRAM_ACCEPT_ZONE_X =0x0326;
static constexpr uint16_t CMD_PROGRAM_ACCEPT_ZONE_Z =0x0327;
static constexpr uint16_t CMD_PROGRAM_ACCEPT_HOME   =0x0328;
static constexpr uint16_t CMD_PROGRAM_ACCEPT_TRAVEL =0x0329;
static constexpr uint16_t CMD_PROGRAM_ACCEPT_DRY_X  =0x032A;
static constexpr uint16_t CMD_PROGRAM_ACCEPT_DRY_Z  =0x032B;

// Настройки RS485/VFD (страница Settings).
static constexpr uint16_t CMD_VFD_SETTINGS_APPLY    = 0x0200;
static constexpr uint16_t CMD_VFD_SETTINGS_SAVE     = 0x0201;
static constexpr uint16_t CMD_VFD_SETTINGS_LOAD     = 0x0202;
static constexpr uint16_t CMD_VFD_SETTINGS_DEFAULTS = 0x0203;
static constexpr uint16_t CMD_VFD_TEST_H1            = 0x0204;
static constexpr uint16_t CMD_VFD_TEST_H2            = 0x0205;
static constexpr uint16_t CMD_VFD_TEST_V1            = 0x0206;
static constexpr uint16_t CMD_VFD_TEST_V2            = 0x0207;
static constexpr uint16_t CMD_VFD_TEST_ALL           = 0x0208;

// =====================================================
// MANUAL JOG COMMANDS
// Пока это безопасные команды-заглушки: физические выходы/VFD не управляются.
// Все кнопки Manual page пишут VP = 0x1100, key value(0x) = код ниже.
// =====================================================
static constexpr uint16_t CMD_JOG_H1_FWD    = 0x0101; // H1 / X1 вперед
static constexpr uint16_t CMD_JOG_H1_BWD    = 0x0102; // H1 / X1 назад
static constexpr uint16_t CMD_JOG_H2_FWD    = 0x0103; // H2 / X2 вперед
static constexpr uint16_t CMD_JOG_H2_BWD    = 0x0104; // H2 / X2 назад
static constexpr uint16_t CMD_JOG_H_BOTH_FWD = 0x0105; // H1+H2 вперед
static constexpr uint16_t CMD_JOG_H_BOTH_BWD = 0x0106; // H1+H2 назад

static constexpr uint16_t CMD_JOG_V1_UP     = 0x0107; // V1 / Z1 вверх
static constexpr uint16_t CMD_JOG_V1_DOWN   = 0x0108; // V1 / Z1 вниз
static constexpr uint16_t CMD_JOG_V2_UP     = 0x0109; // V2 / Z2 вверх
static constexpr uint16_t CMD_JOG_V2_DOWN   = 0x010A; // V2 / Z2 вниз
static constexpr uint16_t CMD_JOG_V_BOTH_UP   = 0x010B; // V1+V2 вверх
static constexpr uint16_t CMD_JOG_V_BOTH_DOWN = 0x010C; // V1+V2 вниз

static constexpr uint16_t CMD_JOG_STOP      = 0x010F; // остановить ручное движение

// Backward-compatible aliases used by earlier system-step code.
// They now point to the existing DWIN command map 0001..0006.
static constexpr uint16_t CMD_MODE_SERVICE = CMD_SETTINGS;
static constexpr uint16_t CMD_MODE_STOP    = CMD_STOP;
static constexpr uint16_t CMD_MODE_MANUAL  = CMD_MANUAL;
static constexpr uint16_t CMD_MODE_AUTO    = CMD_AUTO;

// =====================================================
// MODE VALUES
// Это НЕ команды кнопок.
// Это значения, которые Arduino пишет в VP_MODE = 0x1010,
// чтобы экран показывал текущий режим.
// =====================================================
static constexpr uint16_t MODE_SERVICE     = 0;
static constexpr uint16_t MODE_MANUAL      = 1;
static constexpr uint16_t MODE_AUTO        = 2;
static constexpr uint16_t MODE_HOME        = 3;
static constexpr uint16_t MODE_STOP        = 4;
static constexpr uint16_t MODE_SETTINGS    = 5;
static constexpr uint16_t MODE_CALIBRATION = 6;

// Backward-compatible aliases used by old code/environments.
static constexpr uint16_t MODE_SENSOR_CORE       = MODE_SERVICE;
static constexpr uint16_t MODE_SENSOR_CORE_FAST  = MODE_SERVICE;
static constexpr uint16_t MODE_SERVICE_SENSORS   = MODE_SERVICE;
static constexpr uint16_t MODE_STOP_READY        = MODE_STOP;
static constexpr uint16_t MODE_MANUAL_READY      = MODE_MANUAL;
static constexpr uint16_t MODE_AUTO_READY        = MODE_AUTO;


// =====================================================
// MOTOR STATE VALUES
// Arduino пишет это значение в VP_MOTOR_STATE = 0x1014.
// Сейчас это безопасная программная индикация без физических выходов.
// =====================================================
static constexpr uint16_t MOTOR_STATE_IDLE        = 0;
static constexpr uint16_t MOTOR_STATE_H1_FWD      = 1;
static constexpr uint16_t MOTOR_STATE_H1_BWD      = 2;
static constexpr uint16_t MOTOR_STATE_H2_FWD      = 3;
static constexpr uint16_t MOTOR_STATE_H2_BWD      = 4;
static constexpr uint16_t MOTOR_STATE_H_BOTH_FWD  = 5;
static constexpr uint16_t MOTOR_STATE_H_BOTH_BWD  = 6;
static constexpr uint16_t MOTOR_STATE_V1_UP       = 7;
static constexpr uint16_t MOTOR_STATE_V1_DOWN     = 8;
static constexpr uint16_t MOTOR_STATE_V2_UP       = 9;
static constexpr uint16_t MOTOR_STATE_V2_DOWN     = 10;
static constexpr uint16_t MOTOR_STATE_V_BOTH_UP   = 11;
static constexpr uint16_t MOTOR_STATE_V_BOTH_DOWN = 12;
static constexpr uint16_t MOTOR_STATE_STOPPED     = 20;
static constexpr uint16_t MOTOR_STATE_BLOCKED     = 30;


// =====================================================
// MANUAL JOG SAFETY
// =====================================================
// Step7A: watchdog перенесён в SettingsV6.vfd.manualJogTimeoutMs и EEPROM.
// Значение по умолчанию 2500 мс задаётся в SettingsStorageV6::defaultsVfdSection().
// Это аварийная защита ручного режима, а не время движения до координаты.
static constexpr bool MANUAL_JOG_TIMEOUT_ENABLED = true;
static constexpr uint16_t MANUAL_JOG_TIMEOUT_DEFAULT_MS = 2500;

// =====================================================
// VFD / RS485 LAYER
// Step7B: подключены существующие ModbusMasterRTU + Drives. Полные кадры
// NE200 строятся и печатаются, но НЕ отправляются, пока VFD_RS485_ENABLED=false.
// =====================================================
// Build-time switches. Default environment remains fully dry-run.
#ifndef V6_VFD_RS485_ENABLED
  #define V6_VFD_RS485_ENABLED 0
#endif
#ifndef V6_VFD_DRY_RUN
  #define V6_VFD_DRY_RUN 1
#endif
#ifndef V6_VFD_WRITE_COMMANDS_ENABLED
  #define V6_VFD_WRITE_COMMANDS_ENABLED 0
#endif
#ifndef V6_AUTO_PHYSICAL_ENABLED
  #define V6_AUTO_PHYSICAL_ENABLED 0
#endif
#ifndef V6_HE200_COMMISSIONING
  #define V6_HE200_COMMISSIONING 0
#endif
static constexpr bool VFD_RS485_ENABLED = (V6_VFD_RS485_ENABLED != 0);
static constexpr bool VFD_DRY_RUN       = (V6_VFD_DRY_RUN != 0);
static constexpr bool VFD_WRITE_COMMANDS_ENABLED = (V6_VFD_WRITE_COMMANDS_ENABLED != 0);
// Separate commissioning interlock: physical manual movement can be enabled
// without allowing HOME/AUTO. Only the explicit FIELD AUTO build sets this to 1.
static constexpr bool AUTO_PHYSICAL_ENABLED = (V6_AUTO_PHYSICAL_ENABLED != 0);
static constexpr bool HE200_COMMISSIONING = (V6_HE200_COMMISSIONING != 0);
static constexpr uint8_t PIN_VFD_RS485_DE_RE = 6; // MAX485 DE+/RE, RO=RX1/19, DI=TX1/18
// Baud/parity/stop bits/timeouts are now read from SettingsV6 and EEPROM.

// Step7B uses the already existing ModbusMasterRTU/Drives files.
// While dry-run is active, complete RTU frames (including CRC) are printed
// to the PlatformIO monitor and nothing is transmitted to MAX485.
static constexpr bool VFD_MODBUS_TRACE = true;
static constexpr bool VFD_BACKGROUND_POLL_ENABLED = false; // enable only after the NE200 monitoring map is verified

// Hard compile-time interlock: real drive TX must never be enabled with the
// convenient bench interpretation of the safety chain.
// Physical writes are never allowed with the convenient BENCH safety logic.
// Physical READ-ONLY diagnostics are allowed, because no motion/stop register is written.
static_assert(!VFD_RS485_ENABLED || !VFD_WRITE_COMMANDS_ENABLED || !SAFETY_BENCH_MODE,
              "Physical VFD writes require SAFETY_BENCH_MODE=false and verified NC safety wiring");
static_assert(!AUTO_PHYSICAL_ENABLED || (VFD_RS485_ENABLED && VFD_WRITE_COMMANDS_ENABLED && !SAFETY_BENCH_MODE),
              "Physical AUTO/HOME requires RS485 writes, FIELD safety logic and explicit AUTO enable");
static_assert(!HE200_COMMISSIONING || !VFD_WRITE_COMMANDS_ENABLED,
              "HE200 commissioning profile is READ-ONLY until the HE200 RUN/STOP write map is verified");

// LEGACY NE200/300 write map retained only for dry-run history.
// DO NOT use these writes with HE200 until its command/write table is verified.
// Control register: 0x0001; network setpoint: 0x0002.
static constexpr uint16_t NE200_REG_COMMAND  = 0x0001;
static constexpr uint16_t NE200_REG_SETPOINT = 0x0002;
static constexpr uint16_t NE200_REG_STATUS   = 0x0020; // safe read test
static constexpr uint16_t NE200_REG_FAULT    = 0x0021; // read together with status


// HE200 monitoring addresses from the supplied HE200 User Manual, p.47-48.
// These are used by the Step9E physical RS485 READ-ONLY commissioning build.
static constexpr uint16_t HE200_REG_RUNNING_FREQ   = 0x7000; // 0.01 Hz
static constexpr uint16_t HE200_REG_SET_FREQ       = 0x7001; // 0.01 Hz
static constexpr uint16_t HE200_REG_BUS_VOLT       = 0x7002; // 0.1 V
static constexpr uint16_t HE200_REG_OUTPUT_VOLT    = 0x7003; // 1 V
static constexpr uint16_t HE200_REG_OUTPUT_CURRENT = 0x7004; // 0.01 A when Pd.06=0
static constexpr uint16_t HE200_REG_DIGITAL_INPUT  = 0x7007; // X input state
static constexpr uint16_t HE200_REG_FAULT_INFO     = 0x702D;
static constexpr uint16_t HE200_REG_CUR_SET_FREQ   = 0x703B; // 0.01 %
static constexpr uint16_t HE200_REG_CUR_RUN_FREQ   = 0x703C; // 0.01 %
static constexpr uint16_t HE200_REG_RUN_STATE      = 0x703D;

static constexpr uint16_t NE200_CMD_FORWARD    = 0x0001;
static constexpr uint16_t NE200_CMD_REVERSE    = 0x0002;
static constexpr uint16_t NE200_CMD_STOP       = 0x0003;
static constexpr uint16_t NE200_CMD_COAST_STOP = 0x0004;
static constexpr uint16_t NE200_CMD_RESET_FAULT= 0x0005;

static constexpr uint16_t NE200_SETPOINT_MIN = 0;
static constexpr uint16_t NE200_SETPOINT_MAX = 10000; // 100.00%; direction is selected by register 0x0001

// Статусы будущего VFD слоя для VP_VFD_STATUS = 0x1016
static constexpr uint16_t VFD_STATUS_DISABLED = 0;
static constexpr uint16_t VFD_STATUS_READY    = 1;
static constexpr uint16_t VFD_STATUS_DRY_RUN  = 2;
static constexpr uint16_t VFD_STATUS_MOVING   = 3;
static constexpr uint16_t VFD_STATUS_STOPPED  = 4;
static constexpr uint16_t VFD_STATUS_BLOCKED  = 5;

// =====================================================
// SENSOR STATUS / ERROR MASK
// VP_ERROR = 0x1012
// =====================================================
static constexpr uint16_t ERROR_NONE = 0;

// LOST — датчик долго не даёт данных / нет значения / аппаратная ошибка.
static constexpr uint16_t ERR_X1_LOST = 0x0001;
static constexpr uint16_t ERR_X2_LOST = 0x0002;
static constexpr uint16_t ERR_Z1_LOST = 0x0004;
static constexpr uint16_t ERR_Z2_LOST = 0x0008;

// STALE — значение устарело, но последнее значение удерживаем.
static constexpr uint16_t ERR_X1_STALE = 0x0010;
static constexpr uint16_t ERR_X2_STALE = 0x0020;
static constexpr uint16_t ERR_Z1_STALE = 0x0040;
static constexpr uint16_t ERR_Z2_STALE = 0x0080;

// EEPROM/settings problem.
static constexpr uint16_t ERR_SETTINGS = 0x0100;

// Hardware safety. These bits are intentionally above the sensor/status range.
static constexpr uint16_t ERR_ESTOP = 0x0200;
static constexpr uint16_t ERR_LIMIT = 0x0400;
static constexpr uint16_t ERR_AUTO  = 0x0800; // AutoRunner terminal fault; detail in VP_AUTO_ERROR

// Backward-compatible aliases for earlier code.
static constexpr uint16_t ERROR_X1_INVALID = ERR_X1_LOST;
static constexpr uint16_t ERROR_X2_INVALID = ERR_X2_LOST;
static constexpr uint16_t ERROR_Z1_INVALID = ERR_Z1_LOST;
static constexpr uint16_t ERROR_Z2_INVALID = ERR_Z2_LOST;
static constexpr uint16_t ERROR_X1_STALE   = ERR_X1_STALE;
static constexpr uint16_t ERROR_X2_STALE   = ERR_X2_STALE;
static constexpr uint16_t ERROR_Z1_STALE   = ERR_Z1_STALE;
static constexpr uint16_t ERROR_Z2_STALE   = ERR_Z2_STALE;
static constexpr uint16_t ERROR_SETTINGS   = ERR_SETTINGS;

// =====================================================
// DWIN UPDATE
// =====================================================
static constexpr uint16_t DWIN_UPDATE_MS = 100;
