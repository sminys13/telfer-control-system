// /**
//  * @file strings.cpp
//  * @brief Строки в программной памяти (PROGMEM) для экономии RAM
//  */

// #include <Arduino.h>
// #include <avr/pgmspace.h>
// #include "../include/config.h"

// // Тексты ошибок в программной памяти
// const char error_sensor_laser1[] PROGMEM = "Ошибка лазерного датчика 1";
// const char error_sensor_laser2[] PROGMEM = "Ошибка лазерного датчика 2";
// const char error_sensor_us1[] PROGMEM = "Ошибка УЗ датчика 1";
// const char error_sensor_us2[] PROGMEM = "Ошибка УЗ датчика 2";
// const char error_motor_h1[] PROGMEM = "Ошибка двигателя гориз. 1";
// const char error_motor_h2[] PROGMEM = "Ошибка двигателя гориз. 2";
// const char error_motor_v1[] PROGMEM = "Ошибка двигателя верт. 1";
// const char error_motor_v2[] PROGMEM = "Ошибка двигателя верт. 2";
// const char error_rs485[] PROGMEM = "Ошибка связи RS-485";
// const char error_overload[] PROGMEM = "Перегрузка";
// const char error_limit_switch[] PROGMEM = "Концевой выключатель";
// const char error_position[] PROGMEM = "Отклонение позиции";
// const char error_emergency[] PROGMEM = "Аварийная остановка";
// const char error_memory[] PROGMEM = "Ошибка памяти";
// const char error_display[] PROGMEM = "Ошибка дисплея";

// // Таблица ошибок
// const char* const error_messages[] PROGMEM = {
//     error_sensor_laser1,
//     error_sensor_laser2,
//     error_sensor_us1,
//     error_sensor_us2,
//     error_motor_h1,
//     error_motor_h2,
//     error_motor_v1,
//     error_motor_v2,
//     error_rs485,
//     error_overload,
//     error_limit_switch,
//     error_position,
//     error_emergency,
//     error_memory,
//     error_display
// };

// // Функция для чтения строк из PROGMEM
// const char* getErrorMessage(ErrorType error) {
//     static char buffer[32];
//     if (error >= ERROR_COUNT) {
//         return "Неизвестная ошибка";
//     }
//     strcpy_P(buffer, (const char*)pgm_read_word(&(error_messages[error])));
//     return buffer;
// }