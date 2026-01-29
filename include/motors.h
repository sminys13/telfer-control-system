/**
 * @file motors.h
 * @brief Управление двигателями через частотные преобразователи HE200-T3S-1R5G по RS-485
 * @version 4.0
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "../include/config.h"

// ========== КОНСТАНТЫ УПРАВЛЕНИЯ ДВИГАТЕЛЯМИ ==========

// Адреса частотных преобразователей (должны быть настроены на самих преобразователях)
#define MOTOR_H1_ADDR 0x01   // Горизонтальный левый
#define MOTOR_H2_ADDR 0x02   // Горизонтальный правый
#define MOTOR_V1_ADDR 0x03   // Вертикальный левый
#define MOTOR_V2_ADDR 0x04   // Вертикальный правый
#define MOTOR_BROADCAST 0xFF // Широковещательный адрес

// Команды Modbus RTU (предполагаемый протокол, нужно уточнить по документации HE200)
#define MODBUS_READ_COIL 0x01
#define MODBUS_READ_INPUT 0x02
#define MODBUS_READ_HOLDING_REG 0x03
#define MODBUS_READ_INPUT_REG 0x04
#define MODBUS_WRITE_COIL 0x05
#define MODBUS_WRITE_SINGLE_REG 0x06
#define MODBUS_WRITE_MULTI_COIL 0x0F
#define MODBUS_WRITE_MULTI_REG 0x10

// Регистры частотного преобразователя HE200 (нужно уточнить по документации)
#define REG_MOTOR_SPEED 0x1000     // Регистр скорости (0-1000 = 0-100%)
#define REG_MOTOR_DIRECTION 0x1001 // Регистр направления (0 - стоп, 1 - вперед, 2 - назад)
#define REG_MOTOR_ACCEL 0x1002     // Регистр времени разгона (0.1с)
#define REG_MOTOR_DECEL 0x1003     // Регистр времени торможения (0.1с)
#define REG_MOTOR_STATUS 0x1004    // Регистр статуса
#define REG_MOTOR_FAULT 0x1005     // Регистр ошибок
#define REG_MOTOR_CURRENT 0x1006   // Регистр тока (0.1А)
#define REG_MOTOR_FREQ 0x1007      // Регистр частоты (0.1Гц)
#define REG_MOTOR_TORQUE 0x1008    // Регистр момента (0.1%)

// Состояния двигателей
typedef enum
{
    MOTOR_STATE_IDLE,         // Остановлен
    MOTOR_STATE_ACCELERATING, // Разгон
    MOTOR_STATE_RUNNING,      // Работа на заданной скорости
    MOTOR_STATE_DECELERATING, // Торможение
    MOTOR_STATE_FAULT,        // Ошибка
    MOTOR_STATE_EMERGENCY_STOP // Аварийная остановка
} MotorState;

// Режимы управления
typedef enum
{
    CONTROL_MODE_SPEED,    // Управление скоростью
    CONTROL_MODE_POSITION, // Управление позицией
    CONTROL_MODE_TORQUE,   // Управление моментом
    CONTROL_MODE_SYNC      // Синхронное управление
} ControlMode;

// Ошибки двигателей
typedef enum
{
    MOTOR_ERROR_NONE = 0,      // Нет ошибок
    MOTOR_ERROR_OVER_CURRENT,  // Перегрузка по току
    MOTOR_ERROR_OVER_VOLTAGE,  // Перенапряжение
    MOTOR_ERROR_UNDER_VOLTAGE, // Недовольтаж
    MOTOR_ERROR_OVER_TEMP,     // Перегрев
    MOTOR_ERROR_COMM_FAILURE,  // Ошибка связи
    MOTOR_ERROR_ENCODER,       // Ошибка энкодера
    MOTOR_ERROR_OVERLOAD,      // Перегрузка
    MOTOR_ERROR_SHORT_CIRCUIT, // Короткое замыкание
    MOTOR_ERROR_GROUND_FAULT   // Замыкание на землю
} MotorError;

// Структура состояния двигателя
typedef struct
{
    MotorState state;         // Текущее состояние
    MotorError error;         // Активная ошибка
    uint8_t warningCode;      // Код предупреждения
    uint8_t address;          // Адрес Modbus
    int16_t targetSpeed;      // Целевая скорость (-1000..+1000)
    int16_t currentSpeed;     // Текущая скорость (-1000..+1000)
    int16_t acceleration;     // Ускорение (%/с²)
    int16_t maxSpeed;         // Максимальная скорость (%)
    int32_t position;         // Текущая позиция (импульсы энкодера)
    int32_t targetPosition;   // Целевая позиция
    float current;            // Ток двигателя (А)
    float dcVoltage;          // Напряжение DC (В)
    float frequency;          // Частота двигателя (Гц)
    float temperature;        // Температура (°C)
    uint32_t runtime;         // Время работы (часы)
    uint32_t lastCommandTime; // Время последней команды
    bool enabled;             // Двигатель включен
    bool running;             // Двигатель в движении
    bool fault;               // Неиспраность
    bool faultResetPending;   // Ожидание сброса ошибки
} MotorStatus;

// Структура конфигурации двигателя
typedef struct
{
    uint8_t address;           // Адрес Modbus
    uint16_t maxSpeed;         // Максимальная скорость (об/мин)
    uint16_t ratedCurrent;     // Номинальный ток (А)
    uint16_t accelerationTime; // Время разгона 0-макс (мс)
    uint16_t decelerationTime; // Время торможения макс-0 (мс)
    uint16_t overcurrentLimit; // Лимит перегрузки (%)
    uint8_t polePairs;         // Количество пар полюсов
    uint16_t encoderPPR;       // Импульсов энкодера на оборот
    bool inverted;             // Инвертировано направление
    uint8_t controlMode;       // Режим управления
} MotorConfig;

// Структура управления синхронным движением
typedef struct
{
    MotorStatus *master;      // Ведущий двигатель
    MotorStatus *slave;       // Ведомый двигатель
    int32_t positionError;    // Ошибка позиции
    int32_t maxPositionError; // Максимальная допустимая ошибка
    int16_t syncGain;         // Коэффициент синхронизации
    bool enabled;             // Синхронизация включена
} SyncControl;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация
void motorsInit(void);
bool initRS485Interface(void);
void initMotorConfigurations(void);
bool detectMotors(void);

// Управление отдельными двигателями
bool setMotorSpeed(uint8_t motorID, int16_t speed, bool direction);
bool setMotorSpeedPercent(uint8_t motorID, int8_t percent, bool direction);
bool stopMotor(uint8_t motorID);
bool emergencyStopMotor(uint8_t motorID);
bool resetMotorFault(uint8_t motorID);
bool enableMotor(uint8_t motorID, bool enable);

// Групповое управление
void stopAllMotors(void);
void emergencyStopAll(void);
void enableAllMotors(bool enable);
void setHorizontalSpeed(int16_t leftSpeed, int16_t rightSpeed, bool direction);
void setVerticalSpeed(int16_t leftSpeed, int16_t rightSpeed, bool direction);

// Специальные режимы
void tiltOperation(bool isLowering, uint8_t tiltPercentage);
void levelingOperation(void);
void syncHorizontalMovement(int32_t targetPosition, uint8_t speedPercent);
void syncVerticalMovement(int32_t targetHeight, uint8_t speedPercent);

// Позиционное управление
bool moveToPosition(uint8_t motorID, int32_t position, uint8_t speedPercent);
bool moveHorizontalToPosition(int32_t position, uint8_t speedPercent);
bool moveVerticalToHeight(int32_t height, uint8_t speedPercent);

// Чтение состояния
MotorStatus *getMotorStatus(uint8_t motorID);
bool readMotorParameters(uint8_t motorID);
bool checkMotorFault(uint8_t motorID);
float getMotorCurrent(uint8_t motorID);
int32_t getMotorPosition(uint8_t motorID);

// Конфигурация
bool configureMotor(uint8_t motorID, const MotorConfig *config);
bool saveMotorParameters(uint8_t motorID);
bool loadMotorParameters(uint8_t motorID);
bool setMotorLimits(uint8_t motorID, uint16_t maxSpeed, uint16_t maxCurrent);

// Коммуникация
bool sendModbusCommand(uint8_t address, uint8_t function, uint16_t reg, uint16_t value);
bool sendModbusReadCommand(uint8_t address, uint8_t function, uint16_t reg, uint16_t count);
bool parseModbusResponse(uint8_t *data, uint16_t length, uint8_t *response, uint16_t *responseLength);
uint16_t calculateCRC16(const uint8_t *data, uint16_t length);

// Безопасность и диагностика
bool checkMotorSafety(void);
bool performMotorSelfTest(void);
void monitorMotorHealth(void);
bool isMotorOverloaded(uint8_t motorID);
bool areMotorsSynchronized(void);

// Утилиты
int16_t speedPercentToRaw(int8_t percent);
int8_t speedRawToPercent(int16_t raw);
int16_t calculateRampSpeed(int16_t targetSpeed, int16_t currentSpeed, uint16_t rampTime);
void updateMotorControl(void);

// Отладка
#ifdef DEBUG_MOTORS
void printMotorStatus(uint8_t motorID);
void printAllMotorsStatus(void);
void dumpModbusPacket(const uint8_t *data, uint16_t length);
#endif

#endif // MOTORS_H