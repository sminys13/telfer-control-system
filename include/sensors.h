/**
 * @file sensors.h
 * @brief Управление датчиками системы: лазерные дальномеры и ультразвуковые датчики
 * @version 4.0
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "../include/config.h"

// ========== КОНСТАНТЫ ДАТЧИКОВ ==========

// Команды для лазерных дальномеров
#define LASER_CMD_LASER_ON 0x80, 0x06, 0x05, 0x01, 0x74
#define LASER_CMD_LASER_OFF 0x80, 0x06, 0x05, 0x00, 0x75
#define LASER_CMD_SINGLE_MEAS 0x80, 0x06, 0x02, 0x78
#define LASER_CMD_CONT_MEAS 0x80, 0x06, 0x03, 0x77
#define LASER_CMD_SET_FREQ_5HZ 0x04, 0x0A, 0x05, 0xF3
#define LASER_CMD_SET_FREQ_10HZ 0x04, 0x0A, 0x0A, 0xEE
#define LASER_CMD_SET_FREQ_20HZ 0x04, 0x0A, 0x14, 0xE4
#define LASER_CMD_SET_RES_1MM 0x04, 0x0C, 0x01, 0xF5
#define LASER_CMD_SET_RES_0_1MM 0x04, 0x0C, 0x02, 0xF4

// Структура пакета данных лазерного дальномера
#pragma pack(push, 1)
typedef struct
{
    uint8_t header1;  // 0x80
    uint8_t header2;  // 0x06
    uint8_t dataType; // 0x82 - данные измерения
    char distance[7]; // ASCII расстояние (например: "123.456")
    uint8_t checksum; // Контрольная сумма
} LaserDataPacket;
#pragma pack(pop)

// Структура для хранения отфильтрованных данных датчика
typedef struct
{
    int32_t rawValue;      // Сырое значение (мм)
    int32_t filteredValue; // Отфильтрованное значение (мм)
    int32_t minValue;      // Минимальное зафиксированное значение
    int32_t maxValue;      // Максимальное зафиксированное значение
    int32_t averageValue;  // Скользящее среднее
    uint32_t lastUpdate;   // Время последнего обновления
    bool isValid;          // Данные валидны
    uint8_t errorCount;    // Счетчик ошибок
    float velocity;        // Скорость изменения (мм/с)
} SensorData;

// Структура для фильтра Калмана
typedef struct
{
    float estimate;      // Текущая оценка
    float estimateError; // Ошибка оценки
    float processNoise;  // Шум процесса
    float measureNoise;  // Шум измерения
    float kalmanGain;    // Коэффициент Калмана
} KalmanFilter;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация
void sensorsInit(void);
void laserSensorsInit(HardwareSerial *serial1, HardwareSerial *serial2);
void ultrasonicSensorsInit(void);
bool performSensorSelfTest(void);

// Лазерные дальномеры
bool sendLaserCommand(HardwareSerial *serial, const uint8_t *command, uint8_t length);
float readLaserDistance(HardwareSerial *serial);
bool parseLaserData(const uint8_t *buffer, uint8_t length, float *distance);
void setLaserFrequency(HardwareSerial *serial, uint8_t frequency);
void setLaserResolution(HardwareSerial *serial, bool highResolution);
void setLaserMode(HardwareSerial *serial, bool continuousMode);
void laserPowerControl(HardwareSerial *serial, bool powerOn);

// Ультразвуковые датчики
int32_t readUltrasonicDistance(uint8_t trigPin, uint8_t echoPin);
int32_t measureUltrasonicPulse(uint8_t trigPin, uint8_t echoPin, uint32_t timeout = 30000);
void ultrasonicCalibration(uint8_t trigPin, uint8_t echoPin, int32_t *offset);

// Фильтрация и обработка данных
void initKalmanFilter(KalmanFilter *filter, float initialEstimate, float processNoise, float measureNoise);
float updateKalmanFilter(KalmanFilter *filter, float measurement);
int32_t applyMovingAverage(SensorData *sensor, int32_t newValue, uint8_t windowSize);
int32_t applyMedianFilter(int32_t *values, uint8_t size, int32_t newValue);
int32_t applyLowPassFilter(SensorData *sensor, int32_t newValue, float alpha);

// Обновление данных
void updateAllSensors(SystemStatus *status);
void updateLaserSensors(SystemStatus *status);
void updateUltrasonicSensors(SystemStatus *status);
void calculateDerivedValues(SystemStatus *status);

// Валидация и диагностика
bool validateSensorData(const SensorData *sensor, int32_t minExpected, int32_t maxExpected);
bool checkSensorConsistency(const SystemStatus *status);
void diagnoseSensorIssues(SystemStatus *status);
void resetSensorErrors(SensorData *sensor);

// Утилиты
float calculateVelocity(SensorData *sensor, int32_t newValue, uint32_t currentTime);
int32_t convertMetersToMillimeters(float meters);
float convertMillimetersToMeters(int32_t millimeters);
float calculateSensorVariance(const SensorData *sensor, uint8_t sampleCount);

// Отладка
#ifdef DEBUG_SENSORS
void printSensorData(const SensorData *sensor, const char *name);
void printLaserPacket(const uint8_t *data, uint8_t length);
#endif

#endif // SENSORS_H