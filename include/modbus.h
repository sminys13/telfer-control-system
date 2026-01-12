/**
 * @file modbus.h
 * @brief Modbus RTU протокол для управления частотными преобразователями HE200
 * @version 4.0
 */

#ifndef MODBUS_H
#define MODBUS_H

#include <Arduino.h>
#include <HardwareSerial.h>

// ========== КОНСТАНТЫ MODBUS ==========

// Адреса устройств
#define MODBUS_ADDR_BROADCAST 0x00 // Широковещательный адрес
#define MODBUS_ADDR_H1 0x01        // Горизонтальный левый
#define MODBUS_ADDR_H2 0x02        // Горизонтальный правый
#define MODBUS_ADDR_V1 0x03        // Вертикальный левый
#define MODBUS_ADDR_V2 0x04        // Вертикальный правый

// Функции Modbus
#define MODBUS_READ_COILS 0x01
#define MODBUS_READ_INPUTS 0x02
#define MODBUS_READ_HOLDING_REGS 0x03
#define MODBUS_READ_INPUT_REGS 0x04
#define MODBUS_WRITE_SINGLE_COIL 0x05
#define MODBUS_WRITE_SINGLE_REG 0x06
#define MODBUS_WRITE_MULTIPLE_COILS 0x0F
#define MODBUS_WRITE_MULTIPLE_REGS 0x10
#define MODBUS_DIAGNOSTICS 0x08

// Коды ошибок Modbus
#define MODBUS_ERROR_ILLEGAL_FUNCTION 0x01
#define MODBUS_ERROR_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_ERROR_ILLEGAL_DATA_VALUE 0x03
#define MODBUS_ERROR_SLAVE_DEVICE_FAILURE 0x04
#define MODBUS_ERROR_ACKNOWLEDGE 0x05
#define MODBUS_ERROR_SLAVE_DEVICE_BUSY 0x06
#define MODBUS_ERROR_MEMORY_PARITY_ERROR 0x08
#define MODBUS_ERROR_GATEWAY_PATH_UNAVAILABLE 0x0A
#define MODBUS_ERROR_GATEWAY_TARGET_NO_RESPONSE 0x0B

// Регистры частотного преобразователя HE200
#define REG_COMMAND_START_STOP 0x0001    // Команда пуск/стоп
#define REG_FREQUENCY_SETPOINT 0x0002    // Задание частоты
#define REG_STATUS 0x0020                // Статус привода
#define REG_FAULT_CODE 0x0021            // Код ошибки
#define REG_WARNING_CODE 0x0022          // Код предупреждения
#define REG_OUTPUT_FREQUENCY 0x0023      // Выходная частота
#define REG_SETPOINT_FREQUENCY 0x0024    // Заданная частота
#define REG_DC_BUS_VOLTAGE 0x0025        // Напряжение DC шины
#define REG_OUTPUT_VOLTAGE 0x0026        // Выходное напряжение
#define REG_OUTPUT_CURRENT 0x0027        // Выходной ток
#define REG_OUTPUT_SPEED 0x0028          // Выходная скорость (об/мин)
#define REG_OUTPUT_POWER 0x0029          // Выходная мощность
#define REG_OUTPUT_TORQUE 0x002A         // Выходной момент
#define REG_DIGITAL_INPUTS_STATUS 0x0030 // Статус цифровых входов
#define REG_ANALOG_INPUT_AI1 0x002D      // Аналоговый вход AI1
#define REG_ANALOG_INPUT_AI2 0x002E      // Аналоговый вход AI2

// Команды для регистра 0x0001
#define CMD_STOP 0x0004           // Остановка
#define CMD_STOP_FREEWHEEL 0x0004 // Останов на выбеге
#define CMD_FORWARD 0x0001        // Вперед
#define CMD_REVERSE 0x0002        // Назад
#define CMD_FAULT_RESET 0x0005    // Сброс ошибок

// Значения для регистра задания частоты (0x0002)
#define FREQ_MIN -10000  // -100.00%
#define FREQ_MAX 10000   // +100.00%
#define FREQ_SCALE 100.0 // Масштаб (100.00 = 100.00%)

// Таймауты
#define MODBUS_RESPONSE_TIMEOUT_MS 200   // Таймаут ответа
#define MODBUS_INTER_FRAME_DELAY_US 3500 // Задержка между кадрами (3.5 символа)

// ========== СТРУКТУРЫ ДАННЫХ ==========

// Структура Modbus запроса
typedef struct
{
    uint8_t address;    // Адрес устройства
    uint8_t function;   // Функция
    uint16_t startAddr; // Начальный адрес
    uint16_t quantity;  // Количество регистров/коилов
    uint8_t data[252];  // Данные (для записи)
    uint8_t dataLength; // Длина данных
} ModbusRequest;

// Структура Modbus ответа
typedef struct
{
    uint8_t address;    // Адрес устройства
    uint8_t function;   // Функция
    uint8_t data[252];  // Данные
    uint8_t dataLength; // Длина данных
    uint8_t error;      // Код ошибки (если function > 0x80)
    bool isValid;       // Валидность ответа
    uint16_t crc;       // CRC ответа
} ModbusResponse;

// Структура состояния Modbus устройства
typedef struct
{
    uint8_t address;        // Адрес устройства
    bool connected;         // Устройство подключено
    uint32_t lastResponse;  // Время последнего ответа
    uint16_t faultCode;     // Код ошибки
    uint16_t warningCode;   // Код предупреждения
    uint16_t status;        // Статус
    float outputFrequency;  // Выходная частота (Гц)
    float outputCurrent;    // Выходной ток (А)
    float dcVoltage;        // Напряжение DC шины (В)
    float outputVoltage;    // Выходное напряжение (В)
    float outputPower;      // Выходная мощность (кВт)
    float outputTorque;     // Выходной момент (%)
    uint16_t digitalInputs; // Цифровые входы
    uint16_t analogInput1;  // Аналоговый вход 1
    uint16_t analogInput2;  // Аналоговый вход 2
} ModbusDeviceStatus;

// Структура статистики Modbus
typedef struct
{
    uint32_t totalRequests;  // Всего запросов
    uint32_t totalResponses; // Всего ответов
    uint32_t timeoutErrors;  // Ошибки таймаута
    uint32_t crcErrors;      // Ошибки CRC
    uint32_t formatErrors;   // Ошибки формата
    uint32_t deviceErrors;   // Ошибки устройств
    float successRate;       // Процент успешных запросов
} ModbusStats;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация
bool modbusInit(HardwareSerial *serial, uint8_t dePin, uint32_t baudRate);
void modbusSetTimeout(uint32_t timeoutMs);
void modbusSetRetryCount(uint8_t retries);
bool modbusIsInitialized(void);

// Основные функции
bool modbusReadHoldingRegisters(uint8_t address, uint16_t startAddr,
                                uint16_t quantity, uint16_t *values);
bool modbusWriteSingleRegister(uint8_t address, uint16_t registerAddr,
                               uint16_t value);
bool modbusWriteMultipleRegisters(uint8_t address, uint16_t startAddr,
                                  uint16_t quantity, uint16_t *values);
bool modbusReadInputRegisters(uint8_t address, uint16_t startAddr,
                              uint16_t quantity, uint16_t *values);

// Управление частотными преобразователями
bool modbusDriveStart(uint8_t address, bool forward);
bool modbusDriveStop(uint8_t address);
bool modbusDriveEmergencyStop(uint8_t address);
bool modbusDriveSetFrequency(uint8_t address, float frequencyPercent);
bool modbusDriveFaultReset(uint8_t address);
bool modbusDriveGetStatus(uint8_t address, ModbusDeviceStatus *status);
bool modbusDriveUpdateAllStatus(void);

// Вспомогательные функции
ModbusDeviceStatus *modbusGetDeviceStatus(uint8_t address);
ModbusStats *modbusGetStats(void);
void modbusResetStats(void);
bool modbusCheckDeviceConnection(uint8_t address);
bool modbusScanDevices(uint8_t *foundDevices, uint8_t maxDevices);

// Низкоуровневые функции
bool modbusSendRequest(ModbusRequest *request);
bool modbusReceiveResponse(ModbusResponse *response, uint32_t timeout);
uint16_t modbusCalculateCRC(const uint8_t *data, uint8_t length);
bool modbusValidateResponse(const ModbusResponse *response,
                            const ModbusRequest *request);

// Утилиты
float modbusRegistersToFloat(uint16_t high, uint16_t low);
void modbusFloatToRegisters(float value, uint16_t *high, uint16_t *low);
uint16_t modbusPercentToRegister(float percent);
float modbusRegisterToPercent(uint16_t registerValue);
const char *modbusErrorToString(uint8_t errorCode);
const char *modbusFaultToString(uint16_t faultCode);

// Отладка
#ifdef DEBUG_MODBUS
void modbusPrintRequest(const ModbusRequest *request);
void modbusPrintResponse(const ModbusResponse *response);
void modbusPrintDeviceStatus(uint8_t address);
void modbusPrintStats(void);
#endif

#endif // MODBUS_H