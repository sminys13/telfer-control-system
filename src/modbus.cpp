/**
 * @file modbus.cpp
 * @brief Реализация Modbus RTU протокола
 * @version 4.0
 */

#include "../include/modbus.h"
#include <Arduino.h>

// ========== ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Параметры Modbus
static HardwareSerial *modbusSerial = NULL;
static uint8_t dePin = 0;
static uint32_t baudRate = 9600;
static uint32_t responseTimeout = MODBUS_RESPONSE_TIMEOUT_MS;
static uint8_t retryCount = 3;
static bool initialized = false;

// Буферы для передачи/приема
static uint8_t txBuffer[256];
static uint8_t rxBuffer[256];
static uint8_t rxIndex = 0;

// Состояния устройств
static ModbusDeviceStatus deviceStatus[5]; // 0-4 адреса
static ModbusStats modbusStats = {0};

// ========== ИНИЦИАЛИЗАЦИЯ ==========

/**
 * @brief Инициализация Modbus
 * @param serial Указатель на последовательный порт
 * @param dePin Пин управления направлением DE/RE
 * @param baudRate Скорость передачи
 * @return true если инициализация успешна
 */
bool modbusInit(HardwareSerial *serial, uint8_t dePin, uint32_t baudRate)
{
    if (!serial)
    {
        Serial.println(F("ОШИБКА Modbus: Неверный указатель на порт"));
        return false;
    }

    Serial.println(F("Инициализация Modbus RTU..."));

    modbusSerial = serial;
    modbusSerial->begin(baudRate);

    pinMode(dePin, OUTPUT);
    digitalWrite(dePin, LOW); // Режим приема по умолчанию

    ::dePin = dePin;
    ::baudRate = baudRate;

    // Инициализация статусов устройств
    for (uint8_t i = 0; i < 5; i++)
    {
        deviceStatus[i].address = i;
        deviceStatus[i].connected = false;
        deviceStatus[i].lastResponse = 0;
        deviceStatus[i].faultCode = 0;
        deviceStatus[i].warningCode = 0;
        deviceStatus[i].status = 0;
        deviceStatus[i].outputFrequency = 0;
        deviceStatus[i].outputCurrent = 0;
        deviceStatus[i].dcVoltage = 0;
        deviceStatus[i].outputVoltage = 0;
        deviceStatus[i].outputPower = 0;
        deviceStatus[i].outputTorque = 0;
        deviceStatus[i].digitalInputs = 0;
        deviceStatus[i].analogInput1 = 0;
        deviceStatus[i].analogInput2 = 0;
    }

    // Сброс статистики
    modbusResetStats();

    // Очистка буферов
    while (modbusSerial->available())
    {
        modbusSerial->read();
    }

    initialized = true;

    Serial.print(F("Modbus инициализирован на порту "));
    Serial.print(baudRate);
    Serial.println(F(" бод"));

    return true;
}

// ========== ОСНОВНЫЕ ФУНКЦИИ ==========

/**
 * @brief Чтение регистров хранения
 * @param address Адрес устройства
 * @param startAddr Начальный адрес
 * @param quantity Количество регистров
 * @param values Массив для значений
 * @return true если операция успешна
 */
bool modbusReadHoldingRegisters(uint8_t address, uint16_t startAddr,
                                uint16_t quantity, uint16_t *values)
{
    if (!initialized || quantity == 0 || quantity > 125)
    {
        return false;
    }

    ModbusRequest request;
    request.address = address;
    request.function = MODBUS_READ_HOLDING_REGS;
    request.startAddr = startAddr;
    request.quantity = quantity;
    request.dataLength = 0;

    ModbusResponse response;

    for (uint8_t retry = 0; retry < retryCount; retry++)
    {
        modbusStats.totalRequests++;

        if (modbusSendRequest(&request) &&
            modbusReceiveResponse(&response, responseTimeout))
        {
            modbusStats.totalResponses++;

            if (response.error == 0 && response.dataLength == quantity * 2)
            {
                // Преобразование данных
                for (uint16_t i = 0; i < quantity; i++)
                {
                    values[i] = (response.data[i * 2] << 8) | response.data[i * 2 + 1];
                }

                // Обновление статуса устройства
                deviceStatus[address].connected = true;
                deviceStatus[address].lastResponse = millis();

                return true;
            }
            else if (response.error != 0)
            {
                modbusStats.deviceErrors++;
                Serial.print(F("Modbus ошибка устройства "));
                Serial.print(address, HEX);
                Serial.print(F(": код 0x"));
                Serial.println(response.error, HEX);
            }
        }
        else
        {
            modbusStats.timeoutErrors++;
        }

        delay(10);
    }

    deviceStatus[address].connected = false;
    return false;
}

/**
 * @brief Запись одного регистра
 * @param address Адрес устройства
 * @param registerAddr Адрес регистра
 * @param value Значение
 * @return true если операция успешна
 */
bool modbusWriteSingleRegister(uint8_t address, uint16_t registerAddr, uint16_t value)
{
    if (!initialized)
    {
        return false;
    }

    ModbusRequest request;
    request.address = address;
    request.function = MODBUS_WRITE_SINGLE_REG;
    request.startAddr = registerAddr;
    request.quantity = 1;

    // Данные для записи
    request.data[0] = (value >> 8) & 0xFF;
    request.data[1] = value & 0xFF;
    request.dataLength = 2;

    ModbusResponse response;

    for (uint8_t retry = 0; retry < retryCount; retry++)
    {
        modbusStats.totalRequests++;

        if (modbusSendRequest(&request) &&
            modbusReceiveResponse(&response, responseTimeout))
        {
            modbusStats.totalResponses++;

            if (response.error == 0 && response.dataLength == 4)
            {
                // Проверка, что записанное значение совпадает
                uint16_t writtenAddr = (response.data[0] << 8) | response.data[1];
                uint16_t writtenValue = (response.data[2] << 8) | response.data[3];

                if (writtenAddr == registerAddr && writtenValue == value)
                {
                    deviceStatus[address].connected = true;
                    deviceStatus[address].lastResponse = millis();
                    return true;
                }
            }
        }
        else
        {
            modbusStats.timeoutErrors++;
        }

        delay(10);
    }

    deviceStatus[address].connected = false;
    return false;
}

// ========== УПРАВЛЕНИЕ ЧАСТОТНЫМИ ПРЕОБРАЗОВАТЕЛЯМИ ==========

/**
 * @brief Запуск частотного преобразователя
 * @param address Адрес устройства
 * @param forward Направление (true - вперед, false - назад)
 * @return true если команда успешно отправлена
 */
bool modbusDriveStart(uint8_t address, bool forward)
{
    uint16_t command = forward ? CMD_FORWARD : CMD_REVERSE;
    return modbusWriteSingleRegister(address, REG_COMMAND_START_STOP, command);
}

/**
 * @brief Остановка частотного преобразователя
 * @param address Адрес устройства
 * @return true если команда успешно отправлена
 */
bool modbusDriveStop(uint8_t address)
{
    return modbusWriteSingleRegister(address, REG_COMMAND_START_STOP, CMD_STOP);
}

/**
 * @brief Аварийная остановка частотного преобразователя
 * @param address Адрес устройства
 * @return true если команда успешно отправлена
 */
bool modbusDriveEmergencyStop(uint8_t address)
{
    return modbusWriteSingleRegister(address, REG_COMMAND_START_STOP, CMD_STOP_FREEWHEEL);
}

/**
 * @brief Установка частоты вращения
 * @param address Адрес устройства
 * @param frequencyPercent Частота в процентах (-100..+100)
 * @return true если команда успешно отправлена
 */
bool modbusDriveSetFrequency(uint8_t address, float frequencyPercent)
{
    // Ограничение частоты
    frequencyPercent = constrain(frequencyPercent, -100.0f, 100.0f);

    // Преобразование в значение регистра
    uint16_t registerValue = modbusPercentToRegister(frequencyPercent);

    return modbusWriteSingleRegister(address, REG_FREQUENCY_SETPOINT, registerValue);
}

/**
 * @brief Получение статуса частотного преобразователя
 * @param address Адрес устройства
 * @param status Структура для сохранения статуса
 * @return true если статус успешно получен
 */
bool modbusDriveGetStatus(uint8_t address, ModbusDeviceStatus *status)
{
    if (!status)
    {
        return false;
    }

    uint16_t registers[11];

    // Чтение основных регистров
    if (!modbusReadHoldingRegisters(address, REG_STATUS, 11, registers))
    {
        return false;
    }

    // Заполнение структуры статуса
    status->status = registers[0];                   // REG_STATUS
    status->faultCode = registers[1];                // REG_FAULT_CODE
    status->warningCode = registers[2];              // REG_WARNING_CODE
    status->outputFrequency = registers[3] / 100.0f; // REG_OUTPUT_FREQUENCY (0.1 Гц)
    status->dcVoltage = registers[5] / 10.0f;        // REG_DC_BUS_VOLTAGE (0.1 В)
    status->outputVoltage = registers[6] / 10.0f;    // REG_OUTPUT_VOLTAGE (0.1 В)
    status->outputCurrent = registers[7] / 100.0f;   // REG_OUTPUT_CURRENT (0.1 А)
    status->outputPower = registers[9] / 1000.0f;    // REG_OUTPUT_POWER (0.001 кВт)
    status->outputTorque = registers[10] / 10.0f;    // REG_OUTPUT_TORQUE (0.1%)

    // Чтение статуса цифровых входов
    if (modbusReadHoldingRegisters(address, REG_DIGITAL_INPUTS_STATUS, 1, registers))
    {
        status->digitalInputs = registers[0];
    }

    // Чтение аналоговых входов
    if (modbusReadHoldingRegisters(address, REG_ANALOG_INPUT_AI1, 2, registers))
    {
        status->analogInput1 = registers[0];
        status->analogInput2 = registers[1];
    }

    return true;
}

/**
 * @brief Обновление статуса всех устройств
 * @return Количество успешно обновленных устройств
 */
bool modbusDriveUpdateAllStatus(void)
{
    uint8_t updatedCount = 0;

    for (uint8_t addr = MODBUS_ADDR_H1; addr <= MODBUS_ADDR_V2; addr++)
    {
        ModbusDeviceStatus status;

        if (modbusDriveGetStatus(addr, &status))
        {
            deviceStatus[addr] = status;
            updatedCount++;
        }
    }

    // Обновление статистики успешных запросов
    if (modbusStats.totalRequests > 0)
    {
        modbusStats.successRate = (100.0f * modbusStats.totalResponses) / modbusStats.totalRequests;
    }

    return (updatedCount > 0);
}

// ========== НИЗКОУРОВНЕВЫЕ ФУНКЦИИ ==========

/**
 * @brief Отправка Modbus запроса
 * @param request Указатель на запрос
 * @return true если запрос отправлен
 */
bool modbusSendRequest(ModbusRequest *request)
{
    if (!request || !modbusSerial)
    {
        return false;
    }

    uint8_t index = 0;

    // Формирование пакета
    txBuffer[index++] = request->address;
    txBuffer[index++] = request->function;
    txBuffer[index++] = (request->startAddr >> 8) & 0xFF;
    txBuffer[index++] = request->startAddr & 0xFF;

    if (request->function == MODBUS_READ_HOLDING_REGS ||
        request->function == MODBUS_READ_INPUT_REGS)
    {
        txBuffer[index++] = (request->quantity >> 8) & 0xFF;
        txBuffer[index++] = request->quantity & 0xFF;
    }
    else if (request->function == MODBUS_WRITE_SINGLE_REG)
    {
        txBuffer[index++] = (request->data[0] >> 8) & 0xFF;
        txBuffer[index++] = request->data[0] & 0xFF;
        txBuffer[index++] = (request->data[1] >> 8) & 0xFF;
        txBuffer[index++] = request->data[1] & 0xFF;
    }
    else if (request->function == MODBUS_WRITE_MULTIPLE_REGS)
    {
        txBuffer[index++] = (request->quantity >> 8) & 0xFF;
        txBuffer[index++] = request->quantity & 0xFF;
        txBuffer[index++] = request->dataLength;

        for (uint8_t i = 0; i < request->dataLength; i++)
        {
            txBuffer[index++] = request->data[i];
        }
    }

    // Расчет и добавление CRC
    uint16_t crc = modbusCalculateCRC(txBuffer, index);
    txBuffer[index++] = crc & 0xFF;
    txBuffer[index++] = (crc >> 8) & 0xFF;

    // Очистка входного буфера
    while (modbusSerial->available())
    {
        modbusSerial->read();
    }

    // Переключение в режим передачи
    digitalWrite(dePin, HIGH);
    delayMicroseconds(100);

    // Отправка данных
    modbusSerial->write(txBuffer, index);
    modbusSerial->flush();

    // Ожидание завершения передачи
    delayMicroseconds(MODBUS_INTER_FRAME_DELAY_US);

    // Переключение в режим приема
    digitalWrite(dePin, LOW);

#ifdef DEBUG_MODBUS
    modbusPrintRequest(request);
#endif

    return true;
}

/**
 * @brief Прием Modbus ответа
 * @param response Указатель на структуру ответа
 * @param timeout Таймаут ожидания (мс)
 * @return true если ответ получен
 */
bool modbusReceiveResponse(ModbusResponse *response, uint32_t timeout)
{
    if (!response || !modbusSerial)
    {
        return false;
    }

    memset(response, 0, sizeof(ModbusResponse));
    rxIndex = 0;

    uint32_t startTime = millis();

    // Ожидание начала ответа
    while (millis() - startTime < timeout)
    {
        if (modbusSerial->available())
        {
            break;
        }
    }

    if (!modbusSerial->available())
    {
        return false; // Таймаут
    }

    // Чтение ответа
    while (millis() - startTime < timeout && rxIndex < sizeof(rxBuffer))
    {
        if (modbusSerial->available())
        {
            rxBuffer[rxIndex++] = modbusSerial->read();
            startTime = millis(); // Сброс таймаута при получении данных
        }
    }

    // Проверка минимальной длины ответа
    if (rxIndex < 5) // Адрес + функция + 2 байта CRC
    {
        return false;
    }

    // Проверка CRC
    uint16_t receivedCRC = (rxBuffer[rxIndex - 1] << 8) | rxBuffer[rxIndex - 2];
    uint16_t calculatedCRC = modbusCalculateCRC(rxBuffer, rxIndex - 2);

    if (receivedCRC != calculatedCRC)
    {
        modbusStats.crcErrors++;
        return false;
    }

    // Заполнение структуры ответа
    response->address = rxBuffer[0];
    response->function = rxBuffer[1];

    // Проверка на ошибку
    if (response->function & 0x80)
    {
        response->error = rxBuffer[2];
        response->dataLength = 0;
    }
    else
    {
        response->error = 0;

        if (response->function == MODBUS_READ_HOLDING_REGS ||
            response->function == MODBUS_READ_INPUT_REGS)
        {
            response->dataLength = rxBuffer[2];
            memcpy(response->data, &rxBuffer[3], response->dataLength);
        }
        else if (response->function == MODBUS_WRITE_SINGLE_REG)
        {
            response->dataLength = 4;
            memcpy(response->data, &rxBuffer[2], 4);
        }
    }

    response->crc = receivedCRC;
    response->isValid = true;

#ifdef DEBUG_MODBUS
    modbusPrintResponse(response);
#endif

    return true;
}

/**
 * @brief Расчет CRC16 Modbus
 * @param data Указатель на данные
 * @param length Длина данных
 * @return CRC16
 */
uint16_t modbusCalculateCRC(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t pos = 0; pos < length; pos++)
    {
        crc ^= (uint16_t)data[pos];

        for (uint8_t i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// ========== УТИЛИТЫ ==========

/**
 * @brief Преобразование процентов в значение регистра
 * @param percent Проценты (-100..+100)
 * @return Значение регистра
 */
uint16_t modbusPercentToRegister(float percent)
{
    int32_t value = (int32_t)(percent * FREQ_SCALE);
    value = constrain(value, FREQ_MIN, FREQ_MAX);
    return (uint16_t)value;
}

/**
 * @brief Преобразование значения регистра в проценты
 * @param registerValue Значение регистра
 * @return Проценты
 */
float modbusRegisterToPercent(uint16_t registerValue)
{
    int16_t value = (int16_t)registerValue;
    return value / FREQ_SCALE;
}

/**
 * @brief Получение строкового описания ошибки Modbus
 * @param errorCode Код ошибки
 * @return Описание ошибки
 */
const char *modbusErrorToString(uint8_t errorCode)
{
    static const char *errorStrings[] = {
        "Нет ошибки",
        "Недопустимая функция",
        "Недопустимый адрес данных",
        "Недопустимое значение данных",
        "Ошибка ведомого устройства",
        "Подтверждение",
        "Ведомое устройство занято",
        "Ошибка четности памяти",
        "Недоступен шлюз",
        "Нет ответа от целевого устройства"};

    if (errorCode < sizeof(errorStrings) / sizeof(errorStrings[0]))
    {
        return errorStrings[errorCode];
    }

    return "Неизвестная ошибка";
}

// ========== ОТЛАДКА ==========

#ifdef DEBUG_MODBUS
/**
 * @brief Вывод запроса Modbus
 */
void modbusPrintRequest(const ModbusRequest *request)
{
    Serial.print(F("Modbus запрос: Адрес=0x"));
    if (request->address < 0x10)
        Serial.print('0');
    Serial.print(request->address, HEX);

    Serial.print(F(", Функция=0x"));
    if (request->function < 0x10)
        Serial.print('0');
    Serial.print(request->function, HEX);

    Serial.print(F(", Адрес=0x"));
    Serial.print(request->startAddr, HEX);

    Serial.print(F(", Количество="));
    Serial.println(request->quantity);
}

/**
 * @brief Вывод статистики Modbus
 */
void modbusPrintStats(void)
{
    Serial.println(F("\n=== СТАТИСТИКА MODBUS ==="));

    Serial.print(F("Всего запросов: "));
    Serial.println(modbusStats.totalRequests);

    Serial.print(F("Всего ответов: "));
    Serial.println(modbusStats.totalResponses);

    Serial.print(F("Ошибок таймаута: "));
    Serial.println(modbusStats.timeoutErrors);

    Serial.print(F("Ошибок CRC: "));
    Serial.println(modbusStats.crcErrors);

    Serial.print(F("Ошибок устройств: "));
    Serial.println(modbusStats.deviceErrors);

    Serial.print(F("Успешных запросов: "));
    Serial.print(modbusStats.successRate, 1);
    Serial.println(F("%"));

    Serial.println(F("==========================\n"));
}
#endif