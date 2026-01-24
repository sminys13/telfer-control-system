/**
 * @file motors.cpp
 * @brief Реализация управления двигателями через частотные преобразователи HE200
 * @version 4.0
 */

#include "../include/motors.h"
#include "../include/modbus.h"
#include <HardwareSerial.h>

// ========== ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Серийный порт для RS-485
static HardwareSerial *rs485Serial = &Serial3;

// Статусы двигателей
static MotorStatus motorStatus[MOTOR_COUNT];
static MotorConfig motorConfig[MOTOR_COUNT];

// Синхронное управление
static SyncControl horizontalSync = {0};
static SyncControl verticalSync = {0};

// Временные метки
static uint32_t lastCommandTime = 0;
// static uint32_t lastStatusUpdate = 0;
// static uint32_t lastHealthCheck = 0;

// Статистика
static uint32_t commandCount = 0;
static uint32_t errorCount = 0;
static uint32_t emergencyStopCount = 0;

// Буферы для Modbus
static uint8_t modbusTxBuffer[64];
// static uint8_t modbusRxBuffer[64];
// static uint8_t modbusBufferIndex = 0;

// Флаги
static bool rs485Initialized = false;
static bool motorsEnabled = false;
static bool emergencyState = false;

// ========== ИНИЦИАЛИЗАЦИЯ ==========

/**
 * @brief Инициализация системы управления двигателями
 */
void motorsInit(void)
{
    Serial.println(F("Инициализация системы управления двигателями..."));

    // 1. Инициализация интерфейса RS-485
    if (!initRS485Interface())
    {
        Serial.println(F("ОШИБКА: Не удалось инициализировать RS-485"));
        return;
    }

    // 2. Инициализация конфигураций двигателей
    initMotorConfigurations();

    // 3. Инициализация структур статусов
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        memset(&motorStatus[i], 0, sizeof(MotorStatus));
        motorStatus[i].state = MOTOR_STATE_IDLE;
        motorStatus[i].error = MOTOR_ERROR_NONE;
        motorStatus[i].enabled = false;
        motorStatus[i].faultResetPending = false;
        motorStatus[i].maxSpeed = 1000; // 100%
    }

    // 4. Настройка синхронного управления
    horizontalSync.master = &motorStatus[MOTOR_HORIZONTAL_LEFT];
    horizontalSync.slave = &motorStatus[MOTOR_HORIZONTAL_RIGHT];
    horizontalSync.maxPositionError = 100; // 100 импульсов
    horizontalSync.syncGain = 50;          // 50%
    horizontalSync.enabled = true;

    verticalSync.master = &motorStatus[MOTOR_VERTICAL_LEFT];
    verticalSync.slave = &motorStatus[MOTOR_VERTICAL_RIGHT];
    verticalSync.maxPositionError = 50; // 50 импульсов
    verticalSync.syncGain = 50;         // 50%
    verticalSync.enabled = true;

    // 5. Обнаружение и проверка двигателей
    if (!detectMotors())
    {
        Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: Не все двигатели обнаружены"));
    }

    // 6. Самодиагностика
    if (performMotorSelfTest())
    {
        Serial.println(F("Самодиагностика двигателей: УСПЕХ"));
        motorsEnabled = true;
    }
    else
    {
        Serial.println(F("Самодиагностика двигателей: ОШИБКА"));
        motorsEnabled = false;
    }

    Serial.println(F("Система управления двигателями инициализирована"));
}

/**
 * @brief Инициализация интерфейса RS-485
 * @return true если инициализация успешна
 */
bool initRS485Interface(void)
{
    Serial.println(F("Инициализация интерфейса RS-485..."));

    // Инициализация Modbus
    if (!modbusInit(&Serial3, RS485_RE_DE_PIN, SERIAL_RS485_BAUD))
    {
        Serial.println(F("ОШИБКА: Не удалось инициализировать Modbus"));
        return false;
    }

    // Настройка таймаутов и повторных попыток
    modbusSetTimeout(200);
    modbusSetRetryCount(3);

    // Сканирование устройств на шине
    Serial.println(F("Поиск устройств Modbus..."));

    uint8_t foundDevices[10];
    uint8_t deviceCount = 0;

    if (modbusScanDevices(foundDevices, 10))
    {
        Serial.print(F("Найдено устройств: "));
        Serial.println(deviceCount);

        for (uint8_t i = 0; i < deviceCount; i++)
        {
            Serial.print(F("  Адрес 0x"));
            if (foundDevices[i] < 0x10)
                Serial.print('0');
            Serial.println(foundDevices[i], HEX);
        }
    }
    else
    {
        Serial.println(F("Устройства не найдены. Проверьте подключение."));
    }

    // Проверка связи с каждым двигателем
    bool allMotorsOK = true;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        uint8_t address = motorConfig[i].address;

        Serial.print(F("Двигатель "));
        Serial.print(i + 1);
        Serial.print(F(" (адрес 0x"));
        if (address < 0x10)
            Serial.print('0');
        Serial.print(address, HEX);
        Serial.print(F("): "));

        // Попытка чтения статуса
        ModbusDeviceStatus status;
        if (modbusDriveGetStatus(address, &status))
        {
            Serial.println(F("OK"));
            motorStatus[i].enabled = true;

            // Сброс возможных ошибок
            if (status.faultCode != 0)
            {
                Serial.println(F("  Сброс ошибки..."));
                modbusDriveFaultReset(address);
            }
        }
        else
        {
            Serial.println(F("НЕТ ОТВЕТА"));
            motorStatus[i].enabled = false;
            allMotorsOK = false;
        }

        delay(100);
    }

    rs485Initialized = true;
    return allMotorsOK;
}

/**
 * @brief Инициализация конфигураций двигателей
 */
void initMotorConfigurations(void)
{
    Serial.println(F("Настройка конфигураций двигателей..."));

    // Конфигурация горизонтального левого двигателя
    motorConfig[MOTOR_HORIZONTAL_LEFT].address = MOTOR_H1_ADDR;
    motorConfig[MOTOR_HORIZONTAL_LEFT].maxSpeed = 1500;         // об/мин
    motorConfig[MOTOR_HORIZONTAL_LEFT].ratedCurrent = 3;        // А
    motorConfig[MOTOR_HORIZONTAL_LEFT].accelerationTime = 1000; // мс
    motorConfig[MOTOR_HORIZONTAL_LEFT].decelerationTime = 1000; // мс
    motorConfig[MOTOR_HORIZONTAL_LEFT].overcurrentLimit = 150;  // %
    motorConfig[MOTOR_HORIZONTAL_LEFT].polePairs = 4;
    motorConfig[MOTOR_HORIZONTAL_LEFT].encoderPPR = 1024;
    motorConfig[MOTOR_HORIZONTAL_LEFT].inverted = false;
    motorConfig[MOTOR_HORIZONTAL_LEFT].controlMode = CONTROL_MODE_SPEED;

    // Конфигурация горизонтального правого двигателя
    motorConfig[MOTOR_HORIZONTAL_RIGHT].address = MOTOR_H2_ADDR;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].maxSpeed = 1500;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].ratedCurrent = 3;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].accelerationTime = 1000;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].decelerationTime = 1000;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].overcurrentLimit = 150;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].polePairs = 4;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].encoderPPR = 1024;
    motorConfig[MOTOR_HORIZONTAL_RIGHT].inverted = true; // Инвертирован для симметрии
    motorConfig[MOTOR_HORIZONTAL_RIGHT].controlMode = CONTROL_MODE_SPEED;

    // Конфигурация вертикального левого двигателя
    motorConfig[MOTOR_VERTICAL_LEFT].address = MOTOR_V1_ADDR;
    motorConfig[MOTOR_VERTICAL_LEFT].maxSpeed = 1000;         // Меньше скорость для точности
    motorConfig[MOTOR_VERTICAL_LEFT].ratedCurrent = 5;        // Больший ток (груз)
    motorConfig[MOTOR_VERTICAL_LEFT].accelerationTime = 2000; // Плавнее разгон
    motorConfig[MOTOR_VERTICAL_LEFT].decelerationTime = 2000;
    motorConfig[MOTOR_VERTICAL_LEFT].overcurrentLimit = 120; // Меньший лимит для безопасности
    motorConfig[MOTOR_VERTICAL_LEFT].polePairs = 4;
    motorConfig[MOTOR_VERTICAL_LEFT].encoderPPR = 2048; // Выше разрешение
    motorConfig[MOTOR_VERTICAL_LEFT].inverted = false;
    motorConfig[MOTOR_VERTICAL_LEFT].controlMode = CONTROL_MODE_SPEED;

    // Конфигурация вертикального правого двигателя
    motorConfig[MOTOR_VERTICAL_RIGHT].address = MOTOR_V2_ADDR;
    motorConfig[MOTOR_VERTICAL_RIGHT].maxSpeed = 1000;
    motorConfig[MOTOR_VERTICAL_RIGHT].ratedCurrent = 5;
    motorConfig[MOTOR_VERTICAL_RIGHT].accelerationTime = 2000;
    motorConfig[MOTOR_VERTICAL_RIGHT].decelerationTime = 2000;
    motorConfig[MOTOR_VERTICAL_RIGHT].overcurrentLimit = 120;
    motorConfig[MOTOR_VERTICAL_RIGHT].polePairs = 4;
    motorConfig[MOTOR_VERTICAL_RIGHT].encoderPPR = 2048;
    motorConfig[MOTOR_VERTICAL_RIGHT].inverted = true;
    motorConfig[MOTOR_VERTICAL_RIGHT].controlMode = CONTROL_MODE_SPEED;

    Serial.println(F("Конфигурации двигателей установлены"));
}

/**
 * @brief Обнаружение двигателей на шине RS-485
 * @return true если хотя бы один двигатель обнаружен
 */
bool detectMotors(void)
{
    Serial.println(F("Обнаружение двигателей на шине RS-485..."));

    bool anyMotorDetected = false;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        uint8_t address = motorConfig[i].address;

        // Попытка чтения регистра статуса
        if (sendModbusReadCommand(address, MODBUS_READ_HOLDING_REG, REG_MOTOR_STATUS, 1))
        {
            delay(10);

            if (rs485Serial->available() >= 7)
            { // Минимальный ответ
                uint8_t response[32];
                uint16_t responseLength = 0;

                while (rs485Serial->available())
                {
                    if (responseLength < sizeof(response))
                    {
                        response[responseLength++] = rs485Serial->read();
                    }
                }

                if (responseLength >= 7 && response[0] == address)
                {
                    Serial.print(F("Двигатель "));
                    Serial.print(i + 1);
                    Serial.print(F(" (адрес 0x"));
                    if (address < 0x10)
                        Serial.print('0');
                    Serial.print(address, HEX);
                    Serial.println(F(") обнаружен"));

                    motorStatus[i].enabled = true;
                    anyMotorDetected = true;

                    // Чтение параметров двигателя
                    readMotorParameters(i);
                }
            }
        }

        delay(50); // Пауза между опросами
    }

    if (!anyMotorDetected)
    {
        Serial.println(F("ВНИМАНИЕ: Двигатели не обнаружены. Проверьте подключение RS-485."));
    }

    return anyMotorDetected;
}

// ========== УПРАВЛЕНИЕ ОТДЕЛЬНЫМИ ДВИГАТЕЛЯМИ ==========

/**
 * @brief Установка скорости двигателя в сырых значениях
 * @param motorID ID двигателя (0-3)
 * @param speed Скорость (-1000..+1000)
 * @param direction Направление (true - вперед/вверх, false - назад/вниз)
 * @return true если команда отправлена успешно
 */
bool setMotorSpeed(uint8_t motorID, int16_t speed, bool direction)
{
    if (motorID >= MOTOR_COUNT || !motorsEnabled || emergencyState)
    {
        return false;
    }

    // Ограничение скорости
    speed = constrain(speed, -1000, 1000);

    // Для инвертированных двигателей инвертируем направление
    if (motorConfig[motorID].inverted)
    {
        direction = !direction;
    }

    uint8_t address = motorConfig[motorID].address;

    // Установка направления (пуск/стоп)
    uint16_t command = (speed == 0) ? CMD_STOP : (direction ? CMD_FORWARD : CMD_REVERSE);

    if (!modbusWriteSingleRegister(address, REG_COMMAND_START_STOP, command))
    {
        Serial.print(F("ОШИБКА установки направления для двигателя "));
        Serial.println(motorID);
        errorCount++;
        return false;
    }

    // Установка скорости (если не остановка)
    if (speed != 0)
    {
        float speedPercent = speed / 10.0f; // Преобразование в проценты

        if (!modbusDriveSetFrequency(address, speedPercent))
        {
            Serial.print(F("ОШИБКА установки скорости для двигателя "));
            Serial.println(motorID);
            errorCount++;
            return false;
        }
    }

    // Обновление статуса
    motorStatus[motorID].targetSpeed = direction ? speed : -speed;
    motorStatus[motorID].state = (speed == 0) ? MOTOR_STATE_IDLE : MOTOR_STATE_RUNNING;
    motorStatus[motorID].lastCommandTime = millis();

    commandCount++;
    lastCommandTime = millis();

    return true;
}

/**
 * @brief Установка скорости двигателя в процентах
 * @param motorID ID двигателя (0-3)
 * @param percent Скорость (-100..+100%)
 * @param direction Направление
 * @return true если команда отправлена успешно
 */
bool setMotorSpeedPercent(uint8_t motorID, int8_t percent, bool direction)
{
    if (motorID >= MOTOR_COUNT)
    {
        return false;
    }

    // Ограничение процента
    percent = constrain(percent, -100, 100);

    // Преобразование в сырое значение (0-1000)
    int16_t rawSpeed = speedPercentToRaw(percent);

    return setMotorSpeed(motorID, rawSpeed, direction);
}

/**
 * @brief Остановка двигателя
 * @param motorID ID двигателя
 * @return true если команда отправлена успешно
 */
bool stopMotor(uint8_t motorID)
{
    if (motorID >= MOTOR_COUNT)
    {
        return false;
    }

    // Плавная остановка через установку нулевой скорости
    bool result = setMotorSpeed(motorID, 0, true);

    if (result)
    {
        motorStatus[motorID].state = MOTOR_STATE_DECELERATING;

        // Ждем полной остановки
        delay(motorConfig[motorID].decelerationTime);

        motorStatus[motorID].state = MOTOR_STATE_IDLE;
        motorStatus[motorID].targetSpeed = 0;
        motorStatus[motorID].currentSpeed = 0;
    }

    return result;
}

/**
 * @brief Аварийная остановка двигателя
 * @param motorID ID двигателя
 * @return true если команда отправлена успешно
 */
bool emergencyStopMotor(uint8_t motorID)
{
    if (motorID >= MOTOR_COUNT)
    {
        return false;
    }

    uint8_t address = motorConfig[motorID].address;

    if (modbusDriveEmergencyStop(address))
    {
        motorStatus[motorID].state = MOTOR_STATE_EMERGENCY_STOP;
        motorStatus[motorID].targetSpeed = 0;
        motorStatus[motorID].currentSpeed = 0;
        emergencyStopCount++;

        return true;
    }

    return false;
}

// ========== ГРУППОВОЕ УПРАВЛЕНИЕ ==========

/**
 * @brief Остановка всех двигателей
 */
void stopAllMotors(void)
{
    Serial.println(F("Остановка всех двигателей..."));

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        stopMotor(i);
    }

    // Дополнительно: отправка широковещательной команды остановки
    sendModbusCommand(MOTOR_BROADCAST, MODBUS_WRITE_COIL, 0x0001, 0x0001);

    Serial.println(F("Все двигатели остановлены"));
}

/**
 * @brief Аварийная остановка всех двигателей
 */
void emergencyStopAll(void)
{
    Serial.println(F("!!! АВАРИЙНАЯ ОСТАНОВКА ВСЕХ ДВИГАТЕЛЕЙ !!!"));

    emergencyState = true;

    // Немедленная остановка всех двигателей
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        emergencyStopMotor(i);
    }

    // Широковещательная команда аварийного останова
    sendModbusCommand(MOTOR_BROADCAST, MODBUS_WRITE_COIL, 0x0001, 0x0001);

    emergencyStopCount++;

    // Звуковая сигнализация
    for (uint8_t i = 0; i < 3; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
    }
}

/**
 * @brief Установка скорости горизонтальных двигателей
 * @param leftSpeed Скорость левого двигателя (-1000..+1000)
 * @param rightSpeed Скорость правого двигателя (-1000..+1000)
 * @param direction Направление
 */
void setHorizontalSpeed(int16_t leftSpeed, int16_t rightSpeed, bool direction)
{
    if (!motorsEnabled || emergencyState)
    {
        return;
    }

    // Установка скорости с синхронизацией
    setMotorSpeed(MOTOR_HORIZONTAL_LEFT, leftSpeed, direction);
    setMotorSpeed(MOTOR_HORIZONTAL_RIGHT, rightSpeed, direction);

    // Обновление синхронизации
    if (horizontalSync.enabled)
    {
        // Коррекция скорости ведомого двигателя
        int32_t posError = horizontalSync.positionError;
        int16_t correction = (int16_t)(posError * horizontalSync.syncGain / 100);

        int16_t correctedSpeed = rightSpeed + correction;
        correctedSpeed = constrain(correctedSpeed, -1000, 1000);

        setMotorSpeed(MOTOR_HORIZONTAL_RIGHT, correctedSpeed, direction);
    }
}

/**
 * @brief Установка скорости вертикальных двигателей
 * @param leftSpeed Скорость левого двигателя (-1000..+1000)
 * @param rightSpeed Скорость правого двигателя (-1000..+1000)
 * @param direction Направление
 */
void setVerticalSpeed(int16_t leftSpeed, int16_t rightSpeed, bool direction)
{
    if (!motorsEnabled || emergencyState)
    {
        return;
    }

    // Установка скорости с синхронизацией
    setMotorSpeed(MOTOR_VERTICAL_LEFT, leftSpeed, direction);
    setMotorSpeed(MOTOR_VERTICAL_RIGHT, rightSpeed, direction);

    // Обновление синхронизации
    if (verticalSync.enabled)
    {
        int32_t posError = verticalSync.positionError;
        int16_t correction = (int16_t)(posError * verticalSync.syncGain / 100);

        int16_t correctedSpeed = rightSpeed + correction;
        correctedSpeed = constrain(correctedSpeed, -1000, 1000);

        setMotorSpeed(MOTOR_VERTICAL_RIGHT, correctedSpeed, direction);
    }
}

// ========== СПЕЦИАЛЬНЫЕ РЕЖИМЫ ==========

/**
 * @brief Наклонное опускание/подъем
 * @param isLowering true - опускание, false - подъем
 * @param tiltPercentage Угол наклона (0-100%)
 */
void tiltOperation(bool isLowering, uint8_t tiltPercentage)
{
    if (!motorsEnabled || emergencyState)
    {
        return;
    }

    Serial.print(F("Наклонное "));
    Serial.print(isLowering ? F("опускание") : F("подъем"));
    Serial.print(F(" с углом "));
    Serial.print(tiltPercentage);
    Serial.println(F("%"));

    // Базовая скорость
    int16_t baseSpeed = 500; // 50%

    // Расчет разницы скоростей для наклона
    int16_t speedDifference = map(tiltPercentage, 0, 100, 0, 300); // Макс 30% разницы

    if (isLowering)
    {
        // Опускание: левый двигатель быстрее
        setVerticalSpeed(baseSpeed + speedDifference,
                         baseSpeed - speedDifference,
                         false); // false = вниз
    }
    else
    {
        // Подъем: правый двигатель быстрее
        setVerticalSpeed(baseSpeed - speedDifference,
                         baseSpeed + speedDifference,
                         true); // true = вверх
    }
}

/**
 * @brief Выравнивание груза
 */
void levelingOperation(void)
{
    if (!motorsEnabled || emergencyState)
    {
        return;
    }

    Serial.println(F("Выравнивание груза..."));

    // Чтение текущих высот (должно быть из модуля датчиков)
    // Предполагаем, что есть глобальные переменные cargoHeight1 и cargoHeight2

    extern int32_t cargoHeight1, cargoHeight2; // Из модуля датчиков

    int32_t heightDifference = cargoHeight1 - cargoHeight2;
    int32_t tolerance = 10; // мм

    if (abs(heightDifference) > tolerance)
    {
        // Коррекция высоты
        if (heightDifference > 0)
        {
            // Левый выше - опускаем левый/поднимаем правый
            setVerticalSpeed(200, 300, heightDifference > 0);
        }
        else
        {
            // Правый выше - опускаем правый/поднимаем левый
            setVerticalSpeed(300, 200, heightDifference > 0);
        }

        // Ждем выравнивания
        delay(500);

        // Остановка
        setVerticalSpeed(0, 0, true);
    }

    Serial.println(F("Груз выровнен"));
}

// ========== КОММУНИКАЦИЯ MODBUS ==========

/**
 * @brief Отправка команды Modbus
 * @param address Адрес устройства
 * @param function Функция Modbus
 * @param reg Адрес регистра
 * @param value Значение
 * @return true если команда отправлена успешно
 */
bool sendModbusCommand(uint8_t address, uint8_t function, uint16_t reg, uint16_t value)
{
    if (!rs485Initialized)
    {
        return false;
    }

    // Формирование пакета
    modbusTxBuffer[0] = address;
    modbusTxBuffer[1] = function;
    modbusTxBuffer[2] = highByte(reg);
    modbusTxBuffer[3] = lowByte(reg);
    modbusTxBuffer[4] = highByte(value);
    modbusTxBuffer[5] = lowByte(value);

    // Расчет CRC16
    uint16_t crc = calculateCRC16(modbusTxBuffer, 6);
    modbusTxBuffer[6] = lowByte(crc);
    modbusTxBuffer[7] = highByte(crc);

    // Переключение в режим передачи
    digitalWrite(RS485_RE_DE_PIN, HIGH);
    digitalWrite(RS485_TX_ENABLE, HIGH);
    delayMicroseconds(100);

    // Отправка пакета
    size_t bytesSent = rs485Serial->write(modbusTxBuffer, 8);

    // Ожидание завершения передачи
    rs485Serial->flush();
    delayMicroseconds(100);

    // Переключение в режим приема
    digitalWrite(RS485_TX_ENABLE, LOW);
    digitalWrite(RS485_RE_DE_PIN, LOW);

    // Задержка перед чтением ответа
    delay(10);

#ifdef DEBUG_MOTORS
    Serial.print(F("Отправка Modbus: Адрес=0x"));
    if (address < 0x10)
        Serial.print('0');
    Serial.print(address, HEX);
    Serial.print(F(", Функция=0x"));
    if (function < 0x10)
        Serial.print('0');
    Serial.print(function, HEX);
    Serial.print(F(", Регистр=0x"));
    Serial.print(reg, HEX);
    Serial.print(F(", Значение=0x"));
    Serial.print(value, HEX);
    Serial.print(F(", CRC=0x"));
    Serial.println(crc, HEX);

    dumpModbusPacket(modbusTxBuffer, 8);
#endif

    return (bytesSent == 8);
}

/**
 * @brief Отправка команды чтения Modbus
 * @param address Адрес устройства
 * @param function Функция Modbus
 * @param reg Адрес регистра
 * @param count Количество регистров
 * @return true если команда отправлена успешно
 */
bool sendModbusReadCommand(uint8_t address, uint8_t function, uint16_t reg, uint16_t count)
{
    if (!rs485Initialized)
    {
        return false;
    }

    modbusTxBuffer[0] = address;
    modbusTxBuffer[1] = function;
    modbusTxBuffer[2] = highByte(reg);
    modbusTxBuffer[3] = lowByte(reg);
    modbusTxBuffer[4] = highByte(count);
    modbusTxBuffer[5] = lowByte(count);

    uint16_t crc = calculateCRC16(modbusTxBuffer, 6);
    modbusTxBuffer[6] = lowByte(crc);
    modbusTxBuffer[7] = highByte(crc);

    digitalWrite(RS485_RE_DE_PIN, HIGH);
    digitalWrite(RS485_TX_ENABLE, HIGH);
    delayMicroseconds(100);

    rs485Serial->write(modbusTxBuffer, 8);
    rs485Serial->flush();
    delayMicroseconds(100);

    digitalWrite(RS485_TX_ENABLE, LOW);
    digitalWrite(RS485_RE_DE_PIN, LOW);

    delay(10);

    return true;
}

/**
 * @brief Расчет CRC16 Modbus
 * @param data Указатель на данные
 * @param length Длина данных
 * @return CRC16
 */
uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < length; pos++)
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

// ========== БЕЗОПАСНОСТЬ И ДИАГНОСТИКА ==========

/**
 * @brief Проверка безопасности двигателей
 * @return true если все двигатели в безопасном состоянии
 */
bool checkMotorSafety(void)
{
    if (emergencyState)
    {
        return false;
    }

    bool allSafe = true;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        // Проверка перегрева
        if (motorStatus[i].temperature > 80.0f)
        { // 80°C
            Serial.print(F("ПЕРЕГРЕВ Двигатель "));
            Serial.println(i + 1);
            allSafe = false;
        }

        // Проверка перегрузки по току
        if (motorStatus[i].current > motorConfig[i].ratedCurrent * 1.5f)
        {
            Serial.print(F("ПЕРЕГРУЗКА Двигатель "));
            Serial.println(i + 1);
            allSafe = false;
        }

        // Проверка времени без команды (watchdog)
        if (millis() - motorStatus[i].lastCommandTime > 5000 &&
            motorStatus[i].state != MOTOR_STATE_IDLE)
        {
            Serial.print(F("WATCHDOG Двигатель "));
            Serial.println(i + 1);
            stopMotor(i);
            allSafe = false;
        }
    }

    // Проверка синхронизации горизонтальных двигателей
    if (horizontalSync.enabled && abs(horizontalSync.positionError) > horizontalSync.maxPositionError)
    {
        Serial.println(F("РАССИНХРОНИЗАЦИЯ горизонтальных двигателей"));
        stopAllMotors();
        allSafe = false;
    }

    // Проверка синхронизации вертикальных двигателей
    if (verticalSync.enabled && abs(verticalSync.positionError) > verticalSync.maxPositionError)
    {
        Serial.println(F("РАССИНХРОНИЗАЦИЯ вертикальных двигателей"));
        stopAllMotors();
        allSafe = false;
    }

    return allSafe;
}

/**
 * @brief Самодиагностика двигателей
 * @return true если все тесты пройдены
 */
bool performMotorSelfTest(void)
{
    Serial.println(F("\n--- САМОДИАГНОСТИКА ДВИГАТЕЛЕЙ ---"));

    bool allTestsPassed = true;

    // Тест 1: Проверка связи с каждым двигателем
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        Serial.print(F("Двигатель "));
        Serial.print(i + 1);
        Serial.print(F(": "));

        if (motorStatus[i].enabled)
        {
            // Чтение статуса двигателя
            if (readMotorParameters(i))
            {
                Serial.println(F("OK"));
            }
            else
            {
                Serial.println(F("ОШИБКА СВЯЗИ"));
                allTestsPassed = false;
            }
        }
        else
        {
            Serial.println(F("НЕ ОБНАРУЖЕН"));
            allTestsPassed = false;
        }
    }

    // Тест 2: Проверка аварийной остановки
    Serial.print(F("Тест аварийной остановки: "));
    emergencyStopAll();
    delay(100);

    // Проверка, что все двигатели остановлены
    bool allStopped = true;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        if (motorStatus[i].state != MOTOR_STATE_IDLE &&
            motorStatus[i].state != MOTOR_STATE_EMERGENCY_STOP)
        {
            allStopped = false;
            break;
        }
    }

    if (allStopped)
    {
        Serial.println(F("OK"));
    }
    else
    {
        Serial.println(F("ОШИБКА"));
        allTestsPassed = false;
    }

    // Сброс аварийного состояния
    emergencyState = false;

    // Тест 3: Кратковременное движение каждого двигателя
    if (allTestsPassed)
    {
        Serial.println(F("Тест кратковременного движения..."));

        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        {
            if (motorStatus[i].enabled)
            {
                Serial.print(F("Двигатель "));
                Serial.print(i + 1);
                Serial.print(F(": "));

                // Вращение вперед
                setMotorSpeedPercent(i, 10, true);
                delay(200);

                // Остановка
                stopMotor(i);
                delay(100);

                // Вращение назад
                setMotorSpeedPercent(i, 10, false);
                delay(200);

                // Остановка
                stopMotor(i);
                delay(100);

                Serial.println(F("OK"));
            }
        }
    }

    Serial.println(F("--- САМОДИАГНОСТИКА ЗАВЕРШЕНА ---\n"));

    return allTestsPassed;
}

// ========== УТИЛИТЫ ==========

/**
 * @brief Преобразование процентов в сырое значение скорости
 * @param percent Проценты (-100..+100)
 * @return Сырое значение скорости (-1000..+1000)
 */
int16_t speedPercentToRaw(int8_t percent)
{
    percent = constrain(percent, -100, 100);
    return (int16_t)percent * 10; // 10 единиц на 1%
}

/**
 * @brief Преобразование сырого значения скорости в проценты
 * @param raw Сырое значение скорости (-1000..+1000)
 * @return Проценты (-100..+100)
 */
int8_t speedRawToPercent(int16_t raw)
{
    raw = constrain(raw, -1000, 1000);
    return (int8_t)(raw / 10);
}


/**
 * @brief Обновление состояния двигателей
 */
void updateMotorStatuses(void)
{
    static uint32_t lastUpdate = 0;
    uint32_t currentTime = millis();

    // Обновляем статусы не чаще, чем раз в 500 мс
    if (currentTime - lastUpdate < 500)
    {
        return;
    }

    lastUpdate = currentTime;

    // Обновление статусов всех двигателей
    modbusDriveUpdateAllStatus();

    // Обновление структур motorStatus
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        uint8_t address = motorConfig[i].address;
        ModbusDeviceStatus *device = modbusGetDeviceStatus(address);

        if (device && device->connected)
        {
            // Обновление тока
            motorStatus[i].current = device->outputCurrent;

            // Обновление температуры (если есть данные)
            // motorStatus[i].temperature = ...;

            // Обновление позиции (если есть энкодер)
            // motorStatus[i].position = ...;

            // Проверка ошибок
            if (device->faultCode != 0)
            {
                motorStatus[i].error = MOTOR_ERROR_COMM_FAILURE;
                motorStatus[i].state = MOTOR_STATE_FAULT;

                Serial.print(F("ОШИБКА двигателя "));
                Serial.print(i + 1);
                Serial.print(F(": код 0x"));
                Serial.println(device->faultCode, HEX);
            }
            else
            {
                motorStatus[i].error = MOTOR_ERROR_NONE;
            }
        }
        else
        {
            motorStatus[i].enabled = false;
            motorStatus[i].error = MOTOR_ERROR_COMM_FAILURE;
        }
    }
}


/**
 * @brief Обновление управления двигателями (вызывается в основном цикле)
 */
void updateMotorControl(void)
{
    static uint32_t lastUpdate = 0;
    uint32_t currentTime = millis();

    // Обновление не чаще, чем каждые 100 мс
    if (currentTime - lastUpdate < 100)
    {
        return;
    }

    lastUpdate = currentTime;

    // 1. Обновление статусов двигателей
    updateMotorStatuses();

    // 2. Проверка безопасности
    if (!checkMotorSafety())
    {
        emergencyStopAll();
        return;
    }

    // 3. Плавное изменение скорости (рампы)
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        if (motorStatus[i].state == MOTOR_STATE_ACCELERATING ||
            motorStatus[i].state == MOTOR_STATE_DECELERATING)
        {
            int16_t rampSpeed = calculateRampSpeed(
                motorStatus[i].targetSpeed,
                motorStatus[i].currentSpeed,
                motorConfig[i].accelerationTime);

            // Применение новой скорости
            bool direction = rampSpeed >= 0;
            setMotorSpeed(i, abs(rampSpeed), direction);

            // Проверка достижения целевой скорости
            if (abs(rampSpeed - motorStatus[i].targetSpeed) < 10)
            {
                motorStatus[i].state = MOTOR_STATE_RUNNING;
                motorStatus[i].currentSpeed = motorStatus[i].targetSpeed;
            }
        }
    }

    // 4. Обновление синхронизации
    if (horizontalSync.enabled)
    {
        // Коррекция скорости ведомого двигателя
        int32_t posError = horizontalSync.positionError;
        int16_t correction = (int16_t)(posError * horizontalSync.syncGain / 100);

        // Применение коррекции
        int16_t currentSpeed = motorStatus[MOTOR_HORIZONTAL_RIGHT].targetSpeed;
        int16_t correctedSpeed = currentSpeed + correction;
        correctedSpeed = constrain(correctedSpeed, -1000, 1000);

        if (correctedSpeed != currentSpeed)
        {
            bool direction = correctedSpeed >= 0;
            setMotorSpeed(MOTOR_HORIZONTAL_RIGHT, abs(correctedSpeed), direction);
        }
    }
}

// ========== ОТЛАДКА ==========

#ifdef DEBUG_MOTORS
/**
 * @brief Вывод статуса двигателя
 * @param motorID ID двигателя
 */
void printMotorStatus(uint8_t motorID)
{
    if (motorID >= MOTOR_COUNT)
    {
        return;
    }

    MotorStatus *m = &motorStatus[motorID];

    Serial.print(F("Двигатель "));
    Serial.print(motorID + 1);
    Serial.print(F(": Состояние="));

    switch (m->state)
    {
    case MOTOR_STATE_IDLE:
        Serial.print(F("Остановлен"));
        break;
    case MOTOR_STATE_ACCELERATING:
        Serial.print(F("Разгон"));
        break;
    case MOTOR_STATE_RUNNING:
        Serial.print(F("Работа"));
        break;
    case MOTOR_STATE_DECELERATING:
        Serial.print(F("Торможение"));
        break;
    case MOTOR_STATE_FAULT:
        Serial.print(F("Ошибка"));
        break;
    case MOTOR_STATE_EMERGENCY_STOP:
        Serial.print(F("Аварийный стоп"));
        break;
    }

    Serial.print(F(", Скорость="));
    Serial.print(speedRawToPercent(m->currentSpeed));
    Serial.print(F("%, Ток="));
    Serial.print(m->current, 1);
    Serial.print(F("А, Темп="));
    Serial.print(m->temperature, 1);
    Serial.print(F("°C, Позиция="));
    Serial.print(m->position);
    Serial.println(F(" имп."));
}

/**
 * @brief Вывод статуса всех двигателей
 */
void printAllMotorsStatus(void)
{
    Serial.println(F("\n=== СТАТУС ДВИГАТЕЛЕЙ ==="));

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        printMotorStatus(i);
    }

    Serial.print(F("Горизонтальная синхронизация: Ошибка="));
    Serial.print(horizontalSync.positionError);
    Serial.print(F(" имп., Лимит="));
    Serial.print(horizontalSync.maxPositionError);
    Serial.println(F(" имп."));

    Serial.print(F("Вертикальная синхронизация: Ошибка="));
    Serial.print(verticalSync.positionError);
    Serial.print(F(" имп., Лимит="));
    Serial.print(verticalSync.maxPositionError);
    Serial.println(F(" имп."));

    Serial.println(F("==========================\n"));
}

/**
 * @brief Дамп Modbus пакета
 * @param data Буфер с данными
 * @param length Длина данных
 */
void dumpModbusPacket(const uint8_t *data, uint16_t length)
{
    Serial.print(F("Modbus пакет ("));
    Serial.print(length);
    Serial.print(F(" байт): "));

    for (uint16_t i = 0; i < length; i++)
    {
        if (data[i] < 0x10)
            Serial.print('0');
        Serial.print(data[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}
#endif