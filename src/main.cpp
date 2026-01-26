/**
 * @file main.cpp
 * @brief Главный файл системы управления тельферами
 * @version 4.0
 * @date 2025
 *
 * Система управления двумя тельферами с лазерными дальномерами
 * и ультразвуковыми датчиками высоты.
 * Управление через частотные преобразователи HE200-T3S-1R5G по RS-485.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Encoder.h>

// Заголовочные файлы проекта
#include "../include/config.h"
#include "../include/fonts.h"
#include "../include/sensors.h"
#include "../include/motors.h"
#include "../include/modbus.h"
#include "../include/storage.h"
#include "../include/ui.h"
#include "../include/states.h"
#include "../include/utils.h"

// ========== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Текущее состояние системы
SystemState currentState = STATE_BOOT;
SystemState previousState = STATE_BOOT;

// Флаги состояния системы
SystemFlags systemFlags = {
    .isPaused = false,
    .isEmergency = false,
    .systemInitialized = false,
    .displayInitialized = false,
    .motorsEnabled = false,
    .sensorsActive = false,
    .programRunning = false,
    .manualMode = false,
    .errorAutoReset = false
};

// Текущие позиции и настройки
SystemData systemData = {
    .currentZone = 0,
    .currentProgram = 0,
    .programCount = 0,
    .menuIndex = 0,
    .menuScroll = 0,
    .activeError = ERROR_NONE,
    .errorMessage = {},
};

// Временные метки
SystemTiming systemTiming = {
    .startupTime = 0,
    .stateStartTime = 0,
    .dipStartTime = 0,
    .pauseStartTime = 0,
    .errorTime = 0,
    .lastSensorUpdate = 0,
    .lastDisplayUpdate = 0,
    .lastEncoderCheck = 0,
    .lastMotorCommand = 0,
    .lastSafetyCheck = 0};

// Данные системы
SystemStatus systemStatus = {0};
SystemCalibration calibration = {0};
ProgramSettings programs[MAX_PROGRAMS] = {0};
// ProgramManager programManager;
UserSettings userSettings = {0};
DiagnosticsResult diagnosticsResults[MAX_DIAGNOSTIC_TESTS] = {0};
// Статистика
SystemStatistics systemStats = {0};
PerformanceStats perfStats = {0};

// Объекты оборудования
U8G2_ST7565_EA_DOGM128_1_4W_SW_SPI u8g2(
    U8G2_R1,
    DISPLAY_SCL_PIN,
    DISPLAY_SDA_PIN,
    DISPLAY_CS_PIN,
    DISPLAY_DC_PIN,
    DISPLAY_RESET_PIN);

Encoder rotaryEncoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========
void systemInitialize();
void systemLoop();
void handleEmergencyStop();
void updateSystemStatus();
void handleEncoderInput(Encoder *encoder, uint8_t buttonPin);
bool loadSettings(SystemCalibration *cal, ProgramSettings *progs, uint8_t *count);
bool loadDefaultSettings(SystemCalibration *cal, ProgramSettings *progs, unsigned char *count);
void updateDisplay(U8G2 *display, SystemState state, SystemData *data, SystemStatus *status);
// void showSplashScreen(U8G2 *display);

// ========== ФУНКЦИЯ SETUP ==========

/**
 * @brief Функция инициализации системы
 *
 * Выполняется один раз при запуске контроллера.
 * Инициализирует все компоненты системы.
 */
void setup()
{
  // Задержка для стабилизации питания
  delay(100);

  // Инициализация последовательного порта для отладки
  Serial.begin(SERIAL_DEBUG_BAUD);
  while (!Serial)
  {
    delay(10);
  }

  Serial.println(F("\n========================================"));
  Serial.println(F("СИСТЕМА УПРАВЛЕНИЯ ТЕЛЬФЕРАМИ"));
  Serial.println(F("Версия: 4.0"));
  Serial.println(F("Дата сборки: " __DATE__ " " __TIME__));
  Serial.println(F("========================================"));

  // Инициализация системы
  systemInitialize();

  Serial.println(F("Система инициализирована"));
  Serial.println(F("========================================"));
}

/**
 * @brief Инициализация всех компонентов системы
 */
void systemInitialize()
{
  // 1. Инициализация утилит
  utilsInit();

  // 2. Инициализация пинов
  initPins();

  // 3. Инициализация дисплея
  if (!initDisplay(&u8g2))
  {
    Serial.println(F("ОШИБКА: Не удалось инициализировать дисплей"));
    systemFlags.displayInitialized = false;
  }
  else
  {
    systemFlags.displayInitialized = true;
    // showSplashScreen(&u8g2);
  }

  // 4. Инициализация датчиков
  initSensors();
  systemFlags.sensorsActive = true;

  // 5. Инициализация двигателей
  if (!initRS485Interface())
  {
    Serial.println(F("ОШИБКА: Не удалось инициализировать Modbus"));
    systemFlags.motorsEnabled = false;
  }
  else
  {
    motorsInit();
    systemFlags.motorsEnabled = true;
  }

  // 6. Инициализация хранилища
  if (!storageInit())
  {
    Serial.println(F("ОШИБКА: Не удалось инициализировать хранилище"));
  }
  
  // programManager.init();
  // programManager.loadProgram(0); // Загрузить первую программу
  // 7. Загрузка настроек из EEPROM
  if (!loadSettings(&calibration, programs, &systemData.programCount))
  {
    Serial.println(F("Загрузка настроек по умолчанию"));
    loadDefaultSettings(&calibration, programs, &systemData.programCount);
  }

  // Загрузка пользовательских настроек
  if (!loadUserSettings(&userSettings))
  {
    Serial.println(F("Загрузка пользовательских настроек по умолчанию"));
    resetUserSettings();
    loadUserSettings(&userSettings);
  }

  // Применение пользовательских настроек
  uiSetContrast(userSettings.displayContrast);
  // soundEnabled = userSettings.soundEnabled;

  // 8. Инициализация системы состояний
  statesInit();

  // 9. Инициализация пользовательского интерфейса
  initUI();

  // 10. Самодиагностика
  if (performSelfTest())
  {
    Serial.println(F("Самодиагностика пройдена успешно"));
  }
  else
  {
    Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: Ошибки при самодиагностике"));
  }

  // 11. Установка начального состояния
  systemTiming.startupTime = millis();
  systemFlags.systemInitialized = true;
  // currentState = STATE_IDLE;

  // Переход в состояние загрузки
  statesChangeState(STATE_BOOT, TRANSITION_IMMEDIATE, 0);

  // 12. Звуковой сигнал готовности
  playStartupMelody();
}



// ========== ФУНКЦИЯ LOOP ==========

/**
 * @brief Главный цикл программы
 *
 * Выполняется постоянно после setup().
 * Управляет всеми процессами системы.
 */
void loop()
{
  uint32_t loopStartTime = micros();

  // Обновление времени работы системы
  systemStatus.uptime = millis() - systemTiming.startupTime;

  // ===== ШАГ 1: ПРОВЕРКА АВАРИЙНОЙ КНОПКИ =====
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW && !systemFlags.isEmergency)
  {
    emergencyStop();
    statesPostEvent(EVENT_EMERGENCY_STOP, 0, NULL);
  }

  // ===== ШАГ 2: ОБНОВЛЕНИЕ ДАТЧИКОВ =====
  if (millis() - systemTiming.lastSensorUpdate > SENSOR_UPDATE_INTERVAL)
  {
    updateAllSensors(&systemStatus);
    systemTiming.lastSensorUpdate = millis();
  }

  // ===== ШАГ 3: ОБРАБОТКА ВВОДА =====
  if (millis() - systemTiming.lastEncoderCheck > ENCODER_DEBOUNCE_TIME)
  {
    handleEncoderInput(&rotaryEncoder, ENCODER_SW_PIN);
    systemTiming.lastEncoderCheck = millis();
  }

  // ===== ШАГ 4: ПРОВЕРКА БЕЗОПАСНОСТИ =====
  if (millis() - systemTiming.lastSafetyCheck > SAFETY_CHECK_INTERVAL)
  {
    if (!systemFlags.isEmergency && !checkSafetyLimits(&systemStatus, &calibration))
    {
      emergencyStop();
      statesPostEvent(EVENT_EMERGENCY_STOP, 0, NULL);
    }
    systemTiming.lastSafetyCheck = millis();
  }

  // ===== ШАГ 5: ОБНОВЛЕНИЕ УПРАВЛЕНИЯ =====
  updateMotorControl(); // Из модуля motors.cpp
  updateMelody();       // Из модуля utils.cpp
  timerUpdateAll();     // Из модуля utils.cpp

  // ===== ШАГ 6: ОБРАБОТКА СОСТОЯНИЙ =====
  statesProcess();

  // ===== ШАГ 7: ОБНОВЛЕНИЕ ДИСПЛЕЯ =====
  if (systemFlags.displayInitialized &&
      millis() - systemTiming.lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL)
  {
    uiUpdateScreen();
    systemTiming.lastDisplayUpdate = millis();
  }

  // ===== ШАГ 8: СЕРВИСНЫЕ ОПЕРАЦИИ =====
  updateSystemStatus();

  // ===== ШАГ 9: УПРАВЛЕНИЕ ЗАДЕРЖКОЙ =====
  // Поддержание стабильной частоты цикла
  uint32_t loopTime = micros() - loopStartTime;
  updatePerformanceStats(loopTime);

  if (loopTime < TARGET_LOOP_TIME_US)
  {
    delayMicroseconds(TARGET_LOOP_TIME_US - loopTime);
  }
}

/**
 * @brief Обновление статуса системы
 */
void updateSystemStatus()
{
  // Мигание светодиодом статуса
  static uint32_t lastBlinkTime = 0;
  uint32_t currentTime = millis();

  SystemState currentState = statesGetCurrentState();

  if (currentState == STATE_EMERGENCY)
  {
    digitalWrite(LED_STATUS_PIN, HIGH); // Постоянно горит при аварии
  }
  else if (systemData.activeError != ERROR_NONE)
  {
    // Быстрое мигание при ошибке (2 Гц)
    if (currentTime - lastBlinkTime > 250)
    {
      digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
      lastBlinkTime = currentTime;
    }
  }
  else if (currentState != STATE_IDLE && currentState != STATE_BOOT)
  {
    digitalWrite(LED_STATUS_PIN, HIGH); // Горит при работе
  }
  else
  {
    digitalWrite(LED_STATUS_PIN, LOW); // Выключен в режиме ожидания
  }

// Периодический вывод отладочной информации
#ifdef DEBUG_MODE
  static unsigned long lastDebugOutput = 0;
  if (currentTime - lastDebugOutput > 5000)
  { // Каждые 5 секунд
    debugPrintSystemStatus();
    lastDebugOutput = currentTime;
  }
#endif
}

/**
 * @brief Аварийная остановка системы
 */
void emergencyStop()
{
  if (!systemFlags.isEmergency)
  {
    systemFlags.isEmergency = true;

    // 1. Немедленная остановка всех двигателей
    emergencyStopAll();

    // 2. Включение аварийной индикации
    digitalWrite(LED_STATUS_PIN, HIGH);

    // 3. Звуковая сигнализация
    playEmergencyMelody();

    // 4. Запись в журнал
    Serial.println(F("!!! АВАРИЙНАЯ ОСТАНОВКА !!!"));

    // 5. Переход в состояние аварии
    statesChangeState(STATE_EMERGENCY, TRANSITION_IMMEDIATE, 0);
  }
}

/**
 * @brief Восстановление после аварийной остановки
 */
bool resetEmergency()
{
  if (systemFlags.isEmergency && digitalRead(EMERGENCY_STOP_PIN) == HIGH)
  {
    systemFlags.isEmergency = false;
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1500, 500);

    Serial.println(F("Аварийная остановка сброшена"));

    // Возврат в состояние ожидания
    statesChangeState(STATE_IDLE, TRANSITION_WITH_DELAY, 1000);

    return true;
  }
  return false;
}

/**
 * @brief Обработка ввода с энкодера
 */
void handleEncoderInput(Encoder *encoder, uint8_t buttonPin)
{
  static int32_t lastPosition = 0;
  static uint32_t lastButtonPress = 0;
  static bool buttonPressed = false;

  int32_t currentPosition = encoder->read();

  // Обработка поворота энкодера
  if (currentPosition != lastPosition)
  {
    int32_t change = (currentPosition - lastPosition) / 4; // Учет шага энкодера

    if (change > 0)
    {
      statesPostEvent(EVENT_ENCODER_TURN, 1, NULL); // Вправо
      uiNavigateDown();
    }
    else if (change < 0)
    {
      statesPostEvent(EVENT_ENCODER_TURN, -1, NULL); // Влево
      uiNavigateUp();
    }

    lastPosition = currentPosition;
  }

  // Обработка нажатия кнопки
  bool buttonState = digitalRead(buttonPin) == LOW;
  uint32_t currentTime = millis();

  if (buttonState && !buttonPressed && (currentTime - lastButtonPress > 50))
  {
    // Короткое нажатие
    buttonPressed = true;
    lastButtonPress = currentTime;

    statesPostEvent(EVENT_ENCODER_CLICK, 0, NULL);
    uiNavigateEnter();
  }
  else if (!buttonState && buttonPressed)
  {
    // Отпускание кнопки
    buttonPressed = false;

    // Проверка длительного нажатия
    if (currentTime - lastButtonPress > 1000)
    {
      statesPostEvent(EVENT_MENU_BACK, 0, NULL);
      uiNavigateBack();
    }
  }
}

/**
 * @brief Загрузка настроек из EEPROM
 */
bool loadSettings(SystemCalibration *cal, ProgramSettings *progs, uint8_t *count)
{
  if (!cal || !progs || !count)
  {
    return false;
  }

  // Загрузка калибровки
  if (!loadCalibration(cal))
  {
    Serial.println(F("Не удалось загрузить калибровку"));
    return false;
  }

  // Загрузка программ
  if (!loadAllPrograms(progs, count))
  {
    Serial.println(F("Не удалось загрузить программы"));
    *count = 0;
  }

  Serial.print(F("Загружено программ: "));
  Serial.println(*count);

  return true;
}