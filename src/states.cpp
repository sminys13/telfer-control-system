/**
 * @file states.cpp
 * @brief Реализация конечного автомата системы
 * @version 4.0
 */

#include "../include/config.h"
#include "../include/states.h"
#include "../include/ui.h"
#include "../include/motors.h"
#include "../include/storage.h"
#include "../include/sensors.h"
#include "../include/utils.h"
#include <Arduino.h>

extern SystemStatus systemStatus;
extern SystemFlags systemFlags;
extern SystemData systemData;
extern UserSettings userSettings;
extern ProgramSettings programs[MAX_PROGRAMS];
extern DiagnosticsResult diagnosticsResults[MAX_DIAGNOSTIC_TESTS];

// ========== ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Контекст состояний
static StateContext stateContext = {
    .currentState = STATE_BOOT,
    .previousState = STATE_BOOT,
    .nextState = STATE_BOOT,
    .stateEnterTime = 0,
    .stateDuration = 0,
    .transitionInProgress = 0,
    .transitionStartTime = 0,
    .eventQueueHead = 0,
    .eventQueueTail = 0,
    .eventQueue = {},
    .eventCount = 0
};

// Таблица обработчиков состояний
static StateHandler stateHandlers[] = {
    // BOOT
    {STATE_BOOT, stateEnterBoot, stateExitBoot, stateProcessBoot, NULL, 5000, true},

    // IDLE
    {STATE_IDLE, stateEnterIdle, stateExitIdle, stateProcessIdle, NULL, 0, false},

    // MENU_NAVIGATION
    {STATE_MENU_NAVIGATION, stateEnterMenuNavigation, stateExitMenuNavigation,
     stateProcessMenuNavigation, NULL, STATE_TIMEOUT_MS, false},

    // PROGRAM_SELECTION
    {STATE_PROGRAM_SELECTION, stateEnterProgramSelection, stateExitProgramSelection,
     stateProcessProgramSelection, NULL, STATE_TIMEOUT_MS, false},

    // PROGRAM_EDIT
    {STATE_PROGRAM_EDIT, stateEnterProgramEdit, stateExitProgramEdit,
     stateProcessProgramEdit, NULL, STATE_TIMEOUT_MS, false},

    // ZONE_EDIT
    {STATE_ZONE_EDIT, stateEnterZoneEdit, stateExitZoneEdit,
     stateProcessZoneEdit, NULL, STATE_TIMEOUT_MS, false},

    // AUTO_RUNNING
    {STATE_AUTO_RUNNING, stateEnterAutoRunning, stateExitAutoRunning,
     stateProcessAutoRunning, NULL, 0, true},

    // MANUAL_CONTROL
    {STATE_MANUAL_CONTROL, stateEnterManualControl, stateExitManualControl,
     stateProcessManualControl, NULL, STATE_TIMEOUT_MS, false},

    // CALIBRATION
    {STATE_CALIBRATION, stateEnterCalibration, stateExitCalibration,
     stateProcessCalibration, NULL, STATE_TIMEOUT_MS, false},

    // SETTINGS
    {STATE_SETTINGS, stateEnterSettings, stateExitSettings,
     stateProcessSettings, NULL, STATE_TIMEOUT_MS, false},

    // MONITOR
    {STATE_MONITOR, stateEnterMonitor, stateExitMonitor,
     stateProcessMonitor, NULL, STATE_TIMEOUT_MS, false},

    // DIAGNOSTICS
    {STATE_DIAGNOSTICS, stateEnterDiagnostics, stateExitDiagnostics,
     stateProcessDiagnostics, NULL, STATE_TIMEOUT_MS, false},

    // ERROR
    {STATE_ERROR, stateEnterError, stateExitError,
     stateProcessError, NULL, 0, true},

    // EMERGENCY
    {STATE_EMERGENCY, stateEnterEmergency, stateExitEmergency,
     stateProcessEmergency, NULL, 0, true}};

// Таблица переходов между состояниями
static StateTransition stateTransitions[] = {
    // Из BOOT
    {STATE_BOOT, EVENT_MOVE_COMPLETE, STATE_IDLE, NULL, TRANSITION_IMMEDIATE, 0},

    // Из IDLE
    {STATE_IDLE, EVENT_ENCODER_CLICK, STATE_MENU_NAVIGATION, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_IDLE, EVENT_PROGRAM_START, STATE_AUTO_RUNNING, NULL, TRANSITION_WITH_DELAY, 100},
    {STATE_IDLE, EVENT_MANUAL_MOVE, STATE_MANUAL_CONTROL, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_IDLE, EVENT_EMERGENCY_STOP, STATE_EMERGENCY, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_IDLE, EVENT_SENSOR_ALERT, STATE_ERROR, NULL, TRANSITION_IMMEDIATE, 0},

    // Из MENU_NAVIGATION
    {STATE_MENU_NAVIGATION, EVENT_MENU_SELECT, STATE_PROGRAM_SELECTION, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_MENU_NAVIGATION, EVENT_MENU_BACK, STATE_IDLE, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_MENU_NAVIGATION, EVENT_EMERGENCY_STOP, STATE_EMERGENCY, NULL, TRANSITION_IMMEDIATE, 0},

    // Из PROGRAM_SELECTION
    {STATE_PROGRAM_SELECTION, EVENT_MENU_SELECT, STATE_AUTO_RUNNING, NULL, TRANSITION_WITH_DELAY, 500},
    {STATE_PROGRAM_SELECTION, EVENT_MENU_BACK, STATE_MENU_NAVIGATION, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_PROGRAM_SELECTION, EVENT_MENU_SELECT, STATE_PROGRAM_EDIT, NULL, TRANSITION_IMMEDIATE, 0},

    // Из AUTO_RUNNING
    {STATE_AUTO_RUNNING, EVENT_PROGRAM_PAUSE, STATE_AUTO_RUNNING, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_AUTO_RUNNING, EVENT_PROGRAM_STOP, STATE_IDLE, NULL, TRANSITION_WITH_DELAY, 1000},
    {STATE_AUTO_RUNNING, EVENT_PROGRAM_COMPLETE, STATE_IDLE, NULL, TRANSITION_WITH_ANIMATION, 2000},
    {STATE_AUTO_RUNNING, EVENT_EMERGENCY_STOP, STATE_EMERGENCY, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_AUTO_RUNNING, EVENT_SENSOR_ALERT, STATE_ERROR, NULL, TRANSITION_IMMEDIATE, 0},

    // Из MANUAL_CONTROL
    {STATE_MANUAL_CONTROL, EVENT_MENU_BACK, STATE_IDLE, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_MANUAL_CONTROL, EVENT_EMERGENCY_STOP, STATE_EMERGENCY, NULL, TRANSITION_IMMEDIATE, 0},
    {STATE_MANUAL_CONTROL, EVENT_SENSOR_ALERT, STATE_ERROR, NULL, TRANSITION_IMMEDIATE, 0},

    // Из ERROR
    {STATE_ERROR, EVENT_ERROR_CLEARED, STATE_IDLE, NULL, TRANSITION_WITH_DELAY, 1000},
    {STATE_ERROR, EVENT_EMERGENCY_STOP, STATE_EMERGENCY, NULL, TRANSITION_IMMEDIATE, 0},

    // Из EMERGENCY
    {STATE_EMERGENCY, EVENT_ERROR_CLEARED, STATE_IDLE, NULL, TRANSITION_WITH_DELAY, 2000},

    // Конец таблицы
    {STATE_COUNT, EVENT_NONE, STATE_COUNT, NULL, 0, 0}};

// ========== ИНИЦИАЛИЗАЦИЯ ==========

/**
 * @brief Инициализация системы состояний
 */
void statesInit(void)
{
    Serial.println(F("Инициализация системы состояний..."));

    // Сброс контекста
    memset(&stateContext, 0, sizeof(StateContext));
    stateContext.currentState = STATE_BOOT;
    stateContext.previousState = STATE_BOOT;
    stateContext.stateEnterTime = millis();

    // Очистка очереди событий
    statesClearEventQueue();

    // Вызов функции входа в начальное состояние
    if (stateHandlers[STATE_BOOT].enterFunction)
    {
        stateHandlers[STATE_BOOT].enterFunction();
    }

    Serial.print(F("Начальное состояние: "));
    Serial.println(statesGetStateName(STATE_BOOT));

    // Запись в лог
    statesPostEvent(EVENT_NONE, 0, NULL); // Пустое событие для инициализации

    Serial.println(F("Система состояний инициализирована"));
}

/**
 * @brief Обработка системы состояний (вызывается в главном цикле)
 */
void statesProcess(void)
{
    static uint32_t lastProcessTime = 0;
    uint32_t currentTime = millis();

    // Ограничение частоты обработки (10 Гц)
    if (currentTime - lastProcessTime < 100)
    {
        return;
    }

    lastProcessTime = currentTime;

    // Обновление длительности состояния
    stateContext.stateDuration = currentTime - stateContext.stateEnterTime;

    // Проверка таймаутов состояний
    statesCheckStateTimeouts();

    // Обработка очереди событий
    statesProcessEventQueue();

    // Вызов функции обработки текущего состояния
    StateHandler *handler = &stateHandlers[stateContext.currentState];
    if (handler->processFunction)
    {
        handler->processFunction();
    }

    // Обработка запланированного перехода
    if (stateContext.nextState != stateContext.currentState &&
        stateContext.nextState < STATE_COUNT)
    {
        if (stateContext.transitionInProgress)
        {
            // Проверка завершения задержки перехода
            if (currentTime - stateContext.transitionStartTime >=
                stateHandlers[stateContext.currentState].timeoutMs)
            {
                statesChangeState(stateContext.nextState, TRANSITION_IMMEDIATE, 0);
                stateContext.transitionInProgress = 0;
            }
        }
    }
}

// ========== УПРАВЛЕНИЕ СОСТОЯНИЯМИ ==========

/**
 * @brief Изменение состояния системы
 * @param newState Новое состояние
 * @param flags Флаги перехода
 * @param delayMs Задержка перехода (мс)
 * @return true если переход выполнен
 */
bool statesChangeState(SystemState newState, uint16_t flags, uint32_t delayMs)
{
    if (newState >= STATE_COUNT)
    {
        Serial.print(F("ОШИБКА: Некорректное состояние: "));
        Serial.println(newState);
        return false;
    }

    SystemState currentState = stateContext.currentState;

    // Проверка допустимости перехода
    if (!statesIsTransitionAllowed(currentState, newState))
    {
        Serial.print(F("ПРЕДУПРЕЖДЕНИЕ: Переход из "));
        Serial.print(statesGetStateName(currentState));
        Serial.print(F(" в "));
        Serial.print(statesGetStateName(newState));
        Serial.println(F(" не разрешен"));
        return false;
    }

    // Проверка условия перехода (guard condition)
    StateHandler *currentHandler = &stateHandlers[currentState];
    if (currentHandler->guardCondition && !currentHandler->guardCondition())
    {
        Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: Условие перехода не выполнено"));
        return false;
    }

    // Если требуется задержка
    if ((flags & TRANSITION_WITH_DELAY) && delayMs > 0)
    {
        stateContext.nextState = newState;
        stateContext.transitionInProgress = 1;
        stateContext.transitionStartTime = millis();

        Serial.print(F("Запланирован переход из "));
        Serial.print(statesGetStateName(currentState));
        Serial.print(F(" в "));
        Serial.print(statesGetStateName(newState));
        Serial.print(F(" через "));
        Serial.print(delayMs);
        Serial.println(F(" мс"));

        return true;
    }

    // Вызов функции выхода из текущего состояния
    if (currentHandler->exitFunction)
    {
        currentHandler->exitFunction();
    }

    // Обновление контекста
    stateContext.previousState = currentState;
    stateContext.currentState = newState;
    stateContext.stateEnterTime = millis();
    stateContext.stateDuration = 0;
    stateContext.nextState = newState;
    stateContext.transitionInProgress = 0;

    // Вызов функции входа в новое состояние
    StateHandler *newHandler = &stateHandlers[newState];
    if (newHandler->enterFunction)
    {
        newHandler->enterFunction();
    }

    Serial.print(F("Переход состояния: "));
    Serial.print(statesGetStateName(currentState));
    Serial.print(F(" -> "));
    Serial.println(statesGetStateName(newState));

    // Запись в лог
    char logMsg[64];
    snprintf(logMsg, sizeof(logMsg), "Переход: %s -> %s",
             statesGetStateName(currentState), statesGetStateName(newState));
    // logMessage(LOG_INFO, logMsg, 0);

    return true;
}

/**
 * @brief Получение текущего состояния
 * @return Текущее состояние системы
 */
SystemState statesGetCurrentState(void)
{
    return stateContext.currentState;
}

/**
 * @brief Проверка допустимости перехода между состояниями
 * @param fromState Исходное состояние
 * @param toState Целевое состояние
 * @return true если переход разрешен
 */
bool statesIsTransitionAllowed(SystemState fromState, SystemState toState)
{
    // Аварийное состояние может быть вызвано из любого состояния
    if (toState == STATE_EMERGENCY)
    {
        return true;
    }

    // Из аварийного состояния можно выйти только через сброс ошибки
    if (fromState == STATE_EMERGENCY && toState != STATE_IDLE)
    {
        return false;
    }

    // Проверка таблицы переходов
    for (uint8_t i = 0; i < sizeof(stateTransitions) / sizeof(StateTransition); i++)
    {
        StateTransition *trans = &stateTransitions[i];
        if (trans->fromState == fromState && trans->toState == toState)
        {
            return true;
        }
    }

    return false;
}

// ========== ОБРАБОТЧИКИ СОСТОЯНИЙ ==========

// ----- СОСТОЯНИЕ BOOT -----

/**
 * @brief Вход в состояние загрузки
 */
void stateEnterBoot(void)
{
    Serial.println(F("=== СОСТОЯНИЕ ЗАГРУЗКИ ==="));

    // Инициализация оборудования
    // (уже выполнена в main.cpp)

    // Включение светодиода статуса
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Запуск анимации загрузки
    uiStartAnimation(10, 100, true);

    // Установка таймаута для состояния загрузки
    stateContext.stateEnterTime = millis();
}

/**
 * @brief Обработка состояния загрузки
 */
void stateProcessBoot(void)
{
    static uint8_t bootStage = 0;
    static uint32_t lastStageTime = 0;
    uint32_t currentTime = millis();

    // Последовательная инициализация этапов загрузки
    switch (bootStage)
    {
    case 0: // Инициализация дисплея
        if (currentTime - lastStageTime > 500)
        {
            Serial.println(F("Этап 1: Дисплей..."));
            // Здесь может быть дополнительная инициализация дисплея
            bootStage++;
            lastStageTime = currentTime;
        }
        break;

    case 1: // Инициализация датчиков
        if (currentTime - lastStageTime > 500)
        {
            Serial.println(F("Этап 2: Датчики..."));
            // sensorsInit(); - уже вызвано ранее
            bootStage++;
            lastStageTime = currentTime;
        }
        break;

    case 2: // Инициализация двигателей
        if (currentTime - lastStageTime > 500)
        {
            Serial.println(F("Этап 3: Двигатели..."));
            // motorsInit(); - уже вызвано ранее
            bootStage++;
            lastStageTime = currentTime;
        }
        break;

    case 3: // Самодиагностика
        if (currentTime - lastStageTime > 500)
        {
            Serial.println(F("Этап 4: Самодиагностика..."));
            // performSelfTest(); - уже выполнено
            bootStage++;
            lastStageTime = currentTime;
        }
        break;

    case 4: // Завершение загрузки
        if (currentTime - lastStageTime > 1000)
        {
            Serial.println(F("Загрузка завершена"));
            bootStage++;
            lastStageTime = currentTime;

            // Переход в состояние ожидания
            statesChangeState(STATE_IDLE, TRANSITION_WITH_DELAY, 500);
        }
        break;
    }
}

/**
 * @brief Выход из состояния загрузки
 */
void stateExitBoot(void)
{
    Serial.println(F("Выход из состояния загрузки"));

    // Выключение светодиода статуса
    digitalWrite(LED_STATUS_PIN, LOW);

    // Остановка анимации
    uiStopAnimation();

    // Звуковой сигнал завершения загрузки
    beep(1000, 200);
    delay(100);
    beep(1500, 200);
}

// ----- СОСТОЯНИЕ IDLE (ОЖИДАНИЕ) -----

/**
 * @brief Вход в состояние ожидания
 */
void stateEnterIdle(void)
{
    Serial.println(F("=== СОСТОЯНИЕ ОЖИДАНИЯ ==="));

    // Остановка всех двигателей (на всякий случай)
    stopAllMotors();

    // Обновление дисплея
    uiSetScreen(uiDrawMainMenuScreen, uiHandleMainMenuInput, 100);

    // Мигание светодиодом (медленно)
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(100);
    digitalWrite(LED_STATUS_PIN, LOW);

    // Запись в лог
    statesPostEvent(EVENT_NONE, 0, NULL);
}

/**
 * @brief Обработка состояния ожидания
 */
void stateProcessIdle(void)
{
    // Мигание светодиодом (1 Гц)
    static uint32_t lastBlinkTime = 0;
    uint32_t currentTime = millis();

    if (currentTime - lastBlinkTime > 1000)
    {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlinkTime = currentTime;
    }

    // Проверка энкодера (для входа в меню)
    // Обрабатывается в основном цикле через uiHandleMainMenuInput()
}

/**
 * @brief Выход из состояния ожидания
 */
void stateExitIdle(void)
{
    Serial.print(F("Выход из состояния ожидания в "));
    Serial.println(statesGetStateName(stateContext.nextState));

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);
}

// ----- СОСТОЯНИЕ MENU_NAVIGATION -----

/**
 * @brief Вход в состояние навигации по меню
 */
void stateEnterMenuNavigation(void)
{
    Serial.println(F("=== СОСТОЯНИЕ НАВИГАЦИИ ПО МЕНЮ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка главного меню
    uiCreateMenu(mainMenuItems, mainMenuCount / sizeof(MenuItem), "ГЛАВНОЕ МЕНЮ", MENU_MAIN);

    // Звуковое подтверждение
    beep(800, 100);
}

/**
 * @brief Обработка состояния навигации по меню
 */
void stateProcessMenuNavigation(void)
{
    // Обновление дисплея меню
    uiUpdateScreen();

    // Проверка таймаута бездействия
    if (stateContext.stateDuration > stateHandlers[STATE_MENU_NAVIGATION].timeoutMs)
    {
        Serial.println(F("Таймаут меню - возврат в ожидание"));
        statesChangeState(STATE_IDLE, TRANSITION_IMMEDIATE, 0);
    }
}

/**
 * @brief Выход из состояния навигации по меню
 */
void stateExitMenuNavigation(void)
{
    Serial.println(F("Выход из состояния навигации по меню"));
    digitalWrite(LED_STATUS_PIN, LOW);
}

// ----- СОСТОЯНИЕ AUTO_RUNNING -----

/**
 * @brief Вход в состояние автоматического выполнения
 */
void stateEnterAutoRunning(void)
{
    Serial.println(F("=== СОСТОЯНИЕ АВТОМАТИЧЕСКОГО ВЫПОЛНЕНИЯ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана автоматического режима
    uiSetScreen(uiDrawAutoModeScreen, uiHandleAutoModeInput, 50);

    // Проверка наличия выбранной программы
    if (systemData.currentProgram >= MAX_PROGRAMS || programs[systemData.currentProgram].zoneCount == 0)
    {
        Serial.println(F("ОШИБКА: Нет выбранной программы"));
        uiShowError("Нет программы!");
        statesChangeState(STATE_IDLE, TRANSITION_WITH_DELAY, 1000);
        return;
    }

    // Сброс счетчиков программы
    programs[systemData.currentProgram].currentRepeat = 0;

    // Запуск программы
    Serial.print(F("Запуск программы: "));
    Serial.println(programs[systemData.currentProgram].name);

    // Звуковое подтверждение
    beepSequence(2, 1000, 200);

    // Запись в лог
    char logMsg[64];
    snprintf(logMsg, sizeof(logMsg), "Запуск программы: %s", programs[systemData.currentProgram].name);
    // logMessage(LOG_PROGRAM_START, logMsg, currentProgram);
}

/**
 * @brief Обработка состояния автоматического выполнения
 */
void stateProcessAutoRunning(void)
{
    static uint32_t zoneStartTime = 0;
    static bool movingToZone = false;
    static bool dipping = false;

    uint32_t currentTime = millis();

    if (systemData.currentProgram >= MAX_PROGRAMS)
    {
        statesChangeState(STATE_ERROR, TRANSITION_IMMEDIATE, 0);
        return;
    }

    // Если программа завершена
    if (systemData.currentZone >= programs->zoneCount)
    {
        Serial.println(F("Программа завершена"));

        // Проверка необходимости повторения
        programs->currentRepeat++;
        if (programs->repeatEnabled &&
            (programs->repeatCount == 0 || programs->currentRepeat < programs->repeatCount))
        {
            Serial.print(F("Повторение "));
            Serial.print(programs->currentRepeat);
            Serial.print(F(" из "));
            Serial.println(programs->repeatCount);

            systemData.currentZone = 0;
            movingToZone = false;
            dipping = false;

            // Задержка перед повторением
            delay(1000);

            return;
        }
        else
        {
            // Завершение программы
            statesChangeState(STATE_IDLE, TRANSITION_WITH_ANIMATION, 2000);
            return;
        }
    }

    ZoneSettings *zone = &programs->zones[systemData.currentZone];

    if (!movingToZone && !dipping)
    {
        // Начало движения к зоне
        Serial.print(F("Движение к зоне "));
        Serial.print(systemData.currentZone + 1);
        Serial.print(F(": "));
        Serial.print(zone->position);
        Serial.println(F(" мм"));

        // Установка целевой позиции
        // moveHorizontalToPosition(zone->position, zone->motorSpeed);

        movingToZone = true;
        zoneStartTime = currentTime;
    }
    else if (movingToZone)
    {
        // Проверка достижения позиции
        int32_t positionError = abs(systemStatus.avgHorizontalPos - zone->position);

        if (positionError <= HORIZONTAL_TOLERANCE ||
            (currentTime - zoneStartTime) > 30000) // Таймаут 30 секунд
        {
            Serial.println(F("Позиция достигнута"));

            // Остановка горизонтального движения
            // stopAllMotors();

            movingToZone = false;

            // Начало погружения
            Serial.println(F("Начало погружения"));
            dipping = true;
            zoneStartTime = currentTime;

            // Установка угла наклона для погружения
            // tiltOperation(true, zone->tiltAngle); // true = опускание
        }
        else
        {
            // Обновление прогресса на дисплее
            uint8_t progress = (systemStatus.avgHorizontalPos * 100) / zone->position;
            progress = constrain(progress, 0, 100);

            // Можно обновить индикатор прогресса
        }
    }
    else if (dipping)
    {
        // Проверка завершения погружения
        if ((currentTime - zoneStartTime) >= zone->dipTime)
        {
            Serial.println(F("Погружение завершено"));

            // Подъем с наклоном
            // tiltOperation(false, zone->tiltAngle); // false = подъем

            dipping = false;
            zoneStartTime = currentTime;

            // Ждем время ожидания после подъема
            delay(zone->waitTime);

            // Переход к следующей зоне
            systemData.currentZone++;
            Serial.print(F("Переход к зоне "));
            Serial.println(systemData.currentZone + 1);
        }
    }

    // Обновление дисплея
    uiUpdateScreen();
}

/**
 * @brief Выход из состояния автоматического выполнения
 */
void stateExitAutoRunning(void)
{
    Serial.println(F("Выход из состояния автоматического выполнения"));

    // Остановка всех двигателей
    stopAllMotors();

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1500, 500);

    // Запись в лог
    // logMessage(LOG_PROGRAM_END, "Программа завершена", 0);
}

// ----- СОСТОЯНИЕ MANUAL_CONTROL -----

/**
 * @brief Вход в состояние ручного управления
 */
void stateEnterManualControl(void)
{
    Serial.println(F("=== СОСТОЯНИЕ РУЧНОГО УПРАВЛЕНИЯ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана ручного управления
    uiSetScreen(uiDrawManualControlScreen, uiHandleManualControlInput, 50);

    // Включение режима ручного управления в модуле моторов
    // (если есть такая функция)
    // enableManualMode(true);
    systemFlags.manualMode = true;

    // Сброс текущих целевых позиций
    // extern int32_t manualTargetHorizontal, manualTargetVertical;
    // manualTargetHorizontal = systemStatus.avgHorizontalPos;
    // manualTargetVertical = systemStatus.avgHeight;

    // Звуковое подтверждение
    beep(800, 200);
    delay(100);
    beep(1000, 200);

    // Запись в лог
    logMessage(LOG_INFO, "Ручной режим включен", 0);
}

/**
 * @brief Обработка состояния ручного управления
 */
void stateProcessManualControl(void)
{
    static uint32_t lastUpdate = 0;
    uint32_t currentTime = millis();

    // Обновление каждые 100 мс
    if (currentTime - lastUpdate < 100)
    {
        return;
    }

    lastUpdate = currentTime;

    // Проверка энкодера и кнопок (обрабатывается в uiHandleManualControlInput)

    // Проверка состояния датчиков
    if (!systemStatus.sensorsValid)
    {
        Serial.println(F("ПРЕДУПРЕЖДЕНИЕ: Данные датчиков невалидны в ручном режиме"));
        uiShowWarning("Проверьте датчики!");
    }

    // Проверка таймаута бездействия
    if (stateContext.stateDuration > stateHandlers[STATE_MANUAL_CONTROL].timeoutMs)
    {
        Serial.println(F("Таймаут ручного управления - возврат в ожидание"));
        statesChangeState(STATE_IDLE, TRANSITION_IMMEDIATE, 0);
    }

    // Обновление дисплея
    uiUpdateScreen();
}

/**
 * @brief Выход из состояния ручного управления
 */
void stateExitManualControl(void)
{
    Serial.println(F("Выход из состояния ручного управления"));

    // Остановка всех двигателей
    stopAllMotors();

    // Выключение режима ручного управления
    // enableManualMode(false);

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1000, 200);

    // Запись в лог
    logMessage(LOG_INFO, "Ручной режим выключен", 0);
}

// ----- СОСТОЯНИЕ CALIBRATION -----

/**
 * @brief Вход в состояние калибровки
 */
void stateEnterCalibration(void)
{
    Serial.println(F("=== СОСТОЯНИЕ КАЛИБРОВКИ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана калибровки
    uiSetScreen(uiDrawCalibrationScreen, uiHandleCalibrationInput, 100);

    // // Сброс калибровочных данных
    // extern uint8_t calibrationStep;
    // calibrationStep = 0;

    // Сохранение текущей позиции как начальной
    extern SystemCalibration calibration;
    calibration.homePosition = systemStatus.avgHorizontalPos;

    // Звуковое подтверждение
    beepSequence(3, 1000, 100);

    // Запись в лог
    logMessage(LOG_CALIBRATION, "Начало калибровки", 0);
}

/**
 * @brief Обработка состояния калибровки
 */
void stateProcessCalibration(void)
{
    static uint32_t lastUpdate = 0;
    uint32_t currentTime = millis();

    // Обновление каждые 100 мс
    if (currentTime - lastUpdate < 100)
    {
        return;
    }

    lastUpdate = currentTime;

    // Обновление данных с датчиков для отображения
    updateAllSensors(&systemStatus);

    // Проверка согласованности данных датчиков
    if (!checkSensorConsistency(&systemStatus))
    {
        uiShowWarning("Некорректные данные датчиков!");
    }

    // Обновление дисплея
    uiUpdateScreen();
}

/**
 * @brief Выход из состояния калибровки
 */
void stateExitCalibration(void)
{
    Serial.println(F("Выход из состояния калибровки"));

    // Остановка всех двигателей
    stopAllMotors();

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1500, 300);

    // Запись в лог
    logMessage(LOG_CALIBRATION, "Калибровка завершена", 0);
}

// ----- СОСТОЯНИЕ SETTINGS -----

/**
 * @brief Вход в состояние настроек
 */
void stateEnterSettings(void)
{
    Serial.println(F("=== СОСТОЯНИЕ НАСТРОЕК ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана настроек
    uiSetScreen(uiDrawSettingsScreen, uiHandleSettingsInput, 100);

    // Загрузка текущих настроек
    loadUserSettings(&userSettings);

    // Звуковое подтверждение
    beep(1200, 150);

    // Запись в лог
    logMessage(LOG_INFO, "Вход в настройки", 0);
}

/**
 * @brief Обработка состояния настроек
 */
void stateProcessSettings(void)
{
    // Обновление дисплея
    uiUpdateScreen();

    // Проверка таймаута бездействия
    if (stateContext.stateDuration > stateHandlers[STATE_SETTINGS].timeoutMs)
    {
        Serial.println(F("Таймаут настроек - возврат в меню"));
        statesChangeState(STATE_MENU_NAVIGATION, TRANSITION_IMMEDIATE, 0);
    }
}

/**
 * @brief Выход из состояния настроек
 */
void stateExitSettings(void)
{
    Serial.println(F("Выход из состояния настроек"));

    // Сохранение настроек
    if (saveUserSettings(&userSettings))
    {
        Serial.println(F("Настройки сохранены"));
        beep(1500, 100);
    }
    else
    {
        Serial.println(F("ОШИБКА сохранения настроек"));
        beep(800, 300);
    }

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Запись в лог
    logMessage(LOG_SETTINGS_CHANGE, "Настройки изменены", 0);
}

// ----- СОСТОЯНИЕ DIAGNOSTICS -----

/**
 * @brief Вход в состояние диагностики
 */
void stateEnterDiagnostics(void)
{
    Serial.println(F("=== СОСТОЯНИЕ ДИАГНОСТИКИ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана диагностики
    uiSetScreen(uiDrawDiagnosticsScreen, uiHandleDiagnosticsInput, 250);

    // Запуск диагностических тестов
    Serial.println(F("Запуск диагностических тестов..."));

    // Тест датчиков
    bool sensorsOK = performSensorSelfTest();

    // Тест двигателей
    bool motorsOK = performMotorSelfTest();

    // Тест хранилища
    bool storageOK = storageCheckIntegrity();

    // Сохранение результатов
    diagnosticsResults[0].passed = sensorsOK;
    diagnosticsResults[0].testId = 1;
    strncpy(diagnosticsResults[0].message, "Датчики", 31);

    diagnosticsResults[1].passed = motorsOK;
    diagnosticsResults[1].testId = 2;
    strncpy(diagnosticsResults[1].message, "Двигатели", 31);

    diagnosticsResults[2].passed = storageOK;
    diagnosticsResults[2].testId = 3;
    strncpy(diagnosticsResults[2].message, "Хранилище", 31);

    // Звуковое подтверждение
    if (sensorsOK && motorsOK && storageOK)
    {
        playSuccessMelody();
    }
    else
    {
        playWarningMelody();
    }

    // Запись в лог
    logMessage(LOG_DIAGNOSTICS, "Диагностика выполнена",
               (sensorsOK ? 1 : 0) | (motorsOK ? 2 : 0) | (storageOK ? 4 : 0));
}

/**
 * @brief Обработка состояния диагностики
 */
void stateProcessDiagnostics(void)
{
    // Обновление дисплея с результатами диагностики
    uiUpdateScreen();

    // Проверка таймаута
    if (stateContext.stateDuration > stateHandlers[STATE_DIAGNOSTICS].timeoutMs)
    {
        Serial.println(F("Таймаут диагностики - возврат в меню"));
        statesChangeState(STATE_MENU_NAVIGATION, TRANSITION_IMMEDIATE, 0);
    }
}

/**
 * @brief Выход из состояния диагностики
 */
void stateExitDiagnostics(void)
{
    Serial.println(F("Выход из состояния диагностики"));

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1000, 200);
}

// ----- СОСТОЯНИЕ ERROR -----

/**
 * @brief Вход в состояние ошибки
 */
void stateEnterError(void)
{
    Serial.println(F("=== СОСТОЯНИЕ ОШИБКИ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана ошибки
    uiSetScreen(uiDrawErrorScreen, uiHandleErrorInput, 100);

    // Остановка всех двигателей
    stopAllMotors();

    // Звуковая сигнализация
    playErrorMelody();

    // Получение сообщения об ошибке
    Serial.print(F("Ошибка: "));
    Serial.print(getErrorMessage(systemData.activeError));
    Serial.print(F(" - "));
    Serial.println(systemData.errorMessage);

    // Запись в лог
    char logMsg[128];
    snprintf(logMsg, sizeof(logMsg), "Ошибка %d: %s", systemData.activeError, systemData.errorMessage);
    logMessage(LOG_ERROR, logMsg, systemData.activeError);
}

/**
 * @brief Обработка состояния ошибки
 */
void stateProcessError(void)
{
    // Мигание светодиодом (1 Гц)
    static uint32_t lastBlinkTime = 0;
    uint32_t currentTime = millis();

    if (currentTime - lastBlinkTime > 500)
    {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlinkTime = currentTime;
    }

    // Проверка условий для сброса ошибки
    if (systemFlags.errorAutoReset)
    {
        // Автоматический сброс через 10 секунд
        if (stateContext.stateDuration > 10000)
        {
            Serial.println(F("Автоматический сброс ошибки"));
            statesChangeState(STATE_IDLE, TRANSITION_WITH_DELAY, 1000);
        }
    }

    // Обновление дисплея
    uiUpdateScreen();
}

/**
 * @brief Выход из состояния ошибки
 */
void stateExitError(void)
{
    Serial.println(F("Выход из состояния ошибки"));

    // Сброс ошибки
    clearError();

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1500, 300);

    // Запись в лог
    logMessage(LOG_INFO, "Ошибка сброшена", 0);
}

// ----- СОСТОЯНИЕ PROGRAM_SELECTION -----

/**
 * @brief Вход в состояние выбора программы
 */
void stateEnterProgramSelection(void)
{
    Serial.println(F("=== СОСТОЯНИЕ ВЫБОРА ПРОГРАММЫ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Создание меню программ
    
    // Создание пунктов меню для программ
    static MenuItem programMenuItems[MAX_PROGRAMS + 1];

    for (uint8_t i = 0; i < systemData.programCount; i++)
    {
        programMenuItems[i].text = programs[i].name;
        programMenuItems[i].id = i;
        programMenuItems[i].enabled = (programs[i].zoneCount > 0);
        programMenuItems[i].hasSubmenu = false;
        programMenuItems[i].action = NULL; // Обработка будет в обработчике событий
        programMenuItems[i].value = NULL;
        programMenuItems[i].minValue = 0;
        programMenuItems[i].maxValue = 0;
        programMenuItems[i].options = NULL;
        programMenuItems[i].optionCount = 0;
    }

    // Добавление пункта "Назад"
    programMenuItems[systemData.programCount].text = "<- Назад";
    programMenuItems[systemData.programCount].id = 255;
    programMenuItems[systemData.programCount].enabled = true;
    programMenuItems[systemData.programCount].hasSubmenu = false;
    programMenuItems[systemData.programCount].action = NULL;
    programMenuItems[systemData.programCount].value = NULL;
    programMenuItems[systemData.programCount].minValue = 0;
    programMenuItems[systemData.programCount].maxValue = 0;
    programMenuItems[systemData.programCount].options = NULL;
    programMenuItems[systemData.programCount].optionCount = 0;

    // Создание меню
    uiCreateMenu(programMenuItems, systemData.programCount + 1, "ВЫБОР ПРОГРАММЫ", 1);

    // Звуковое подтверждение
    beep(1000, 150);
}

/**
 * @brief Обработка состояния выбора программы
 */
void stateProcessProgramSelection(void)
{
    // Обновление меню
    uiUpdateScreen();

    // Проверка таймаута
    if (stateContext.stateDuration > stateHandlers[STATE_PROGRAM_SELECTION].timeoutMs)
    {
        Serial.println(F("Таймаут выбора программы - возврат в меню"));
        statesChangeState(STATE_MENU_NAVIGATION, TRANSITION_IMMEDIATE, 0);
    }
}

/**
 * @brief Выход из состояния выбора программы
 */
void stateExitProgramSelection(void)
{
    Serial.println(F("Выход из состояния выбора программы"));

    // Сохранение выбранной программы
    uint8_t selectedId = uiGetSelectedId();

    if (selectedId < 255) // Не пункт "Назад"
    {
        systemData.currentProgram = selectedId;

        Serial.print(F("Выбрана программа: "));
        Serial.println(programs[systemData.currentProgram].name);

        // Запись в лог
        char logMsg[64];
        snprintf(logMsg, sizeof(logMsg), "Выбрана программа: %s", programs[systemData.currentProgram].name);
        logMessage(LOG_INFO, logMsg, systemData.currentProgram);
    }

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);
}

// ----- СОСТОЯНИЕ PROGRAM_EDIT -----

/**
 * @brief Вход в состояние редактирования программы
 */
void stateEnterProgramEdit(void)
{
    Serial.println(F("=== СОСТОЯНИЕ РЕДАКТИРОВАНИЯ ПРОГРАММЫ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана редактирования программы
    uiSetScreen(uiDrawProgramEditScreen, uiHandleProgramEditInput, 100);

    // Получение текущей программы
    if (systemData.currentProgram < MAX_PROGRAMS)
    {
        Serial.print(F("Редактирование программы: "));
        Serial.println(programs[systemData.currentProgram].name);
    }

    // Звуковое подтверждение
    beep(800, 200);
}

/**
 * @brief Обработка состояния редактирования программы
 */
void stateProcessProgramEdit(void)
{
    // Обновление дисплея
    uiUpdateScreen();

    // Проверка таймаута
    if (stateContext.stateDuration > stateHandlers[STATE_PROGRAM_EDIT].timeoutMs)
    {
        Serial.println(F("Таймаут редактирования - возврат к выбору программы"));
        statesChangeState(STATE_PROGRAM_SELECTION, TRANSITION_IMMEDIATE, 0);
    }
}

/**
 * @brief Выход из состояния редактирования программы
 */
void stateExitProgramEdit(void)
{
    Serial.println(F("Выход из состояния редактирования программы"));

    // Сохранение изменений программы
    if (saveProgram(&programs[systemData.currentProgram], systemData.currentProgram))
    {
        Serial.println(F("Программа сохранена"));
        beep(1500, 100);
    }
    else
    {
        Serial.println(F("ОШИБКА сохранения программы"));
        beep(800, 300);
    }

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);
}

// ----- СОСТОЯНИЕ ZONE_EDIT -----

/**
 * @brief Вход в состояние редактирования зоны
 */
void stateEnterZoneEdit(void)
{
    Serial.println(F("=== СОСТОЯНИЕ РЕДАКТИРОВАНИЯ ЗОНЫ ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана редактирования зоны
    uiSetScreen(uiDrawZoneEditScreen, uiHandleZoneEditInput, 100);

    // Получение текущей зоны
    if (systemData.currentProgram < MAX_PROGRAMS && systemData.currentZone < programs[systemData.currentProgram].zoneCount)
    {
        Serial.print(F("Редактирование зоны: "));
        Serial.println(programs[systemData.currentProgram].zones[systemData.currentZone].name);
    }

    // Звуковое подтверждение
    beep(900, 200);
}

/**
 * @brief Обработка состояния редактирования зоны
 */
void stateProcessZoneEdit(void)
{
    // Обновление дисплея
    uiUpdateScreen();

    // Проверка таймаута
    if (stateContext.stateDuration > stateHandlers[STATE_ZONE_EDIT].timeoutMs)
    {
        Serial.println(F("Таймаут редактирования зоны - возврат к программе"));
        statesChangeState(STATE_PROGRAM_EDIT, TRANSITION_IMMEDIATE, 0);
    }
}

/**
 * @brief Выход из состояния редактирования зоны
 */
void stateExitZoneEdit(void)
{
    Serial.println(F("Выход из состояния редактирования зоны"));

    // Сохранение изменений зоны (уже сохранено при редактировании или будет сохранено с программой)

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1200, 150);
}

// ----- СОСТОЯНИЕ MONITOR -----

/**
 * @brief Вход в состояние мониторинга
 */
void stateEnterMonitor(void)
{
    Serial.println(F("=== СОСТОЯНИЕ МОНИТОРИНГА ==="));

    // Включение светодиода
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана мониторинга
    uiSetScreen(uiDrawMonitorScreen, uiHandleMonitorInput, 250);

    // Сброс статистики мониторинга
    // extern uint32_t monitorStartTime;
    // monitorStartTime = millis();

    // Звуковое подтверждение
    beep(1000, 100);
    delay(50);
    beep(1200, 100);
}

/**
 * @brief Обработка состояния мониторинга
 */
void stateProcessMonitor(void)
{
    extern SystemTiming systemTiming;

    // Обновление данных с датчиков
    updateAllSensors(&systemStatus);

    // Обновление статистики мониторинга
    // extern PerformanceStats monitorStats;
    updatePerformanceStats(micros() - systemTiming.lastDisplayUpdate);

    // Обновление дисплея
    uiUpdateScreen();

    // Проверка таймаута
    if (stateContext.stateDuration > stateHandlers[STATE_MONITOR].timeoutMs)
    {
        Serial.println(F("Таймаут мониторинга - возврат в меню"));
        statesChangeState(STATE_MENU_NAVIGATION, TRANSITION_IMMEDIATE, 0);
    }
}

/**
 * @brief Выход из состояния мониторинга
 */
void stateExitMonitor(void)
{
    Serial.println(F("Выход из состояния мониторинга"));

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1200, 200);
}

// ----- СОСТОЯНИЕ EMERGENCY -----

/**
 * @brief Вход в аварийное состояние
 */
void stateEnterEmergency(void)
{
    Serial.println(F("!!! АВАРИЙНОЕ СОСТОЯНИЕ !!!"));

    // Немедленная остановка всех двигателей
    stopAllMotors();
    emergencyStopAll();

    // Включение светодиода (постоянно)
    digitalWrite(LED_STATUS_PIN, HIGH);

    // Загрузка экрана аварии
    uiSetScreen(uiDrawEmergencyScreen, uiHandleEmergencyInput, 100);

    // Звуковая сигнализация
    for (int i = 0; i < 3; i++)
    {
        beep(2000, 300);
        delay(200);
    }

    // Запись в лог
    // logMessage(LOG_EMERGENCY, "Аварийная остановка", 0);
}

/**
 * @brief Обработка аварийного состояния
 */
void stateProcessEmergency(void)
{
    // Мигание светодиодом (быстрое)
    static uint32_t lastBlinkTime = 0;
    uint32_t currentTime = millis();

    if (currentTime - lastBlinkTime > 200)
    {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlinkTime = currentTime;
    }

    // Проверка состояния аварийной кнопки
    if (digitalRead(EMERGENCY_STOP_PIN) == HIGH)
    {
        // Кнопка отпущена - можно сбросить аварию
        Serial.println(F("Аварийная кнопка отпущена"));

        // Требуется подтверждение сброса (например, поворот энкодера)
        // В реальной системе здесь может быть проверка условий для сброса
    }

    // Обновление дисплея
    uiUpdateScreen();
}

/**
 * @brief Выход из аварийного состояния
 */
void stateExitEmergency(void)
{
    Serial.println(F("Выход из аварийного состояния"));

    // Выключение светодиода
    digitalWrite(LED_STATUS_PIN, LOW);

    // Звуковое подтверждение
    beep(1500, 1000);

    // Сброс флагов аварии
    systemFlags.isEmergency = false;

    // Запись в лог
    // logMessage(LOG_INFO, "Авария сброшена", 0);
}

// (Остальные обработчики состояний будут реализованы аналогично)

// ========== УПРАВЛЕНИЕ СОБЫТИЯМИ ==========

/**
 * @brief Помещение события в очередь
 * @param type Тип события
 * @param data Дополнительные данные
 * @param context Контекст события
 * @return true если событие помещено в очередь
 */
bool statesPostEvent(EventType type, int32_t data, void *context)
{
    if (stateContext.eventCount >= 16)
    {
        Serial.println(F("ОШИБКА: Очередь событий переполнена"));
        return false;
    }

    Event *event = &stateContext.eventQueue[stateContext.eventQueueTail];
    event->type = type;
    event->timestamp = millis();
    event->data = data;
    event->context = context;

    stateContext.eventQueueTail = (stateContext.eventQueueTail + 1) % 16;
    stateContext.eventCount++;

#ifdef DEBUG_STATES
    Serial.print(F("Событие добавлено: "));
    Serial.print(statesGetEventName(type));
    Serial.print(F(" (в очереди: "));
    Serial.print(stateContext.eventCount);
    Serial.println(F(")"));
#endif

    return true;
}

/**
 * @brief Обработка очереди событий
 * @return true если обработано хотя бы одно событие
 */
bool statesProcessEventQueue(void)
{
    if (stateContext.eventCount == 0)
    {
        return false;
    }

    Event *event = &stateContext.eventQueue[stateContext.eventQueueHead];

    // Обработка события в зависимости от текущего состояния
    switch (stateContext.currentState)
    {
    case STATE_IDLE:
        // Обработка событий в состоянии ожидания
        switch (event->type)
        {
        case EVENT_ENCODER_CLICK:
            statesChangeState(STATE_MENU_NAVIGATION, TRANSITION_IMMEDIATE, 0);
            break;

        case EVENT_PROGRAM_START:
            statesChangeState(STATE_AUTO_RUNNING, TRANSITION_WITH_DELAY, 100);
            break;

        default:
            break;
        }
        break;

        // Обработка для других состояний...

    default:
        break;
    }

    // Удаление обработанного события
    stateContext.eventQueueHead = (stateContext.eventQueueHead + 1) % 16;
    stateContext.eventCount--;

    return true;
}

// ========== УТИЛИТЫ ==========

/**
 * @brief Получение имени состояния
 * @param state Состояние
 * @return Строковое представление состояния
 */
const char *statesGetStateName(SystemState state)
{
    static const char *stateNames[] = {
        "BOOT",
        "IDLE",
        "MENU_NAVIGATION",
        "PROGRAM_SELECTION",
        "PROGRAM_EDIT",
        "ZONE_EDIT",
        "AUTO_RUNNING",
        "MANUAL_CONTROL",
        "CALIBRATION",
        "SETTINGS",
        "MONITOR",
        "DIAGNOSTICS",
        "ERROR",
        "EMERGENCY",
        "COUNT"};

    if (state < STATE_COUNT)
    {
        return stateNames[state];
    }

    return "UNKNOWN";
}

/**
 * @brief Проверка таймаутов состояний
 */
void statesCheckStateTimeouts(void)
{
    StateHandler *handler = &stateHandlers[stateContext.currentState];

    if (handler->timeoutMs > 0 && stateContext.stateDuration > handler->timeoutMs)
    {
        Serial.print(F("Таймаут состояния: "));
        Serial.println(statesGetStateName(stateContext.currentState));

        // Отправка события таймаута
        statesPostEvent(EVENT_TIMEOUT, stateContext.currentState, NULL);

        // Возврат в состояние ожидания
        statesChangeState(STATE_IDLE, TRANSITION_WITH_DELAY, 100);
    }
}

// ========== ОТЛАДКА ==========

#ifdef DEBUG_STATES
/**
 * @brief Вывод очереди событий
 */
void statesPrintEventQueue(void)
{
    Serial.println(F("\n=== ОЧЕРЕДЬ СОБЫТИЙ ==="));

    if (stateContext.eventCount == 0)
    {
        Serial.println(F("Пусто"));
    }
    else
    {
        Serial.print(F("Событий в очереди: "));
        Serial.println(stateContext.eventCount);

        for (uint8_t i = 0; i < stateContext.eventCount; i++)
        {
            uint8_t index = (stateContext.eventQueueHead + i) % 16;
            Event *event = &stateContext.eventQueue[index];

            Serial.print(F("  ["));
            Serial.print(i);
            Serial.print(F("] "));
            Serial.print(statesGetEventName(event->type));
            Serial.print(F(" @ "));
            Serial.print(event->timestamp);
            Serial.print(F(" data="));
            Serial.println(event->data);
        }
    }

    Serial.println(F("=====================\n"));
}
#endif