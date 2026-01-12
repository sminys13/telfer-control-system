/**
 * @file states.h
 * @brief Управление состояниями системы - конечный автомат (FSM)
 * @version 4.0
 */

#ifndef STATES_H
#define STATES_H

#include "../include/config.h"
#include <stdint.h>
#include <stdbool.h>

// ========== КОНСТАНТЫ УПРАВЛЕНИЯ СОСТОЯНИЯМИ ==========

// Максимальное время пребывания в состоянии (таймаут безопасности)
#define STATE_TIMEOUT_MS 300000 // 5 минут

// Минимальное время для стабильного состояния
#define STATE_DEBOUNCE_MS 50

// Флаги перехода
#define TRANSITION_IMMEDIATE 0x01
#define TRANSITION_WITH_DELAY 0x02
#define TRANSITION_WITH_ANIMATION 0x04
#define TRANSITION_WITH_CONFIRMATION 0x08

// Типы событий
typedef enum
{
    EVENT_NONE,             // Нет события
    EVENT_BUTTON_PRESS,     // Нажатие кнопки
    EVENT_ENCODER_TURN,     // Поворот энкодера
    EVENT_ENCODER_CLICK,    // Нажатие энкодера
    EVENT_EMERGENCY_STOP,   // Аварийная остановка
    EVENT_PROGRAM_START,    // Запуск программы
    EVENT_PROGRAM_PAUSE,    // Пауза программы
    EVENT_PROGRAM_RESUME,   // Продолжение программы
    EVENT_PROGRAM_STOP,     // Остановка программы
    EVENT_PROGRAM_COMPLETE, // Программа выполнена
    EVENT_MOVE_COMPLETE,    // Движение завершено
    EVENT_DIP_COMPLETE,     // Погружение завершено
    EVENT_CALIBRATION_DONE, // Калибровка завершена
    EVENT_ERROR_CLEARED,    // Ошибка устранена
    EVENT_TIMEOUT,          // Таймаут состояния
    EVENT_SENSOR_ALERT,     // Предупреждение от датчика
    EVENT_MENU_SELECT,      // Выбор в меню
    EVENT_MENU_BACK,        // Возврат в меню
    EVENT_MANUAL_MOVE,      // Ручное движение
    EVENT_AUTO_MOVE,        // Автоматическое движение
    EVENT_COUNT             // Количество событий
} EventType;

// Структура события
typedef struct
{
    EventType type;     // Тип события
    uint32_t timestamp; // Время события
    int32_t data;       // Дополнительные данные
    void *context;      // Контекст события
} Event;

// Структура перехода между состояниями
typedef struct
{
    SystemState fromState; // Исходное состояние
    EventType trigger;     // Событие-триггер
    SystemState toState;   // Целевое состояние
    void (*action)();      // Действие при переходе
    uint16_t flags;        // Флаги перехода
    uint32_t delayMs;      // Задержка перехода (мс)
} StateTransition;

// Структура обработчика состояния
typedef struct
{
    SystemState state;         // Состояние
    void (*enterFunction)();   // Функция входа в состояние
    void (*exitFunction)();    // Функция выхода из состояния
    void (*processFunction)(); // Функция обработки состояния
    bool (*guardCondition)();  // Условие перехода (может быть NULL)
    uint32_t timeoutMs;        // Таймаут состояния (0 = нет таймаута)
    bool isBlocking;           // Блокирующее состояние
} StateHandler;

// Структура контекста состояния
typedef struct
{
    SystemState currentState;     // Текущее состояние
    SystemState previousState;    // Предыдущее состояние
    SystemState nextState;        // Следующее состояние (если запланирован)
    uint32_t stateEnterTime;      // Время входа в текущее состояние
    uint32_t stateDuration;       // Длительность текущего состояния
    uint8_t transitionInProgress; // Флаг перехода
    uint32_t transitionStartTime; // Время начала перехода
    uint32_t eventQueueHead;      // Голова очереди событий
    uint32_t eventQueueTail;      // Хвост очереди событий
    Event eventQueue[16];         // Очередь событий (кольцевой буфер)
    uint8_t eventCount;           // Количество событий в очереди
} StateContext;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация и управление
void statesInit(void);
void statesProcess(void);
void statesReset(void);
StateContext *statesGetContext(void);

// Управление состояниями
bool statesChangeState(SystemState newState, uint16_t flags, uint32_t delayMs);
bool statesRequestStateChange(SystemState newState);
void statesForceState(SystemState newState);
SystemState statesGetCurrentState(void);
SystemState statesGetPreviousState(void);
uint32_t statesGetStateDuration(void);
bool statesIsStateValid(SystemState state);
bool statesIsTransitionAllowed(SystemState fromState, SystemState toState);

// Управление событиями
bool statesPostEvent(EventType type, int32_t data, void *context);
bool statesPostEventFromISR(EventType type, int32_t data, void *context);
bool statesProcessEventQueue(void);
bool statesClearEventQueue(void);
uint8_t statesGetEventCount(void);
bool statesHasPendingEvents(void);

// Обработчики состояний (объявления)
void stateEnterBoot(void);
void stateProcessBoot(void);
void stateExitBoot(void);

void stateEnterIdle(void);
void stateProcessIdle(void);
void stateExitIdle(void);

void stateEnterMenuNavigation(void);
void stateProcessMenuNavigation(void);
void stateExitMenuNavigation(void);

void stateEnterProgramSelection(void);
void stateProcessProgramSelection(void);
void stateExitProgramSelection(void);

void stateEnterProgramEdit(void);
void stateProcessProgramEdit(void);
void stateExitProgramEdit(void);

void stateEnterZoneEdit(void);
void stateProcessZoneEdit(void);
void stateExitZoneEdit(void);

void stateEnterAutoRunning(void);
void stateProcessAutoRunning(void);
void stateExitAutoRunning(void);

void stateEnterManualControl(void);
void stateProcessManualControl(void);
void stateExitManualControl(void);

void stateEnterCalibration(void);
void stateProcessCalibration(void);
void stateExitCalibration(void);

void stateEnterSettings(void);
void stateProcessSettings(void);
void stateExitSettings(void);

void stateEnterMonitor(void);
void stateProcessMonitor(void);
void stateExitMonitor(void);

void stateEnterDiagnostics(void);
void stateProcessDiagnostics(void);
void stateExitDiagnostics(void);

void stateEnterError(void);
void stateProcessError(void);
void stateExitError(void);

void stateEnterEmergency(void);
void stateProcessEmergency(void);
void stateExitEmergency(void);

// Специальные функции
void statesHandleEmergencyStop(void);
void statesHandleError(ErrorType error, const char *message);
bool statesRecoverFromEmergency(void);
bool statesRecoverFromError(void);
void statesCheckStateTimeouts(void);

// Утилиты
const char *statesGetStateName(SystemState state);
const char *statesGetEventName(EventType event);
void statesPrintCurrentState(void);
void statesPrintStateHistory(void);

// Отладка
#ifdef DEBUG_STATES
void statesPrintEventQueue(void);
void statesPrintTransitions(void);
void statesDumpContext(void);
#endif

#endif // STATES_H