/**
 * @file globals.cpp
 * @brief Глобальные переменные системы
 */

#include "../include/config.h"
#include "../include/storage.h"
#include "../include/ui.h"
#include "../include/common_definitions.h"

// ========== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Текущие данные системы
// uint8_t currentZone = 0;
// uint8_t currentProgram = 0;
uint8_t programCount = 0;
// bool isPaused = false;
// bool isEmergency = false;
// bool systemInitialized = false;
bool errorAutoReset = true;

// Диагностика
DiagnosticsResult diagnosticsResults[MAX_DIAGNOSTIC_TESTS] = {0};
uint32_t diagnosticsTime = 0;

// Временные метки
uint32_t dipStartTime = 0;
uint32_t systemStartTime = 0;

// Ручное управление
int32_t manualTargetHorizontal = 0;
int32_t manualTargetVertical = 0;

// Ошибки
// ErrorType activeError = ERROR_NONE;
// char errorMessage[64] = {0};
uint32_t errorTime = 0;

// Калибровка
uint8_t calibrationStep = 0;

// UI
uint8_t uiSelectedId = 0;