/**
 * @file common_definitions.h
 * @brief Общие определения структур для всей системы
 */

#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

#include <stdint.h>
#include <stdbool.h>

// ========== ОСНОВНЫЕ СТРУКТУРЫ ==========

// Эти структуры должны быть ОДИНАКОВЫМИ во всех модулях

// typedef struct {
//     char name[16];
//     int32_t position;
//     int32_t targetHeight;
//     uint32_t dipTime;
//     uint8_t tiltAngle;
//     uint32_t waitTime;
//     bool enabled;
//     uint8_t motorSpeed;
// } ZoneSettings;

// typedef struct {
//     char name[20];
//     ZoneSettings zones[20];
//     uint8_t zoneCount;
//     uint8_t zoneOrder[20];
//     bool repeatEnabled;
//     uint16_t repeatCount;
//     uint16_t currentRepeat;
//     uint32_t totalRuntime;
// } ProgramSettings;

// typedef struct {
//     int32_t homePosition;
//     int32_t maxHorizontalTravel;
//     int32_t maxVerticalTravel;
//     uint8_t tiltSpeed;
//     uint8_t levelingSpeed;
//     uint16_t accelerationTime;
//     uint16_t decelerationTime;
//     int16_t safetyMargin;
//     bool manualOverrideAllowed;
//     uint8_t displayContrast;
//     uint16_t sensorFilterTime;
// } SystemCalibration;

// typedef struct {
//     int32_t telfer1Pos;
//     int32_t telfer2Pos;
//     int32_t cargoHeight1;
//     int32_t cargoHeight2;
//     int32_t avgHorizontalPos;
//     int32_t avgHeight;
//     int32_t tiltDifference;
//     uint32_t uptime;
//     float batteryVoltage;
//     int8_t temperature;
//     bool sensorsValid;
//     bool motorsEnabled;
//     bool emergencyActive;
// } SystemStatus;

// typedef struct {
//     uint8_t testId;
//     bool passed;
//     char message[32];
//     uint32_t timestamp;
// } DiagnosticsResult;

// typedef struct {
//     uint8_t displayContrast;
//     uint8_t displayTimeout;
//     uint8_t soundVolume;
//     bool soundEnabled;
//     bool beepOnAction;
//     bool autoSave;
//     uint8_t language;
//     uint8_t units;
//     uint16_t logRetention;
//     uint8_t logLevel;
//     uint8_t brightness;
//     bool showHelp;
//     bool confirmActions;
// } UserSettings;

#endif // COMMON_DEFINITIONS_H