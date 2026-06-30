#pragma once

#include <Arduino.h>
#include <stdint.h>

#define FW_VERSION_V6_BRINGUP "v6-sensor-core"

static constexpr uint32_t DBG_BAUD = 115200;
static constexpr uint32_t DWIN_BAUD = 115200;

// SC16IS752 #1 -> X1/X2
static constexpr uint8_t PIN_SC16_1_CS = 10;
static constexpr uint8_t PIN_SC16_1_IRQ = 40; // reserved

// SC16IS752 #2 -> Z1/Z2
static constexpr uint8_t PIN_SC16_2_CS = 8;
static constexpr uint8_t PIN_SC16_2_IRQ = 41; // reserved

static constexpr uint8_t MEGA_SPI_SS_PIN = 53;

// Confirmed on the user's CJMCU-752 module.
static constexpr uint32_t SC16_XTAL_HZ = 1843200UL;
static constexpr uint16_t SC16_DIV_9600 = 12;

static constexpr uint32_t LASER_BAUD = 9600;
static constexpr uint16_t LASER_MAX_MM = 10000;

// This is the safe blocking driver period used for calibration/sensor core.
// Fast motion optimization will be a separate sensor-driver step.
static constexpr uint16_t LASER_SAMPLE_PERIOD_MS = 120;
static constexpr uint8_t LASER_RETRY_COUNT = 3;

// DWIN VP map.
static constexpr uint16_t VP_X1 = 0x1000;
static constexpr uint16_t VP_X2 = 0x1002;
static constexpr uint16_t VP_Z1 = 0x1004;
static constexpr uint16_t VP_Z2 = 0x1006;
static constexpr uint16_t VP_MODE = 0x1010;
static constexpr uint16_t VP_ERROR = 0x1012;
static constexpr uint16_t VP_CMD = 0x1100;

// Main DWIN modes/status.
static constexpr uint16_t MODE_SENSOR_CORE = 20;

// DWIN command values written to VP_CMD=0x1100.
static constexpr uint16_t CMD_NONE = 0x0000;

static constexpr uint16_t CMD_ZERO_X1 = 0x0021;
static constexpr uint16_t CMD_ZERO_X2 = 0x0022;
static constexpr uint16_t CMD_ZERO_Z1 = 0x0023;
static constexpr uint16_t CMD_ZERO_Z2 = 0x0024;

static constexpr uint16_t CMD_SAVE = 0x0030;
static constexpr uint16_t CMD_LOAD = 0x0031;
static constexpr uint16_t CMD_RESET_CAL = 0x0032;

// Sensor error bitmask written to VP_ERROR.
static constexpr uint16_t ERROR_NONE = 0;
static constexpr uint16_t ERROR_X1_INVALID = 1 << 0;
static constexpr uint16_t ERROR_X2_INVALID = 1 << 1;
static constexpr uint16_t ERROR_Z1_INVALID = 1 << 2;
static constexpr uint16_t ERROR_Z2_INVALID = 1 << 3;
static constexpr uint16_t ERROR_SETTINGS = 1 << 8;
