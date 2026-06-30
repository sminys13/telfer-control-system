#pragma once

#include <Arduino.h>
#include <stdint.h>

#define FW_VERSION_V6_BRINGUP "v6-step3"

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
static constexpr uint16_t LASER_SAMPLE_PERIOD_MS = 120;
static constexpr uint8_t LASER_RETRY_COUNT = 3;

// DWIN VP map.
static constexpr uint16_t VP_X1    = 0x1000;
static constexpr uint16_t VP_X2    = 0x1002;
static constexpr uint16_t VP_Z1    = 0x1004;
static constexpr uint16_t VP_Z2    = 0x1006;
static constexpr uint16_t VP_MODE  = 0x1010;
static constexpr uint16_t VP_ERROR = 0x1012;
static constexpr uint16_t VP_CMD   = 0x1100;

static constexpr uint16_t MODE_BRINGUP = 6;
static constexpr uint16_t ERROR_NONE   = 0;
static constexpr uint16_t ERROR_LASER1 = 101;
static constexpr uint16_t ERROR_LASER2 = 102;
static constexpr uint16_t ERROR_LASER3 = 103;
static constexpr uint16_t ERROR_LASER4 = 104;
