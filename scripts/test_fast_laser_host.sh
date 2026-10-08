#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STUB="${TMPDIR:-/tmp}/arduino_stubs_step9h"
OUT="${TMPDIR:-/tmp}/fast_laser_protocol_step9h_test"
rm -rf "$STUB"
mkdir -p "$STUB"
cat > "$STUB/Arduino.h" <<'STUBEOF'
#pragma once
#include <stdint.h>
#include <stddef.h>
#define OUTPUT 1
#define HIGH 1
#define LOW 0
static inline void pinMode(uint8_t, uint8_t) {}
static inline void digitalWrite(uint8_t, uint8_t) {}
static inline void delayMicroseconds(unsigned int) {}
STUBEOF
cat > "$STUB/SPI.h" <<'STUBEOF'
#pragma once
#include <stdint.h>
#define MSBFIRST 1
#define SPI_MODE0 0
class SPISettings {
public:
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};
class SPIClass {
public:
  void begin() {}
  void beginTransaction(const SPISettings&) {}
  uint8_t transfer(uint8_t value) { return value; }
};
extern SPIClass SPI;
STUBEOF
cat > "$STUB/SPI.cpp" <<'STUBEOF'
#include "SPI.h"
SPIClass SPI;
STUBEOF
g++ -std=c++11 -Wall -Wextra -Werror \
  -I"$STUB" -I"$ROOT/include" \
  "$STUB/SPI.cpp" \
  "$ROOT/src/sc16is752.cpp" \
  "$ROOT/src/fast_laser_sensor.cpp" \
  "$ROOT/test/host/test_fast_laser_protocol.cpp" \
  -o "$OUT"
"$OUT"
