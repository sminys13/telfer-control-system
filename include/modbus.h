\
/**
 * @file modbus.h
 * @brief Минимальный Modbus RTU master для управления частотниками по RS-485.
 *
 * Почему "минимальный":
 *  - Mega2560 имеет мало RAM.
 *  - Нам достаточно Write Single Register (0x06) и Read Holding Registers (0x03).
 *
 * Подключение MAX485:
 *  - RO -> RX1 (pin 19)
 *  - DI -> TX1 (pin 18)
 *  - DE и /RE объединить -> PIN_RS485_DE_RE (pin 6)
 *  - A/B -> линия RS-485
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

struct ModbusResult {
  bool ok;
  uint8_t error; // 0=ok, 1=timeout, 2=crc, 3=exception, 4=bad_response
};

class ModbusMasterRTU {
public:
  ModbusMasterRTU();

  // serialConfig: SERIAL_8E1 / SERIAL_8N1 и т.п. (см. Fd.03 в мануале ПЧ).
  // По умолчанию SERIAL_8E1 (even parity) — типичная заводская настройка для NE200/300.
  void begin(HardwareSerial& serial, uint8_t deRePin, uint32_t baud,
            uint16_t timeoutMs = 120, uint16_t serialConfig = SERIAL_8E1);

  ModbusResult writeSingleRegister(uint8_t addr, uint16_t reg, uint16_t value);
  ModbusResult readHoldingRegisters(uint8_t addr, uint16_t reg, uint16_t count, uint16_t* outValues);

  void setTimeout(uint16_t timeoutMs) { _timeoutMs = timeoutMs; }
  void setInterFrameDelayUs(uint16_t us) { _ifDelayUs = us; }

private:
  HardwareSerial* _ser;
  uint8_t _de;
  uint16_t _timeoutMs;
  uint16_t _ifDelayUs;

  void txEnable(bool on);
  void clearRx();

  ModbusResult sendRequest(const uint8_t* req, uint8_t reqLen, uint8_t* resp, uint8_t respMax, uint8_t& respLen, uint8_t expectedMinLen);

  uint16_t crc16(const uint8_t* data, size_t len);
};

