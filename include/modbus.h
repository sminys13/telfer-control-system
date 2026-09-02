/**
 * @file modbus.h
 * @brief Compact Modbus RTU master reused by the V6 NE200 layer.
 *
 * MAX485 wiring:
 *  - RO -> Mega RX1 (pin 19)
 *  - DI -> Mega TX1 (pin 18)
 *  - DE and /RE tied together -> PIN_VFD_RS485_DE_RE
 *  - A/B -> RS-485 bus
 */
#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

enum ModbusErrorCode : uint8_t {
  MODBUS_ERROR_NONE = 0,
  MODBUS_ERROR_TIMEOUT = 1,
  MODBUS_ERROR_CRC = 2,
  MODBUS_ERROR_EXCEPTION = 3,
  MODBUS_ERROR_BAD_RESPONSE = 4,
  MODBUS_ERROR_DRY_RUN = 5
};

struct ModbusResult {
  bool ok;
  uint8_t error;
  bool simulated;
};

class ModbusMasterRTU {
public:
  ModbusMasterRTU();

  void begin(HardwareSerial& serial,
             uint8_t deRePin,
             uint32_t baud,
             uint16_t timeoutMs,
             uint16_t serialConfig,
             uint8_t retries,
             bool transportEnabled,
             bool dryRun,
             bool traceFrames);

  void reconfigure(uint32_t baud,
                   uint16_t timeoutMs,
                   uint16_t serialConfig,
                   uint8_t retries,
                   bool transportEnabled,
                   bool dryRun,
                   bool traceFrames);

  ModbusResult writeSingleRegister(uint8_t addr, uint16_t reg, uint16_t value);
  ModbusResult readHoldingRegisters(uint8_t addr, uint16_t reg, uint16_t count, uint16_t* outValues);
  ModbusResult readInputRegisters(uint8_t addr, uint16_t reg, uint16_t count, uint16_t* outValues);

  uint8_t buildWriteSingleRegisterFrame(uint8_t addr, uint16_t reg, uint16_t value,
                                        uint8_t* out, uint8_t outSize) const;
  uint8_t buildReadRegistersFrame(uint8_t addr, uint8_t functionCode,
                                  uint16_t reg, uint16_t count,
                                  uint8_t* out, uint8_t outSize) const;

  void setTimeout(uint16_t timeoutMs) { _timeoutMs = timeoutMs; }
  void setInterFrameDelayUs(uint16_t us) { _ifDelayUs = us; }

  bool isDryRun() const { return _dryRun || !_transportEnabled; }
  bool transportEnabled() const { return _transportEnabled; }
  uint32_t baud() const { return _baud; }

private:
  HardwareSerial* _ser;
  uint8_t _de;
  uint16_t _timeoutMs;
  uint16_t _ifDelayUs;
  uint16_t _serialConfig;
  uint32_t _baud;
  uint8_t _retries;
  bool _transportEnabled;
  bool _dryRun;
  bool _traceFrames;
  bool _serialStarted;

  void txEnable(bool on);
  void clearRx();
  void startOrStopSerial();
  void traceFrame(const __FlashStringHelper* label, const uint8_t* data, uint8_t len) const;

  ModbusResult sendRequest(const uint8_t* req, uint8_t reqLen,
                           uint8_t* resp, uint8_t respMax,
                           uint8_t& respLen, uint8_t expectedMinLen);
  ModbusResult sendRequestOnce(const uint8_t* req, uint8_t reqLen,
                               uint8_t* resp, uint8_t respMax,
                               uint8_t& respLen, uint8_t expectedMinLen);

  uint16_t crc16(const uint8_t* data, size_t len) const;
  static uint16_t defaultInterFrameUs(uint32_t baud);
};
