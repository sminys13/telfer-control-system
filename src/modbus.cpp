/**
 * @file modbus.cpp
 * @brief Compact Modbus RTU master with a safe frame-trace dry-run mode.
 */
#include "modbus.h"
#include "utils.h"

ModbusMasterRTU::ModbusMasterRTU()
: _ser(nullptr),
  _de(0),
  _timeoutMs(120),
  _ifDelayUs(4000),
  _serialConfig(SERIAL_8N1),
  _baud(9600),
  _retries(0),
  _transportEnabled(false),
  _dryRun(true),
  _traceFrames(true),
  _serialStarted(false) {}

uint16_t ModbusMasterRTU::defaultInterFrameUs(uint32_t baud) {
  if (baud == 0) return 4000;
  // 3.5 characters, conservatively assuming 11 bits/character.
  uint32_t us = 38500000UL / baud;
  if (us < 1750UL) us = 1750UL;
  if (us > 20000UL) us = 20000UL;
  return (uint16_t)us;
}

void ModbusMasterRTU::begin(HardwareSerial& serial,
                            uint8_t deRePin,
                            uint32_t baud,
                            uint16_t timeoutMs,
                            uint16_t serialConfig,
                            uint8_t retries,
                            bool transportEnabled,
                            bool dryRun,
                            bool traceFrames) {
  _ser = &serial;
  _de = deRePin;
  pinMode(_de, OUTPUT);
  txEnable(false);
  reconfigure(baud, timeoutMs, serialConfig, retries,
              transportEnabled, dryRun, traceFrames);
}

void ModbusMasterRTU::reconfigure(uint32_t baud,
                                  uint16_t timeoutMs,
                                  uint16_t serialConfig,
                                  uint8_t retries,
                                  bool transportEnabled,
                                  bool dryRun,
                                  bool traceFrames) {
  _baud = baud;
  _timeoutMs = timeoutMs;
  _serialConfig = serialConfig;
  _retries = retries;
  _transportEnabled = transportEnabled;
  _dryRun = dryRun;
  _traceFrames = traceFrames;
  _ifDelayUs = defaultInterFrameUs(baud);
  startOrStopSerial();
}

void ModbusMasterRTU::startOrStopSerial() {
  if (!_ser) return;

  if (_serialStarted) {
    _ser->end();
    _serialStarted = false;
  }

  // In dry-run with the physical transport disabled we deliberately do not
  // touch TX1/RX1. The exact frame is still built and printed to Serial.
  if (_transportEnabled) {
    _ser->begin(_baud, _serialConfig);
    _serialStarted = true;
    clearRx();
  }
  txEnable(false);
}

uint16_t ModbusMasterRTU::crc16(const uint8_t* data, size_t len) const {
  return crc16_modbus(data, len);
}

void ModbusMasterRTU::txEnable(bool on) {
  digitalWrite(_de, on ? HIGH : LOW);
}

void ModbusMasterRTU::clearRx() {
  if (!_ser || !_serialStarted) return;
  while (_ser->available()) (void)_ser->read();
}

void ModbusMasterRTU::traceFrame(const __FlashStringHelper* label,
                                 const uint8_t* data, uint8_t len) const {
  if (!_traceFrames) return;
  Serial.print(label);
  for (uint8_t i = 0; i < len; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.println();
}

uint8_t ModbusMasterRTU::buildWriteSingleRegisterFrame(uint8_t addr,
                                                        uint16_t reg,
                                                        uint16_t value,
                                                        uint8_t* out,
                                                        uint8_t outSize) const {
  if (!out || outSize < 8 || addr == 0) return 0;
  out[0] = addr;
  out[1] = 0x06;
  out[2] = (uint8_t)(reg >> 8);
  out[3] = (uint8_t)(reg & 0xFF);
  out[4] = (uint8_t)(value >> 8);
  out[5] = (uint8_t)(value & 0xFF);
  const uint16_t c = crc16(out, 6);
  out[6] = (uint8_t)(c & 0xFF);
  out[7] = (uint8_t)(c >> 8);
  return 8;
}

uint8_t ModbusMasterRTU::buildReadRegistersFrame(uint8_t addr,
                                                  uint8_t functionCode,
                                                  uint16_t reg,
                                                  uint16_t count,
                                                  uint8_t* out,
                                                  uint8_t outSize) const {
  if (!out || outSize < 8 || addr == 0 || count == 0) return 0;
  if (functionCode != 0x03 && functionCode != 0x04) return 0;
  out[0] = addr;
  out[1] = functionCode;
  out[2] = (uint8_t)(reg >> 8);
  out[3] = (uint8_t)(reg & 0xFF);
  out[4] = (uint8_t)(count >> 8);
  out[5] = (uint8_t)(count & 0xFF);
  const uint16_t c = crc16(out, 6);
  out[6] = (uint8_t)(c & 0xFF);
  out[7] = (uint8_t)(c >> 8);
  return 8;
}

ModbusResult ModbusMasterRTU::sendRequestOnce(const uint8_t* req, uint8_t reqLen,
                                               uint8_t* resp, uint8_t respMax,
                                               uint8_t& respLen,
                                               uint8_t expectedMinLen) {
  ModbusResult r{false, MODBUS_ERROR_NONE, false};
  respLen = 0;
  if (!_ser || !_serialStarted || !req || reqLen < 4 ||
      !resp || respMax < expectedMinLen) {
    r.error = MODBUS_ERROR_BAD_RESPONSE;
    return r;
  }

  delayMicroseconds(_ifDelayUs);
  clearRx();
  txEnable(true);
  delayMicroseconds(80);
  _ser->write(req, reqLen);
  _ser->flush();
  delayMicroseconds(80);
  txEnable(false);

  const uint32_t t0 = millis();
  uint32_t lastByteMs = t0;
  while ((uint32_t)(millis() - t0) < _timeoutMs && respLen < respMax) {
    if (_ser->available()) {
      resp[respLen++] = (uint8_t)_ser->read();
      lastByteMs = millis();
      continue;
    }
    if (respLen >= expectedMinLen &&
        (uint32_t)(millis() - lastByteMs) >= 3U) {
      break;
    }
  }

  if (respLen < expectedMinLen) {
    r.error = MODBUS_ERROR_TIMEOUT;
    return r;
  }

  const uint16_t crcCalc = crc16(resp, respLen - 2);
  const uint16_t crcRx = (uint16_t)resp[respLen - 2] |
                         ((uint16_t)resp[respLen - 1] << 8);
  if (crcCalc != crcRx) {
    r.error = MODBUS_ERROR_CRC;
    return r;
  }

  if (resp[1] & 0x80) {
    r.error = MODBUS_ERROR_EXCEPTION;
    return r;
  }

  r.ok = true;
  return r;
}

ModbusResult ModbusMasterRTU::sendRequest(const uint8_t* req, uint8_t reqLen,
                                           uint8_t* resp, uint8_t respMax,
                                           uint8_t& respLen,
                                           uint8_t expectedMinLen) {
  ModbusResult last{false, MODBUS_ERROR_BAD_RESPONSE, false};
  for (uint8_t attempt = 0; attempt <= _retries; ++attempt) {
    last = sendRequestOnce(req, reqLen, resp, respMax, respLen, expectedMinLen);
    if (last.ok) return last;
  }
  return last;
}

ModbusResult ModbusMasterRTU::writeSingleRegister(uint8_t addr,
                                                   uint16_t reg,
                                                   uint16_t value) {
  uint8_t req[8];
  const uint8_t reqLen = buildWriteSingleRegisterFrame(addr, reg, value,
                                                        req, sizeof(req));
  if (reqLen == 0) return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};

  if (isDryRun()) {
    traceFrame(F("MODBUS DRY TX FC06: "), req, reqLen);
    return ModbusResult{true, MODBUS_ERROR_NONE, true};
  }

  traceFrame(F("MODBUS TX FC06: "), req, reqLen);
  uint8_t resp[16];
  uint8_t respLen = 0;
  ModbusResult r = sendRequest(req, reqLen, resp, sizeof(resp), respLen, 8);
  if (!r.ok) return r;

  if (respLen < 8) return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};
  for (uint8_t i = 0; i < 6; ++i) {
    if (resp[i] != req[i])
      return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};
  }
  return r;
}

ModbusResult ModbusMasterRTU::readHoldingRegisters(uint8_t addr,
                                                    uint16_t reg,
                                                    uint16_t count,
                                                    uint16_t* outValues) {
  if (count == 0 || count > 8 || !outValues)
    return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};

  uint8_t req[8];
  const uint8_t reqLen = buildReadRegistersFrame(addr, 0x03, reg, count,
                                                  req, sizeof(req));
  if (reqLen == 0) return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};

  if (isDryRun()) {
    for (uint16_t i = 0; i < count; ++i) outValues[i] = 0;
    traceFrame(F("MODBUS DRY TX FC03: "), req, reqLen);
    return ModbusResult{false, MODBUS_ERROR_DRY_RUN, true};
  }

  traceFrame(F("MODBUS TX FC03: "), req, reqLen);
  const uint8_t expectedMin = (uint8_t)(5 + count * 2);
  uint8_t resp[64];
  uint8_t respLen = 0;
  ModbusResult r = sendRequest(req, reqLen, resp, sizeof(resp), respLen, expectedMin);
  if (!r.ok) return r;

  if (resp[0] != addr || resp[1] != 0x03 || resp[2] != count * 2)
    return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};
  if (respLen < (uint8_t)(3 + count * 2 + 2))
    return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};

  uint8_t pos = 3;
  for (uint16_t i = 0; i < count; ++i) {
    outValues[i] = ((uint16_t)resp[pos] << 8) | resp[pos + 1];
    pos += 2;
  }
  return r;
}

ModbusResult ModbusMasterRTU::readInputRegisters(uint8_t addr,
                                                  uint16_t reg,
                                                  uint16_t count,
                                                  uint16_t* outValues) {
  if (count == 0 || count > 8 || !outValues)
    return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};

  uint8_t req[8];
  const uint8_t reqLen = buildReadRegistersFrame(addr, 0x04, reg, count,
                                                  req, sizeof(req));
  if (reqLen == 0) return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};

  if (isDryRun()) {
    for (uint16_t i = 0; i < count; ++i) outValues[i] = 0;
    traceFrame(F("MODBUS DRY TX FC04: "), req, reqLen);
    return ModbusResult{false, MODBUS_ERROR_DRY_RUN, true};
  }

  traceFrame(F("MODBUS TX FC04: "), req, reqLen);
  const uint8_t expectedMin = (uint8_t)(5 + count * 2);
  uint8_t resp[64];
  uint8_t respLen = 0;
  ModbusResult r = sendRequest(req, reqLen, resp, sizeof(resp), respLen, expectedMin);
  if (!r.ok) return r;

  if (resp[0] != addr || resp[1] != 0x04 || resp[2] != count * 2)
    return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};
  if (respLen < (uint8_t)(3 + count * 2 + 2))
    return ModbusResult{false, MODBUS_ERROR_BAD_RESPONSE, false};

  uint8_t pos = 3;
  for (uint16_t i = 0; i < count; ++i) {
    outValues[i] = ((uint16_t)resp[pos] << 8) | resp[pos + 1];
    pos += 2;
  }
  return r;
}
