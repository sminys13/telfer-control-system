\
/**
 * @file modbus.cpp
 * @brief Реализация минимального Modbus RTU master.
 */
#include "modbus.h"
#include "utils.h"

ModbusMasterRTU::ModbusMasterRTU()
: _ser(nullptr), _de(0), _timeoutMs(120), _ifDelayUs(250) {}

void ModbusMasterRTU::begin(HardwareSerial& serial, uint8_t deRePin, uint32_t baud, uint16_t timeoutMs, uint16_t serialConfig) {
  _ser = &serial;
  _de = deRePin;
  _timeoutMs = timeoutMs;

  pinMode(_de, OUTPUT);
  txEnable(false);

  // Параметры порта должны совпадать с Fd.02 (baud) и Fd.03 (parity) в ПЧ.
  // По умолчанию используем SERIAL_8E1 (even parity) — типичный заводской режим NE200/300.
  _ser->begin(baud, serialConfig);
  clearRx();
}

uint16_t ModbusMasterRTU::crc16(const uint8_t* data, size_t len) {
  return crc16_modbus(data, len);
}

void ModbusMasterRTU::txEnable(bool on) {
  digitalWrite(_de, on ? HIGH : LOW);
}

void ModbusMasterRTU::clearRx() {
  if (!_ser) return;
  while (_ser->available()) (void)_ser->read();
}

ModbusResult ModbusMasterRTU::sendRequest(const uint8_t* req, uint8_t reqLen, uint8_t* resp, uint8_t respMax,
                                          uint8_t& respLen, uint8_t expectedMinLen) {
  ModbusResult r{false, 0};
  respLen = 0;
  if (!_ser || !req || reqLen < 4 || !resp || respMax < expectedMinLen) {
    r.error = 4;
    return r;
  }

  // межкадровая пауза
  delayMicroseconds(_ifDelayUs);

  clearRx();
  txEnable(true);
  _ser->write(req, reqLen);
  _ser->flush();
  txEnable(false);

  // --- IMPORTANT ---
  // В предыдущих сборках был критический баг:
  // мы читали "до таймаута" даже если ответ уже пришёл.
  // На 4 ПЧ это превращало интерфейс в "не реагирует".
  //
  // Теперь делаем так:
  //  - ждём байты до общего timeout,
  //  - как только набрали expectedMinLen, выходим после короткой паузы без новых байт.
  const uint32_t t0 = millis();
  uint32_t lastByteMs = t0;
  while ((millis() - t0) < _timeoutMs && respLen < respMax) {
    if (_ser->available()) {
      resp[respLen++] = (uint8_t)_ser->read();
      lastByteMs = millis();
      continue;
    }
    // Если уже получили минимум кадра и новых байт нет несколько миллисекунд — считаем кадр завершённым.
    if (respLen >= expectedMinLen && (millis() - lastByteMs) >= 3) {
      break;
    }
  }

  if (respLen < expectedMinLen) {
    r.error = 1; // timeout/short
    return r;
  }

  // CRC проверка
  if (respLen >= 5) {
    uint16_t crcCalc = crc16(resp, respLen - 2);
    uint16_t crcRx = (uint16_t)resp[respLen - 2] | ((uint16_t)resp[respLen - 1] << 8);
    if (crcCalc != crcRx) {
      r.error = 2;
      return r;
    }
  }

  // Exception response: addr, func|0x80, code, crcLo, crcHi
  if (respLen >= 5 && (resp[1] & 0x80)) {
    r.error = 3;
    return r;
  }

  r.ok = true;
  return r;
}

ModbusResult ModbusMasterRTU::writeSingleRegister(uint8_t addr, uint16_t reg, uint16_t value) {
  // Request: [addr][06][regHi][regLo][valHi][valLo][crcLo][crcHi]
  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint16_t c = crc16(req, 6);
  req[6] = (uint8_t)(c & 0xFF);
  req[7] = (uint8_t)(c >> 8);

  uint8_t resp[16];
  uint8_t respLen = 0;
  auto r = sendRequest(req, sizeof(req), resp, sizeof(resp), respLen, 8);
  if (!r.ok) return r;

  // Ответ должен эхом повторить запрос (первые 6 байт)
  if (respLen < 8) { r.ok=false; r.error=4; return r; }
  for (uint8_t i=0;i<6;i++) {
    if (resp[i] != req[i]) { r.ok=false; r.error=4; return r; }
  }
  return r;
}

ModbusResult ModbusMasterRTU::readHoldingRegisters(uint8_t addr, uint16_t reg, uint16_t count, uint16_t* outValues) {
  // Request: [addr][03][regHi][regLo][countHi][countLo][crcLo][crcHi]
  if (count == 0 || count > 8 || !outValues) { // ограничим маленьким числом для экономии RAM
    return ModbusResult{false, 4};
  }

  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x03;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(count >> 8);
  req[5] = (uint8_t)(count & 0xFF);
  uint16_t c = crc16(req, 6);
  req[6] = (uint8_t)(c & 0xFF);
  req[7] = (uint8_t)(c >> 8);

  // Response: [addr][03][byteCount][data...][crcLo][crcHi]
  const uint8_t expectedMin = 5 + (uint8_t)(count * 2);
  uint8_t resp[64];
  uint8_t respLen = 0;
  auto r = sendRequest(req, sizeof(req), resp, sizeof(resp), respLen, expectedMin);
  if (!r.ok) return r;

  if (resp[0] != addr || resp[1] != 0x03) { return ModbusResult{false,4}; }
  if (resp[2] != count*2) { return ModbusResult{false,4}; }
  if (respLen < (uint8_t)(3 + count*2 + 2)) { return ModbusResult{false,4}; }

  uint8_t idx = 3;
  for (uint16_t i=0;i<count;i++) {
    outValues[i] = ((uint16_t)resp[idx] << 8) | (uint16_t)resp[idx+1];
    idx += 2;
  }
  return r;
}

