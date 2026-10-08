#pragma once
#include "modbus.h"

// Fixed FC03 allowlist, one word/transaction. No write/reset/probe fallback.
class He200AuditV6 {
public:
  bool start(ModbusMasterRTU& bus, const uint8_t addresses[4]);
  bool tick(uint32_t nowMs);
  void cancel();
  bool active() const { return _active; }
private:
  ModbusMasterRTU* _bus = nullptr;
  uint8_t _addresses[4] = {};
  bool _active = false;
  uint8_t _drive = 0;
  uint8_t _item = 0;
  uint16_t _errors = 0;
  uint32_t _lastMs = 0;
  uint16_t _scan = 0;
};
