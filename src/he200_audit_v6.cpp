#include "he200_audit_v6.h"
#include <avr/pgmspace.h>
namespace {
struct Item { uint16_t reg; char name[9]; };
// HE200 CN A6 V3.2 pp. 11-14, 17, 21, 34-35, 41-42, 48-50.
const Item items[] PROGMEM = {
  {0xF001, "P0.01"},
  {0xF002, "P0.02"},
  {0xF003, "P0.03"},
  {0xF004, "P0.04"},
  {0xF007, "P0.07"},
  {0xF009, "P0.09"},
  {0xF00A, "P0.10"},
  {0xF00B, "P0.11"},
  {0xF00C, "P0.12"},
  {0xF00E, "P0.14"},
  {0xF011, "P0.17"},
  {0xF012, "P0.18"},
  {0xF013, "P0.19"},
  {0xF016, "P0.22"},
  {0xF018, "P0.24"},
  {0xF019, "P0.25"},
  {0xF01B, "P0.27"},
  {0xF01C, "P0.28"},
  {0xF30D, "P3.13"},
  {0xF30E, "P3.14"},
  {0xF400, "P4.00"},
  {0xF401, "P4.01"},
  {0xF402, "P4.02"},
  {0xF403, "P4.03"},
  {0xF404, "P4.04"},
  {0xF405, "P4.05"},
  {0xF40A, "P4.10"},
  {0xF40B, "P4.11"},
  {0xF423, "P4.35"},
  {0xF424, "P4.36"},
  {0xF425, "P4.37"},
  {0xF426, "P4.38"},
  {0xF427, "P4.39"},
  {0xF502, "P5.02"},
  {0xF512, "P5.18"},
  {0xF516, "P5.22"},
  {0xF600, "P6.00"},
  {0xF601, "P6.01"},
  {0xF603, "P6.03"},
  {0xF604, "P6.04"},
  {0xF605, "P6.05"},
  {0xF606, "P6.06"},
  {0xF607, "P6.07"},
  {0xF60A, "P6.10"},
  {0xF60B, "P6.11"},
  {0xF60C, "P6.12"},
  {0xF60D, "P6.13"},
  {0xF60E, "P6.14"},
  {0xF70A, "P7.10"},
  {0xF70B, "P7.11"},
  {0xF80C, "P8.12"},
  {0xF80D, "P8.13"},
  {0xF80E, "P8.14"},
  {0xF812, "P8.18"},
  {0xA000, "A0.00"},
  {0xA001, "A0.01"},
  {0xA003, "A0.03"},
  {0xA005, "A0.05"},
  {0xA006, "A0.06"},
  {0xA100, "A1.00"},
  {0xA101, "A1.01"},
  {0xA102, "A1.02"},
  {0xA103, "A1.03"},
  {0xA104, "A1.04"},
  {0xA105, "A1.05"},
  {0xA106, "A1.06"},
  {0xFD00, "Pd.00"},
  {0xFD01, "Pd.01"},
  {0xFD02, "Pd.02"},
  {0xFD03, "Pd.03"},
  {0xFD04, "Pd.04"},
  {0xFD05, "Pd.05"},
  {0x3000, "STATUS"},
  {0x1000, "COMM"},
  {0x1008, "DI_ALT"},
  {0x7000, "RUN_HZ"},
  {0x7001, "SET_HZ"},
  {0x7002, "BUS_V"},
  {0x7003, "OUT_V"},
  {0x7004, "OUT_I"},
  {0x7007, "DI"},
  {0x7008, "DO"},
  {0x701C, "COMM_D"},
  {0x701E, "FREQ_A"},
  {0x701F, "FREQ_B"},
  {0x7029, "DI_VIEW"},
  {0x702A, "DO_VIEW"},
  {0x702B, "FUNC1"},
  {0x702C, "FUNC2"},
  {0x702D, "FAULT"},
  {0x703B, "SET_PCT"},
  {0x703C, "RUN_PCT"},
  {0x703D, "STATE"},
};
const uint8_t count = sizeof(items)/sizeof(items[0]);
}
bool He200AuditV6::start(ModbusMasterRTU& bus, const uint8_t addresses[4]) {
  if (_active || bus.isDryRun()) return false;
  for (uint8_t i=0;i<4;++i) {
    if (!addresses[i] || addresses[i]>247) return false;
    for (uint8_t j=0;j<i;++j) if (addresses[j]==addresses[i]) return false;
    _addresses[i]=addresses[i];
  }
  _bus=&bus; _drive=0; _item=0; _errors=0; _lastMs=millis(); _active=true; ++_scan;
  Serial.print(F("@AUDIT state=START scan=")); Serial.print(_scan);
  Serial.print(F(" rows=")); Serial.print(count);
  Serial.println(F(" drives=4 fc=03 writes=0 motionPermit=0"));
  return true;
}
void He200AuditV6::cancel() {
  Serial.print(F("@AUDIT state=CANCELLED scan=")); Serial.println(_scan);
  _active=false;
}
bool He200AuditV6::tick(uint32_t nowMs) {
  if (!_active || (uint32_t)(nowMs-_lastMs)<30) return false;
  Item item; memcpy_P(&item,&items[_item],sizeof(item));
  uint16_t value=0;
  const ModbusResult result=_bus->readHoldingRegisters(_addresses[_drive],item.reg,1,&value);
  Serial.print(F("@AUDIT_ROW scan=")); Serial.print(_scan);
  Serial.print(F(" drive=")); Serial.print(_drive);
  Serial.print(F(" addr=")); Serial.print(_addresses[_drive]);
  Serial.print(F(" key=")); Serial.print(item.name);
  Serial.print(F(" reg=0x")); Serial.print(item.reg,HEX);
  Serial.print(F(" ok=")); Serial.print(result.ok && !result.simulated ? 1 : 0);
  if (result.ok && !result.simulated) {
    Serial.print(F(" value=")); Serial.print(value);
    Serial.print(F(" raw=0x")); Serial.print(value,HEX);
  } else { ++_errors; }
  Serial.print(F(" error=")); Serial.print(result.error);
  Serial.print(F(" exception=")); Serial.print(result.exception);
  Serial.print(F(" ms=")); Serial.println(millis());
  _lastMs=millis();
  if (++_drive==4) { _drive=0; ++_item; }
  if (_item==count) {
    _active=false;
    Serial.print(F("@AUDIT state=DONE scan=")); Serial.print(_scan);
    Serial.print(F(" errors=")); Serial.print(_errors);
    Serial.println(F(" motionPermit=0"));
  }
  return true;
}
