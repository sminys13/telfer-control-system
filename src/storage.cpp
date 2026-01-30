\
/**
 * @file storage.cpp
 * @brief EEPROM storage.
 */
#include "storage.h"
#include "utils.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>

static constexpr uint32_t MAGIC_TLFR = 0x52464C54UL; // 'T''L''F''R' little-endian
static constexpr uint8_t  STORAGE_VER = 1;

uint16_t Storage::crc16(const uint8_t* data, uint16_t len) const {
  return crc16_modbus(data, len);
}

uint16_t Storage::addrSettings() const {
  return (uint16_t)(addrHeader() + sizeof(Header));
}

uint16_t Storage::addrProgram(uint8_t slot) const {
  // после settings идёт массив программ
  return (uint16_t)(addrSettings() + sizeof(GlobalSettings) + (uint16_t)slot * sizeof(ProgramConfig));
}

bool Storage::readHeader(bool& ok) {
  Header h;
  EEPROM.get(addrHeader(), h);

  ok = (h.magic == MAGIC_TLFR) && (h.ver == STORAGE_VER) && (h.activeSlot < PROGRAM_SLOTS);
  if (!ok) return false;

  // CRC
  Header tmp = h;
  tmp.crc = 0;
  uint16_t c = crc16((const uint8_t*)&tmp, sizeof(tmp));
  ok = (c == h.crc);
  if (ok) _activeSlot = h.activeSlot;
  return ok;
}

bool Storage::writeHeader() {
  Header h;
  h.magic = MAGIC_TLFR;
  h.ver = STORAGE_VER;
  h.activeSlot = _activeSlot;
  h.crc = 0;
  h.crc = crc16((const uint8_t*)&h, sizeof(h));

  EEPROM.put(addrHeader(), h);
  return true;
}

void Storage::makeDefaultSettings(GlobalSettings& s) const {
  memset(&s, 0, sizeof(s));
  s.home_x_mm[0] = 0;
  s.home_x_mm[1] = 0;

  // "безопасная" высота для перемещения — оператор калибрует позже
  s.travel_us_mm[0] = 800;
  s.travel_us_mm[1] = 800;

  s.h_tol_mm = DEFAULT_H_TOL_MM;
  s.v_tol_mm = DEFAULT_V_TOL_MM;
  s.drip_wait_s = 45;

  s.h_speed_pct = DEFAULT_H_SPEED_PCT;
  s.v_speed_pct = DEFAULT_V_SPEED_PCT;
  s.v_tilt_speed_pct = DEFAULT_V_TILT_PCT;
  s.manual_h_sync_default = false;
}

void Storage::makeDefaultProgram(ProgramConfig& p) const {
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "PROG1", sizeof(p.name)-1);
  p.zone_count = 1;
  p.order[0] = 0;

  for (uint8_t i=0;i<MAX_ZONES;i++) {
    p.zones[i].x_mm[0] = 0;
    p.zones[i].x_mm[1] = 0;
    p.zones[i].us_target_mm[0] = 400; // по умолчанию "погружение" на 400мм
    p.zones[i].us_target_mm[1] = 400;
    p.zones[i].dip_time_s = 60;
    p.zones[i].tilt_step_mm = 30;
    p.zones[i].step_wait_s = 30;
    p.zones[i].move_speed_pct = DEFAULT_H_SPEED_PCT;
    p.zones[i].v_speed_pct = DEFAULT_V_SPEED_PCT;
    p.zones[i].enabled = (i==0);
  }
}

void Storage::factoryReset() {
  _activeSlot = 0;
  writeHeader();

  GlobalSettings s;
  makeDefaultSettings(s);
  EEPROM.put(addrSettings(), s);

  for (uint8_t slot=0; slot<PROGRAM_SLOTS; slot++) {
    ProgramConfig p;
    makeDefaultProgram(p);
    // имя слота
    char nm[12];
    snprintf(nm, sizeof(nm), "PROG%u", (unsigned)(slot+1));
    strncpy(p.name, nm, sizeof(p.name)-1);

    EEPROM.put(addrProgram(slot), p);

    // CRC программы отдельным полем в конце не храним, потому что ProgramConfig фиксирован.
    // Если будет нужно — добавим обёртку struct {ProgramConfig p; uint16_t crc;}
  }
}

bool Storage::begin() {
  bool ok = false;
  readHeader(ok);
  if (!ok) {
    factoryReset();
  }
  return true;
}

bool Storage::loadSettings(GlobalSettings& out) {
  EEPROM.get(addrSettings(), out);
  // минимальная валидация
  if (out.h_tol_mm <= 0 || out.h_tol_mm > 200) out.h_tol_mm = DEFAULT_H_TOL_MM;
  if (out.v_tol_mm <= 0 || out.v_tol_mm > 200) out.v_tol_mm = DEFAULT_V_TOL_MM;
  if (out.h_speed_pct < 10 || out.h_speed_pct > 100) out.h_speed_pct = DEFAULT_H_SPEED_PCT;
  if (out.v_speed_pct < 10 || out.v_speed_pct > 100) out.v_speed_pct = DEFAULT_V_SPEED_PCT;
  if (out.v_tilt_speed_pct < 10 || out.v_tilt_speed_pct > 100) out.v_tilt_speed_pct = DEFAULT_V_TILT_PCT;
  return true;
}

bool Storage::saveSettings(const GlobalSettings& s) {
  EEPROM.put(addrSettings(), s);
  return true;
}

bool Storage::loadProgramSlot(uint8_t slot, ProgramConfig& out) {
  if (slot >= PROGRAM_SLOTS) return false;
  EEPROM.get(addrProgram(slot), out);

  // Валидация
  if (out.zone_count < 1 || out.zone_count > MAX_ZONES) out.zone_count = 1;
  for (uint8_t i=0;i<out.zone_count;i++) {
    if (out.order[i] >= out.zone_count) out.order[i] = i;
  }
  return true;
}

bool Storage::saveProgramSlot(uint8_t slot, const ProgramConfig& p) {
  if (slot >= PROGRAM_SLOTS) return false;
  EEPROM.put(addrProgram(slot), p);
  return true;
}

bool Storage::loadActiveProgram(ProgramConfig& out) {
  return loadProgramSlot(_activeSlot, out);
}

bool Storage::saveActiveProgram(const ProgramConfig& p) {
  return saveProgramSlot(_activeSlot, p);
}

bool Storage::setActiveSlot(uint8_t slot) {
  if (slot >= PROGRAM_SLOTS) return false;
  _activeSlot = slot;
  return writeHeader();
}

bool Storage::copySlot(uint8_t from, uint8_t to) {
  if (from >= PROGRAM_SLOTS || to >= PROGRAM_SLOTS) return false;
  ProgramConfig p;
  if (!loadProgramSlot(from, p)) return false;
  return saveProgramSlot(to, p);
}

