\
/**
 * @file storage.h
 * @brief Хранение настроек и программ в EEPROM (flash-память Arduino Mega).
 *
 * Требования:
 *  - До 10 зон.
 *  - Можно сохранять несколько программ (слоты), но в RAM держим только активную.
 *  - Параметры сохраняются редко (не "в цикле"), чтобы не убить EEPROM.
 *
 * В EEPROM хранится:
 *  - Заголовок (сигнатура, версия, активный слот)
 *  - GlobalSettings
 *  - ProgramConfig для каждого слота (фиксированный размер)
 */
#pragma once
#include <stdint.h>
#include "config.h"

class Storage {
public:
  bool begin();

  bool loadSettings(GlobalSettings& out);
  bool saveSettings(const GlobalSettings& s);

  bool loadProgramSlot(uint8_t slot, ProgramConfig& out);
  bool saveProgramSlot(uint8_t slot, const ProgramConfig& p);

  bool loadActiveProgram(ProgramConfig& out);
  bool saveActiveProgram(const ProgramConfig& p);

  uint8_t getActiveSlot() const { return _activeSlot; }
  bool setActiveSlot(uint8_t slot);

  bool copySlot(uint8_t from, uint8_t to);

  // Сброс к заводским (создаёт 1 программу по умолчанию)
  void factoryReset();

private:
  uint8_t _activeSlot = 0;

  // layout helpers
  uint16_t addrHeader() const { return 0; }
  uint16_t addrSettings() const;
  uint16_t addrProgram(uint8_t slot) const;

  uint16_t crc16(const uint8_t* data, uint16_t len) const;

  void makeDefaultSettings(GlobalSettings& s) const;
  void makeDefaultProgram(ProgramConfig& p) const;

  bool writeHeader();
  bool readHeader(bool& ok);

  struct Header {
    uint32_t magic;      // 'TLFR'
    uint8_t  ver;        // 1
    uint8_t  activeSlot; // 0..PROGRAM_SLOTS-1
    uint16_t crc;        // CRC заголовка без crc
  };
};

