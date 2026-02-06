/**
 * @file ui.cpp
 * @brief Реализация UI (U8g2 + Keypad/Encoder).
 */
#include "ui.h"
#include "utils.h"

#include <Arduino.h>
#include <U8g2lib.h>
#if USE_ENCODER
#include <Encoder.h>
#endif

#include <avr/pgmspace.h>

// Используем "_1_" буфер (экономия RAM).
static U8G2_ST7565_ERC12864_1_4W_HW_SPI u8g2(U8G2_R0, PIN_LCD_CS, PIN_LCD_DC, PIN_LCD_RST);
#if USE_ENCODER
static Encoder enc(PIN_ENC_CLK, PIN_ENC_DT);
#endif

static constexpr uint16_t LONG_PRESS_MS = 1200;
static constexpr uint8_t  MENU_VISIBLE = 4;

// ---- Меню (строки в PROGMEM, чтобы не жрать RAM) ----
// ВАЖНО: нельзя использовать F("...") в глобальных инициализаторах массивов
// (это даёт ошибки компиляции на AVR). Поэтому строки кладём в PROGMEM явно.

// MAIN
static const char S_BACK[] PROGMEM = "<-Back";
static const char S_AUTO[] PROGMEM = "Auto";
static const char S_MANUAL[] PROGMEM = "Manual";
static const char S_PROGS[] PROGMEM = "Programs";
static const char S_CAL[] PROGMEM = "Calibration";
static const char S_SET[] PROGMEM = "Settings";
static const char S_SRV[] PROGMEM = "Service";
static const char* const MENU_MAIN[] PROGMEM = {
  S_BACK, S_AUTO, S_MANUAL, S_PROGS, S_CAL, S_SET, S_SRV
};

// AUTO
static const char S_START[] PROGMEM = "Start";
static const char S_PAUSE[] PROGMEM = "Pause/Res";
static const char S_STOP[] PROGMEM = "Stop";
static const char S_HOME[] PROGMEM = "Home";
static const char* const MENU_AUTO[] PROGMEM = {
  S_BACK, S_START, S_PAUSE, S_STOP, S_HOME
};

// PROGRAMS
static const char S_SLOT_SEL[] PROGMEM = "Slot (select)";
static const char S_SLOT_LOAD[] PROGMEM = "Load slot";
static const char S_SLOT_SAVE[] PROGMEM = "Save slot";
static const char S_SLOT_COPY[] PROGMEM = "Copy active";
static const char S_ZONE_COUNT[] PROGMEM = "Zones count";
static const char S_ORDER_STEP[] PROGMEM = "Order: step";
static const char S_ORDER_ZONE[] PROGMEM = "Order: zone";
static const char* const MENU_PROG[] PROGMEM = {
  S_BACK, S_SLOT_SEL, S_SLOT_LOAD, S_SLOT_SAVE, S_SLOT_COPY, S_ZONE_COUNT, S_ORDER_STEP, S_ORDER_ZONE
};

// CALIBRATION
static const char S_ZONE_SEL[] PROGMEM = "Zone (select)";
static const char S_ZONE_ONOFF[] PROGMEM = "Zone on/off";
static const char S_CAP_X[] PROGMEM = "Save X (laser)";
static const char S_CAP_H[] PROGMEM = "Save H (ultra)";
static const char S_DIP_TIME[] PROGMEM = "Dip time (s)";
static const char S_TILT_STEP[] PROGMEM = "Tilt step (mm)";
static const char S_STEP_WAIT[] PROGMEM = "Step wait (s)";
static const char S_CAP_HOME[] PROGMEM = "Save HOME X";
static const char S_CAP_TRAVEL[] PROGMEM = "Save TRAVEL";
static const char* const MENU_CAL[] PROGMEM = {
  S_BACK, S_ZONE_SEL, S_ZONE_ONOFF, S_CAP_X, S_CAP_H, S_DIP_TIME, S_TILT_STEP, S_STEP_WAIT, S_CAP_HOME, S_CAP_TRAVEL
};

// SETTINGS
static const char S_H_TOL[] PROGMEM = "H tol (mm)";
static const char S_V_TOL[] PROGMEM = "V tol (mm)";
static const char S_H_SPD[] PROGMEM = "H speed (%)";
static const char S_V_SPD[] PROGMEM = "V speed (%)";
static const char S_TILT_SPD[] PROGMEM = "Tilt spd (%)";
static const char S_DRIP[] PROGMEM = "Drip wait (s)";
static const char S_HSYNC[] PROGMEM = "Manual H-sync";
static const char* const MENU_SET[] PROGMEM = {
  S_BACK, S_H_TOL, S_V_TOL, S_H_SPD, S_V_SPD, S_TILT_SPD, S_DRIP, S_HSYNC
};

// SERVICE
static const char S_FACTORY[] PROGMEM = "Factory reset";
static const char S_SAVE_ALL[] PROGMEM = "Save all";
static const char* const MENU_SRV[] PROGMEM = {
  S_BACK, S_FACTORY, S_SAVE_ALL
};

static void readMenuItem(const char* const* menuPgm, uint8_t idx, char* out, size_t outSize) {
  // menuPgm находится в PROGMEM → читаем указатель через pgm_read_ptr
  const char* p = (const char*)pgm_read_ptr(&menuPgm[idx]);
  if (!p) { out[0] = 0; return; }
  strncpy_P(out, (PGM_P)p, outSize - 1);
  out[outSize - 1] = 0;
}

bool UI::readBtn(uint8_t pin) const {
  return readActiveLow(pin, BUTTON_ACTIVE_LOW);
}

void UI::begin() {
  // Пины
  // ВАЖНО: многие энкодер-модули EC11 имеют лишь «сухие» контакты (S1/S2 замыкают на GND).
  // Поэтому для стабильной работы ОБЯЗАТЕЛЬНО включаем подтяжку вверх.
  // (Если на модуле уже стоят внешние подтяжки — это не мешает: получится параллельная подтяжка.)
#if USE_ENCODER
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
#else
  /* Encoder disabled (USE_ENCODER=0). */
#endif

  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_START, INPUT_PULLUP);

  pinMode(PIN_BTN_H_BOTH_FWD, INPUT_PULLUP);
  pinMode(PIN_BTN_H_BOTH_BWD, INPUT_PULLUP);

  pinMode(PIN_BTN_H1_FWD, INPUT_PULLUP);
  pinMode(PIN_BTN_H1_BWD, INPUT_PULLUP);
  pinMode(PIN_BTN_H2_FWD, INPUT_PULLUP);
  pinMode(PIN_BTN_H2_BWD, INPUT_PULLUP);

  pinMode(PIN_BTN_V1_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_V1_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_V2_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_V2_DOWN, INPUT_PULLUP);

  pinMode(PIN_ESTOP, INPUT_PULLUP);

  pinMode(PIN_LIM_H1_LEFT, INPUT_PULLUP);
  pinMode(PIN_LIM_H1_RIGHT, INPUT_PULLUP);
  pinMode(PIN_LIM_H2_LEFT, INPUT_PULLUP);
  pinMode(PIN_LIM_H2_RIGHT, INPUT_PULLUP);

#if USE_KEYPAD
  _kp.begin();
#endif

  // Дисплей
  u8g2.begin();
  // Контраст (пользователь просил, иначе "засвечено")
  u8g2.setContrast(LCD_CONTRAST);
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.setFontMode(1);

  // Очистка мусора после прошивки
  for (uint8_t i=0;i<2;i++) {
    u8g2.firstPage();
    do { } while (u8g2.nextPage());
    delay(20);
  }

#if USE_ENCODER
  _encLast = enc.read() / ENCODER_DIV;
  _encBtnLast = readBtn(PIN_ENC_SW);
#endif
  _screen = Screen::STATUS;
  _sel = 0; _scroll = 0; _editing = false;
  _tmpSlotSel = 0;
  _tmpZoneSel = 0;
  _tmpOrderStep = 0;
  _tmpOrderZone = 0;
}

void UI::menuMove(int8_t delta, uint8_t itemCount) {
  if (delta == 0 || _editing) return;
  int16_t n = (int16_t)_sel + delta;
  if (n < 0) n = 0;
  if (n >= (int16_t)itemCount) n = itemCount - 1;
  _sel = (uint8_t)n;

  if (_sel < _scroll) _scroll = _sel;
  if (_sel >= _scroll + MENU_VISIBLE) _scroll = _sel - (MENU_VISIBLE - 1);
}

#if USE_ENCODER
void UI::menuClickLogic(bool pressed, bool& click, bool& longPress, uint32_t nowMs) {
  click = false;
  longPress = false;

  if (pressed && !_encBtnLast) {
    _encBtnDownMs = nowMs;
  } else if (!pressed && _encBtnLast) {
    uint32_t dur = nowMs - _encBtnDownMs;
    if (dur < LONG_PRESS_MS) click = true;
  } else if (pressed && _encBtnLast) {
    uint32_t dur = nowMs - _encBtnDownMs;
    if (dur >= LONG_PRESS_MS) {
      longPress = true;
      _encBtnDownMs = nowMs + 60000UL;
    }
  }
  _encBtnLast = pressed;
}
#endif

static const __FlashStringHelper* errToText(ErrorCode e) {
  switch (e) {
    case ErrorCode::NONE: return F("OK");
    case ErrorCode::ESTOP: return F("E-STOP!");
    case ErrorCode::LIMIT_SWITCH: return F("LIMIT!");
    case ErrorCode::MODBUS_COMM: return F("RS485/Modbus");
    case ErrorCode::LASER1_FAIL: return F("Laser1");
    case ErrorCode::LASER2_FAIL: return F("Laser2");
    case ErrorCode::US1_FAIL: return F("US1");
    case ErrorCode::US2_FAIL: return F("US2");
    case ErrorCode::SENSOR_TIMEOUT: return F("Sensor timeout");
    case ErrorCode::INVALID_PROGRAM: return F("Bad program");
    case ErrorCode::DRIVE_FAULT: return F("Drive fault");
    default: return F("ERR");
  }
}

void UI::drawStatus(const SensorsSnapshot& sensors, const UiStateSummary& st) {
  char b1[14], b2[14];
  char fH1[8], fH2[8], fV1[8], fV2[8];

  auto formatFreq = [&](uint8_t idx, bool showSet, char* out, size_t outSz) {
    const bool connected = (st.mbConnectedMask & (1u << idx)) != 0;
    if (!connected) {
      strncpy(out, "--.--", outSz);
      out[outSz - 1] = 0;
      return;
    }
    const uint16_t v = showSet ? st.mbSetFreq01Hz[idx] : st.mbRunFreq01Hz[idx];
    const uint16_t a = (uint16_t)(v / 100);
    const uint16_t b = (uint16_t)(v % 100);
    snprintf(out, outSz, "%u.%02u", (unsigned)a, (unsigned)b);
  };

  const bool extended = _statusExtended;

  bool holdSet = false;
#if USE_KEYPAD
  // В компактном режиме удержание 5 временно показывает SET вместо RUN.
  holdSet = _kp.isDown('5');
#endif

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_5x8_tf);

    // 1) Mode + Slot
    u8g2.setCursor(0, 8);
    if (st.mode == RunMode::STOP) u8g2.print(F("STOP "));
    else if (st.mode == RunMode::MANUAL) u8g2.print(F("MAN "));
    else u8g2.print(st.autoPaused ? F("AUTO(P) ") : F("AUTO "));
    u8g2.print(F("S"));
    u8g2.print((int)(st.activeSlot + 1));

    // 2) ERR (always)
    u8g2.setCursor(0, 16);
    u8g2.print(F("ERR:"));
    u8g2.print(errToText(st.error));

    // 3) Lasers
    u8g2.setCursor(0, 24);
    u8g2.print(F("X1="));
    if (sensors.laser[0].valid) { i32toa(sensors.laser[0].mm, b1, sizeof(b1)); u8g2.print(b1); }
    else u8g2.print(F("---"));
    u8g2.print(F(" X2="));
    if (sensors.laser[1].valid) { i32toa(sensors.laser[1].mm, b2, sizeof(b2)); u8g2.print(b2); }
    else u8g2.print(F("---"));

    // 4) Ultrasound
    u8g2.setCursor(0, 32);
    u8g2.print(F("H1="));
    if (sensors.us[0].valid) { i32toa(sensors.us[0].mm, b1, sizeof(b1)); u8g2.print(b1); }
    else u8g2.print(F("---"));
    u8g2.print(F(" H2="));
    if (sensors.us[1].valid) { i32toa(sensors.us[1].mm, b2, sizeof(b2)); u8g2.print(b2); }
    else u8g2.print(F("---"));

    // 5-8) Drives
    // Compact: show RUN by default; hold '5' -> show SET.
    // Extended: show both RUN and SET.

    const bool compactShowSet = (!extended) && holdSet;
    const bool showSetLine1 = extended ? false : compactShowSet;

    // RUN/SET line for H drives
    formatFreq(0, showSetLine1, fH1, sizeof(fH1));
    formatFreq(1, showSetLine1, fH2, sizeof(fH2));
    u8g2.setCursor(0, 40);
    u8g2.print(showSetLine1 ? 'S' : 'R');
    u8g2.print(F(" H1=")); u8g2.print(fH1);
    u8g2.print(F(" H2=")); u8g2.print(fH2);

    // RUN/SET line for V drives
    formatFreq(2, showSetLine1, fV1, sizeof(fV1));
    formatFreq(3, showSetLine1, fV2, sizeof(fV2));
    u8g2.setCursor(0, 48);
    u8g2.print(showSetLine1 ? 'S' : 'R');
    u8g2.print(F(" V1=")); u8g2.print(fV1);
    u8g2.print(F(" V2=")); u8g2.print(fV2);

    if (extended) {
      // SET lines
      formatFreq(0, true, fH1, sizeof(fH1));
      formatFreq(1, true, fH2, sizeof(fH2));
      u8g2.setCursor(0, 56);
      u8g2.print(F("S H1=")); u8g2.print(fH1);
      u8g2.print(F(" H2=")); u8g2.print(fH2);

      formatFreq(2, true, fV1, sizeof(fV1));
      formatFreq(3, true, fV2, sizeof(fV2));
      u8g2.setCursor(0, 64);
      u8g2.print(F("S V1=")); u8g2.print(fV1);
      u8g2.print(F(" V2=")); u8g2.print(fV2);
    }
  } while (u8g2.nextPage());
}

void UI::drawMenu(const __FlashStringHelper* title,
                  const char* const* itemsPgm,
                  uint8_t itemCount,
                  const char* footerLine1,
                  const char* footerLine2) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x13_tf);

    u8g2.setCursor(0, 12);
    u8g2.print(title);
    u8g2.drawHLine(0, 14, 128);

    for (uint8_t row=0; row<MENU_VISIBLE; row++) {
      uint8_t idx = _scroll + row;
      if (idx >= itemCount) break;
      uint8_t y = 28 + row*12;

      bool selected = (idx == _sel);
      if (selected) { u8g2.drawBox(0, y-10, 128, 12); u8g2.setDrawColor(0); }
      else { u8g2.setDrawColor(1); }

      char lineBuf[48];
      readMenuItem(itemsPgm, idx, lineBuf, sizeof(lineBuf));
      u8g2.setCursor(2, y);
      u8g2.print(lineBuf);

      u8g2.setDrawColor(1);
    }

    if (footerLine1) { u8g2.setCursor(0, 54); u8g2.print(footerLine1); }
    if (footerLine2) { u8g2.setCursor(0, 64); u8g2.print(footerLine2); }
  } while (u8g2.nextPage());
}

static void ensureZoneExists(ProgramConfig& p, uint8_t z) {
  if (z < p.zone_count) return;
  // расширяем зону
  uint8_t newCount = z + 1;
  if (newCount > MAX_ZONES) newCount = MAX_ZONES;
  // по умолчанию порядок 0..n-1
  for (uint8_t i=p.zone_count;i<newCount;i++) p.order[i] = i;
  p.zone_count = newCount;
  // включим новую зону
  p.zones[z].enabled = true;
}

void UI::screenMainMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&,
                        AppActions& a, bool click, bool, int8_t encDelta) {
  const uint8_t cnt = (uint8_t)(sizeof(MENU_MAIN)/sizeof(MENU_MAIN[0]));
  menuMove(encDelta, cnt);

  if (click) {
    switch (_sel) {
      case 0: _screen = Screen::STATUS; _sel=0; _scroll=0; break;
      case 1: _screen = Screen::AUTO_MENU; _sel=0; _scroll=0; break;
      case 2: a.toManual = true; _screen = Screen::MANUAL_SCREEN; _sel=0; _scroll=0; break;
      case 3: _screen = Screen::PROGRAM_MENU; _sel=0; _scroll=0; break;
      case 4: _screen = Screen::CAL_MENU; _sel=0; _scroll=0; break;
      case 5: _screen = Screen::SETTINGS_MENU; _sel=0; _scroll=0; break;
      case 6: _screen = Screen::SERVICE_MENU; _sel=0; _scroll=0; break;
      default: break;
    }
  }
  drawMenu(F("MENU"), MENU_MAIN, cnt);
}

void UI::screenAutoMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&,
                        AppActions& a, bool click, bool, int8_t encDelta) {
  const uint8_t cnt = (uint8_t)(sizeof(MENU_AUTO)/sizeof(MENU_AUTO[0]));
  menuMove(encDelta, cnt);

  if (click) {
    switch (_sel) {
      case 0: _screen = Screen::MAIN_MENU; _sel=0; _scroll=0; break;
      case 1: a.startAuto = true; break;
      case 2: a.pauseResumeAuto = true; break;
      case 3: a.stopAuto = true; break;
      case 4: a.returnHome = true; break;
      default: break;
    }
  }
  drawMenu(F("AUTO"), MENU_AUTO, cnt);
}

void UI::screenProgramMenu(const SensorsSnapshot&, const UiStateSummary& st, GlobalSettings&, ProgramConfig& program,
                           AppActions& a, bool click, bool, int8_t encDelta) {
  const uint8_t cnt = (uint8_t)(sizeof(MENU_PROG)/sizeof(MENU_PROG[0]));
  menuMove(encDelta, cnt);

  // --- Режим редактирования конкретных пунктов ---
  if (_editing) {
    // 1) выбор слота
    if (_sel == 1) {
      if (encDelta) {
        int16_t v = (int16_t)_tmpSlotSel + encDelta;
        if (v < 0) v = 0;
        if (v >= PROGRAM_SLOTS) v = PROGRAM_SLOTS - 1;
        _tmpSlotSel = (uint8_t)v;
      }
      if (click) _editing = false;
    }
    // 5) количество зон
    else if (_sel == 5) {
      if (encDelta) {
        int16_t v = (int16_t)program.zone_count + encDelta;
        v = clampT<int16_t>(v, 1, MAX_ZONES);
        // при увеличении дополним order по умолчанию 0..n-1
        if ((uint8_t)v > program.zone_count) {
          for (uint8_t i = program.zone_count; i < (uint8_t)v; i++) {
            program.order[i] = i;
            program.zones[i].enabled = true;
          }
        }
        program.zone_count = (uint8_t)v;
        if (_tmpOrderStep >= program.zone_count) _tmpOrderStep = 0;
      }
      if (click) _editing = false;
    }
    // 6) порядок: шаг
    else if (_sel == 6) {
      if (encDelta) {
        int16_t v = (int16_t)_tmpOrderStep + encDelta;
        v = clampT<int16_t>(v, 0, (int16_t)program.zone_count - 1);
        _tmpOrderStep = (uint8_t)v;
        // подтянем текущее значение зоны для этого шага
        _tmpOrderZone = program.order[_tmpOrderStep];
        if (_tmpOrderZone >= program.zone_count) _tmpOrderZone = 0;
      }
      if (click) _editing = false;
    }
    // 7) порядок: зона
    else if (_sel == 7) {
      if (encDelta) {
        int16_t v = (int16_t)_tmpOrderZone + encDelta;
        v = clampT<int16_t>(v, 0, (int16_t)program.zone_count - 1);
        _tmpOrderZone = (uint8_t)v;
      }
      if (click) {
        // применяем
        program.order[_tmpOrderStep] = _tmpOrderZone;
        _editing = false;
      }
    }
  }

  // --- Клики по пунктам ---
  if (!_editing && click) {
    switch (_sel) {
      case 0: _screen = Screen::MAIN_MENU; _sel=0; _scroll=0; break;
      case 1: _editing = true; break;
      case 2: a.loadSlot = true; a.slot = _tmpSlotSel; break;
      case 3: a.saveSlot = true; a.slot = _tmpSlotSel; break;
      case 4: a.copySlot = true; a.copyFrom = st.activeSlot; a.copyTo = _tmpSlotSel; break;
      case 5: _editing = true; break;
      case 6: _editing = true; _tmpOrderZone = program.order[_tmpOrderStep]; break;
      case 7: _editing = true; _tmpOrderZone = program.order[_tmpOrderStep]; break;
      default: break;
    }
  }

  char f1[32], f2[32];
  snprintf(f1, sizeof(f1), "Act:%u  Sel:%u", (unsigned)(st.activeSlot+1), (unsigned)(_tmpSlotSel+1));
  snprintf(f2, sizeof(f2), "Zones:%u  Ord%u->Z%u", (unsigned)program.zone_count,
           (unsigned)(_tmpOrderStep+1), (unsigned)(program.order[_tmpOrderStep]+1));
  drawMenu(F("PROGRAMS"), MENU_PROG, cnt, f1, _editing ? "Edit: rotate, click OK" : f2);
}

void UI::screenCalMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig& program,
                       AppActions& a, bool click, bool, int8_t encDelta) {
  const uint8_t cnt = (uint8_t)(sizeof(MENU_CAL)/sizeof(MENU_CAL[0]));
  menuMove(encDelta, cnt);

  // zone select
  if (_sel == 1 && _editing) {
    if (encDelta) {
      int16_t v = (int16_t)_tmpZoneSel + encDelta;
      if (v < 0) v = 0;
      if (v >= MAX_ZONES) v = MAX_ZONES - 1;
      _tmpZoneSel = (uint8_t)v;
    }
    if (click) _editing = false;
  }
  // dip time (пункт 5)
  else if (_sel == 5 && _editing) {
    ensureZoneExists(program, _tmpZoneSel);
    auto& z = program.zones[_tmpZoneSel];
    if (encDelta) {
      int32_t v = (int32_t)z.dip_time_s + encDelta * 5; // шаг 5 сек
      v = clampT<int32_t>(v, 0, 600);
      z.dip_time_s = (uint16_t)v;
    }
    if (click) _editing = false;
  }
  // tilt step (пункт 6)
  else if (_sel == 6 && _editing) {
    ensureZoneExists(program, _tmpZoneSel);
    auto& z = program.zones[_tmpZoneSel];
    if (encDelta) {
      int32_t v = (int32_t)z.tilt_step_mm + encDelta * 2; // шаг 2 мм
      v = clampT<int32_t>(v, 0, 200);
      z.tilt_step_mm = (uint16_t)v;
    }
    if (click) _editing = false;
  }
  // step wait (пункт 7)
  else if (_sel == 7 && _editing) {
    ensureZoneExists(program, _tmpZoneSel);
    auto& z = program.zones[_tmpZoneSel];
    if (encDelta) {
      int32_t v = (int32_t)z.step_wait_s + encDelta * 5; // шаг 5 сек
      v = clampT<int32_t>(v, 0, 300);
      z.step_wait_s = (uint16_t)v;
    }
    if (click) _editing = false;
  }
  else if (click) {
    switch (_sel) {
      case 0: _screen = Screen::MAIN_MENU; _sel=0; _scroll=0; break;
      case 1: _editing = true; break;
      case 2: // toggle enable
        ensureZoneExists(program, _tmpZoneSel);
        program.zones[_tmpZoneSel].enabled = !program.zones[_tmpZoneSel].enabled;
        break;
      case 3: a.captureZoneX = true; a.zoneIndex = _tmpZoneSel; break;
      case 4: a.captureZoneHeight = true; a.zoneIndex = _tmpZoneSel; break;
      case 5: _editing = true; break;
      case 6: _editing = true; break;
      case 7: _editing = true; break;
      case 8: a.captureHome = true; break;
      case 9: a.captureTravel = true; break;
      default: break;
    }
  }

  char f1[32], f2[32];
  ensureZoneExists(program, _tmpZoneSel);
  auto& z = program.zones[_tmpZoneSel];
  snprintf(f1, sizeof(f1), "Zone:%u %s Dip:%us", (unsigned)(_tmpZoneSel+1), z.enabled ? "ON" : "OFF", (unsigned)z.dip_time_s);
  snprintf(f2, sizeof(f2), "Tilt:%umm Wait:%us", (unsigned)z.tilt_step_mm, (unsigned)z.step_wait_s);
  drawMenu(F("CALIB"), MENU_CAL, cnt, f1, f2);
}

void UI::screenSettingsMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings& settings, ProgramConfig&,
                            AppActions& a, bool click, bool, int8_t encDelta) {
  const uint8_t cnt = (uint8_t)(sizeof(MENU_SET)/sizeof(MENU_SET[0]));
  menuMove(encDelta, cnt);

  auto applyEdit = [&](int32_t& val, int32_t lo, int32_t hi, int32_t step){
    if (encDelta) val = clampT<int32_t>(val + encDelta*step, lo, hi);
  };

  if (_editing) {
    switch (_sel) {
      case 1: {
        int32_t v = settings.h_tol_mm;
        applyEdit(v, 1, 200, 1);
        settings.h_tol_mm = (int16_t)v;
      } break;
      case 2: {
        int32_t v = settings.v_tol_mm;
        applyEdit(v, 1, 200, 1);
        settings.v_tol_mm = (int16_t)v;
      } break;
      case 3: {
        int32_t v = settings.h_speed_pct;
        applyEdit(v, 10, 100, 1);
        settings.h_speed_pct = (uint8_t)v;
      } break;
      case 4: {
        int32_t v = settings.v_speed_pct;
        applyEdit(v, 10, 100, 1);
        settings.v_speed_pct = (uint8_t)v;
      } break;
      case 5: {
        int32_t v = settings.v_tilt_speed_pct;
        applyEdit(v, 10, 100, 1);
        settings.v_tilt_speed_pct = (uint8_t)v;
      } break;
      case 6: {
        int32_t v = settings.drip_wait_s;
        applyEdit(v, 0, 600, 5);
        settings.drip_wait_s = (uint16_t)v;
      } break;
      default: break;
    }
    if (click) { _editing = false; a.saveSettings = true; }
  } else if (click) {
    switch (_sel) {
      case 0: _screen = Screen::MAIN_MENU; _sel=0; _scroll=0; break;
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        _editing = true;
        break;
      case 7:
        _manualSync = !_manualSync;
        settings.manual_h_sync_default = _manualSync;
        a.saveSettings = true;
        break;
      default: break;
    }
  }

  char f1[32], f2[32];
  snprintf(f1, sizeof(f1), "Htol:%d Vtol:%d", (int)settings.h_tol_mm, (int)settings.v_tol_mm);
  snprintf(f2, sizeof(f2), "H%u V%u Tilt%u", (unsigned)settings.h_speed_pct, (unsigned)settings.v_speed_pct, (unsigned)settings.v_tilt_speed_pct);
  drawMenu(F("SETTINGS"), MENU_SET, cnt, f1, _editing ? "Edit: rotate, click save" : f2);
}

void UI::screenServiceMenu(const SensorsSnapshot&, const UiStateSummary& st, GlobalSettings&, ProgramConfig&,
                           AppActions& a, bool click, bool, int8_t encDelta) {
  const uint8_t cnt = (uint8_t)(sizeof(MENU_SRV)/sizeof(MENU_SRV[0]));
  menuMove(encDelta, cnt);

  if (click) {
    switch (_sel) {
      case 0: _screen = Screen::MAIN_MENU; _sel=0; _scroll=0; break;
      case 1: a.factoryReset = true; break;
      case 2: a.saveSettings = true; a.saveSlot = true; a.slot = st.activeSlot; break;
      default: break;
    }
  }
  drawMenu(F("SERVICE"), MENU_SRV, cnt, " ", " ");
}


void UI::screenManual(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings& settings, ProgramConfig&,
                      AppActions& a, bool click, bool, int8_t) {
  if (click) { _screen = Screen::MAIN_MENU; _sel=0; _scroll=0; }

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.setCursor(0, 12);
    u8g2.print(F("MANUAL MODE"));

#if USE_KEYPAD
    u8g2.setCursor(0, 28);
    u8g2.print(F("H1:4< 6>  H2:1< 3>"));
    u8g2.setCursor(0, 40);
    u8g2.print(F("V1:7^ 9v  V2:*^ #v"));
    u8g2.setCursor(0, 52);
    u8g2.print(F("Sync 2/8: "));
    u8g2.print(settings.manual_h_sync_default ? F("ON") : F("OFF"));
    u8g2.setCursor(0, 64);
    u8g2.print(F("B=BACK   D=STOP"));
#else
    u8g2.setCursor(0, 28);
    u8g2.print(F("Buttons: H1/H2/V1/V2"));
    u8g2.setCursor(0, 40);
    u8g2.print(F("H-sync: "));
    u8g2.print(_manualSync ? F("ON") : F("OFF"));
    u8g2.setCursor(0, 52);
    u8g2.print(F("Use BOTH buttons"));
    u8g2.setCursor(0, 64);
    u8g2.print(F("Press knob: menu"));
#endif
  } while (u8g2.nextPage());

  (void)a;
}



void UI::tick(uint32_t nowMs,
              const SensorsSnapshot& sensors,
              const UiStateSummary& st,
              GlobalSettings& settings,
              ProgramConfig& program,
              AppActions& actionsOut,
              ManualButtons& manualButtonsOut) {
  // Всегда начинаем с чистого набора действий, чтобы не тянуть "хвост".
  actionsOut = AppActions{};

#if USE_KEYPAD
  // Скан клавиатуры делаем один раз за тик UI.
  _kp.tick(nowMs);
  // Одно событие "нажатия" (edge). Для меню нам обычно достаточно одного ключа.
  const char key = _kp.popKey();

  // Debug: маска и последняя клавиша.
  _kpMaskDbg = _kp.stableMask();
  if (key != 0) {
    _kpLastKeyDbg = key;
    _kpLastKeyMs = nowMs;
  }
  // "Стираем" показ последней клавиши спустя 2 секунды, чтобы было видно свежие события.
  if (_kpLastKeyDbg != 0 && (uint32_t)(nowMs - _kpLastKeyMs) > 2000) {
    _kpLastKeyDbg = 0;
  }
#else
  const char key = 0;
#endif

  // -----------------------------------------------------------------------
  // 1) Снимаем входы безопасности (E-STOP + концевики) ВСЕГДА с физических пинов.
  // -----------------------------------------------------------------------
  manualButtonsOut.estop = readActiveLow(PIN_ESTOP, ESTOP_ACTIVE_LOW);

  manualButtonsOut.lim_h1_left  = readActiveLow(PIN_LIM_H1_LEFT,  LIMIT_ACTIVE_LOW);
  manualButtonsOut.lim_h1_right = readActiveLow(PIN_LIM_H1_RIGHT, LIMIT_ACTIVE_LOW);
  manualButtonsOut.lim_h2_left  = readActiveLow(PIN_LIM_H2_LEFT,  LIMIT_ACTIVE_LOW);
  manualButtonsOut.lim_h2_right = readActiveLow(PIN_LIM_H2_RIGHT, LIMIT_ACTIVE_LOW);

  // -----------------------------------------------------------------------
  // 2) Ручные кнопки движения: по умолчанию читаем отдельные входы.
  //    В BENCH_MODE на макетке часто всё заменено перемычками — это нормально.
  // -----------------------------------------------------------------------
  manualButtonsOut.h1_fwd = readBtn(PIN_BTN_H1_FWD);
  manualButtonsOut.h1_bwd = readBtn(PIN_BTN_H1_BWD);
  manualButtonsOut.h2_fwd = readBtn(PIN_BTN_H2_FWD);
  manualButtonsOut.h2_bwd = readBtn(PIN_BTN_H2_BWD);

  manualButtonsOut.h_both_fwd = readBtn(PIN_BTN_H_BOTH_FWD);
  manualButtonsOut.h_both_bwd = readBtn(PIN_BTN_H_BOTH_BWD);

  manualButtonsOut.v1_up   = readBtn(PIN_BTN_V1_UP);
  manualButtonsOut.v1_down = readBtn(PIN_BTN_V1_DOWN);
  manualButtonsOut.v2_up   = readBtn(PIN_BTN_V2_UP);
  manualButtonsOut.v2_down = readBtn(PIN_BTN_V2_DOWN);

  manualButtonsOut.start = readBtn(PIN_BTN_START);
  manualButtonsOut.stop  = readBtn(PIN_BTN_STOP);

#if USE_KEYPAD
  // -----------------------------------------------------------------------
  // 2.1) Замена ручных кнопок матричной клавиатурой (только для MANUAL).
  //      В остальных режимах клавиатура НЕ должна случайно запускать движение.
  // -----------------------------------------------------------------------
  const bool inManual = (st.mode == RunMode::MANUAL);

  // STOP по клавише D — разрешаем ВСЕГДА (это безопасно).
  if (_kp.isDown('D')) manualButtonsOut.stop = true;

  if (inManual) {
    // Горизонталь
    manualButtonsOut.h1_bwd = _kp.isDown('4');
    manualButtonsOut.h1_fwd = _kp.isDown('6');
    manualButtonsOut.h2_bwd = _kp.isDown('1');
    manualButtonsOut.h2_fwd = _kp.isDown('3');

    // Опциональная синхронная горизонталь (оба тельфера вместе)
    const bool allowSync = settings.manual_h_sync_default;
    manualButtonsOut.h_both_bwd = allowSync && _kp.isDown('2');
    manualButtonsOut.h_both_fwd = allowSync && _kp.isDown('8');

    // Вертикаль: V Forward = DOWN
    manualButtonsOut.v1_up   = _kp.isDown('7');
    manualButtonsOut.v1_down = _kp.isDown('9');
    manualButtonsOut.v2_up   = _kp.isDown('*');
    manualButtonsOut.v2_down = _kp.isDown('#');

    // Старт ручного режима (если понадобится) — клавиша A.
    if (_kp.isDown('A')) manualButtonsOut.start = true;
  }
#endif

  // Любой STOP/E-STOP → немедленно уходим в STOP и возвращаемся на STATUS.
  if (manualButtonsOut.stop || manualButtonsOut.estop) {
    actionsOut.toStop = true;
    _screen = Screen::STATUS;
    _editing = false;
  }

  // -----------------------------------------------------------------------
  // 3) Управление UI (меню): энкодер или клавиатура.
  // -----------------------------------------------------------------------
  int8_t encDelta = 0;
  bool click = false;
  bool longPress = false;

#if USE_ENCODER
  // Энкодер: делим на ENCODER_DIV и ограничиваем шаг за тик (защита от глитчей).
  long det = enc.read() / ENCODER_DIV;
  long d = det - _encLast;
  _encLast = det;
  if (d > 4) d = 4;
  if (d < -4) d = -4;
  encDelta = (int8_t)d;

  bool encPressed = readBtn(PIN_ENC_SW);
  menuClickLogic(encPressed, click, longPress, nowMs);
#endif

#if USE_KEYPAD
  // Клавиатура: 2/8 = вверх/вниз, A = enter, B = back, C = status (домой).
  if (key == '2') encDelta = -1;
  else if (key == '8') encDelta = +1;
  else if (key == 'A') click = true;

  if (key == 'C') {
    _screen = Screen::STATUS;
    _sel = 0; _scroll = 0; _editing = false;
  }

  if (key == 'B') {
    // Универсальный "back":
    //  - из MAIN_MENU → STATUS
    //  - из любых подменю → MAIN_MENU
    if (_screen == Screen::MAIN_MENU) _screen = Screen::STATUS;
    else if (_screen != Screen::STATUS) _screen = Screen::MAIN_MENU;

    _sel = 0; _scroll = 0; _editing = false;
  }
#endif

  if (longPress) {
    _screen = Screen::STATUS;
    _sel = 0; _scroll = 0; _editing = false;
  }

  // STATUS: клик → меню
  if (_screen == Screen::STATUS) {
#if USE_KEYPAD
    // Переключение режима статуса (compact/extended) по клавише 0
    if (key == '0') {
      _statusExtended = !_statusExtended;
    }
    // В статусе хотим заходить в меню по A.
    if (key == 'A') click = true;
#endif
    if (click) {
      _screen = Screen::MAIN_MENU;
      _sel = 0; _scroll = 0; _editing = false;
      // ВАЖНО: "съедаем" этот клик, чтобы он не сработал сразу внутри MAIN_MENU (иначе можно мгновенно выбрать первый пункт).
      click = false;
    } else {
      drawStatus(sensors, st);
      return;
    }
  }

  // -----------------------------------------------------------------------
  // 4) Рисуем/обрабатываем текущий экран.
  // -----------------------------------------------------------------------
  switch (_screen) {
    case Screen::MAIN_MENU:
      screenMainMenu(sensors, st, settings, program, actionsOut, click, longPress, encDelta);
      break;
    case Screen::AUTO_MENU:
      screenAutoMenu(sensors, st, settings, program, actionsOut, click, longPress, encDelta);
      break;
    case Screen::PROGRAM_MENU:
      screenProgramMenu(sensors, st, settings, program, actionsOut, click, longPress, encDelta);
      break;
    case Screen::CAL_MENU:
      screenCalMenu(sensors, st, settings, program, actionsOut, click, longPress, encDelta);
      break;
    case Screen::SETTINGS_MENU:
      screenSettingsMenu(sensors, st, settings, program, actionsOut, click, longPress, encDelta);
      break;
    case Screen::SERVICE_MENU:
      screenServiceMenu(sensors, st, settings, program, actionsOut, click, longPress, encDelta);
      break;
    case Screen::MANUAL_SCREEN:
      screenManual(sensors, st, settings, program, actionsOut, click, longPress, encDelta);
      break;
    default:
      _screen = Screen::STATUS;
      break;
  }
}
