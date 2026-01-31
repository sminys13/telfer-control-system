/**
 * @file ui.h
 * @brief UI на дисплее 128x64 (ST7565R) + управление (клавиатура 4x4 или энкодер).
 */
#pragma once

#include <stdint.h>
#include "config.h"
#include "sensors.h"

#if USE_KEYPAD
#include "keypad.h"
#endif

enum class RunMode : uint8_t { STOP=0, MANUAL=1, AUTO=2 };

struct ManualButtons {
  bool h1_fwd, h1_bwd;
  bool h2_fwd, h2_bwd;
  bool h_both_fwd, h_both_bwd;
  bool v1_up, v1_down;
  bool v2_up, v2_down;
  bool start, stop;

  // safety inputs
  bool estop;
  bool lim_h1_left, lim_h1_right, lim_h2_left, lim_h2_right;
};

struct AppActions {
  // режимы/авто
  bool toStop = false;
  bool toManual = false;
  bool startAuto = false;
  bool pauseResumeAuto = false;
  bool stopAuto = false;
  bool returnHome = false;

  // storage
  bool saveSettings = false;

  bool loadSlot = false;
  bool saveSlot = false;
  uint8_t slot = 0;

  bool copySlot = false;
  uint8_t copyFrom = 0;
  uint8_t copyTo = 0;

  bool factoryReset = false;

  // калибровки
  bool captureHome = false;
  bool captureTravel = false;
  bool captureZoneX = false;
  bool captureZoneHeight = false;
  uint8_t zoneIndex = 0;
};

struct UiStateSummary {
  RunMode mode;
  bool autoPaused;
  uint8_t activeSlot;
  uint8_t autoOrderIndex;
  uint8_t autoZoneIndex;
  ErrorCode error;

  // Modbus/RS485 связь с 4 ПЧ (H1,H2,V1,V2).
  // UI специально НЕ включает motors.h: сюда передаём только то, что нужно для отображения.
  static constexpr uint8_t MB_DRIVES = 4;
  uint8_t mbConnectedMask = 0;              // bit0=H1, bit1=H2, bit2=V1, bit3=V2
  uint16_t mbStatus[MB_DRIVES] = {0,0,0,0}; // регистр 0x0020 (Status)
  uint16_t mbFault [MB_DRIVES] = {0,0,0,0}; // регистр 0x0021 (Fault)
};

class UI {
public:
  void begin();
  void tick(uint32_t nowMs,
            const SensorsSnapshot& sensors,
            const UiStateSummary& st,
            GlobalSettings& settings,
            ProgramConfig& program,
            AppActions& actionsOut,
            ManualButtons& manualButtonsOut);

  bool manualSyncEnabled() const { return _manualSync; }
  void setManualSyncEnabled(bool v) { _manualSync = v; }

private:
  bool _manualSync = false;

  enum class Screen : uint8_t {
    STATUS,
    MAIN_MENU,
    AUTO_MENU,
    PROGRAM_MENU,
    CAL_MENU,
    SETTINGS_MENU,
    MANUAL_SCREEN,
    SERVICE_MENU
  };
  Screen _screen = Screen::STATUS;

  uint8_t _sel = 0;
  uint8_t _scroll = 0;
  bool _editing = false;

#if USE_ENCODER
  // энкодер
  long _encLast = 0;
  bool _encBtnLast = false;
  uint32_t _encBtnDownMs = 0;
#endif

#if USE_KEYPAD
  Keypad4x4 _kp;
#endif

  // временные значения
  uint8_t _tmpSlotSel = 0;
  uint8_t _tmpZoneSel = 0;
  uint8_t _tmpOrderStep = 0; // какой шаг в order[] редактируем
  uint8_t _tmpOrderZone = 0; // какое значение зоны ставим (0..zone_count-1)

  bool readBtn(uint8_t pin) const;

  void menuMove(int8_t delta, uint8_t itemCount);
#if USE_ENCODER
  void menuClickLogic(bool pressed, bool& click, bool& longPress, uint32_t nowMs);
#endif

  void drawStatus(const SensorsSnapshot& sensors, const UiStateSummary& st);
  void drawMenu(const __FlashStringHelper* title,
                const char* const* itemsPgm,
                uint8_t itemCount,
                const char* footerLine1 = nullptr,
                const char* footerLine2 = nullptr);

  void screenMainMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenAutoMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenProgramMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenCalMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenSettingsMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenServiceMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenManual(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
};
