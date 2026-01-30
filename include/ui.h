\
/**
 * @file ui.h
 * @brief Минимальный UI на дисплее 128x64 (ST7565R) + энкодер.
 */
#pragma once
#include <stdint.h>
#include "config.h"
#include "sensors.h"

enum class RunMode : uint8_t { STOP=0, MANUAL=1, AUTO=2 };

struct ManualButtons {
  bool h1_fwd, h1_bwd;
  bool h2_fwd, h2_bwd;
  bool h_both_fwd, h_both_bwd;
  bool v1_up, v1_down;
  bool v2_up, v2_down;
  bool start, stop;
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
  bool modbusOk;
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

  // энкодер
  long _encLast = 0;
  bool _encBtnLast = false;
  uint32_t _encBtnDownMs = 0;

  // временные значения
  uint8_t _tmpSlotSel = 0;
  uint8_t _tmpZoneSel = 0;
  uint8_t _tmpOrderStep = 0; // какой шаг в order[] редактируем
  uint8_t _tmpOrderZone = 0; // какое значение зоны ставим (0..zone_count-1)

  bool readBtn(uint8_t pin) const;

  void menuMove(int8_t delta, uint8_t itemCount);
  void menuClickLogic(bool pressed, bool& click, bool& longPress, uint32_t nowMs);

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

