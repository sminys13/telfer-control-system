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

  // laser device config
  bool applyLaserConfig = false; // отправить команды 0x04/0x80 в лазеры
  bool restartLaserStreaming = false; // отправить только laser ON + continuous

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

  // DRY zone (special) calibration
  bool captureDryX = false;
  bool captureDryHeight = false;

  // Operator confirmation (used in DRY sequence on STATUS screen)
  bool operatorNext = false;
  uint8_t zoneIndex = 0;
};

struct UiStateSummary {
  RunMode mode;
  bool autoPaused;
  bool autoRunning;
  uint8_t activeSlot;
  uint8_t autoOrderIndex;
  uint8_t autoZoneIndex;
  uint8_t autoPhase;       // AutoRunner::Phase (как число), чтобы UI мог показать стадию
  uint8_t autoZoneNow;     // 1..N (человеческий номер), 0 = n/a
  uint8_t autoZoneNext;    // 1..N, 0 = home/end/unknown
  int8_t  autoZoneDir;     // -1 = влево, +1 = вправо, 0 = неизвестно/нет
  uint16_t autoDipRemainS; // оставшееся время выдержки (сек), 0xFFFF = не в выдержке
  uint16_t autoDryRemainS; // оставшееся время сушки (сек), 0xFFFF = не в сушке
  bool autoWaitOperator;   // ждём команду оператора (A на статус-экране)
  bool autoDryAlarm;       // таймер сушки закончился, активен сигнал
  ErrorCode error;

  // Modbus/RS485 связь с 4 ПЧ (H1,H2,V1,V2).
  // UI специально НЕ включает motors.h: сюда передаём только то, что нужно для отображения.
  static constexpr uint8_t MB_DRIVES = 4;
  uint8_t mbConnectedMask = 0;                // bit0=H1, bit1=H2, bit2=V1, bit3=V2
  uint16_t mbRunFreq01Hz[MB_DRIVES] = {0,0,0,0}; // 0x7000 (Running frequency), 0.01Hz
  uint16_t mbSetFreq01Hz[MB_DRIVES] = {0,0,0,0}; // 0x7001 (Set frequency), 0.01Hz
  uint16_t mbBusV01V[MB_DRIVES]     = {0,0,0,0}; // 0x7002 (DC bus voltage), 0.1V
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

#if USE_KEYPAD
  // Ускорение навигации по меню: авто-повтор при удержании клавиш 2/8.
  int8_t _navDir = 0;               // -1=up(2), +1=down(8)
  uint32_t _navNextRepeatMs = 0;

  // В ручном режиме: удержание '0' включает/выключает синхронный ход по горизонтали.
  bool _key0Latched = false;
  uint32_t _key0DownMs = 0;

  // Для Back (B): latch, чтобы не пропускать нажатия при редком тике.
  bool _keyBLatched = false;
  // Долгое удержание B на статус-экране = START/ACK (сброс аварии/выход из STOP).
  bool _keyBLongLatched = false;
  uint32_t _keyBDownMs = 0;
#endif

  enum class Screen : uint8_t {
    STATUS,
    MAIN_MENU,
    AUTO_MENU,
    PROGRAM_MENU,
    PROGRAM_VIEW,
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

  // Отладка клавиатуры: показываем на STATUS последнюю клавишу и маску.
  char _kpLastKeyDbg = 0;
  uint16_t _kpMaskDbg = 0;
  uint32_t _kpLastKeyMs = 0;
#endif

  // STATUS screen mode toggle (0): compact vs extended
  bool _statusExtended = false;

  // Optional header tag on the right side (AUTO state, SAVE confirm, etc.)
  const __FlashStringHelper* _hdrRight = nullptr;

  // временные значения
  uint8_t _tmpSlotSel = 0;
  uint8_t _tmpZoneSel = 0;
  uint8_t _tmpOrderStep = 0; // какой шаг в order[] редактируем
  uint8_t _tmpOrderZone = 0; // какое значение зоны ставим (0..zone_count-1)

  // PROGRAM VIEW screen
  uint8_t _progViewMode = 0; // 0=OVERVIEW, 1=ORDER, 2=ZONES

  // Calibration: confirm SAVE operations (A=yes, B=no)
  struct SaveConfirm {
    bool active = false;
    uint8_t itemIndex = 0;
    uint8_t zoneIndex = 0;
    enum class Op : uint8_t { NONE, ZONE_X, ZONE_H, HOME_X, TRAVEL_H, DRY_X, DRY_H } op = Op::NONE;
  } _saveConfirm;

  uint32_t _saveFlashUntilMs = 0;

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

  // Меню с отображением значений справа (без подвала)
  typedef void (*MenuValueFn)(uint8_t idx, char* out, size_t outSize, void* ctx);
  void drawMenuValues(const __FlashStringHelper* title,
                      const char* const* itemsPgm,
                      uint8_t itemCount,
                      MenuValueFn valueFn,
                      void* ctx = nullptr);

  void screenMainMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenAutoMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenProgramMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenProgramView(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenCalMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenSettingsMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenServiceMenu(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
  void screenManual(const SensorsSnapshot&, const UiStateSummary&, GlobalSettings&, ProgramConfig&, AppActions&, bool click, bool longPress, int8_t encDelta);
};
