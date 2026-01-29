/**
 * @file ui.h
 * @brief Пользовательский интерфейс системы: дисплей, меню, навигация
 * @version 4.0
 */

#ifndef UI_H
#define UI_H

#include <U8g2lib.h>
#include "../include/config.h"

// ========== КОНСТАНТЫ ИНТЕРФЕЙСА ==========

// Размеры экрана
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64



// Размеры шрифтов (в пикселях)
#define FONT_SMALL_HEIGHT 10
#define FONT_MEDIUM_HEIGHT 13
#define FONT_LARGE_HEIGHT 20
#define FONT_TITLE_HEIGHT 14

// Позиции на экране
#define HEADER_HEIGHT 16
#define HEADER_TEXT_Y 0
#define MENU_START_Y 18
#define MENU_ITEM_HEIGHT 12
#define MENU_MAX_VISIBLE 4 // Максимально видимых пунктов меню
#define FOOTER_HEIGHT 10

// Цвета (1-битный дисплей: 0 - черный, 1 - белый)
#define COLOR_BLACK 0
#define COLOR_WHITE 1
#define COLOR_INVERT 2

// Анимации
#define BLINK_INTERVAL 500 // Интервал мигания (мс)
#define PROGRESS_BAR_HEIGHT 8
#define PROGRESS_BAR_WIDTH 100

// Типы элементов интерфейса
typedef enum
{
    UI_ELEMENT_NONE,
    UI_ELEMENT_MENU,
    UI_ELEMENT_VALUE,
    UI_ELEMENT_BUTTON,
    UI_ELEMENT_PROGRESS,
    UI_ELEMENT_GAUGE,
    UI_ELEMENT_GRAPH
} UIElementType;

// Режимы редактирования значений
typedef enum
{
    EDIT_MODE_NONE,      // Не редактируется
    EDIT_MODE_INCREMENT, // Увеличение значения
    EDIT_MODE_DECREMENT, // Уменьшение значения
    EDIT_MODE_DIRECT,    // Прямой ввод
    EDIT_MODE_SELECT     // Выбор из списка
} EditMode;

// Структура пункта меню
typedef struct
{
    const char *text;     // Текст пункта
    uint8_t id;           // ID пункта
    bool enabled;         // Доступен ли пункт
    bool hasSubmenu;      // Есть подменю
    void (*action)();     // Функция при выборе
    int32_t *value;       // Указатель на значение (если есть)
    int32_t minValue;     // Минимальное значение
    int32_t maxValue;     // Максимальное значение
    const char **options; // Опции для выбора (если есть)
    uint8_t optionCount;  // Количество опций
} MenuItem;

// Структура меню
typedef struct
{
    MenuItem *items;       // Массив пунктов меню
    uint8_t itemCount;     // Количество пунктов
    uint8_t selectedIndex; // Выбранный индекс
    uint8_t scrollOffset;  // Смещение прокрутки
    uint8_t level;         // Уровень меню
    const char *title;     // Заголовок меню
    bool editing;          // Режим редактирования
    EditMode editMode;     // Режим редактирования
} Menu;

// Структура экрана
typedef struct
{
    void (*drawFunction)(); // Функция отрисовки
    void (*inputHandler)(); // Обработчик ввода
    uint8_t updateInterval; // Интервал обновления (мс)
    uint32_t lastUpdate;    // Время последнего обновления
} Screen;

// Структура анимации
typedef struct
{
    uint8_t frame;          // Текущий кадр
    uint8_t totalFrames;    // Всего кадров
    uint32_t frameTime;     // Время кадра (мс)
    uint32_t lastFrameTime; // Время последнего кадра
    bool loop;              // Зациклить анимацию
    bool active;            // Активна ли анимация
} Animation;

// Структура прогресс-бара
typedef struct
{
    int32_t value;       // Текущее значение
    int32_t min;         // Минимальное значение
    int32_t max;         // Максимальное значение
    uint8_t width;       // Ширина (пиксели)
    uint8_t height;      // Высота (пиксели)
    bool showValue;      // Показывать значение
    bool showPercentage; // Показывать проценты
} ProgressBar;

// Структура пользовательского интерфейса
typedef struct
{
    U8G2 *display;          // Указатель на объект дисплея
    Screen currentScreen;   // Текущий экран
    Menu currentMenu;       // Текущее меню
    Animation animation;    // Текущая анимация
    bool needsRedraw;       // Требуется перерисовка
    bool blinkState;        // Состояние мигания
    uint32_t lastBlinkTime; // Время последнего мигания
    uint8_t contrast;       // Контраст дисплея
    uint8_t brightness;     // Яркость (если поддерживается)
} UserInterface;

// Словарь с элементами главного меню
extern MenuItem mainMenuItems[];
extern const int mainMenuCount;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

// Инициализация
bool uiInit(U8G2 *display);
void uiSetContrast(uint8_t contrast);
void uiSetBrightness(uint8_t brightness);
void uiSetFontSmall(void);
void uiSetFontMedium(void);
void uiSetFontLarge(void);
void uiSetFontTitle(void);

// Управление экранами
void uiSetScreen(void (*drawFunction)(), void (*inputHandler)(), uint8_t updateInterval);
void uiUpdateScreen(void);
void uiForceRedraw(void);

// Меню и навигация
void uiCreateMenu(MenuItem *items, uint8_t count, const char *title, uint8_t level);
void uiNavigateUp(void);
void uiNavigateDown(void);
void uiNavigateEnter(void);
void uiNavigateBack(void);
void uiNavigateHome(void);
uint8_t uiGetSelectedId(void);
void uiSetSelectedIndex(uint8_t index);
void uiScrollToItem(uint8_t index);
void uiUpdateMenuScroll(void);

// Отрисовка элементов
void uiDrawHeader(const char *title);
void uiDrawFooter(const char *text);
void uiDrawMenu(const Menu *menu);
void uiDrawMenuItem(uint8_t x, uint8_t y, const MenuItem *item, bool selected, bool editing);
void uiDrawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                       int32_t value, int32_t min, int32_t max,
                       const char *label, bool showValue);
void uiDrawGauge(uint8_t x, uint8_t y, uint8_t radius,
                 int32_t value, int32_t min, int32_t max,
                 const char *label, const char *unit);
void uiDrawValueBox(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                    const char *label, const char *value, const char *unit,
                    bool highlighted);
void uiDrawStatusBar(uint8_t y, const char *leftText, const char *centerText, const char *rightText);
void uiDrawMessageBox(const char *title, const char *message, uint8_t type);
void uiDrawConfirmDialog(const char *question, bool *result);

// Текстовые утилиты
uint8_t uiGetTextWidth(const char *text);
uint8_t uiGetTextHeight(const char *text);
void uiDrawTextCentered(uint8_t y, const char *text);
void uiDrawTextRight(uint8_t x, uint8_t y, const char *text);
void uiDrawTextTruncated(uint8_t x, uint8_t y, uint8_t maxWidth, const char *text);
void uiDrawMultilineText(uint8_t x, uint8_t y, uint8_t lineHeight, const char *text);
void uiDrawTextWithBackground(uint8_t x, uint8_t y, const char *text, uint8_t color);

// Графические утилиты
void uiDrawHorizontalLine(uint8_t x, uint8_t y, uint8_t length);
void uiDrawVerticalLine(uint8_t x, uint8_t y, uint8_t length);
void uiDrawBox(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
void uiDrawRoundedBox(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t radius, uint8_t color);
void uiDrawCircle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color);
void uiDrawFilledCircle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color);

// Анимации
void uiStartAnimation(uint8_t totalFrames, uint32_t frameTime, bool loop);
void uiStopAnimation(void);
void uiUpdateAnimation(void);
bool uiIsAnimationActive(void);
void uiDrawLoadingAnimation(uint8_t x, uint8_t y);
void uiDrawProgressAnimation(uint8_t x, uint8_t y, uint8_t radius);

// Экранные компоненты
void uiDrawTelferPositions(int32_t pos1, int32_t pos2, int32_t targetPos, int32_t maxTravel);
void uiDrawCargoHeights(int32_t height1, int32_t height2, int32_t targetHeight, int32_t maxHeight);
void uiDrawTiltIndicator(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                         int32_t height1, int32_t height2, uint8_t tiltPercent);
void uiDrawProgramInfo(uint8_t programIndex, uint8_t zoneIndex, uint8_t totalZones,
                       uint32_t dipTime, uint32_t remainingTime);
void uiDrawSystemStatus(bool motorsEnabled, bool sensorsOk, bool emergency, uint32_t uptime);

// Конкретные экраны (объявления)
void uiDrawMainMenuScreen(void);
void uiDrawManualControlScreen(void);
void uiDrawAutoModeScreen(void);
void uiDrawCalibrationScreen(void);
void uiDrawSettingsScreen(void);
void uiDrawProgramEditScreen(void);
void uiDrawZoneEditScreen(void);
void uiDrawMonitorScreen(void);
void uiDrawDiagnosticsScreen(void);
void uiDrawEmergencyScreen(void);
void uiDrawSplashScreen(void);
void uiDrawErrorScreen(void);

// Обработчики ввода для экранов
void uiHandleMainMenuInput(void);
void uiHandleManualControlInput(void);
void uiHandleAutoModeInput(void);
void uiHandleCalibrationInput(void);
void uiHandleSettingsInput(void);
void uiHandleProgramEditInput(void);
void uiHandleZoneEditInput(void);
void uiHandleMonitorInput(void);
void uiHandleDiagnosticsInput(void);
void uiHandleEmergencyInput(void);
void uiHandleErrorInput(void);

// Вспомогательные функции
void uiBeep(uint8_t type);
void uiVibrate(uint8_t intensity);
void uiShowToast(const char *message, uint16_t duration);
void uiShowError(const char *error);
void uiShowSuccess(const char *message);
void uiShowWarning(const char *warning);

// Отладка
#ifdef DEBUG_UI
void uiPrintMenuInfo(const Menu *menu);
void uiPrintScreenInfo(const Screen *screen);
#endif

#endif // UI_H