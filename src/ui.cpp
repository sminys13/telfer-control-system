/**
 * @file ui.cpp
 * @brief Реализация пользовательского интерфейса системы
 * @version 4.0
 */

#include "../include/ui.h"
#include "../include/utils.h"
#include "../include/config.h"
#include "../include/storage.h"
#include "../include/common_definitions.h"
#include <Arduino.h>

//  Макрос-помощник
#define FMT_FLOAT(f) ((double)(f))

// ========== ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========

// Основной объект интерфейса
static UserInterface ui;

// Буферы для текста
static char textBuffer[64];
static char valueBuffer[16];
// static char unitBuffer[8];

// Текущие меню
// static Menu mainMenu;
// static Menu autoMenu;
// static Menu manualMenu;
// static Menu settingsMenu;
// static Menu calibrationMenu;
// static Menu programMenu;

// Элементы главного меню
MenuItem mainMenuItems[] = {
    {"Авто режим", 1, true, true, NULL, NULL, 0, 0, NULL, 0},
    {"Ручной режим", 2, true, true, NULL, NULL, 0, 0, NULL, 0},
    {"Программы", 3, true, true, NULL, NULL, 0, 0, NULL, 0},
    {"Калибровка", 4, true, true, NULL, NULL, 0, 0, NULL, 0},
    {"Настройки", 5, true, true, NULL, NULL, 0, 0, NULL, 0},
    {"Мониторинг", 6, true, false, NULL, NULL, 0, 0, NULL, 0},
    {"Диагностика", 7, true, false, NULL, NULL, 0, 0, NULL, 0},
    {"Информация", 8, true, false, NULL, NULL, 0, 0, NULL, 0}};

int getMainMenuItemsCount()
{
    return sizeof(mainMenuItems) / sizeof(MenuItem);
}

const int mainMenuCount = getMainMenuItemsCount();

// Элементы меню авто режима
// static MenuItem autoMenuItems[] = {
//     {"Старт", 11, true, false, NULL, NULL, 0, 0, NULL, 0},
//     {"Пауза", 12, false, false, NULL, NULL, 0, 0, NULL, 0},
//     {"Стоп", 13, false, false, NULL, NULL, 0, 0, NULL, 0},
//     {"Выбор прогр.", 14, true, true, NULL, NULL, 0, 0, NULL, 0},
//     {"Настройка", 15, true, true, NULL, NULL, 0, 0, NULL, 0},
//     {"Вернуться", 16, true, false, NULL, NULL, 0, 0, NULL, 0}};

// Элементы меню ручного режима
// static MenuItem manualMenuItems[] = {
//     {"Движение гориз.", 21, true, false, NULL, NULL, 0, 0, NULL, 0},
//     {"Движение верт.", 22, true, false, NULL, NULL, 0, 0, NULL, 0},
//     {"Наклон", 23, true, false, NULL, NULL, 0, 0, NULL, 0},
//     {"Стоп все", 24, true, false, NULL, NULL, 0, 0, NULL, 0},
//     {"Настройки", 25, true, true, NULL, NULL, 0, 0, NULL, 0},
//     {"Вернуться", 26, true, false, NULL, NULL, 0, 0, NULL, 0}};

// Состояние UI
static uint8_t currentScreen = 0;
static uint8_t previousScreen = 0;
static uint32_t screenEnterTime = 0;
static bool screenInitialized = false;

// ========== ИНИЦИАЛИЗАЦИЯ ==========

/**
 * @brief Инициализация пользовательского интерфейса
 * @param display Указатель на объект дисплея
 * @return true если инициализация успешна
 */
bool uiInit(U8G2 *display)
{
    if (!display)
    {
        Serial.println(F("ОШИБКА UI: Неверный указатель на дисплей"));
        return false;
    }

    Serial.println(F("Инициализация пользовательского интерфейса..."));

    // Сохранение указателя на дисплей
    ui.display = display;

    // Инициализация дисплея (уже должна быть выполнена в main)
    ui.display->begin();
    ui.display->setPowerSave(0);

    // Установка контраста
    ui.contrast = 150;
    ui.display->setContrast(ui.contrast);

    // Установка шрифта по умолчанию
    uiSetFontMedium();

    // Инициализация меню
    uiCreateMenu(mainMenuItems, sizeof(mainMenuItems) / sizeof(MenuItem),
                 "ГЛАВНОЕ МЕНЮ", 0);

    // Инициализация экрана
    ui.currentScreen.drawFunction = uiDrawSplashScreen;
    ui.currentScreen.inputHandler = NULL;
    ui.currentScreen.updateInterval = 100;
    ui.currentScreen.lastUpdate = 0;

    // Инициализация анимации
    ui.animation.frame = 0;
    ui.animation.totalFrames = 0;
    ui.animation.frameTime = 0;
    ui.animation.lastFrameTime = 0;
    ui.animation.loop = false;
    ui.animation.active = false;

    // Инициализация состояния
    ui.needsRedraw = true;
    ui.blinkState = false;
    ui.lastBlinkTime = 0;

    // Установка времени входа на экран
    screenEnterTime = millis();
    screenInitialized = true;

    Serial.println(F("Пользовательский интерфейс инициализирован"));
    return true;
}

/**
 * @brief Установка контраста дисплея
 * @param contrast Контраст (0-255)
 */
void uiSetContrast(uint8_t contrast)
{
    ui.contrast = contrast;
    ui.display->setContrast(contrast);
}

/**
 * @brief Установка мелкого шрифта
 */
void uiSetFontSmall(void)
{
    ui.display->setFont(u8g2_font_6x13_t_cyrillic);
}

/**
 * @brief Установка среднего шрифта
 */
void uiSetFontMedium(void)
{
    ui.display->setFont(u8g2_font_6x13_t_cyrillic);
}

/**
 * @brief Установка крупного шрифта
 */
void uiSetFontLarge(void)
{
    ui.display->setFont(u8g2_font_10x20_t_cyrillic);
}

/**
 * @brief Установка шрифта заголовка
 */
void uiSetFontTitle(void)
{
    ui.display->setFont(u8g2_font_7x14B_tf);
}

// ========== УПРАВЛЕНИЕ ЭКРАНАМИ ==========

/**
 * @brief Установка текущего экрана
 * @param drawFunction Функция отрисовки экрана
 * @param inputHandler Обработчик ввода для экрана
 * @param updateInterval Интервал обновления (мс)
 */
void uiSetScreen(void (*drawFunction)(), void (*inputHandler)(), uint8_t updateInterval)
{
    previousScreen = currentScreen;
    ui.currentScreen.drawFunction = drawFunction;
    ui.currentScreen.inputHandler = inputHandler;
    ui.currentScreen.updateInterval = updateInterval;
    ui.currentScreen.lastUpdate = millis();

    // Сброс состояния меню при смене экрана
    ui.currentMenu.selectedIndex = 0;
    ui.currentMenu.scrollOffset = 0;

    // Запись времени входа на экран
    screenEnterTime = millis();

    // Требуется перерисовка
    ui.needsRedraw = true;
}

/**
 * @brief Обновление текущего экрана
 */
void uiUpdateScreen(void)
{
    uint32_t currentTime = millis();

    // Проверка необходимости обновления
    if (currentTime - ui.currentScreen.lastUpdate >= ui.currentScreen.updateInterval)
    {
        // Обновление анимации
        uiUpdateAnimation();

        // Обновление мигания
        if (currentTime - ui.lastBlinkTime >= BLINK_INTERVAL)
        {
            ui.blinkState = !ui.blinkState;
            ui.lastBlinkTime = currentTime;
            ui.needsRedraw = true;
        }

        // Вызов функции отрисовки если требуется
        if (ui.needsRedraw && ui.currentScreen.drawFunction)
        {
            ui.display->clearBuffer();
            ui.currentScreen.drawFunction();
            ui.display->sendBuffer();
            ui.needsRedraw = false;
        }

        ui.currentScreen.lastUpdate = currentTime;
    }
}

/**
 * @brief Принудительная перерисовка экрана
 */
void uiForceRedraw(void)
{
    ui.needsRedraw = true;
    ui.currentScreen.lastUpdate = 0; // Сразу обновить
}

// ========== МЕНЮ И НАВИГАЦИЯ ==========

/**
 * @brief Создание меню
 * @param items Массив пунктов меню
 * @param count Количество пунктов
 * @param title Заголовок меню
 * @param level Уровень меню
 */
void uiCreateMenu(MenuItem *items, uint8_t count, const char *title, uint8_t level)
{
    ui.currentMenu.items = items;
    ui.currentMenu.itemCount = count;
    ui.currentMenu.selectedIndex = 0;
    ui.currentMenu.scrollOffset = 0;
    ui.currentMenu.level = level;
    ui.currentMenu.title = title;
    ui.currentMenu.editing = false;
    ui.currentMenu.editMode = EDIT_MODE_NONE;

    uiUpdateMenuScroll();
}

/**
 * @brief Навигация вверх по меню
 */
void uiNavigateUp(void)
{
    if (ui.currentMenu.editing)
    {
        // В режиме редактирования - увеличение значения
        if (ui.currentMenu.editMode == EDIT_MODE_INCREMENT)
        {
            MenuItem *item = &ui.currentMenu.items[ui.currentMenu.selectedIndex];
            if (item->value && *item->value < item->maxValue)
            {
                (*item->value)++;
                ui.needsRedraw = true;
                uiBeep(1);
            }
        }
    }
    else
    {
        // Обычная навигация
        if (ui.currentMenu.selectedIndex > 0)
        {
            ui.currentMenu.selectedIndex--;
            uiUpdateMenuScroll();
            ui.needsRedraw = true;
            uiBeep(1);
        }
    }
}

/**
 * @brief Навигация вниз по меню
 */
void uiNavigateDown(void)
{
    if (ui.currentMenu.editing)
    {
        // В режиме редактирования - уменьшение значения
        if (ui.currentMenu.editMode == EDIT_MODE_DECREMENT)
        {
            MenuItem *item = &ui.currentMenu.items[ui.currentMenu.selectedIndex];
            if (item->value && *item->value > item->minValue)
            {
                (*item->value)--;
                ui.needsRedraw = true;
                uiBeep(1);
            }
        }
    }
    else
    {
        // Обычная навигация
        if (ui.currentMenu.selectedIndex < ui.currentMenu.itemCount - 1)
        {
            ui.currentMenu.selectedIndex++;
            uiUpdateMenuScroll();
            ui.needsRedraw = true;
            uiBeep(1);
        }
    }
}

/**
 * @brief Вход в пункт меню/подтверждение
 */
void uiNavigateEnter(void)
{
    MenuItem *item = &ui.currentMenu.items[ui.currentMenu.selectedIndex];

    if (ui.currentMenu.editing)
    {
        // Выход из режима редактирования
        ui.currentMenu.editing = false;
        ui.currentMenu.editMode = EDIT_MODE_NONE;
        uiBeep(2);
    }
    else if (item->hasSubmenu)
    {
        // Вход в подменю
        uiBeep(1);
        // Здесь должна быть загрузка подменю
    }
    else if (item->value && item->minValue != item->maxValue)
    {
        // Вход в режим редактирования значения
        ui.currentMenu.editing = true;
        ui.currentMenu.editMode = EDIT_MODE_INCREMENT;
        uiBeep(1);
    }
    else if (item->action)
    {
        // Выполнение действия
        item->action();
        uiBeep(1);
    }

    ui.needsRedraw = true;
}

/**
 * @brief Возврат назад/отмена
 */
void uiNavigateBack(void)
{
    if (ui.currentMenu.editing)
    {
        // Отмена редактирования
        ui.currentMenu.editing = false;
        ui.currentMenu.editMode = EDIT_MODE_NONE;
        uiBeep(2);
    }
    else if (ui.currentMenu.level > 0)
    {
        // Возврат на уровень выше
        uiBeep(2);
        // Здесь должна быть загрузка предыдущего меню
    }
    else
    {
        // Возврат на главный экран
        uiSetScreen(uiDrawMainMenuScreen, uiHandleMainMenuInput, 100);
        uiBeep(2);
    }

    ui.needsRedraw = true;
}

/**
 * @brief Обновление прокрутки меню
 */
void uiUpdateMenuScroll(void)
{
    // Если выбранный элемент выше видимой области
    if (ui.currentMenu.selectedIndex < ui.currentMenu.scrollOffset)
    {
        ui.currentMenu.scrollOffset = ui.currentMenu.selectedIndex;
    }

    // Если выбранный элемент ниже видимой области
    if (ui.currentMenu.selectedIndex >= ui.currentMenu.scrollOffset + MENU_MAX_VISIBLE)
    {
        ui.currentMenu.scrollOffset = ui.currentMenu.selectedIndex - MENU_MAX_VISIBLE + 1;
    }

    // Ограничение смещения
    if (ui.currentMenu.scrollOffset > ui.currentMenu.itemCount - MENU_MAX_VISIBLE)
    {
        ui.currentMenu.scrollOffset = max(0, ui.currentMenu.itemCount - MENU_MAX_VISIBLE);
    }
}

// ========== ОТРИСОВКА ЭЛЕМЕНТОВ ==========

/**
 * @brief Отрисовка заголовка экрана
 * @param title Текст заголовка
 */
void uiDrawHeader(const char *title)
{
    // Черный фон заголовка
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawBox(0, 0, SCREEN_WIDTH, HEADER_HEIGHT);

    // Белый текст
    ui.display->setDrawColor(COLOR_WHITE);
    uiSetFontTitle();
    uiDrawTextCentered(HEADER_TEXT_Y, title);

    // Возврат к обычному шрифту
    uiSetFontMedium();
    ui.display->setDrawColor(COLOR_BLACK);

    // Линия под заголовком
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawHLine(0, HEADER_HEIGHT, SCREEN_WIDTH);
}

/**
 * @brief Отрисовка индикатора прокрутки
 */
void uiDrawScrollIndicator(uint8_t scrollOffset, uint8_t totalItems, uint8_t visibleItems)
{
    if (totalItems <= visibleItems)
    {
        return;
    }

    uint8_t indicatorHeight = 30;
    uint8_t indicatorWidth = 4;
    uint8_t indicatorX = SCREEN_WIDTH - indicatorWidth - 2;
    uint8_t indicatorY = MENU_START_Y;

    // Фон индикатора
    ui.display->setDrawColor(1);
    ui.display->drawFrame(indicatorX, indicatorY, indicatorWidth, indicatorHeight);

    // Позиция ползунка
    uint8_t sliderHeight = (visibleItems * indicatorHeight) / totalItems;
    sliderHeight = max(sliderHeight, 4);

    uint8_t sliderY = indicatorY + (scrollOffset * (indicatorHeight - sliderHeight)) /
                                       (totalItems - visibleItems);

    // Ползунок
    ui.display->setDrawColor(0);
    ui.display->drawBox(indicatorX + 1, sliderY + 1, indicatorWidth - 2, sliderHeight - 2);
}

/**
 * @brief Отрисовка меню
 * @param menu Указатель на меню
 */
void uiDrawMenu(const Menu *menu)
{
    if (!menu || !menu->items)
    {
        return;
    }

    // Отрисовка заголовка
    uiDrawHeader(menu->title);

    // Отрисовка пунктов меню
    uint8_t y = MENU_START_Y;

    for (uint8_t i = menu->scrollOffset;
         i < min(menu->itemCount, menu->scrollOffset + MENU_MAX_VISIBLE);
         i++)
    {

        bool selected = (i == menu->selectedIndex);
        bool editing = (selected && menu->editing);

        uiDrawMenuItem(0, y, &menu->items[i], selected, editing);
        y += MENU_ITEM_HEIGHT;
    }

    // Индикатор прокрутки, если меню не помещается
    if (menu->itemCount > MENU_MAX_VISIBLE)
    {
        uiDrawScrollIndicator(menu->scrollOffset, menu->itemCount, MENU_MAX_VISIBLE);
    }
}

/**
 * @brief Отрисовка одного пункта меню
 */
void uiDrawMenuItem(uint8_t x, uint8_t y, const MenuItem *item, bool selected, bool editing)
{
    if (!item)
    {
        return;
    }

    // Выделение выбранного пункта
    if (selected)
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawBox(x, y, SCREEN_WIDTH, MENU_ITEM_HEIGHT);
        ui.display->setDrawColor(COLOR_WHITE);
    }
    else
    {
        ui.display->setDrawColor(COLOR_BLACK);
    }

    // Отступ для стрелки
    uint8_t textX = x + 8;

    // Стрелка для выбранного пункта
    if (selected)
    {
        ui.display->drawTriangle(textX - 6, y + 3,
                                 textX - 6, y + MENU_ITEM_HEIGHT - 3,
                                 textX - 2, y + MENU_ITEM_HEIGHT / 2);
    }

    // Текст пункта
    ui.display->setFont(u8g2_font_6x13_t_cyrillic);
    ui.display->drawStr(textX, y + MENU_ITEM_HEIGHT - 2, item->text);

    // Если есть значение
    if (item->value)
    {
        snprintf(valueBuffer, sizeof(valueBuffer), "%ld", *item->value);

        uint8_t valueWidth = uiGetTextWidth(valueBuffer);
        uint8_t valueX = SCREEN_WIDTH - valueWidth - 8;

        // Скобки для режима редактирования
        if (editing)
        {
            ui.display->drawStr(valueX - 6, y + MENU_ITEM_HEIGHT - 2, "[");
            ui.display->drawStr(valueX + valueWidth + 2, y + MENU_ITEM_HEIGHT - 2, "]");
        }

        ui.display->drawStr(valueX, y + MENU_ITEM_HEIGHT - 2, valueBuffer);
    }

    // Если пункт недоступен
    if (!item->enabled)
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->setBitmapMode(1);
        for (uint8_t i = 0; i < SCREEN_WIDTH; i += 2)
        {
            ui.display->drawPixel(i, y + MENU_ITEM_HEIGHT / 2);
        }
        ui.display->setBitmapMode(0);
    }

    ui.display->setDrawColor(COLOR_BLACK);
}

/**
 * @brief Отрисовка прогресс-бара
 */
void uiDrawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                       int32_t value, int32_t min, int32_t max,
                       const char *label, bool showValue)
{
    if (max <= min)
    {
        return;
    }

    // Фон прогресс-бара
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawFrame(x, y, width, height);

    // Заполнение
    uint8_t fillWidth = (uint8_t)((int64_t)(value - min) * width / (max - min));
    fillWidth = constrain(fillWidth, 0, width - 2);

    ui.display->drawBox(x + 1, y + 1, fillWidth, height - 2);

    // Текст (если помещается)
    if (label)
    {
        uint8_t textY = y + height + 10;
        if (textY < SCREEN_HEIGHT - 8)
        {
            ui.display->setDrawColor(COLOR_BLACK);
            ui.display->drawStr(x, textY, label);
        }
    }

    // Значение (если нужно)
    if (showValue)
    {
        uint8_t percent = (uint8_t)((int64_t)(value - min) * 100 / (max - min));
        snprintf(valueBuffer, sizeof(valueBuffer), "%d%%", percent);

        uint8_t textWidth = uiGetTextWidth(valueBuffer);
        uint8_t textX = x + (width - textWidth) / 2;
        uint8_t textY = y + height / 2 + 3;

        // Инвертированный цвет для текста на заполненной части
        if (fillWidth > textX - x && fillWidth < textX - x + textWidth)
        {
            ui.display->setDrawColor(COLOR_WHITE);
            ui.display->setFontMode(1);
            ui.display->drawStr(textX, textY, valueBuffer);
            ui.display->setFontMode(0);
        }
        else
        {
            ui.display->setDrawColor(COLOR_BLACK);
            ui.display->drawStr(textX, textY, valueBuffer);
        }
    }

    ui.display->setDrawColor(COLOR_BLACK);
}

/**
 * @brief Отрисовка позиций тельферов
 */
void uiDrawTelferPositions(int32_t pos1, int32_t pos2, int32_t targetPos, int32_t maxTravel)
{
    uint8_t y = MENU_START_Y;

    // Заголовок
    ui.display->drawStr(0, y, "Позиции тельферов:");
    y += 12;

    // Тельфер 1
    snprintf(textBuffer, sizeof(textBuffer), "Т1: %ld мм", pos1);
    ui.display->drawStr(0, y, textBuffer);

    // Прогресс1
    uiDrawProgressBar(40, y - 8, 80, 8, pos1, 0, maxTravel, NULL, false);
    y += 12;

    // Тельфер 2
    snprintf(textBuffer, sizeof(textBuffer), "Т2: %ld мм", pos2);
    ui.display->drawStr(0, y, textBuffer);

    // Прогресс2
    uiDrawProgressBar(40, y - 8, 80, 8, pos2, 0, maxTravel, NULL, false);
    y += 12;

    // Целевая позиция
    snprintf(textBuffer, sizeof(textBuffer), "Цель: %ld мм", targetPos);
    ui.display->drawStr(0, y, textBuffer);

    // Разница
    int32_t avgPos = (pos1 + pos2) / 2;
    int32_t diff = targetPos - avgPos;
    snprintf(textBuffer, sizeof(textBuffer), "Осталось: %ld мм", diff);
    ui.display->drawStr(0, y + 12, textBuffer);
}

// ========== ТЕКСТОВЫЕ УТИЛИТЫ ==========

/**
 * @brief Получение ширины текста
 * @param text Текст
 * @return Ширина в пикселях
 */
uint8_t uiGetTextWidth(const char *text)
{
    return ui.display->getStrWidth(text);
}

/**
 * @brief Отрисовка текста по центру
 * @param y Y  координата
 * @param text Текст
 */
void uiDrawTextCentered(uint8_t y, const char *text)
{
    uint8_t textWidth = uiGetTextWidth(text);
    uint8_t x = (SCREEN_WIDTH - textWidth) / 2;
    ui.display->drawStr(x, y, text);
}

/**
 * @brief Отрисовка текста справа
 * @param x X координата правого края
 * @param y Y координата
 * @param text Текст
 */
void uiDrawTextRight(uint8_t x, uint8_t y, const char *text)
{
    uint8_t textWidth = uiGetTextWidth(text);
    ui.display->drawStr(x - textWidth, y, text);
}

/**
 * @brief Отрисовка текста с обрезкой
 * @param x X координата
 * @param y Y координата
 * @param maxWidth Максимальная ширина
 * @param text Текст
 */
void uiDrawTextTruncated(uint8_t x, uint8_t y, uint8_t maxWidth, const char *text)
{
    uint8_t textWidth = uiGetTextWidth(text);

    if (textWidth <= maxWidth)
    {
        ui.display->drawStr(x, y, text);
    }
    else
    {
        // Обрезка с добавлением "..."
        char truncated[32];
        strncpy(truncated, text, sizeof(truncated) - 4);
        truncated[sizeof(truncated) - 4] = '\0';

        // Поиск места для "..."
        uint8_t len = strlen(truncated);
        while (len > 0)
        {
            truncated[len] = '\0';
            strcat(truncated, "...");

            if (uiGetTextWidth(truncated) <= maxWidth)
            {
                break;
            }

            len--;
            truncated[len] = '\0';
        }

        ui.display->drawStr(x, y, truncated);
    }
}

// ========== КОНКРЕТНЫЕ ЭКРАНЫ ==========

/**
 * @brief Отрисовка экрана главного меню
 */
void uiDrawMainMenuScreen(void)
{
    uiDrawMenu(&ui.currentMenu);

    // Статус бар внизу
    extern bool systemInitialized;
    extern bool isEmergency;
    extern uint32_t systemStartTime;

    uint32_t uptime = (millis() - systemStartTime) / 1000;
    uint8_t hours = uptime / 3600;
    uint8_t minutes = (uptime % 3600) / 60;
    uint8_t seconds = uptime % 60;

    snprintf(textBuffer, sizeof(textBuffer), "%02d:%02d:%02d", hours, minutes, seconds);
    uiDrawStatusBar(SCREEN_HEIGHT - 8,
                    systemInitialized ? "ГОТОВ" : "ИНИЦ...",
                    isEmergency ? "АВАРИЯ" : "",
                    textBuffer);
}

/**
 * @brief Отрисовка экрана ручного управления
 */
void uiDrawManualControlScreen(void)
{
    uiDrawHeader("РУЧНОЕ УПРАВЛЕНИЕ");

    // Получение данных из других модулей
    extern SystemStatus systemStatus;
    extern SystemCalibration calibration;

    uint8_t y = MENU_START_Y;

    // Позиции тельферов
    uiDrawTelferPositions(systemStatus.telfer1Pos,
                          systemStatus.telfer2Pos,
                          0, // Нет целевой позиции в ручном режиме
                          calibration.maxHorizontalTravel);

    y += 50;

    // Высоты груза
    uiDrawCargoHeights(systemStatus.cargoHeight1,
                       systemStatus.cargoHeight2,
                       0,
                       calibration.maxVerticalTravel);

    // Индикатор наклона
    if (abs(systemStatus.cargoHeight1 - systemStatus.cargoHeight2) > 10)
    {
        uint8_t tiltPercent = map(abs(systemStatus.cargoHeight1 - systemStatus.cargoHeight2), 0, 500, 0, 100);
        uiDrawTiltIndicator(SCREEN_WIDTH - 40, MENU_START_Y, 30, 40,
                            systemStatus.cargoHeight1,
                            systemStatus.cargoHeight2,
                            tiltPercent);
    }

    // Инструкция внизу
    uiDrawStatusBar(SCREEN_HEIGHT - 8,
                    "Энкодер: скорость",
                    "",
                    "Долго: выход");
}

/**
 * @brief Отрисовка экрана автоматического режима
 */
void uiDrawAutoModeScreen(void)
{
    uiDrawHeader("АВТОМАТИЧЕСКИЙ РЕЖИМ");

    // Получение данных
    extern SystemStatus systemStatus;
    extern ProgramSettings programs[MAX_PROGRAMS];
    // extern uint8_t currentProgram;
    // extern uint8_t currentZone;
    extern SystemData SystemData;
    extern bool isPaused;
    extern unsigned long dipStartTime;

    uint8_t y = MENU_START_Y;

    // Информация о программе
    if (SystemData.currentProgram < MAX_PROGRAMS)
    {
        snprintf(textBuffer, sizeof(textBuffer), "Программа: %s",
                 programs[SystemData.currentProgram].name);
        ui.display->drawStr(0, y, textBuffer);
        y += 12;

        snprintf(textBuffer, sizeof(textBuffer), "Зона: %d/%d",
                 SystemData.currentZone + 1,
                 programs[SystemData.currentProgram].zoneCount);
        ui.display->drawStr(0, y, textBuffer);
        y += 12;
    }

    // Статус выполнения
    if (isPaused)
    {
        ui.display->drawStr(0, y, "ПАУЗА");
        y += 12;
    }
    else
    {
        ui.display->drawStr(0, y, "ВЫПОЛНЕНИЕ...");
        y += 12;
    }

    // Прогресс текущей зоны
    if (SystemData.currentZone < programs[SystemData.currentProgram].zoneCount)
    {
        ZoneSettings *zone = &programs[SystemData.currentProgram].zones[SystemData.currentZone];

        // Прогресс движения
        int progress = (systemStatus.avgHorizontalPos * 100) / zone->position;
        progress = constrain(progress, 0, 100);

        snprintf(textBuffer, sizeof(textBuffer), "Движение: %d%%", progress);
        ui.display->drawStr(0, y, textBuffer);
        uiDrawProgressBar(60, y - 8, 60, 8, progress, 0, 100, NULL, false);
        y += 12;

        // Время погружения
        if (dipStartTime > 0)
        {
            uint32_t elapsed = (millis() - dipStartTime) / 1000;
            uint32_t remaining = max(0, (zone->dipTime / 1000) - elapsed);

            snprintf(textBuffer, sizeof(textBuffer), "Погружение: %luс", remaining);
            ui.display->drawStr(0, y, textBuffer);

            uint8_t dipProgress = (elapsed * 100) / (zone->dipTime / 1000);
            dipProgress = constrain(dipProgress, 0, 100);
            uiDrawProgressBar(70, y - 8, 50, 8, dipProgress, 0, 100, NULL, false);
            y += 12;
        }
    }

    // Кнопки управления
    y = SCREEN_HEIGHT - 24;
    ui.display->drawFrame(0, y, 42, 20);
    ui.display->drawStr(10, y + 14, "СТОП");

    ui.display->drawFrame(43, y, 42, 20);
    ui.display->drawStr(isPaused ? 53 : 50, y + 14, isPaused ? "ПУСК" : "ПАУЗА");

    ui.display->drawFrame(86, y, 42, 20);
    ui.display->drawStr(96, y + 14, "МЕНЮ");
}

/**
 * @brief Отрисовка экрана калибровки
 */
void uiDrawCalibrationScreen(void)
{
    uiDrawHeader("КАЛИБРОВКА");

    extern SystemStatus systemStatus;
    // extern uint8_t currentZone;
    extern SystemData SystemData;

    uint8_t y = MENU_START_Y;

    // Инструкция
    ui.display->drawStr(0, y, "Используйте энкодер");
    y += 12;
    ui.display->drawStr(0, y, "для перемещения");
    y += 12;

    // Текущая позиция
    snprintf(textBuffer, sizeof(textBuffer), "Позиция: %ld мм",
             systemStatus.avgHorizontalPos);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Текущая зона
    snprintf(textBuffer, sizeof(textBuffer), "Зона калибровки: %d",
             SystemData.currentZone + 1);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Кнопки
    y = SCREEN_HEIGHT - 24;
    ui.display->drawFrame(0, y, 64, 20);
    ui.display->drawStr(15, y + 14, "СОХРАНИТЬ");

    ui.display->drawFrame(65, y, 63, 20);
    ui.display->drawStr(75, y + 14, "ОТМЕНА");

    // Индикатор точности
    int32_t diff = abs(systemStatus.telfer1Pos - systemStatus.telfer2Pos);
    if (diff > 50)
    {
        snprintf(textBuffer, sizeof(textBuffer), "ВНИМАНИЕ! Расх: %ldмм", diff);
        ui.display->drawStr(0, SCREEN_HEIGHT - 8, textBuffer);
    }
}

/**
 * @brief Отрисовка экрана заставки
 */
void uiDrawSplashScreen(void)
{
    static bool firstDraw = true;
    static uint32_t startTime = 0;

    if (firstDraw)
    {
        startTime = millis();
        firstDraw = false;
    }

    // Анимация появления
    uint32_t elapsed = millis() - startTime;
    // uint8_t alpha = min(255, elapsed / 4);

    // Черный фон
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawBox(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

    // Белый текст с анимацией
    ui.display->setDrawColor(COLOR_WHITE);

    // Название системы
    uiSetFontLarge();
    uiDrawTextCentered(15, "ТЕЛЬФЕРЫ");
    uiDrawTextCentered(35, "УПРАВЛЕНИЕ");

    // Версия
    uiSetFontSmall();
    uiDrawTextCentered(50, "Версия 4.0");

    // Автор
    uiDrawTextCentered(SCREEN_HEIGHT - 10, "© 2024");

    // Индикатор загрузки
    if (elapsed < 2000)
    { // 2 секунды анимации
        uint8_t progressWidth = map(elapsed, 0, 2000, 0, 100);
        uiDrawProgressBar(14, SCREEN_HEIGHT - 20, 100, 6, progressWidth, 0, 100, NULL, false);
    }
    else
    {
        // Переход на главный экран
        uiSetScreen(uiDrawMainMenuScreen, uiHandleMainMenuInput, 100);
    }
}

/**
 * @brief Отрисовка экрана мониторинга
 */
void uiDrawMonitorScreen(void)
{
    uiDrawHeader("МОНИТОРИНГ СИСТЕМЫ");

    extern SystemStatus systemStatus;
    extern SystemFlags systemFlags;
    extern PerformanceStats perfStats;

    uint8_t y = MENU_START_Y;

    // Время работы
    uint32_t uptime = systemStatus.uptime / 1000;
    uint8_t hours = uptime / 3600;
    uint8_t minutes = (uptime % 3600) / 60;
    uint8_t seconds = uptime % 60;

    snprintf(textBuffer, sizeof(textBuffer), "Время: %02d:%02d:%02d", hours, minutes, seconds);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Напряжение питания
    snprintf(textBuffer, sizeof(textBuffer), "Напряжение: %.1f В", FMT_FLOAT(systemStatus.batteryVoltage));
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Температура
    snprintf(textBuffer, sizeof(textBuffer), "Температура: %d °C", systemStatus.temperature);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Загрузка ЦП
    snprintf(textBuffer, sizeof(textBuffer), "Загрузка ЦП: %.1f%%", FMT_FLOAT(perfStats.cpuLoad));
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Статус датчиков
    snprintf(textBuffer, sizeof(textBuffer), "Датчики: %s", systemStatus.sensorsValid ? "OK" : "ОШИБКА");
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Статус двигателей
    snprintf(textBuffer, sizeof(textBuffer), "Двигатели: %s", systemFlags.motorsEnabled ? "ВКЛ" : "ВЫКЛ");
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Позиции тельферов
    snprintf(textBuffer, sizeof(textBuffer), "Т1: %ld мм", systemStatus.telfer1Pos);
    ui.display->drawStr(0, y, textBuffer);

    snprintf(textBuffer, sizeof(textBuffer), "Т2: %ld мм", systemStatus.telfer2Pos);
    ui.display->drawStr(64, y, textBuffer);
    y += 12;

    // Высоты груза
    snprintf(textBuffer, sizeof(textBuffer), "В1: %ld мм", systemStatus.cargoHeight1);
    ui.display->drawStr(0, y, textBuffer);

    snprintf(textBuffer, sizeof(textBuffer), "В2: %ld мм", systemStatus.cargoHeight2);
    ui.display->drawStr(64, y, textBuffer);

    // Индикатор наклона
    int32_t tiltDiff = abs(systemStatus.cargoHeight1 - systemStatus.cargoHeight2);
    if (tiltDiff > 10)
    {
        uint8_t tiltPercent = map(tiltDiff, 0, 500, 0, 100);
        uiDrawTiltIndicator(SCREEN_WIDTH - 30, MENU_START_Y, 25, 35,
                            systemStatus.cargoHeight1, systemStatus.cargoHeight2,
                            tiltPercent);
    }

    // Инструкция внизу
    uiDrawStatusBar(SCREEN_HEIGHT - 8,
                    "Энк: прокрутка",
                    "",
                    "Долго: выход");
}

/**
 * @brief Отрисовка экрана диагностики
 */
void uiDrawDiagnosticsScreen(void)
{
    uiDrawHeader("ДИАГНОСТИКА");

    extern DiagnosticsResult diagnosticsResults[MAX_DIAGNOSTIC_TESTS];
    extern uint32_t diagnosticsTime;

    uint8_t y = MENU_START_Y;

    // Результаты тестов
    ui.display->drawStr(0, y, "Результаты тестов:");
    y += 12;

    // Датчики
    snprintf(textBuffer, sizeof(textBuffer), "Датчики: %s",
             diagnosticsResults[0].passed ? "OK" : "ОШИБКА");
    ui.display->drawStr(10, y, textBuffer);

    // Цветной индикатор
    if (diagnosticsResults[0].passed)
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawBox(70, y - 8, 8, 8);
    }
    else
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawFrame(70, y - 8, 8, 8);
        ui.display->drawBox(72, y - 6, 4, 4);
    }
    y += 12;

    // Двигатели
    snprintf(textBuffer, sizeof(textBuffer), "Двигатели: %s",
             diagnosticsResults[1].passed ? "OK" : "ОШИБКА");
    ui.display->drawStr(10, y, textBuffer);

    if (diagnosticsResults[1].passed)
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawBox(70, y - 8, 8, 8);
    }
    else
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawFrame(70, y - 8, 8, 8);
        ui.display->drawBox(72, y - 6, 4, 4);
    }
    y += 12;

    // Хранилище
    snprintf(textBuffer, sizeof(textBuffer), "Память: %s",
             diagnosticsResults[2].passed ? "OK" : "ОШИБКА");
    ui.display->drawStr(10, y, textBuffer);

    if (diagnosticsResults[2].passed)
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawBox(70, y - 8, 8, 8);
    }
    else
    {
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawFrame(70, y - 8, 8, 8);
        ui.display->drawBox(72, y - 6, 4, 4);
    }
    y += 16;

    // Время выполнения диагностики
    uint32_t elapsed = (millis() - diagnosticsTime) / 1000;
    snprintf(textBuffer, sizeof(textBuffer), "Время: %lu с", elapsed);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Общий результат
    bool allTestsPassed = diagnosticsResults[0].passed &&
                          diagnosticsResults[1].passed &&
                          diagnosticsResults[2].passed;

    ui.display->setFont(u8g2_font_7x14B_tf);
    if (allTestsPassed)
    {
        ui.display->drawStr(0, y, "ВСЕ ТЕСТЫ ПРОЙДЕНЫ");
    }
    else
    {
        ui.display->drawStr(0, y, "ЕСТЬ ОШИБКИ!");
    }
    uiSetFontMedium();

    // Индикатор выполнения
    if (elapsed < 5) // Диагностика еще выполняется
    {
        uint8_t progress = (elapsed * 100) / 5;
        uiDrawProgressBar(0, SCREEN_HEIGHT - 20, SCREEN_WIDTH, 6,
                          progress, 0, 100, "Выполнение...", false);
    }

    // Инструкция внизу
    uiDrawStatusBar(SCREEN_HEIGHT - 8,
                    "",
                    "Нажмите для выхода",
                    "");
}

/**
 * @brief Отрисовка экрана настроек
 */
void uiDrawSettingsScreen(void)
{
    static MenuItem settingsMenuItems[] = {
        {"Контраст дисплея", 1, true, false, NULL, NULL, 50, 250, NULL, 0},
        {"Громкость звука", 2, true, false, NULL, NULL, 0, 100, NULL, 0},
        {"Автосохранение", 3, true, false, NULL, NULL, 0, 1, NULL, 0},
        {"Язык", 4, true, false, NULL, NULL, 0, 1, NULL, 0},
        {"Единицы измерения", 5, true, false, NULL, NULL, 0, 2, NULL, 0},
        {"Уровень логов", 6, true, false, NULL, NULL, 0, 4, NULL, 0},
        {"Сброс настроек", 7, true, false, NULL, NULL, 0, 0, NULL, 0},
        {"<- Назад", 8, true, false, NULL, NULL, 0, 0, NULL, 0}};

    static bool menuInitialized = false;

    if (!menuInitialized)
    {
        // Инициализируем глобальное меню
        ui.currentMenu.items = settingsMenuItems;
        ui.currentMenu.itemCount = sizeof(settingsMenuItems) / sizeof(settingsMenuItems[0]);
        ui.currentMenu.selectedIndex = 0;
        // Загрузка текущих настроек в меню
        extern UserSettings userSettings;

        // Контраст
        static int32_t contrastValue = userSettings.displayContrast;
        settingsMenuItems[0].value = &contrastValue;

        // Громкость
        static int32_t volumeValue = userSettings.soundVolume;
        settingsMenuItems[1].value = &volumeValue;

        // Автосохранение
        static int32_t autoSaveValue = userSettings.autoSave ? 1 : 0;
        settingsMenuItems[2].value = &autoSaveValue;

        // Язык
        static int32_t languageValue = userSettings.language;
        settingsMenuItems[3].value = &languageValue;

        // Единицы измерения
        static int32_t unitsValue = userSettings.units;
        settingsMenuItems[4].value = &unitsValue;

        // Уровень логов
        static int32_t logLevelValue = userSettings.logLevel;
        settingsMenuItems[5].value = &logLevelValue;

        menuInitialized = true;
    }

    // Отрисовка меню настроек
    uiDrawMenu(&ui.currentMenu);
}

/**
 * @brief Отрисовка экрана ошибки
 */
void uiDrawErrorScreen(void)
{
    uiDrawHeader("ОШИБКА СИСТЕМЫ");

    extern ErrorType activeError;
    extern char errorMessage[64];

    uint8_t y = MENU_START_Y;

    // Код ошибки
    snprintf(textBuffer, sizeof(textBuffer), "Код: %d", activeError);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Описание ошибки
    const char *errorDesc = getErrorMessage(activeError);
    ui.display->drawStr(0, y, errorDesc);
    y += 12;

    // Сообщение об ошибке
    uiDrawMultilineText(0, y, 10, errorMessage);

    // Время возникновения ошибки
    y = SCREEN_HEIGHT - 24;
    extern uint32_t errorTime;
    uint32_t elapsed = (millis() - errorTime) / 1000;
    snprintf(textBuffer, sizeof(textBuffer), "Прошло: %lu с", elapsed);
    ui.display->drawStr(0, y, textBuffer);

    // Кнопки
    y = SCREEN_HEIGHT - 10;
    ui.display->drawStr(0, y, "СБРОС");
    ui.display->drawStr(SCREEN_WIDTH - 30, y, "ИГНОР");

    // Индикатор мигания для кнопки сброса
    if (ui.blinkState)
    {
        ui.display->drawBox(0, SCREEN_HEIGHT - 12, 30, 10);
        ui.display->setDrawColor(COLOR_WHITE);
        ui.display->drawStr(5, SCREEN_HEIGHT - 3, "СБРОС");
        ui.display->setDrawColor(COLOR_BLACK);
    }
}

/**
 * @brief Отрисовка экрана редактирования программы
 */
void uiDrawProgramEditScreen(void)
{
    uiDrawHeader("РЕДАКТИРОВАНИЕ ПРОГРАММЫ");

    extern uint8_t currentProgram;
    extern ProgramSettings programs[MAX_PROGRAMS];

    if (currentProgram >= MAX_PROGRAMS)
    {
        ui.display->drawStr(0, MENU_START_Y, "Нет программы");
        return;
    }

    ProgramSettings *program = &programs[currentProgram];

    uint8_t y = MENU_START_Y;

    // Название программы
    snprintf(textBuffer, sizeof(textBuffer), "Название: %s", program->name);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Количество зон
    snprintf(textBuffer, sizeof(textBuffer), "Зон: %d", program->zoneCount);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Повторение
    snprintf(textBuffer, sizeof(textBuffer), "Повтор: %s",
             program->repeatEnabled ? "Да" : "Нет");
    ui.display->drawStr(0, y, textBuffer);

    if (program->repeatEnabled)
    {
        snprintf(textBuffer, sizeof(textBuffer), "%d раз", program->repeatCount);
        ui.display->drawStr(60, y, textBuffer);
    }
    y += 12;

    // Общее время выполнения
    uint32_t totalSeconds = program->totalRuntime / 1000;
    uint8_t hours = totalSeconds / 3600;
    uint8_t minutes = (totalSeconds % 3600) / 60;
    uint8_t seconds = totalSeconds % 60;

    snprintf(textBuffer, sizeof(textBuffer), "Время: %02d:%02d:%02d",
             hours, minutes, seconds);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Список зон (первые несколько)
    ui.display->drawStr(0, y, "Зоны:");
    y += 12;

    for (uint8_t i = 0; i < min(3, program->zoneCount); i++)
    {
        ZoneSettings *zone = &program->zones[i];
        snprintf(textBuffer, sizeof(textBuffer), "%d. %s: %ld мм",
                 i + 1, zone->name, zone->position);
        uiDrawTextTruncated(0, y, SCREEN_WIDTH - 10, textBuffer);
        y += 10;
    }

    if (program->zoneCount > 3)
    {
        snprintf(textBuffer, sizeof(textBuffer), "... и еще %d зон", program->zoneCount - 3);
        ui.display->drawStr(0, y, textBuffer);
        y += 10;
    }

    // Кнопки управления
    y = SCREEN_HEIGHT - 24;
    uint8_t buttonWidth = 40;
    uint8_t spacing = 5;

    // Кнопка "Изменить"
    ui.display->drawFrame(spacing, y, buttonWidth, 20);
    ui.display->drawStr(spacing + 5, y + 14, "ИЗМ");

    // Кнопка "Зоны"
    ui.display->drawFrame(spacing * 2 + buttonWidth, y, buttonWidth, 20);
    ui.display->drawStr(spacing * 2 + buttonWidth + 8, y + 14, "ЗОНЫ");

    // Кнопка "Назад"
    ui.display->drawFrame(spacing * 3 + buttonWidth * 2, y, buttonWidth, 20);
    ui.display->drawStr(spacing * 3 + buttonWidth * 2 + 5, y + 14, "НАЗ");
}

/**
 * @brief Отрисовка экрана редактирования зоны
 */
void uiDrawZoneEditScreen(void)
{
    uiDrawHeader("РЕДАКТИРОВАНИЕ ЗОНЫ");

    // extern uint8_t currentProgram;
    // extern uint8_t currentZone;
    extern SystemData SystemData;
    extern ProgramSettings programs[MAX_PROGRAMS];
    extern SystemStatus systemStatus;

    if (SystemData.currentProgram >= MAX_PROGRAMS || SystemData.currentZone >= MAX_ZONES_PER_PROGRAM)
    {
        ui.display->drawStr(0, MENU_START_Y, "Ошибка данных");
        return;
    }

    ZoneSettings *zone = &programs[SystemData.currentProgram].zones[SystemData.currentZone];

    uint8_t y = MENU_START_Y;

    // Название зоны
    snprintf(textBuffer, sizeof(textBuffer), "Название: %s", zone->name);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Позиция
    snprintf(textBuffer, sizeof(textBuffer), "Позиция: %ld мм", zone->position);
    ui.display->drawStr(0, y, textBuffer);

    // Индикатор текущей позиции
    int32_t posError = abs(systemStatus.avgHorizontalPos - zone->position);
    snprintf(textBuffer, sizeof(textBuffer), "Ост: %ld мм", posError);
    ui.display->drawStr(80, y, textBuffer);
    y += 12;

    // Высота
    snprintf(textBuffer, sizeof(textBuffer), "Высота: %ld мм", zone->targetHeight);
    ui.display->drawStr(0, y, textBuffer);

    // Текущая высота
    snprintf(textBuffer, sizeof(textBuffer), "Тек: %ld мм", systemStatus.avgHeight);
    ui.display->drawStr(80, y, textBuffer);
    y += 12;

    // Время погружения
    snprintf(textBuffer, sizeof(textBuffer), "Время погруж: %lu мс", zone->dipTime);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Угол наклона
    snprintf(textBuffer, sizeof(textBuffer), "Угол наклона: %d %%", zone->tiltAngle);
    ui.display->drawStr(0, y, textBuffer);
    y += 12;

    // Скорость движения
    snprintf(textBuffer, sizeof(textBuffer), "Скорость: %d %%", zone->motorSpeed);
    ui.display->drawStr(0, y, textBuffer);

    // Прогресс-бар для скорости
    uiDrawProgressBar(70, y - 8, 50, 8, zone->motorSpeed, 0, 100, NULL, false);

    // Кнопки
    y = SCREEN_HEIGHT - 24;
    uint8_t buttonWidth = 60;

    // Кнопка "Захват позиции"
    ui.display->drawFrame(5, y, buttonWidth, 20);
    ui.display->drawStr(10, y + 14, "ЗАХВАТ");

    // Кнопка "Сохранение"
    ui.display->drawFrame(70, y, buttonWidth, 20);
    ui.display->drawStr(75, y + 14, "СОХР");
}

// ========== ОБРАБОТЧИКИ ВВОДА ==========

/**
 * @brief Обработка ввода в главном меню
 */
void uiHandleMainMenuInput(void)
{
    // Здесь будет обработка энкодера и кнопок
    // Пока заглушка
}

/**
 * @brief Обработка ввода на экране мониторинга
 */
void uiHandleMonitorInput(void)
{
    // Обработка энкодера для прокрутки информации
    // (если информация не помещается на экране)

    // Длительное нажатие для выхода
    // Обрабатывается в основном цикле через handleEncoderInput
}

/**
 * @brief Обработка ввода на экране диагностики
 */
void uiHandleDiagnosticsInput(void)
{
    // При нажатии кнопки - выход из диагностики
    // Обрабатывается в основном цикле
}

/**
 * @brief Обработка ввода на экране настроек
 */
void uiHandleSettingsInput(void)
{
    // Обработка выбора пункта меню
    uint8_t selectedId = uiGetSelectedId();

    switch (selectedId)
    {
    case 7: // Сброс настроек
        // uiShowConfirmDialog("Сбросить все настройки?", NULL);
        break;

    case 8: // Назад
        uiNavigateBack();
        break;
    }
}

/**
 * @brief Обработка ввода на экране ошибки
 */
void uiHandleErrorInput(void)
{
    // Сброс ошибки при нажатии кнопки
    // Обрабатывается в основном цикле
}

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ==========

/**
 * @brief Звуковой сигнал
 * @param type Тип сигнала (1-короткий, 2-длинный, 3-ошибка)
 */
void uiBeep(uint8_t type)
{
    // Заглушка - будет реализовано в модуле utils
    // tone(BUZZER_PIN, frequency, duration);
}

/**
 * @brief Показать сообщение об ошибке
 * @param error Текст ошибки
 */
void uiShowError(const char *error)
{
    // Сохранение текущего экрана
    static void (*previousDrawFunction)() = NULL;
    static void (*previousInputHandler)() = NULL;
    static const char *lastError = nullptr; // статическая переменная для хранения строки ошибки

    lastError = error; // сохраняем текущую ошибку

    if (previousDrawFunction == NULL)
    {
        previousDrawFunction = ui.currentScreen.drawFunction;
        previousInputHandler = ui.currentScreen.inputHandler;
    }

    // Установка экрана ошибки
    uiSetScreen([]()
                { uiDrawMessageBox("ОШИБКА", lastError, 0); }, NULL, 100);

    // Через 3 секунды вернуться
    static uint32_t errorStartTime = 0;
    if (errorStartTime == 0)
    {
        errorStartTime = millis();
    }
    else if (millis() - errorStartTime > 3000)
    {
        uiSetScreen(previousDrawFunction, previousInputHandler, 100);
        previousDrawFunction = NULL;
        previousInputHandler = NULL;
        errorStartTime = 0;
        lastError = nullptr; // очищаем ошибку
    }
}

/**
 * @brief Отрисовка диалогового окна
 */
void uiDrawMessageBox(const char *title, const char *message, uint8_t type)
{
    // Размеры окна
    uint8_t boxWidth = SCREEN_WIDTH - 20;
    uint8_t boxHeight = 60;
    uint8_t boxX = 10;
    uint8_t boxY = (SCREEN_HEIGHT - boxHeight) / 2;

    // Фон
    ui.display->setDrawColor(COLOR_WHITE);
    ui.display->drawRBox(boxX, boxY, boxWidth, boxHeight, 3);

    // Рамка
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawRFrame(boxX, boxY, boxWidth, boxHeight, 3);

    // Заголовок
    ui.display->setDrawColor(COLOR_BLACK);
    uiSetFontMedium();
    uiDrawTextCentered(boxY + 12, title);

    // Разделитель
    ui.display->drawHLine(boxX + 5, boxY + 20, boxWidth - 10);

    // Сообщение
    uiSetFontSmall();
    uiDrawMultilineText(boxX + 10, boxY + 25, 10, message);

    // Иконка в зависимости от типа
    switch (type)
    {
    case 0: // Ошибка
        ui.display->drawCircle(boxX + 15, boxY + 40, 8);
        ui.display->drawStr(boxX + 13, boxY + 44, "!");
        break;
    case 1: // Предупреждение
        ui.display->drawTriangle(boxX + 15, boxY + 32,
                                 boxX + 7, boxY + 48,
                                 boxX + 23, boxY + 48);
        ui.display->drawStr(boxX + 13, boxY + 44, "!");
        break;
    case 2: // Успех
        ui.display->drawCircle(boxX + 15, boxY + 40, 8);
        ui.display->drawStr(boxX + 12, boxY + 44, "✓");
        break;
    }
}

/**
 * @brief Отрисовка многострочного текста
 */
void uiDrawMultilineText(uint8_t x, uint8_t y, uint8_t lineHeight, const char *text)
{
    char buffer[64];
    strncpy(buffer, text, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token = strtok(buffer, "\n");
    uint8_t currentY = y;

    while (token != NULL && currentY < SCREEN_HEIGHT - 10)
    {
        uiDrawTextTruncated(x, currentY, SCREEN_WIDTH - x - 10, token);
        currentY += lineHeight;
        token = strtok(NULL, "\n");
    }
}

/**
 * @brief Отрисовка статус-бара
 */
void uiDrawStatusBar(uint8_t y, const char *leftText, const char *centerText, const char *rightText)
{
    // Линия разделителя
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawHLine(0, y - 2, SCREEN_WIDTH);

    // Текст слева
    if (leftText)
    {
        ui.display->drawStr(2, y + 8, leftText);
    }

    // Текст по центру
    if (centerText)
    {
        uiDrawTextCentered(y + 8, centerText);
    }

    // Текст справа
    if (rightText)
    {
        uiDrawTextRight(SCREEN_WIDTH - 2, y + 8, rightText);
    }
}

/**
 * @brief Отрисовка высот груза
 */
void uiDrawCargoHeights(int32_t height1, int32_t height2, int32_t targetHeight, int32_t maxHeight)
{
    uint8_t y = MENU_START_Y + 50; // После позиций тельферов

    // Заголовок
    ui.display->drawStr(0, y, "Высоты груза:");
    y += 12;

    // Высота 1
    snprintf(textBuffer, sizeof(textBuffer), "В1: %ld мм", height1);
    ui.display->drawStr(0, y, textBuffer);

    // Прогресс высоты 1
    uiDrawProgressBar(40, y - 8, 80, 8, height1, 0, maxHeight, NULL, false);
    y += 12;

    // Высота 2
    snprintf(textBuffer, sizeof(textBuffer), "В2: %ld мм", height2);
    ui.display->drawStr(0, y, textBuffer);

    // Прогресс высоты 2
    uiDrawProgressBar(40, y - 8, 80, 8, height2, 0, maxHeight, NULL, false);
    y += 12;

    // Целевая высота
    if (targetHeight > 0)
    {
        snprintf(textBuffer, sizeof(textBuffer), "Цель: %ld мм", targetHeight);
        ui.display->drawStr(0, y, textBuffer);
        y += 12;
    }

    // Разница высот
    int32_t diff = abs(height1 - height2);
    if (diff > 10)
    {
        snprintf(textBuffer, sizeof(textBuffer), "Наклон: %ld мм", diff);
        ui.display->drawStr(0, y, textBuffer);
    }
}

/**
 * @brief Отрисовка индикатора наклона
 */
void uiDrawTiltIndicator(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                         int32_t height1, int32_t height2, uint8_t tiltPercent)
{
    // Фон индикатора
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawFrame(x, y, width, height);

    // Расчет угла наклона
    int32_t diff = height1 - height2;
    int8_t direction = (diff > 0) ? 1 : -1;
    uint8_t tiltLevel = min(100, tiltPercent) * (height - 4) / 100;

    // Отрисовка наклона
    if (direction > 0)
    {
        // Наклон влево (левый выше)
        ui.display->drawBox(x + 2, y + 2 + (height - 4 - tiltLevel),
                            width / 2 - 2, tiltLevel);
        ui.display->drawBox(x + width / 2, y + 2,
                            width / 2 - 2, height - 4);
    }
    else
    {
        // Наклон вправо (правый выше)
        ui.display->drawBox(x + 2, y + 2,
                            width / 2 - 2, height - 4);
        ui.display->drawBox(x + width / 2, y + 2 + (height - 4 - tiltLevel),
                            width / 2 - 2, tiltLevel);
    }

    // Центральная линия
    ui.display->setDrawColor(COLOR_WHITE);
    ui.display->drawVLine(x + width / 2, y + 2, height - 4);
    ui.display->setDrawColor(COLOR_BLACK);

    // Процент наклона
    snprintf(textBuffer, sizeof(textBuffer), "%d%%", tiltPercent);
    uint8_t textWidth = uiGetTextWidth(textBuffer);
    ui.display->drawStr(x + (width - textWidth) / 2, y + height + 8, textBuffer);
}

/**
 * @brief Отрисовка диалога подтверждения
 */
void uiDrawConfirmDialog(const char *question, bool *result)
{
    static bool dialogActive = false;
    static bool userChoice = false;
    static uint32_t dialogStartTime = 0;

    if (!dialogActive)
    {
        dialogActive = true;
        userChoice = false;
        dialogStartTime = millis();
    }

    // Размеры диалога
    uint8_t dialogWidth = SCREEN_WIDTH - 20;
    uint8_t dialogHeight = 60;
    uint8_t dialogX = 10;
    uint8_t dialogY = (SCREEN_HEIGHT - dialogHeight) / 2;

    // Фон
    ui.display->setDrawColor(COLOR_WHITE);
    ui.display->drawRBox(dialogX, dialogY, dialogWidth, dialogHeight, 3);

    // Рамка
    ui.display->setDrawColor(COLOR_BLACK);
    ui.display->drawRFrame(dialogX, dialogY, dialogWidth, dialogHeight, 3);

    // Вопрос
    uiSetFontMedium();
    uiDrawTextCentered(dialogY + 12, "ПОДТВЕРЖДЕНИЕ");

    // Разделитель
    ui.display->drawHLine(dialogX + 5, dialogY + 20, dialogWidth - 10);

    // Текст вопроса
    uiSetFontSmall();
    uiDrawMultilineText(dialogX + 10, dialogY + 25, 10, question);

    // Кнопки
    uint8_t buttonY = dialogY + dialogHeight - 20;

    // Кнопка "Да"
    ui.display->drawFrame(dialogX + 10, buttonY, 40, 16);
    ui.display->drawStr(dialogX + 20, buttonY + 12, "ДА");

    // Кнопка "Нет"
    ui.display->drawFrame(dialogX + dialogWidth - 50, buttonY, 40, 16);
    ui.display->drawStr(dialogX + dialogWidth - 40, buttonY + 12, "НЕТ");

    // Выбор кнопки (мигание)
    if (ui.blinkState)
    {
        if (userChoice)
        {
            ui.display->setDrawColor(COLOR_BLACK);
            ui.display->drawBox(dialogX + dialogWidth - 50, buttonY, 40, 16);
            ui.display->setDrawColor(COLOR_WHITE);
            ui.display->drawStr(dialogX + dialogWidth - 40, buttonY + 12, "НЕТ");
            ui.display->setDrawColor(COLOR_BLACK);
        }
        else
        {
            ui.display->setDrawColor(COLOR_BLACK);
            ui.display->drawBox(dialogX + 10, buttonY, 40, 16);
            ui.display->setDrawColor(COLOR_WHITE);
            ui.display->drawStr(dialogX + 20, buttonY + 12, "ДА");
            ui.display->setDrawColor(COLOR_BLACK);
        }
    }

    // Обработка таймаута (10 секунд)
    if (millis() - dialogStartTime > 10000)
    {
        dialogActive = false;
        if (result)
            *result = false; // Автоматический отказ по таймауту
    }

    // Возврат результата
    if (result && dialogActive)
    {
        // Здесь должна быть обработка ввода пользователя
        // *result = userChoice;
    }
}

/**
 * @brief Показать всплывающее сообщение
 */
void uiShowToast(const char *message, uint16_t duration)
{
    static char toastMessage[48];
    static uint32_t toastStartTime = 0;
    static bool toastActive = false;

    if (!toastActive && message)
    {
        strncpy(toastMessage, message, sizeof(toastMessage) - 1);
        toastMessage[sizeof(toastMessage) - 1] = '\0';
        toastStartTime = millis();
        toastActive = true;
    }

    if (toastActive)
    {
        // Проверка времени показа
        if (millis() - toastStartTime > duration)
        {
            toastActive = false;
            return;
        }

        // Отрисовка тоста
        uint8_t toastHeight = 20;
        uint8_t toastY = SCREEN_HEIGHT - toastHeight - 5;

        // Полупрозрачный фон
        ui.display->setDrawColor(COLOR_BLACK);
        ui.display->drawBox(0, toastY, SCREEN_WIDTH, toastHeight);

        // Текст
        ui.display->setDrawColor(COLOR_WHITE);
        uiDrawTextCentered(toastY + 13, toastMessage);
        ui.display->setDrawColor(COLOR_BLACK);
    }
}

// ========== ОТЛАДКА ==========

#ifdef DEBUG_UI
/**
 * @brief Вывод информации о меню
 */
void uiPrintMenuInfo(const Menu *menu)
{
    Serial.print(F("Меню: "));
    Serial.print(menu->title);
    Serial.print(F(", Элементов: "));
    Serial.print(menu->itemCount);
    Serial.print(F(", Выбран: "));
    Serial.print(menu->selectedIndex);
    Serial.print(F(", Смещение: "));
    Serial.print(menu->scrollOffset);
    Serial.print(F(", Уровень: "));
    Serial.println(menu->level);
}

/**
 * @brief Вывод информации об экране
 */
void uiPrintScreenInfo(const Screen *screen)
{
    Serial.print(F("Экран: Обновление каждые "));
    Serial.print(screen->updateInterval);
    Serial.print(F("мс, Последнее обновление: "));
    Serial.println(screen->lastUpdate);
}
#endif