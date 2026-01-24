/**
 * @file fonts.h
 * @brief Шрифты с поддержкой кириллицы для U8g2
 * @version 4.0
 */

#ifndef FONTS_H
#define FONTS_H

#include <U8g2lib.h>

// ========== ШРИФТЫ С ПОДДЕРЖКОЙ КИРИЛЛИЦЫ ==========

// Мелкий шрифт (6x13 пикселей, кириллица)
// #define FONT_SMALL u8g2_font_6x12_t_cyrillic
#define FONT_SMALL u8g2_font_5x7_t_cyrillic

// Средний шрифт (7x14 пикселей, кириллица)
// #define FONT_MEDIUM u8g2_font_7x13_t_cyrillic
#define FONT_MEDIUM u8g2_font_6x12_t_cyrillic

// Крупный шрифт (10x20 пикселей, кириллица)
// #define FONT_LARGE u8g2_font_10x20_t_cyrillic
#define FONT_LARGE u8g2_font_7x13_t_cyrillic

// Жирный шрифт (7x14 пикселей, жирный, кириллица)
#define FONT_BOLD u8g2_font_6x13B_t_cyrillic

// Очень мелкий шрифт (5x7 пикселей, базовый латинский)
#define FONT_TINY u8g2_font_5x7_t_cyrillic

// ========== ФУНКЦИИ ДЛЯ РАБОТЫ СО ШРИФТАМИ ==========

/**
 * @brief Установка мелкого шрифта
 * @param display Указатель на объект дисплея
 */
inline void setFontSmall(U8G2 *display)
{
    display->setFont(FONT_SMALL);
}

/**
 * @brief Установка среднего шрифта
 * @param display Указатель на объект дисплея
 */
inline void setFontMedium(U8G2 *display)
{
    display->setFont(FONT_MEDIUM);
}

/**
 * @brief Установка крупного шрифта
 * @param display Указатель на объект дисплея
 */
inline void setFontLarge(U8G2 *display)
{
    display->setFont(FONT_LARGE);
}

/**
 * @brief Установка жирного шрифта
 * @param display Указатель на объект дисплея
 */
inline void setFontBold(U8G2 *display)
{
    display->setFont(FONT_BOLD);
}

/**
 * @brief Установка очень мелкого шрифта
 * @param display Указатель на объект дисплея
 */
inline void setFontTiny(U8G2 *display)
{
    display->setFont(FONT_TINY);
}

/**
 * @brief Получение высоты текущего шрифта
 * @param display Указатель на объект дисплея
 * @return Высота шрифта в пикселях
 */
inline uint8_t getFontHeight(U8G2 *display)
{
    return display->getMaxCharHeight();
}

/**
 * @brief Получение ширины строки
 * @param display Указатель на объект дисплея
 * @param str Строка
 * @return Ширина строки в пикселях
 */
inline uint8_t getStringWidth(U8G2 *display, const char *str)
{
    return display->getStrWidth(str);
}

/**
 * @brief Обрезка строки для помещения в заданную ширину
 * @param str Исходная строка
 * @param maxWidth Максимальная ширина в пикселях
 * @param font Шрифт (указатель на функцию установки шрифта)
 * @return Обрезанная строка (статический буфер)
 */
const char *truncateStringToWidth(const char *str, uint8_t maxWidth,
                                  void (*font)(U8G2 *), U8G2 *display)
{
    static char buffer[64];
    strncpy(buffer, str, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    font(display);

    uint8_t width = getStringWidth(display, buffer);
    uint8_t len = strlen(buffer);

    while (width > maxWidth && len > 0)
    {
        buffer[len - 1] = '\0';
        len--;
        width = getStringWidth(display, buffer);
    }

    // Если все равно не помещается, добавляем "..."
    if (width > maxWidth || (strlen(str) > len && len > 3))
    {
        if (len > 3)
        {
            buffer[len - 3] = '.';
            buffer[len - 2] = '.';
            buffer[len - 1] = '.';
            buffer[len] = '\0';
        }
    }

    return buffer;
}

#endif // FONTS_H