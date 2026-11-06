#ifndef RGBCOLOR_H
#define RGBCOLOR_H

/***********************************************************************
    RGB LED colours (WS2812 / SK6812 "NeoPixel" chains), shared by the
    hub's own "RGB" module and the CAN nodes; no MCU dependencies.

    LinuxCNC sends one colour per segment in a set point (SP[i]): the
    float holds 0xRRGGBB as an integer (exact, a float has a 24-bit
    mantissa), e.g. M150 R255 G64 B0 writes 16728064.0. The chips want
    their bytes in a fixed order, "Color Order" (default GRB, WS2812).
************************************************************************/

#include <stdint.h>
#include <string.h>

#define RGB_ORDERS          6
#define RGB_ORDER_GRB       2           // default: WS2812, SK6812
#define RGB_MAX_LEDS        64          // per chain

static const char* const rgbOrderNames[RGB_ORDERS] = {"RGB", "RBG", "GRB", "GBR", "BRG", "BGR"};

// "GRB" -> 2, nullptr / "" -> RGB_ORDER_GRB, unknown -> -1
static inline int rgbOrderCode(const char* name)
{
    if (!name || !*name) return RGB_ORDER_GRB;
    for (int i = 0; i < RGB_ORDERS; i++)
    {
        if (!strcmp(name, rgbOrderNames[i])) return i;
    }
    return -1;
}

// 0xRRGGBB in the chip's byte order
static inline void rgbBytes(uint32_t color, uint8_t order, uint8_t out[3])
{
    const char* names = rgbOrderNames[order < RGB_ORDERS ? order : RGB_ORDER_GRB];
    for (int i = 0; i < 3; i++)
    {
        int shift = names[i] == 'R' ? 16 : names[i] == 'G' ? 8 : 0;
        out[i] = (uint8_t)(color >> shift);
    }
}

// the set point as 0xRRGGBB, limited to 0 ... 0xFFFFFF (NaN, negative: off)
static inline uint32_t rgbFromSetPoint(float sp)
{
    if (!(sp > 0.0f)) return 0;
    if (sp >= 16777215.0f) return 0xFFFFFF;
    return (uint32_t)(sp + 0.5f);
}

#endif
