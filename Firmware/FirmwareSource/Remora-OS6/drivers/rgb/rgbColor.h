#ifndef RGBCOLOR_H
#define RGBCOLOR_H

/***********************************************************************
    RGB LED colours (WS2812 / SK6812 "NeoPixel" chains), shared by the
    hub's own "RGB" module and the CAN nodes; no MCU dependencies.

    LinuxCNC sends one colour per segment in a set point (SP[i]): the
    float holds 0xRRGGBB as an integer (exact, a float has a 24-bit
    mantissa), e.g. M150 R255 G64 B0 writes 16728064.0. The chips want
    their bytes in a fixed order, "Color Order" like Klipper's
    color_order: R, G, B once each, W once for RGBW chips (SK6812 RGBW,
    Stealthburner "GRBW"); default GRB (WS2812).

    RGBW: the white LED is not sent over SPI / CAN, it is taken from the
    colour at the end: W = min(R, G, B), and R, G, B less that. White
    (R = G = B) lights the white LED only, a pale colour its white part.

    Order code (one byte, CAN module record): permutation of R, G, B
    0 ... 5 (RGB RBG GRB GBR BRG BGR) + 6 * W position (0: no W, 1 ... 4:
    W as byte 1 ... 4).
************************************************************************/

#include <stdint.h>
#include <string.h>

#define RGB_ORDERS          30
#define RGB_ORDER_GRB       2           // default: WS2812, SK6812
#define RGB_MAX_LEDS        64          // per chain
#define RGB_MAX_BYTES       4           // per LED (RGBW)

static const char* const rgbPermutations[6] = {"RGB", "RBG", "GRB", "GBR", "BRG", "BGR"};

static inline int rgbBytesPerLed(uint8_t code) { return code >= 6 ? 4 : 3; }

// the order as text, "GRBW"; buf at least 5 bytes
static inline const char* rgbOrderName(uint8_t code, char* buf)
{
    if (code >= RGB_ORDERS) code = RGB_ORDER_GRB;
    const char* p = rgbPermutations[code % 6];
    int wpos = code / 6;                // 0: none, 1 ... 4
    int n = 0;
    for (int i = 0; i < 4; i++)
    {
        if (wpos && i == wpos - 1) buf[n++] = 'W';
        else if (*p) buf[n++] = *p++;
    }
    buf[n] = 0;
    return buf;
}

// "GRB" -> 2, "GRBW" -> 26, nullptr / "" -> RGB_ORDER_GRB, unknown -> -1
static inline int rgbOrderCode(const char* name)
{
    if (!name || !*name) return RGB_ORDER_GRB;
    size_t len = strlen(name);
    if (len != 3 && len != 4) return -1;
    char rgb[4];
    int n = 0, wpos = 0;
    for (size_t i = 0; i < len; i++)
    {
        char c = name[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        if (c == 'W' && len == 4 && !wpos) wpos = (int)i + 1;
        else if ((c == 'R' || c == 'G' || c == 'B') && n < 3) rgb[n++] = c;
        else return -1;
    }
    if (n != 3 || (len == 4 && !wpos)) return -1;
    rgb[3] = 0;
    for (int i = 0; i < 6; i++)
    {
        if (!strcmp(rgb, rgbPermutations[i])) return i + 6 * wpos;
    }
    return -1;                          // a colour twice
}

// 0xRRGGBB as the chip's bytes (3, or 4 with W); returns the byte count
static inline int rgbBytes(uint32_t color, uint8_t code, uint8_t out[RGB_MAX_BYTES])
{
    if (code >= RGB_ORDERS) code = RGB_ORDER_GRB;
    uint8_t r = (uint8_t)(color >> 16), g = (uint8_t)(color >> 8), b = (uint8_t)color, w = 0;
    int wpos = code / 6;
    if (wpos)
    {
        w = r < g ? r : g;
        if (b < w) w = b;
        r = (uint8_t)(r - w);
        g = (uint8_t)(g - w);
        b = (uint8_t)(b - w);
    }
    const char* p = rgbPermutations[code % 6];
    int n = 0;
    for (int i = 0; i < (wpos ? 4 : 3); i++)
    {
        if (wpos && i == wpos - 1) { out[n++] = w; continue; }
        char c = *p++;
        out[n++] = c == 'R' ? r : c == 'G' ? g : b;
    }
    return n;
}

// the set point as 0xRRGGBB, limited to 0 ... 0xFFFFFF (NaN, negative: off)
static inline uint32_t rgbFromSetPoint(float sp)
{
    if (!(sp > 0.0f)) return 0;
    if (sp >= 16777215.0f) return 0xFFFFFF;
    return (uint32_t)(sp + 0.5f);
}

#endif
