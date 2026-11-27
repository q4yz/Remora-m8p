#ifndef LEDCHAIN_H
#define LEDCHAIN_H

/***********************************************************************
    A WS2812 / SK6812 LED chain on a pin, as the "RGB" module
    (modules/rgb) and the CAN nodes use it

    The chips latch after about 6 us low, so the bits cannot be bit
    banged next to the base thread: a family with REMORA_HAS_LED_CHAIN
    sends them with a peripheral that needs no interrupts held (STM32G0:
    TARGET_STM32G0/drivers/ws2812, a timer channel fed by DMA).
************************************************************************/

#include <stdint.h>

#include "PinNames.h"

class LedChain
{
    public:

        // start sending leds * bytesPerLed bytes (in the chip's byte order); false while the last
        // transfer is still running or its reset time (> 280 us low) has not passed: try again
        virtual bool send(const uint8_t* bytes) = 0;
        virtual bool busy(void) = 0;
        virtual void release(void) = 0;     // pin low, the peripheral free again; delete afterwards
        virtual ~LedChain() {}
};

// nullptr (and why on the console, with who asked) when the pin cannot drive a chain or no
// peripheral is left; bytesPerLed: 3 RGB, 4 RGBW
LedChain* ledChainCreate(PinName pin, int leds, int bytesPerLed, const char* who);
bool ledChainCapable(PinName pin);          // the pin can drive a chain

#endif
