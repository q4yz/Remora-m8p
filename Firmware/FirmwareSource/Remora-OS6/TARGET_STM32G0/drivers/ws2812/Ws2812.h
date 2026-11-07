#ifndef WS2812_H
#define WS2812_H

/***********************************************************************
    WS2812 / SK6812 LED chain on an STM32G0 timer channel, fed by DMA

    The bits are PWM periods of 1.25 us (800 kHz): a short high pulse is a
    0, a long one a 1. The DMA writes the next compare value at every
    timer update, the CPU only fills the buffer, so the transfer needs no
    interrupts at all and none are held: bit banging is not an option
    next to the Remora base thread, the chips latch after about 6 us low.

    The pin must be a timer channel output (also a complementary CHxN) of
    TIM1, TIM3, TIM4, TIM15, TIM16 or TIM17, one chain per timer; the
    timer must not be used by a hardware PWM module. A DMA2 channel per
    chain (5 on the G0B1).
************************************************************************/

#include <stdint.h>

#include "mbed.h"
#include "stm32g0xx_hal.h"

class Ws2812
{
    private:

        TIM_TypeDef*            tim;
        DMA_Channel_TypeDef*    dma;
        int                     dmaIndex;       // DMA2 channel 1 ... 5
        volatile uint32_t*      ccr;
        PinName                 pin;
        int                     leds;
        int                     bytesPerLed;    // 3 RGB, 4 RGBW
        uint16_t*               buffer;         // one compare value per bit, 2 zeros at the end
        int                     length;
        bool                    running;
        uint32_t                doneTick;
        uint16_t                bit0, bit1;
        uint16_t                period;
        bool                    conflictReported;

        Ws2812(void) {}

    public:

        // nullptr (and why on the console) when the pin has no free timer channel or no DMA is left
        static Ws2812* create(PinName pin, int leds, int bytesPerLed, const char* who);
        static bool capable(PinName pin);       // the pin is a channel of a supported timer

        // start sending leds * bytesPerLed bytes (in the chip's byte order); false while the last
        // transfer is still running or its reset time (> 280 us low) has not passed: try again
        bool send(const uint8_t* bytes);
        bool busy(void);
        int getLeds(void) const { return this->leds; }
        PinName getPin(void) const { return this->pin; }

        void release(void);                     // pin low, timer and DMA free again; delete afterwards
};

#endif
