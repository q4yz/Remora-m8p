#ifndef WS2812_H
#define WS2812_H

/***********************************************************************
    WS2812 / SK6812 LED chain on an STM32F4 timer channel, fed by DMA

    The bits are PWM periods of 1.25 us (800 kHz): a short high pulse is a
    0, a long one a 1. The DMA writes the next compare value at every
    timer update, the CPU only fills the buffer, so the transfer needs no
    interrupts at all and none are held: bit banging is not an option
    next to the Remora base thread, the chips latch after about 6 us low.

    The pin must be a timer channel output (also a complementary CHxN) of
    a timer with a DMA update request: TIM1, TIM2, TIM3, TIM4 or TIM8 (one
    chain per timer, each its own DMA stream); the timer must not be used
    by a hardware PWM module. TIM5 is mbed's us ticker, TIM9 ... TIM12 the
    threads and the step timer, they have no update DMA anyway.
    BTT Octopus / Octopus Pro: the RGB header is PB0 (TIM3_CH3, TIM1_CH2N,
    TIM8_CH2N).
************************************************************************/

#include <stdint.h>

#include "mbed.h"
#include "stm32f4xx_hal.h"

#include "ledChain.h"

// the STM32F4 LedChain (ledChainCreate / ledChainCapable are create / capable)
class Ws2812 : public LedChain
{
    private:

        TIM_TypeDef*            tim;
        DMA_TypeDef*            dmaController;
        DMA_Stream_TypeDef*     stream;
        int                     streamIndex;    // 0 ... 7
        volatile uint32_t*      ccr;
        PinName                 pin;
        int                     leds;
        int                     bytesPerLed;    // 3 RGB, 4 RGBW
        uint32_t*               buffer;         // one compare value per bit (32 bit: the stream's direct
                                                // mode needs memory and timer of the same size), 2 zeros at the end
        int                     length;
        bool                    running;
        uint32_t                doneTick;
        uint32_t                bit0, bit1;
        uint32_t                period;
        bool                    conflictReported;

        Ws2812(void) {}
        void clearFlags(void);

    public:

        // nullptr (and why on the console) when the pin has no free timer channel
        static Ws2812* create(PinName pin, int leds, int bytesPerLed, const char* who);
        static bool capable(PinName pin);       // the pin is a channel of a supported timer

        // start sending leds * bytesPerLed bytes (in the chip's byte order); false while the last
        // transfer is still running or its reset time (> 280 us low) has not passed: try again
        bool send(const uint8_t* bytes) override;
        bool busy(void) override;
        int getLeds(void) const { return this->leds; }
        PinName getPin(void) const { return this->pin; }

        void release(void) override;            // pin low, timer and DMA free again; delete afterwards
};

#endif
