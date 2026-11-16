#pragma once
#include "canNodeIo.h"

/* pin state of one simulated node, indexed by the pin byte (CAN_PIN) */
struct SimIo
{
    bool configured[96] = {};
    bool output[96] = {};
    bool level[96] = {};          /* output: driven level, input: level applied from outside */
    uint8_t modifier[96] = {};
    int rises[96] = {};           /* low -> high edges written, e.g. step pulses */
    bool latch[96] = {};          /* output register: the level written, kept while the pin is an input */
    uint16_t analog[96] = {};     /* ADC reading applied from outside, 0 ... 65535 */
    bool analogOn[96] = {};
    int highTicks[96] = {};       /* base ticks the pin was high, e.g. PWM duty */
    int configures = 0;
    int rgbLeds[96] = {};         /* LED chain configured on the pin, 0: none */
    int rgbSends[96] = {};        /* transfers to the chain */
    uint8_t rgb[96][64 * 4] = {}; /* the bytes of the last transfer */
    int rgbBpl[96] = {};          /* bytes per LED, 3 or 4 */

    /* an ADXL345 on SPI (canIoSpi*): registers, FIFO in samples, the acceleration it measures */
    bool spiOn = false;
    uint8_t spiPins[3] = {};
    bool adxlPresent = true;
    uint8_t adxlReg[64] = {};
    int adxlFifo = 0;             /* samples waiting, the sensor holds 32 */
    int adxlPhase = 0;            /* sample rate accumulator */
    int16_t adxlXyz[3] = {};
    int adxlReads = 0;            /* samples read */
    void adxlMs() {               /* one ms of the sensor sampling at its BW_RATE in stream mode */
        if (!adxlPresent || !(adxlReg[0x2D] & 0x08) || (adxlReg[0x38] & 0xC0) != 0x80) return;
        int hz = 3200 >> (0x0F - (adxlReg[0x2C] & 0x0F));
        adxlPhase += hz;
        while (adxlPhase >= 1000) { adxlPhase -= 1000; if (adxlFifo < 32) adxlFifo++; }
    }
};

extern SimIo* simIo;
