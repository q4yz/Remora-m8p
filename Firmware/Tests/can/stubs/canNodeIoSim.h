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
    int configures = 0;
};

extern SimIo* simIo;
