#pragma once
#include "canNodeIo.h"

/* pin state of one simulated node, indexed by the pin byte (CAN_PIN) */
struct SimIo
{
    bool configured[96] = {};
    bool output[96] = {};
    bool level[96] = {};          /* output: driven level, input: level applied from outside */
    uint8_t modifier[96] = {};
    int configures = 0;
};

extern SimIo* simIo;
