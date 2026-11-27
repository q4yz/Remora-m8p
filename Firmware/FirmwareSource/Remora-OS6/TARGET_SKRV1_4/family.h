#ifndef FAMILY_H
#define FAMILY_H

/***********************************************************************
    LPC176x (BTT SKR V1.4: the board folder is the family folder too)

    What this MCU family implements of Remora's hardware interfaces
    (PORTING.md). Shared code asks for a feature with REMORA_HAS_...,
    never for a board or a family.
************************************************************************/

#include "mbed.h"

#define FAMILY_NAME             "LPC176x"

// pin names are "P0_0" ... "P4_31": not checked against a pattern (no CONFIG_PIN_LAST_PORT)

// milliseconds since boot, wraps
static inline uint32_t remoraMillis(void) { return (uint32_t)(us_ticker_read() / 1000); }

#endif
