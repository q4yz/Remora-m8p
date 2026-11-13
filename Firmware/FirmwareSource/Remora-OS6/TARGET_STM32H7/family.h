#ifndef FAMILY_H
#define FAMILY_H

/***********************************************************************
    STM32H7

    What this MCU family implements of Remora's hardware interfaces
    (PORTING.md). Shared code asks for a feature with REMORA_HAS_...,
    never for a board or a family.
************************************************************************/

#include "stm32h7xx_hal.h"

#define FAMILY_NAME             "STM32H7"

// pin names are "PA_0" ... "PH_15": config.txt is checked against them
#define CONFIG_PIN_LAST_PORT    'H'

// milliseconds since boot, wraps
static inline uint32_t remoraMillis(void) { return HAL_GetTick(); }

#endif
