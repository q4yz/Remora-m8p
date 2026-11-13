#ifndef FAMILY_H
#define FAMILY_H

/***********************************************************************
    STM32F1

    What this MCU family implements of Remora's hardware interfaces
    (PORTING.md). Shared code asks for a feature with REMORA_HAS_...,
    never for a board or a family.
************************************************************************/

#include "stm32f1xx_hal.h"

#define FAMILY_NAME             "STM32F1"

// pin names are "PA_0" ... "PE_15": config.txt is checked against them
#define CONFIG_PIN_LAST_PORT    'E'

// milliseconds since boot, wraps
static inline uint32_t remoraMillis(void) { return HAL_GetTick(); }

#endif
