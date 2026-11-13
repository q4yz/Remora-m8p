#ifndef FAMILY_H
#define FAMILY_H

/***********************************************************************
    STM32G0 (Cortex-M0+, 64 MHz): the reference implementation

    What this MCU family implements of Remora's hardware interfaces
    (PORTING.md). Shared code asks for a feature with REMORA_HAS_...,
    never for a board or a family.
************************************************************************/

#include "stm32g0xx_hal.h"

#define FAMILY_NAME             "STM32G0"

// pin names are "PA_0" ... "PF_15": config.txt is checked against them
#define CONFIG_PIN_LAST_PORT    'F'

// milliseconds since boot, wraps
static inline uint32_t remoraMillis(void) { return HAL_GetTick(); }

// default without a "Threads" entry in config.txt: the 64 MHz Cortex-M0+ runs its base thread
// modules at 20 kHz, at 40 kHz the main loop starves (no SPI replies, watchdog reset)
#define PRU_BASEFREQ            20000

// the optional interfaces this family implements, each in TARGET_STM32G0/drivers/
#define REMORA_HAS_STEP_TIMER       // stepTimer.h: interrupt step generation (TIM3)
#define REMORA_HAS_TICK_TIMER       // tickTimer.h: the tick of the bit-banged TMC2209 UART / a node's base thread
#define REMORA_HAS_NODE_IO          // canNodeIo.h: GPIO / ADC by pin byte (CAN nodes, the TMC2209 UART)
#define REMORA_HAS_CAN              // CanBus.h: canBusCreate (FDCAN)
#define REMORA_HAS_LED_CHAIN        // ledChain.h: WS2812 chains (timer + DMA)
#define REMORA_HAS_MCU_TEMP         // mcuTempSensor.h: the internal temperature sensor
#define REMORA_HAS_USB_CONSOLE      // nodeConsole.h: printf on the USB port of a node
#define REMORA_HAS_THREAD_STATS     // pruThread: run times and overruns, kept over a reset (debug builds)
#define REMORA_HAS_BOOT_INFO        // bootInfo.h: reset cause and the thread statistics before it (debug builds)

#endif
