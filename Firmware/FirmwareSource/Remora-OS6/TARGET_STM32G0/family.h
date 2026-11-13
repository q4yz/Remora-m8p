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

// the timers of the shared STM32 drivers (TARGET_STM/drivers): step timer TIM3; tick timer TIM14 on a
// hub (TIM6 / TIM7 are the threads), the basic timer TIM6 on a CAN node (no threads, no pins)
#define STEP_TIMER              TIM3
#define STEP_TIMER_NAME         "TIM3"
#define STEP_TIMER_IRQ          TIM3_TIM4_IRQn
#define STEP_TIMER_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
#define STEP_TIMER_HZ()         SystemCoreClock
#ifdef REMORA_CAN_NODE
#define TICK_TIM                TIM6
#define TICK_TIMER_NAME         "TIM6"
#define TICK_IRQ                TIM6_DAC_LPTIM1_IRQn
#define TICK_CLK_ENABLE()       __HAL_RCC_TIM6_CLK_ENABLE()
#else
#define TICK_TIM                TIM14
#define TICK_TIMER_NAME         "TIM14"
#define TICK_IRQ                TIM14_IRQn
#define TICK_CLK_ENABLE()       __HAL_RCC_TIM14_CLK_ENABLE()
#endif
#define TICK_TIMER_HZ()         SystemCoreClock

// the optional interfaces this family implements, in TARGET_STM32G0/drivers/ or the shared TARGET_STM/drivers/
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
