#ifndef FAMILY_H
#define FAMILY_H

/***********************************************************************
    STM32F4 (Cortex-M4F, 168 / 180 MHz): BTT Octopus / Octopus Pro F446 and
    F429, SKR 2, Monster8, Robin Nano 3, Spider, Spider King

    What this MCU family implements of Remora's hardware interfaces
    (PORTING.md). Shared code asks for a feature with REMORA_HAS_...,
    never for a board or a family.
************************************************************************/

#include "stm32f4xx_hal.h"

#define FAMILY_NAME             "STM32F4"

// pin names are "PA_0" ... "PH_15": config.txt is checked against them
#define CONFIG_PIN_LAST_PORT    'H'

// milliseconds since boot, wraps
static inline uint32_t remoraMillis(void) { return HAL_GetTick(); }

// the clock of the APB1 timers: PCLK1, doubled when APB1 is divided (180 MHz F446: 90 MHz)
static inline uint32_t apb1TimerHz(void)
{
    return HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1_2) ? 2u : 1u);
}

// the timers of the shared STM32 drivers (TARGET_STM/drivers). The threads use TIM9 / TIM10 / TIM11,
// mbed's us ticker TIM5. Step timer TIM12: its channels are only on alternate pins (PB14 / PB15 ALT1),
// no heater or fan output; tick timer the basic timer TIM7 (no pins)
#define STEP_TIMER              TIM12
#define STEP_TIMER_NAME         "TIM12"
#define STEP_TIMER_IRQ          TIM8_BRK_TIM12_IRQn
#define STEP_TIMER_CLK_ENABLE() __HAL_RCC_TIM12_CLK_ENABLE()
#define STEP_TIMER_HZ()         apb1TimerHz()
#define TICK_TIM                TIM7
#define TICK_TIMER_NAME         "TIM7"
#define TICK_IRQ                TIM7_IRQn
#define TICK_CLK_ENABLE()       __HAL_RCC_TIM7_CLK_ENABLE()
#define TICK_TIMER_HZ()         apb1TimerHz()

// the optional interfaces this family implements, in TARGET_STM32F4/drivers/ or the shared TARGET_STM/drivers/
#define REMORA_HAS_STEP_TIMER       // stepTimer.h: interrupt step generation (TIM12)
#define REMORA_HAS_TICK_TIMER       // tickTimer.h: the tick of the bit-banged TMC2209 UART (TIM7)
#define REMORA_HAS_NODE_IO          // canNodeIo.h: GPIO / ADC by pin byte (the TMC2209 UART)
#define REMORA_HAS_CAN              // CanBus.h: canBusCreate (bxCAN, polled)
#define REMORA_HAS_LED_CHAIN        // ledChain.h: WS2812 chains (timer + DMA stream)
#define REMORA_HAS_MCU_TEMP         // mcuTempSensor.h: the internal temperature sensor
#define REMORA_HAS_THREAD_STATS     // pruThread: run times and overruns (debug builds)
#define REMORA_HAS_BOOT_INFO        // bootInfo.h: reset cause (debug builds)

#endif
