#include "extern.h"


void createThreads(void)
{
    // Create the thread objects and set the interrupt vectors to RAM. This is needed
    // as we are using the SD bootloader that requires a different code starting
    // address. Also set interrupt priority with NVIC_SetPriority.

    // TIM2 is the mbed us_ticker on the G0 (wait_us, HAL_GetTick) and must not be used here.
    // The basic timers TIM6 and TIM7 have no pins, so they don't clash with hardware PWM either.
    // The Cortex-M0+ has 4 interrupt priority levels (0-3).

    baseThread = new pruThread(TIM6, TIM6_DAC_LPTIM1_IRQn, base_freq);
    NVIC_SetVector(TIM6_DAC_LPTIM1_IRQn, (uint32_t)TIM6_IRQHandler);
    NVIC_SetPriority(TIM6_DAC_LPTIM1_IRQn, 1);

    servoThread = new pruThread(TIM7, TIM7_LPTIM2_IRQn, servo_freq);
    NVIC_SetVector(TIM7_LPTIM2_IRQn, (uint32_t)TIM7_IRQHandler);
    NVIC_SetPriority(TIM7_LPTIM2_IRQn, 2);

    commsThread = new pruThread(TIM3, TIM3_TIM4_IRQn, PRU_COMMSFREQ);
    NVIC_SetVector(TIM3_TIM4_IRQn, (uint32_t)TIM3_IRQHandler);
    NVIC_SetPriority(TIM3_TIM4_IRQn, 3);
}