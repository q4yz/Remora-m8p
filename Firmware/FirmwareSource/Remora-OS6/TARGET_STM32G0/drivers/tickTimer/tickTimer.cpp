#include "tickTimer.h"

#include "mbed.h"
#include "stm32g0xx_hal.h"

/***********************************************************************
    STM32G0 tick timer. A hub has TIM6 / TIM7 for its threads and uses
    TIM14 (a hardware PWM on PA4 / PA7 / PB1 / PC12 / PF0 would take it
    away: tickTimerIntact()); a CAN node has no threads and uses the
    basic timer TIM6, which has no pins (TIM2 is the mbed us ticker).
    Highest priority, with the SPI NSS edge and the step timer: steady
    bit timing, a tick is a few us.
************************************************************************/

#ifdef REMORA_CAN_NODE
#define TICK_TIM            TIM6
#define TICK_IRQ            TIM6_DAC_LPTIM1_IRQn
#define TICK_CLK_ENABLE()   __HAL_RCC_TIM6_CLK_ENABLE()
#define TICK_NAME           "TIM6"
#else
#define TICK_TIM            TIM14
#define TICK_IRQ            TIM14_IRQn
#define TICK_CLK_ENABLE()   __HAL_RCC_TIM14_CLK_ENABLE()
#define TICK_NAME           "TIM14"
#endif

static void (*tickFn)(void) = nullptr;
static uint32_t reload = 0;
static bool running = false;


static void tickIrq()
{
    if (TICK_TIM->SR & TIM_SR_UIF)
    {
        TICK_TIM->SR = ~TIM_SR_UIF;
        tickFn();
    }
}


bool tickTimerStart(uint32_t hz, void (*tick)(void))
{
    TICK_CLK_ENABLE();
    if (TICK_TIM->CR1 & TIM_CR1_CEN)
    {
        printf("Tick timer: " TICK_NAME " is used by another module (hardware PWM?)\n");
        return false;
    }
    tickFn = tick;
    reload = SystemCoreClock / hz - 1;
    TICK_TIM->CR1 = 0;
    TICK_TIM->PSC = 0;
    TICK_TIM->ARR = reload;
    TICK_TIM->EGR = TIM_EGR_UG;
    TICK_TIM->SR = 0;
    TICK_TIM->DIER = TIM_DIER_UIE;
    NVIC_SetVector(TICK_IRQ, (uint32_t)tickIrq);
    NVIC_SetPriority(TICK_IRQ, 0);
    NVIC_EnableIRQ(TICK_IRQ);
    running = false;
    return true;
}


void tickTimerRun(bool run)
{
    if (run == running) return;
    running = run;
    if (run)
    {
        TICK_TIM->CNT = 0;
        TICK_TIM->CR1 = TIM_CR1_CEN;
    }
    else
    {
        TICK_TIM->CR1 = 0;
    }
}


bool tickTimerIntact(void)
{
    return tickFn != nullptr && TICK_TIM->ARR == reload;
}
