#include "tickTimer.h"

#include "mbed.h"
#include "configuration.h"                  // family.h: TICK_TIM, TICK_IRQ, TICK_CLK_ENABLE(), TICK_TIMER_HZ(), TICK_TIMER_NAME

/***********************************************************************
    STM32 tick timer (every STM32 family with REMORA_HAS_TICK_TIMER): a
    timer with no other use, named by the family in family.h, separately
    for a hub and a CAN node (REMORA_CAN_NODE).

    STM32G0: a hub has TIM6 / TIM7 for its threads and uses TIM14 (a
    hardware PWM on PA4 / PA7 / PB1 / PC12 / PF0 would take it away:
    tickTimerIntact()); a CAN node has no threads and uses the basic timer
    TIM6, which has no pins (TIM2 is the mbed us ticker). STM32F4: the
    basic timer TIM7. Highest priority, with the SPI NSS edge and the step
    timer: steady bit timing, a tick is a few us.
************************************************************************/

#ifdef REMORA_HAS_TICK_TIMER

#define TICK_NAME           TICK_TIMER_NAME

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
    reload = TICK_TIMER_HZ() / hz - 1;
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

#endif
