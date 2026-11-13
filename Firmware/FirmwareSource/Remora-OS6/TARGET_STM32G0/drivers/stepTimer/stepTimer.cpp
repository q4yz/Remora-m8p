#include "stepTimer.h"
#include "StepScheduler.h"

#include "mbed.h"
#include "stm32g0xx_hal.h"

/***********************************************************************
    STM32G0 step timer: TIM3 (16 bit, SystemCoreClock = 64 MHz, 15.6 ns),
    its period set at every interrupt to the time to the next edge (ARPE
    off: the new ARR counts from the update that started this
    interrupt). TIM3 is free on the G0: the comms thread object that owns
    it is never started there (the hub TMC2209s run on TIM14). Highest
    priority, with the SPI NSS edge and the TMC UART: an interrupt is a
    few us.
************************************************************************/

#define TIMER_MARGIN        64              // ticks: the next update at least 1 us after this interrupt

static StepScheduler* sched = nullptr;
static volatile uint32_t programmed = 0;


static void timerIrq()
{
    if (!(TIM3->SR & TIM_SR_UIF)) return;
    TIM3->SR = ~TIM_SR_UIF;
    uint32_t d = sched->onTimer(programmed);
    // the period counts from the update: it must end after now, or the counter runs to 65535
    uint32_t cnt = TIM3->CNT;
    if (d < cnt + TIMER_MARGIN) d = cnt + TIMER_MARGIN;
    TIM3->ARR = d - 1;
    programmed = d;
}


bool stepTimerAvailable(void)
{
    return true;
}


uint32_t stepTimerHz(void)
{
    return SystemCoreClock;
}


bool stepTimerStart(StepScheduler* scheduler)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    if (TIM3->CR1 & TIM_CR1_CEN)
    {
        printf("Step Generator Interrupt: TIM3 is used by another module (RGB on PC6, hardware PWM)\n");
        return false;
    }
    sched = scheduler;

    TIM3->CR1 = 0;                          // ARPE off, up counting
    TIM3->PSC = 0;
    programmed = sched->getIdleTicks();
    TIM3->ARR = programmed - 1;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->DIER = TIM_DIER_UIE;
    NVIC_SetVector(TIM3_TIM4_IRQn, (uint32_t)timerIrq);
    NVIC_SetPriority(TIM3_TIM4_IRQn, 0);
    NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;
    printf("Step Generator Interrupt: TIM3 at %lu Hz\n", (unsigned long)SystemCoreClock);
    return true;
}
