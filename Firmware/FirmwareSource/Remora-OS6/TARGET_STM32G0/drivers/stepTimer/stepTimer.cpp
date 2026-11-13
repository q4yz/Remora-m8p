#include "stepTimer.h"
#include "StepScheduler.h"

#include "mbed.h"
#include "stm32g0xx_hal.h"

/***********************************************************************
    STM32G0 step timer: TIM3 (16 bit, SystemCoreClock = 64 MHz, 15.6 ns),
    free running (ARR 0xFFFF), the next event by output compare (CCR1,
    frozen mode, no pin). TIM3 is free on the G0: the comms thread
    object that owns it is never started there (the hub TMC2209s run on
    TIM14). Highest priority, with the SPI NSS edge and the TMC UART: an
    interrupt is a few us.

    The elapsed time handed to the scheduler is the difference of two
    compare values, i.e. of two absolute counter readings, so it is
    exact however late the interrupt is entered. (Setting the period
    with ARR, as before, assumed that the last period was exactly the
    one programmed: an interrupt that came later than the next period,
    e.g. 3 us after a step for the end of its pulse, saw a second update
    and ran twice, and the scheduler's clock gained a period each time.
    The steps then came early: ~7 % too many at 1 kHz on the M8P with
    three Z motors, a following error that grows with the speed.)
************************************************************************/

#define TIMER_MARGIN        64              // ticks: the next compare at least 1 us after this interrupt
#define TIMER_MAX_STEP      0x7FFF          // ticks: compare targets ahead by less than half the counter

static StepScheduler* sched = nullptr;
static volatile uint16_t target = 0;        // the compare value of this event (the scheduler's "now")
static uint16_t lastTarget = 0;
static volatile uint16_t busyMax = 0;       // ticks, the longest interrupt (from its compare time to its end)


static void timerIrq()
{
    if (!(TIM3->SR & TIM_SR_CC1IF)) return;
    TIM3->SR = ~TIM_SR_CC1IF;

    uint16_t now = target;
    uint32_t d = sched->onTimer((uint16_t)(now - lastTarget));
    lastTarget = now;
    if (d > TIMER_MAX_STEP) d = TIMER_MAX_STEP;

    // the next event, in the future: a late interrupt moves it, the time base stays exact
    uint16_t next = (uint16_t)(now + d);
    uint16_t cnt = (uint16_t)TIM3->CNT;
    if ((int16_t)(next - cnt) < TIMER_MARGIN) next = (uint16_t)(cnt + TIMER_MARGIN);
    TIM3->CCR1 = next;
    target = next;

    uint16_t busy = (uint16_t)(cnt - now);
    if (busy > busyMax) busyMax = busy;
}


// busy wait in the interrupt (the width of a step pulse), ticks < 32768
void stepTimerWait(uint32_t ticks)
{
    uint16_t start = (uint16_t)TIM3->CNT;
    while ((uint16_t)((uint16_t)TIM3->CNT - start) < ticks) { }
}


uint32_t stepTimerBusyMax(void)
{
    return busyMax;
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

    uint32_t idle = sched->getIdleTicks();
    if (idle > TIMER_MAX_STEP) idle = TIMER_MAX_STEP;

    TIM3->CR1 = 0;                          // up counting, free running
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 = 0;                        // channel 1 output compare, frozen: no pin, only the flag
    TIM3->CCER = 0;
    TIM3->CNT = 0;
    TIM3->EGR = TIM_EGR_UG;                 // load PSC
    lastTarget = 0;
    target = (uint16_t)idle;                // the first event: the scheduler's first elapsed is idle
    TIM3->CCR1 = target;
    TIM3->SR = 0;
    TIM3->DIER = TIM_DIER_CC1IE;
    NVIC_SetVector(TIM3_TIM4_IRQn, (uint32_t)timerIrq);
    NVIC_SetPriority(TIM3_TIM4_IRQn, 0);
    NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;
    printf("Step Generator Interrupt: TIM3 at %lu Hz, output compare\n", (unsigned long)SystemCoreClock);
    return true;
}
