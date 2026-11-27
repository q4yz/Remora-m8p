#include "stepTimer.h"
#include "StepScheduler.h"

#include "mbed.h"
#include "configuration.h"                  // family.h: STEP_TIMER, STEP_TIMER_IRQ, STEP_TIMER_CLK_ENABLE(), STEP_TIMER_HZ(), STEP_TIMER_NAME

/***********************************************************************
    STM32 step timer (every STM32 family with REMORA_HAS_STEP_TIMER): a
    general purpose timer the family names in family.h, counting 16 bit
    (ARR 0xFFFF, also on a 32 bit timer), the next event by output compare
    (CCR1, frozen mode, no pin). Highest priority, with the SPI NSS edge
    and the tick timer: an interrupt is a few us.

    STM32G0: TIM3 at 64 MHz. STM32F4: TIM2 at 90 MHz (APB1 x 2): a 16 bit
    period is 728 us, more than twice the longest step (TIMER_MAX_STEP).

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

#ifdef REMORA_HAS_STEP_TIMER

#define TIMER_MARGIN        64              // ticks: the next compare at least 1 us after this interrupt
#define TIMER_MAX_STEP      0x7FFF          // ticks: compare targets ahead by less than half the counter

static StepScheduler* sched = nullptr;
static volatile uint16_t target = 0;        // the compare value of this event (the scheduler's "now")
static uint16_t lastTarget = 0;
static volatile uint16_t busyMax = 0;       // ticks, the longest interrupt (from its compare time to its end)


static void timerIrq()
{
    if (!(STEP_TIMER->SR & TIM_SR_CC1IF)) return;
    STEP_TIMER->SR = ~TIM_SR_CC1IF;

    uint16_t now = target;
    uint32_t d = sched->onTimer((uint16_t)(now - lastTarget));
    lastTarget = now;
    if (d > TIMER_MAX_STEP) d = TIMER_MAX_STEP;

    // the next event, in the future: a late interrupt moves it, the time base stays exact
    uint16_t next = (uint16_t)(now + d);
    uint16_t cnt = (uint16_t)STEP_TIMER->CNT;
    if ((int16_t)(next - cnt) < TIMER_MARGIN) next = (uint16_t)(cnt + TIMER_MARGIN);
    STEP_TIMER->CCR1 = next;
    target = next;

    uint16_t busy = (uint16_t)(cnt - now);
    if (busy > busyMax) busyMax = busy;
}


// busy wait in the interrupt (the width of a step pulse), ticks < 32768
void stepTimerWait(uint32_t ticks)
{
    uint16_t start = (uint16_t)STEP_TIMER->CNT;
    while ((uint16_t)((uint16_t)STEP_TIMER->CNT - start) < ticks) { }
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
    return STEP_TIMER_HZ();
}


bool stepTimerStart(StepScheduler* scheduler)
{
    STEP_TIMER_CLK_ENABLE();
    if (STEP_TIMER->CR1 & TIM_CR1_CEN)
    {
        printf("Step Generator Interrupt: " STEP_TIMER_NAME " is used by another module (RGB, hardware PWM)\n");
        return false;
    }
    sched = scheduler;

    uint32_t idle = sched->getIdleTicks();
    if (idle > TIMER_MAX_STEP) idle = TIMER_MAX_STEP;

    STEP_TIMER->CR1 = 0;                          // up counting, free running
    STEP_TIMER->PSC = 0;
    STEP_TIMER->ARR = 0xFFFF;
    STEP_TIMER->CCMR1 = 0;                        // channel 1 output compare, frozen: no pin, only the flag
    STEP_TIMER->CCER = 0;
    STEP_TIMER->CNT = 0;
    STEP_TIMER->EGR = TIM_EGR_UG;                 // load PSC
    lastTarget = 0;
    target = (uint16_t)idle;                // the first event: the scheduler's first elapsed is idle
    STEP_TIMER->CCR1 = target;
    STEP_TIMER->SR = 0;
    STEP_TIMER->DIER = TIM_DIER_CC1IE;
    NVIC_SetVector(STEP_TIMER_IRQ, (uint32_t)timerIrq);
    NVIC_SetPriority(STEP_TIMER_IRQ, 0);
    NVIC_EnableIRQ(STEP_TIMER_IRQ);
    STEP_TIMER->CR1 = TIM_CR1_CEN;
    printf("Step Generator Interrupt: " STEP_TIMER_NAME " at %lu Hz, output compare\n", (unsigned long)STEP_TIMER_HZ());
    return true;
}

#endif
