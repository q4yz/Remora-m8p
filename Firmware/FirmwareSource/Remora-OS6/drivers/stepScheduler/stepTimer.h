#ifndef STEPTIMER_H
#define STEPTIMER_H

/***********************************************************************
    The board family's step timer for the StepScheduler

    A family that supports "Step Generator": "Interrupt" implements both
    (STM32G0: TARGET_STM32G0/drivers/stepTimer, TIM3); without them the
    weak defaults in modules/stepgen/stepgenInterrupt.cpp say "not here"
    and the Stepgen modules stay in the base thread.

    stepTimerStart(): from now on call sched->onTimer(elapsed) in a high
    priority interrupt, elapsed being the ticks since the last call, and
    the next call that many ticks later as it returns. The timer clock
    is the timerHz the scheduler was made with (stepTimerHz()).
************************************************************************/

#include <stdint.h>

class StepScheduler;

bool stepTimerAvailable(void);
uint32_t stepTimerHz(void);
bool stepTimerStart(StepScheduler* sched);      // false: the timer is used by something else

#endif
