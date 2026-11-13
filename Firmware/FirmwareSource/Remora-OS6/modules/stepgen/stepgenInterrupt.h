#ifndef STEPGENINTERRUPT_H
#define STEPGENINTERRUPT_H

/***********************************************************************
    "Stepgen" modules on the event driven step generation
    (drivers/stepScheduler), instead of the base thread DDS

    The default where the board family has a step timer (STM32G0);
    config.txt, top level:  "Step Generator": "Base Thread" keeps the
    Stepgen modules on the base thread DDS, as before. If the timer is
    taken (e.g. an RGB chain on PC6) they fall back to it on their own.

    One timer interrupt makes every joint's steps at their exact times;
    the servo thread passes LinuxCNC's frequencies and drives the enable
    pins. The base thread frequency (PRU_base_freq) is then only the
    driver's step rate limit, not the step timing.

    Board families: the timer is the family's (drivers/stepScheduler/
    stepTimer.h, STM32G0: TARGET_STM32G0/drivers/stepTimer, TIM3); a family
    without one keeps the Stepgen modules in the base thread.
************************************************************************/

#include <cstdint>

extern bool stepgenInterrupt;           // "Step Generator": "Interrupt" (loadModules)

bool stepgenInterruptSupported(void);
// a "Stepgen" module on the scheduler; false: not possible here (the caller uses the base thread)
bool createStepgenInterrupt(int joint, const char* step, const char* dir, const char* enable);
void stepgenInterruptReport(void);      // debug console: interrupts, steps, latest edge

#endif
