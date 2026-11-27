#ifndef TICKTIMER_H
#define TICKTIMER_H

/***********************************************************************
    A periodic interrupt at the highest priority, for bit timing

    On a hub: the bit clock of the bit-banged TMC2209 UART
    (modules/tmc/tmc2209Uart.cpp), running only while a transfer runs.
    On a CAN node: its base thread (node/nodeMain.cpp: step pulses, PWM
    and the same UART). One per firmware.

    A family with REMORA_HAS_TICK_TIMER implements it on a timer that
    has no other use (STM32G0: TARGET_STM32G0/drivers/tickTimer, TIM14 on
    a hub, TIM6 on a node).
************************************************************************/

#include <stdint.h>

// set the timer up for tick() at hz, stopped; false (and why on the console): the timer is in use
bool tickTimerStart(uint32_t hz, void (*tick)(void));
void tickTimerRun(bool run);
// false: another module has taken the timer since tickTimerStart() (e.g. a hardware PWM on its pin)
bool tickTimerIntact(void);

#endif
