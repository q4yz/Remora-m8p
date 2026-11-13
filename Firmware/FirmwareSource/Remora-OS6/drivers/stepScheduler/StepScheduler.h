#ifndef STEPSCHEDULER_H
#define STEPSCHEDULER_H

/***********************************************************************
    Event driven step generation, independent of the MCU

    The Klipper principle on Remora's data: LinuxCNC still sends a step
    frequency per joint every servo period and reads back the step count,
    but the steps are not sampled on a fixed base thread tick. One timer
    interrupt fires exactly when the next edge of any joint is due,
    emits the edges that are due and programs the timer for the next one:

    - the step times are exact (timer resolution, 15.6 ns at 64 MHz)
      instead of rounded to a base period, no 1-or-2-tick ripple near
      the top speed, idle joints cost nothing
    - a new frequency keeps the step rhythm: the next step comes one new
      interval after the last one
    - the step pulse is always pulseTicks high and at least as long low,
      after a direction change the next step waits dirSetupTicks
    - a late interrupt loses no step: the rate is kept on the scheduled
      times, missed steps follow as fast as the pulse allows

    Split like the SPI link: this is the logic (Firmware/Tests/stepsched
    runs it on a PC), StepPins and the timer are the board family's.
    The base thread calls command() when a frequency changed (a division, the M0+ has
    no divide instruction), the timer interrupt only compares and adds.
************************************************************************/

#include <stdint.h>

#define STEPSCHED_JOINTS        8
#define STEPSCHED_IDLE_US       100     // longest timer period: new commands are taken within this
#define STEPSCHED_SLACK_TICKS   32      // edges due this close are made in the same interrupt (0.5 us)


// the joint's step / direction outputs, the board family's
class StepPins
{
    public:
        virtual void step(uint8_t joint, bool high) = 0;
        virtual void dir(uint8_t joint, bool forward) = 0;
};


class StepScheduler
{
    private:

        struct Joint
        {
            bool              used;
            bool              active;       // an edge is scheduled
            bool              high;         // step pin high, the next edge ends the pulse
            bool              forward;
            uint32_t          interval;     // ticks per step, 0: stopped
            uint32_t          last;         // scheduled time of the last step (the rhythm)
            uint32_t          next;         // time of the next edge
            uint32_t          notBefore;    // direction setup: no step before this
            int32_t           count;        // step position
            volatile int32_t* feedback;     // where the count goes, at every step

            // from command(), taken by the interrupt when seq changes
            volatile uint32_t pInterval;
            volatile bool     pForward;
            volatile uint8_t  pSeq;
            uint8_t           seen;
        };

        StepPins&   pins;
        Joint       joint[STEPSCHED_JOINTS];
        uint32_t    timerHz;
        uint32_t    pulseTicks;
        uint32_t    dirSetupTicks;
        uint32_t    minInterval;
        uint32_t    idleTicks;
        uint32_t    now;                    // ticks, wraps (differences only)

        void apply(uint8_t j);
        void edge(uint8_t j);

    public:

        // timerHz: the timer clock; pulse / dirSetup in ns; maxRate: steps / s per joint
        StepScheduler(StepPins& pins, uint32_t timerHz, uint32_t pulseNs, uint32_t dirSetupNs, uint32_t maxRate);

        bool addJoint(uint8_t j, volatile int32_t* feedback);

        // base thread, on a change: the joint's frequency (steps / s, signed); 0 or !enabled stops it
        void command(uint8_t j, int32_t frequency, bool enabled);

        // timer interrupt: elapsed = the ticks programmed last time (the time since the last call);
        // returns the ticks to the next call, 1 ... idle period
        uint32_t onTimer(uint32_t elapsed);

        int32_t getCount(uint8_t j) const { return this->joint[j].count; }
        uint32_t getMinInterval(void) const { return this->minInterval; }
        uint32_t getIdleTicks(void) const { return this->idleTicks; }

        // statistics (debug)
        volatile uint32_t   events;         // onTimer() calls
        volatile uint32_t   steps;
        volatile uint32_t   lateMax;        // ticks, the most an edge was made after its time
};

#endif
