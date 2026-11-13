#include "StepScheduler.h"

#include <string.h>


StepScheduler::StepScheduler(StepPins& pins, uint32_t timerHz, uint32_t pulseNs, uint32_t dirSetupNs, uint32_t maxRate) :
    pins(pins),
    timerHz(timerHz),
    now(0),
    events(0),
    steps(0),
    lateMax(0)
{
    memset((void*)this->joint, 0, sizeof(this->joint));
    this->pulseTicks = (uint32_t)((uint64_t)timerHz * pulseNs / 1000000000u);
    if (this->pulseTicks < 1) this->pulseTicks = 1;
    this->dirSetupTicks = (uint32_t)((uint64_t)timerHz * dirSetupNs / 1000000000u);
    // a step is a pulse high and at least as long low; and not above maxRate
    this->minInterval = 2 * this->pulseTicks;
    if (maxRate && timerHz / maxRate > this->minInterval) this->minInterval = timerHz / maxRate;
    this->idleTicks = (uint32_t)((uint64_t)timerHz * STEPSCHED_IDLE_US / 1000000u);
}


bool StepScheduler::addJoint(uint8_t j, volatile int32_t* feedback)
{
    if (j >= STEPSCHED_JOINTS || this->joint[j].used) return false;
    Joint& J = this->joint[j];
    J.feedback = feedback;
    J.forward = true;
    J.pForward = true;
    this->pins.dir(j, true);
    this->pins.step(j, false);
    J.used = true;                          // last: the interrupt looks at it from now on
    return true;
}


void StepScheduler::command(uint8_t j, int32_t frequency, bool enabled)
{
    if (j >= STEPSCHED_JOINTS || !this->joint[j].used) return;
    Joint& J = this->joint[j];

    uint32_t interval = 0;
    if (enabled && frequency != 0)
    {
        uint32_t f = frequency < 0 ? 0u - (uint32_t)frequency : (uint32_t)frequency;
        interval = this->timerHz / f;       // software division on the M0+: here, not in the interrupt
        if (interval < this->minInterval) interval = this->minInterval;
    }

    // the interrupt takes all three when pSeq changes, pSeq last
    J.pInterval = interval;
    if (frequency != 0) J.pForward = frequency > 0;     // 0 keeps the direction (no dir pin edge)
    J.pSeq = (uint8_t)(J.pSeq + 1);
}


// a new command for joint j (interrupt)
void StepScheduler::apply(uint8_t j)
{
    Joint& J = this->joint[j];
    J.seen = J.pSeq;
    uint32_t interval = J.pInterval;
    bool forward = J.pForward;

    if (interval && forward != J.forward)
    {
        J.forward = forward;
        this->pins.dir(j, forward);
        J.notBefore = this->now + this->dirSetupTicks;
    }

    bool wasRunning = J.interval != 0;
    J.interval = interval;
    if (!interval)
    {
        if (!J.high) J.active = false;      // a pulse still running ends first
        return;
    }
    if (J.high) return;                     // the pulse ends first, then last + interval

    // running: one new interval after the last step; from a stop: half an interval (the mean phase)
    uint32_t next = wasRunning ? J.last + interval : this->now + interval / 2;
    if ((int32_t)(next - this->now) < 0) next = this->now;
    if ((int32_t)(next - J.notBefore) < 0) next = J.notBefore;
    J.next = next;
    J.active = true;
}


// the edge of joint j that is due (interrupt)
void StepScheduler::edge(uint8_t j)
{
    Joint& J = this->joint[j];

    if (J.high)
    {
        this->pins.step(j, false);
        J.high = false;
        if (!J.interval)
        {
            J.active = false;
            return;
        }
        uint32_t next = J.last + J.interval;
        uint32_t lowEnd = this->now + this->pulseTicks;         // low at least a pulse long
        if ((int32_t)(next - lowEnd) < 0) next = lowEnd;
        if ((int32_t)(next - J.notBefore) < 0) next = J.notBefore;
        J.next = next;
        return;
    }

    int32_t late = (int32_t)(this->now - J.next);
    if (late > 0 && (uint32_t)late > this->lateMax) this->lateMax = (uint32_t)late;

    this->pins.step(j, true);
    J.high = true;
    J.count += J.forward ? 1 : -1;
    if (J.feedback) *J.feedback = J.count;
    this->steps++;
    J.last = J.next;                        // the rhythm on the scheduled time: a late edge is no lost step
    J.next = this->now + this->pulseTicks;  // the pulse on the real time
}


uint32_t StepScheduler::onTimer(uint32_t elapsed)
{
    this->now += elapsed;
    this->events++;

    for (uint8_t j = 0; j < STEPSCHED_JOINTS; j++)
    {
        Joint& J = this->joint[j];
        if (J.used && J.pSeq != J.seen) this->apply(j);
    }

    uint32_t best = this->idleTicks;
    for (uint8_t j = 0; j < STEPSCHED_JOINTS; j++)
    {
        Joint& J = this->joint[j];
        if (!J.used || !J.active) continue;
        // the edges due now (or within the slack): at most a step and the end of the one before
        for (int k = 0; k < 2 && J.active && (int32_t)(J.next - this->now) <= STEPSCHED_SLACK_TICKS; k++)
        {
            this->edge(j);
        }
        if (!J.active) continue;
        int32_t d = (int32_t)(J.next - this->now);
        if (d < 1) d = 1;
        if ((uint32_t)d < best) best = (uint32_t)d;
    }
    return best;
}
