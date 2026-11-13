#include "StepScheduler.h"

#include <string.h>


StepScheduler::StepScheduler(StepPins& pins, uint32_t timerHz, uint32_t pulseNs, uint32_t dirSetupNs, uint32_t maxRate) :
    pins(pins),
    nUsed(0),
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
    this->inlinePulse = pulseNs <= STEPSCHED_INLINE_PULSE_NS;
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
    J.used = true;
    this->used[this->nUsed] = &J;
    this->nUsed = (uint8_t)(this->nUsed + 1);   // last: the interrupt looks at it from now on
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


// a new command for the joint (interrupt)
void StepScheduler::apply(Joint& J)
{
    J.seen = J.pSeq;
    uint32_t interval = J.pInterval;
    bool forward = J.pForward;

    if (interval && forward != J.forward)
    {
        J.forward = forward;
        this->pins.dir(this->index(J), forward);
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

    // running: one new interval after the last step; from a stop: half an interval (the mean phase).
    // A legal "last + interval" is at most an interval (plus the slack of an early edge) ahead of now,
    // or less than an interval behind (the interrupt is late, the step follows at once and the rhythm
    // stays). Anything else is long past (the joint got faster), or J.last is so old that the 32 bit
    // time wrapped (2^31 ticks, 33.5 s at 64 MHz: a joint standing still that long and then started
    // with a slow command first, as LinuxCNC's homing does, waited up to 33 s, no steps and a
    // following error): then now.
    uint32_t next;
    if (wasRunning)
    {
        next = J.last + interval;
        int32_t ahead = (int32_t)(next - this->now);
        if (ahead > (int32_t)(interval + STEPSCHED_SLACK_TICKS) || ahead < -(int32_t)interval) next = this->now;
    }
    else
    {
        next = this->now + interval / 2;
    }
    if (this->dirWaiting(J) && (int32_t)(next - J.notBefore) < 0) next = J.notBefore;
    J.next = next;
    J.active = true;
}


// the direction setup of a joint is still running: notBefore lies at most dirSetupTicks ahead.
// An old notBefore (a direction change long ago) is no limit, however the 32 bit time has wrapped.
bool StepScheduler::dirWaiting(const Joint& J) const
{
    return (uint32_t)(J.notBefore - this->now) <= this->dirSetupTicks;
}


// the step that is due: pin high, count (interrupt)
void StepScheduler::rise(Joint& J)
{
    int32_t late = (int32_t)(this->now - J.next);
    if (late > 0 && (uint32_t)late > this->lateMax) this->lateMax = (uint32_t)late;

    this->pins.step(this->index(J), true);
    J.count += J.forward ? 1 : -1;
    if (J.feedback) *J.feedback = J.count;
    this->steps++;
    J.last = J.next;                        // the rhythm on the scheduled time: a late edge is no lost step
}


// a pulse with an interrupt of its own for its end: the edge of the joint that is due (interrupt)
void StepScheduler::edge(Joint& J)
{
    if (J.high)
    {
        this->pins.step(this->index(J), false);
        J.high = false;
        if (!J.interval)
        {
            J.active = false;
            return;
        }
        uint32_t next = J.last + J.interval;
        uint32_t lowEnd = this->now + this->pulseTicks;         // low at least a pulse long
        if ((int32_t)(next - lowEnd) < 0) next = lowEnd;
        if (this->dirWaiting(J) && (int32_t)(next - J.notBefore) < 0) next = J.notBefore;
        J.next = next;
        return;
    }

    this->rise(J);
    J.high = true;
    J.next = this->now + this->pulseTicks;  // the pulse on the real time
}


uint32_t StepScheduler::onTimer(uint32_t elapsed)
{
    uint32_t now = this->now + elapsed;
    this->now = now;
    this->events++;

    uint32_t best = this->idleTicks;
    uint32_t risen = 0;                     // joints whose pulse ends in this interrupt
    uint8_t n = this->nUsed;
    for (uint8_t i = 0; i < n; i++)
    {
        Joint& J = *this->used[i];
        if (J.pSeq != J.seen) this->apply(J);
        if (!J.active) continue;

        int32_t d = (int32_t)(J.next - now);
        if (d <= STEPSCHED_SLACK_TICKS)     // due now, or within the slack
        {
            if (this->inlinePulse)
            {
                this->rise(J);
                risen |= 1u << i;
                // the pulse ends below, about a pulse from now: low at least as long after that
                uint32_t next = J.last + J.interval;
                uint32_t lowEnd = now + 2 * this->pulseTicks;
                if ((int32_t)(next - lowEnd) < 0) next = lowEnd;
                J.next = next;
            }
            else
            {
                // at most a step and the end of the one before
                this->edge(J);
                if (J.active && (int32_t)(J.next - now) <= STEPSCHED_SLACK_TICKS) this->edge(J);
                if (!J.active) continue;
            }
            d = (int32_t)(J.next - now);
        }
        if (d < 1) d = 1;
        if ((uint32_t)d < best) best = (uint32_t)d;
    }

    if (risen)
    {
        this->pins.wait(this->pulseTicks);
        for (uint8_t i = 0; i < n; i++)
        {
            if (risen & (1u << i)) this->pins.step(this->index(*this->used[i]), false);
        }
    }
    return best;
}
