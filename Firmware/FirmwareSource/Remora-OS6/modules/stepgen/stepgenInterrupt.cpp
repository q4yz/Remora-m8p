#include "stepgenInterrupt.h"

#include "configuration.h"
#include "extern.h"

#include "StepScheduler.h"
#include "stepTimer.h"
#include "drivers/pin/pin.h"
#include "modules/module.h"

/***********************************************************************
    "Stepgen" modules on the event driven step generation, board family
    independent: the logic is drivers/stepScheduler, the timer the
    family's (stepTimer.h, REMORA_HAS_STEP_TIMER; STM32G0: TIM3). A
    family without one keeps the Stepgen modules in the base thread.
************************************************************************/

#define STEP_DIR_SETUP_NS   5000            // direction stable before the next step (external drivers ~5 us)
#define STEP_MAX_RATE       100000          // steps / s per joint

bool stepgenInterrupt = false;

#ifndef REMORA_HAS_STEP_TIMER
// a family without a step timer: the Stepgen modules stay in the base thread
bool stepTimerAvailable(void) { return false; }
uint32_t stepTimerHz(void) { return 0; }
bool stepTimerStart(StepScheduler*) { return false; }
void stepTimerWait(uint32_t) { }
uint32_t stepTimerBusyMax(void) { return 0; }
#endif


class BoardStepPins : public StepPins
{
    public:
        Pin* stepPin[STEPSCHED_JOINTS] = {};
        Pin* dirPin[STEPSCHED_JOINTS] = {};
        void step(uint8_t j, bool high) override { this->stepPin[j]->set(high); }
        void dir(uint8_t j, bool forward) override { this->dirPin[j]->set(forward); }
        void wait(uint32_t ticks) override { stepTimerWait(ticks); }
};

static BoardStepPins pins;
static StepScheduler* sched = nullptr;
static Pin* enablePin[STEPSCHED_JOINTS];
static uint8_t jointMask = 0;


// base thread: LinuxCNC's frequencies and enables to the scheduler, the enable pins (active low).
// Not the servo thread: the link copies each SPI frame to rxData in the main loop, and the servo
// thread runs unsynchronised to LinuxCNC's period, so it would miss a frame now and then and use
// the one before for two periods (each time freq x 1 ms of steps wrong: a following error).
// The base thread takes every frame within a base period, like the base thread DDS did; it only
// compares, the division in command() runs when a frequency changed (once per servo period).
class StepgenCommand : public Module
{
    private:
        int32_t freq[STEPSCHED_JOINTS] = {};
        bool    on[STEPSCHED_JOINTS] = {};
        bool    first = true;

    public:
        void update(void) override
        {
            uint8_t enable = rxData.jointEnable;
            for (uint8_t j = 0; j < STEPSCHED_JOINTS; j++)
            {
                if (!(jointMask & (1u << j))) continue;
                bool o = !mcuShutdown && (enable & (1u << j));
                int32_t f = o ? rxData.jointFreqCmd[j] : 0;
                if (!this->first && o == this->on[j] && f == this->freq[j]) continue;
                this->on[j] = o;
                this->freq[j] = f;
                enablePin[j]->set(!o);
                sched->command(j, f, o);
            }
            this->first = false;
        }
};


static bool startScheduler()
{
    if (sched) return true;
    if (!stepTimerAvailable()) return false;

    StepScheduler* s = new StepScheduler(pins, stepTimerHz(), STEP_PULSE_MIN_NS, STEP_DIR_SETUP_NS, STEP_MAX_RATE);
    if (!stepTimerStart(s))
    {
        delete s;
        return false;
    }
    sched = s;
    baseThread->registerModule(new StepgenCommand());
    printf("Step Generator Interrupt: pulse %d ns (%s), direction setup %d ns, up to %d steps/s per joint\n",
           STEP_PULSE_MIN_NS, s->getInlinePulse() ? "one interrupt per step" : "an interrupt for each edge",
           STEP_DIR_SETUP_NS, STEP_MAX_RATE);
    return true;
}


bool stepgenInterruptSupported(void)
{
    return stepTimerAvailable();
}


bool createStepgenInterrupt(int joint, const char* step, const char* dir, const char* enable)
{
    if (joint < 0 || joint >= STEPSCHED_JOINTS || joint >= JOINTS)
    {
        printf("Stepgen: Joint Number %d out of range\n", joint);
        return false;
    }
    if (jointMask & (1u << joint))
    {
        printf("Stepgen: joint %d used twice\n", joint);
        return false;
    }
    if (!startScheduler()) return false;

    pins.stepPin[joint] = new Pin(step, OUTPUT);
    pins.dirPin[joint] = new Pin(dir, OUTPUT);
    enablePin[joint] = new Pin(enable, OUTPUT);
    enablePin[joint]->set(true);            // disabled until LinuxCNC enables the joint

    ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    jointMask |= (uint8_t)(1u << joint);    // the base thread module passes its commands from now on
    sched->addJoint((uint8_t)joint, &txData.jointFeedback[joint]);
    printf("Stepgen joint %d on the interrupt step generation: step %s, direction %s, enable %s\n", joint, step, dir,
           enable);
    return true;
}


void stepgenInterruptReport(void)
{
    if (!sched) return;
    uint32_t tpu = stepTimerHz() / 1000000;
    if (!tpu) return;
    printf("[debug] stepgen: interrupt, %lu interrupts, %lu steps, latest edge %lu us late, longest interrupt %lu us\n",
           (unsigned long)sched->events, (unsigned long)sched->steps, (unsigned long)(sched->lateMax / tpu),
           (unsigned long)(stepTimerBusyMax() / tpu));
}
