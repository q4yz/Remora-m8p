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
    family's (stepTimer.h, STM32G0: TIM3). A family without one keeps
    the weak defaults below and the Stepgen modules in the base thread.
************************************************************************/

#define STEP_DIR_SETUP_NS   5000            // direction stable before the next step (external drivers ~5 us)
#define STEP_MAX_RATE       100000          // steps / s per joint

bool stepgenInterrupt = false;

// a family without a step timer (no TARGET_.../drivers/stepTimer)
__attribute__((weak)) bool stepTimerAvailable(void) { return false; }
__attribute__((weak)) uint32_t stepTimerHz(void) { return 0; }
__attribute__((weak)) bool stepTimerStart(StepScheduler*) { return false; }


class BoardStepPins : public StepPins
{
    public:
        Pin* stepPin[STEPSCHED_JOINTS] = {};
        Pin* dirPin[STEPSCHED_JOINTS] = {};
        void step(uint8_t j, bool high) override { this->stepPin[j]->set(high); }
        void dir(uint8_t j, bool forward) override { this->dirPin[j]->set(forward); }
};

static BoardStepPins pins;
static StepScheduler* sched = nullptr;
static Pin* enablePin[STEPSCHED_JOINTS];
static uint8_t jointMask = 0;


// servo thread: LinuxCNC's frequencies and enables to the scheduler, the enable pins (active low)
class StepgenServo : public Module
{
    public:
        void update(void) override
        {
            uint8_t enable = rxData.jointEnable;
            for (uint8_t j = 0; j < STEPSCHED_JOINTS; j++)
            {
                if (!(jointMask & (1u << j))) continue;
                bool on = !mcuShutdown && (enable & (1u << j));
                enablePin[j]->set(!on);
                sched->command(j, on ? rxData.jointFreqCmd[j] : 0, on);
            }
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
    servoThread->registerModule(new StepgenServo());
    printf("Step Generator Interrupt: pulse %d ns, direction setup %d ns, up to %d steps/s per joint\n",
           STEP_PULSE_MIN_NS, STEP_DIR_SETUP_NS, STEP_MAX_RATE);
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

    jointMask |= (uint8_t)(1u << joint);    // the servo module passes its commands from now on
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
    printf("[debug] stepgen: interrupt, %lu interrupts, %lu steps, latest edge %lu us late\n",
           (unsigned long)sched->events, (unsigned long)sched->steps, (unsigned long)(sched->lateMax / tpu));
}
