#include "probeLatch.h"

#include <cstring>

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

// The probe must stay released this long before an edge counts (chatter at the
// switching point re-triggers within microseconds, a new probe move after much longer).
#define PROBE_LATCH_ARM_US   2000

void createProbeLatch()
{
    const char* comment = module["Comment"] | "probe";
    const char* pin = module["Pin"] | "";
    const char* modifier = module["Modifier"] | "";
    const char* invert = module["Invert"] | "False";
    int joint = module["Joint Number"] | -1;
    int pv = module["PV[i]"] | -1;
    printf("%s\n", comment);

    if (joint < 0 || joint >= JOINTS || pv < 0 || pv >= VARIABLES || !pin[0])
    {
        printf("Error - Probe Latch needs a Pin, a Joint Number below %d and a PV[i] below %d\n", JOINTS, VARIABLES);
        return;
    }

    int mod = NONE;
    if (!strcmp(modifier, "Pull Up")) mod = PULLUP;
    else if (!strcmp(modifier, "Pull Down")) mod = PULLDOWN;
    else if (!strcmp(modifier, "Pull None")) mod = PULLNONE;

    // the step count the joint's step generator reports (stepgen.cpp writes it every base tick)
    volatile int32_t &count = txData.jointFeedback[joint];
    ptrProcessVariable[pv] = &txData.processVariable[pv];
    uint32_t armTicks = (uint32_t)((int64_t)base_freq * PROBE_LATCH_ARM_US / 1000000);
    if (armTicks < 1) armTicks = 1;

    printf("Probe latch at pin %s, joint %d step count into PV[%d]\n", pin, joint, pv);
    Module* latch = new ProbeLatch(pin, mod, !strcmp(invert, "True"), count, *ptrProcessVariable[pv], armTicks);
    // the base thread's main list (the post list runs updatePost only); after the step
    // generators in config.txt order it reads this tick's count, before them the last one (25 us)
    baseThread->registerModule(latch);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

ProbeLatch::ProbeLatch(std::string portAndPin, int modifier, bool invert, volatile int32_t &ptrStepCount,
                       volatile float &ptrPV, uint32_t armTicks) :
    invert(invert),
    ptrStepCount(&ptrStepCount),
    ptrPV(&ptrPV),
    core(armTicks)
{
    this->pin = new Pin(portAndPin, INPUT, modifier);
    *(this->ptrPV) = this->core.packed();
}


void ProbeLatch::update()
{
    bool triggered = this->pin->get() != this->invert;
    if (this->core.tick(triggered, *(this->ptrStepCount)))
    {
        *(this->ptrPV) = this->core.packed();
    }
}
