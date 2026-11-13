#include "stepgenInterrupt.h"

#include "configuration.h"
#include "extern.h"

bool stepgenInterrupt = false;

#if defined(TARGET_STM32G0)

#include "StepScheduler.h"
#include "drivers/pin/pin.h"
#include "modules/module.h"

/***********************************************************************
    STM32G0 hardware: TIM3 (16 bit, 64 MHz, 15.6 ns), its period set at
    every interrupt to the time to the next edge (ARPE off: the new ARR
    counts from the update that started this interrupt). TIM3 is free on
    the G0: the comms thread object that owns it is never started there
    (the hub TMC2209s run on TIM14). Highest priority, with the SPI NSS
    edge and the TMC UART: an interrupt is a few us.
************************************************************************/

#define STEP_DIR_SETUP_NS   5000            // direction stable before the next step (external drivers ~5 us)
#define STEP_MAX_RATE       100000          // steps / s per joint
#define TIMER_MARGIN        64              // ticks: the next update at least 1 us after this interrupt

class HubStepPins : public StepPins
{
    public:
        Pin* stepPin[STEPSCHED_JOINTS] = {};
        Pin* dirPin[STEPSCHED_JOINTS] = {};
        void step(uint8_t j, bool high) override { this->stepPin[j]->set(high); }
        void dir(uint8_t j, bool forward) override { this->dirPin[j]->set(forward); }
};

static HubStepPins pins;
static StepScheduler* sched = nullptr;
static Pin* enablePin[STEPSCHED_JOINTS];
static uint8_t jointMask = 0;
static volatile uint32_t programmed = 0;


static void timerIrq()
{
    if (!(TIM3->SR & TIM_SR_UIF)) return;
    TIM3->SR = ~TIM_SR_UIF;
    uint32_t d = sched->onTimer(programmed);
    // the period counts from the update: it must end after now, or the counter runs to 65535
    uint32_t cnt = TIM3->CNT;
    if (d < cnt + TIMER_MARGIN) d = cnt + TIMER_MARGIN;
    TIM3->ARR = d - 1;
    programmed = d;
}


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

    __HAL_RCC_TIM3_CLK_ENABLE();
    if (TIM3->CR1 & TIM_CR1_CEN)
    {
        printf("Step Generator Interrupt: TIM3 is used by another module (RGB on PC6, hardware PWM)\n");
        return false;
    }
    sched = new StepScheduler(pins, SystemCoreClock, STEP_PULSE_MIN_NS, STEP_DIR_SETUP_NS, STEP_MAX_RATE);
    servoThread->registerModule(new StepgenServo());

    TIM3->CR1 = 0;                          // ARPE off, up counting
    TIM3->PSC = 0;
    programmed = sched->getIdleTicks();
    TIM3->ARR = programmed - 1;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->DIER = TIM_DIER_UIE;
    NVIC_SetVector(TIM3_TIM4_IRQn, (uint32_t)timerIrq);
    NVIC_SetPriority(TIM3_TIM4_IRQn, 0);
    NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;
    printf("Step Generator Interrupt: TIM3 at %lu Hz, pulse %d ns, direction setup %d ns, up to %d steps/s per joint\n",
           (unsigned long)SystemCoreClock, STEP_PULSE_MIN_NS, STEP_DIR_SETUP_NS, STEP_MAX_RATE);
    return true;
}


bool stepgenInterruptSupported(void)
{
    return true;
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

    jointMask |= (uint8_t)(1u << joint);    // before addJoint: the servo module passes its commands then
    sched->addJoint((uint8_t)joint, &txData.jointFeedback[joint]);
    printf("Stepgen joint %d on the interrupt step generation: step %s, direction %s, enable %s\n", joint, step, dir,
           enable);
    return true;
}


void stepgenInterruptReport(void)
{
    if (!sched) return;
    uint32_t tpu = SystemCoreClock / 1000000;
    printf("[debug] stepgen: interrupt, %lu interrupts, %lu steps, latest edge %lu us late\n",
           (unsigned long)sched->events, (unsigned long)sched->steps, (unsigned long)(sched->lateMax / tpu));
}

#else

bool stepgenInterruptSupported(void) { return false; }
bool createStepgenInterrupt(int, const char*, const char*, const char*) { return false; }
void stepgenInterruptReport(void) {}

#endif
