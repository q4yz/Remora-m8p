/***********************************************************************
    TMC2209 drivers on the STM32G0 hub (Manta M8P), "TMC2209" module

    The same driver as on the CAN nodes (TARGET_STM32G0/drivers/can/
    NodeTmc2209): a bit-banged single wire UART, 3 timer ticks per bit at
    40 kHz = 13333 baud, as on the EBB42 / SB2209 where it answers. The
    TMCStepper library with SoftwareSerial reached none of the M8P's
    drivers (they ran at 1/8 from their MS1 / MS2 pins).

    - the writes are checked: IFCNT read before and after, retried
    - GSTAT every second: a driver reset (motor power off and on again,
      e.g. by a relay) or no answer -> configured again
    - the result per driver on the console on every change:
      "stepper_x TMC2209 PF_13 ok, version 0x21, 16 microsteps, 800 mA"

    The UART ticks come from TIM14, switched on only while a transfer
    runs (main loop, tmcService()), so it costs nothing in between.
    Internal current reference (I_scale_analog 0): VREF is not used.
************************************************************************/

#include "tmc.h"

#if defined(TARGET_STM32G0)

#include "NodeTmc2209.h"
#include "canNodeIo.h"
#include "canProtocol.h"

#define HUB_TMC_MAX         8
#define HUB_TMC_TICK_HZ     40000           // CAN_NODE_BASE_FREQ: TMC_OVERSAMPLE ticks per bit, 13333 baud

static NodeTmc2209 drivers[HUB_TMC_MAX];
static char names[HUB_TMC_MAX][32];
static uint16_t microstepsOf[HUB_TMC_MAX];
static uint16_t currentOf[HUB_TMC_MAX];
static volatile uint8_t driverCount = 0;
static bool timerReady = false;
static bool timerRunning = false;
static bool conflictReported = false;


static void tmcTickIrq()
{
    if (TIM14->SR & TIM_SR_UIF)
    {
        TIM14->SR = ~TIM_SR_UIF;
        for (int i = 0; i < driverCount; i++) drivers[i].tick();
    }
}


static void timerSetup()
{
    if (timerReady) return;
    __HAL_RCC_TIM14_CLK_ENABLE();
    if (TIM14->CR1 & TIM_CR1_CEN) printf("TMC2209: TIM14 is used by another module (hardware PWM?)\n");
    TIM14->CR1 = 0;
    TIM14->PSC = 0;
    TIM14->ARR = SystemCoreClock / HUB_TMC_TICK_HZ - 1;
    TIM14->EGR = TIM_EGR_UG;
    TIM14->SR = 0;
    TIM14->DIER = TIM_DIER_UIE;
    NVIC_SetVector(TIM14_IRQn, (uint32_t)tmcTickIrq);
    // highest priority (with the SPI NSS edge): steady bit timing, a tick is a few us
    NVIC_SetPriority(TIM14_IRQn, 0);
    NVIC_EnableIRQ(TIM14_IRQn);
    timerReady = true;
}


static void timerRun(bool run)
{
    if (run == timerRunning) return;
    timerRunning = run;
    if (run)
    {
        TIM14->CNT = 0;
        TIM14->CR1 = TIM_CR1_CEN;
    }
    else
    {
        TIM14->CR1 = 0;
    }
}


void createTMC2209()
{
    const char* comment = module["Comment"] | "TMC2209";
    const char* pinName = module["RX pin"] | "";
    float rsense = module["RSense"] | 0.11f;
    int address = module["Address"] | 0;
    int current = module["Current"] | 0;
    int microsteps = module["Microsteps"] | 16;
    bool stealth = !strcmp(module["Stealth chop"] | "off", "on");
    int stall = module["Stall sensitivity"] | 0;
    uint8_t pin;
    char name[6];

    printf("%s: TMC2209\n", comment);

    if (driverCount >= HUB_TMC_MAX)
    {
        printf("TMC2209: more than %d drivers\n", HUB_TMC_MAX);
        return;
    }
    if (!canParsePin(pinName, &pin) || !canIoPinExists(pin))
    {
        printf("TMC2209: invalid RX pin '%s'\n", pinName);
        return;
    }
    int mstep = 0;
    while (mstep < 8 && (1 << mstep) < microsteps) mstep++;
    if ((1 << mstep) != microsteps || address < 0 || address > 3 || current < 10 || current > TMC_CURRENT_MAX ||
        !(rsense > 0.0f) || rsense > 1.0f || stall < 0 || stall > 255)
    {
        printf("TMC2209 %s: Microsteps a power of 2 (1 ... 256), Address 0 ... 3, Current 10 ... %d mA, "
               "RSense 0 ... 1 Ohm, Stall sensitivity 0 ... 255: not configured\n",
               canPinName(pin, name), TMC_CURRENT_MAX);
        return;
    }

    // UART line idle high, the pull up holds it while the driver answers
    canIoConfigure(pin, true, DPIN_MOD_PULLUP, true);

    int i = driverCount;
    drivers[i].setup(pin, (uint8_t)address, (uint16_t)(rsense * 1000.0f + 0.5f), (uint16_t)current,
                     (uint16_t)microsteps, stealth, (uint8_t)stall);
    drivers[i].start();
    snprintf(names[i], sizeof(names[i]), "%s", comment);
    microstepsOf[i] = (uint16_t)microsteps;
    currentOf[i] = (uint16_t)current;

    timerSetup();
    driverCount = (uint8_t)(i + 1);         // last: the tick interrupt uses it from now on

    printf("TMC2209 %s: address %d, %d mA, %d microsteps, %s, RSense %.3f Ohm, configured from the main loop\n",
           canPinName(pin, name), address, current, microsteps, stealth ? "StealthChop" : "SpreadCycle",
           (double)rsense);
}


void tmcService()
{
    if (!driverCount) return;

    // another module set TIM14 up after us (hardware PWM on PA4 / PA7 / PB1 / PC12 / PF0)
    if (TIM14->ARR != SystemCoreClock / HUB_TMC_TICK_HZ - 1)
    {
        if (!conflictReported) printf("TMC2209: TIM14 is used by another module, the drivers are not configured\n");
        conflictReported = true;
        return;
    }

    uint32_t now = HAL_GetTick();
    bool busy = false;
    for (int i = 0; i < driverCount; i++)
    {
        NodeTmc2209& d = drivers[i];
        if (d.service(now))
        {
            char name[6];
            if (d.getStatus() == TMC_STATUS_OK)
            {
                printf("%s TMC2209 %s ok, version 0x%02x, %u microsteps, %u mA\n", names[i],
                       canPinName(d.getPin(), name), d.getVersion(), microstepsOf[i], currentOf[i]);
            }
            else
            {
                printf("%s TMC2209 %s %s\n", names[i], canPinName(d.getPin(), name), tmcStatusText(d.getStatus()));
            }
        }
        busy |= d.busy();
    }
    timerRun(busy);
}

#endif
