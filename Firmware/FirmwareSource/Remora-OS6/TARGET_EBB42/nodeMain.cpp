/***********************************************************************
    Remora CAN node firmware for the BTT EBB42 V1.1 / V1.2 (STM32G0B1CBT6)

    Takes its node number and its modules (digital pins, step generators,
    TMC2209 drivers, thermistors) from the hub and exchanges the process data
    with it (CAN_PROTOCOL.md, CanNode). Heater and fans as digital or PWM
    outputs. The step pulses and the PWM are
    made in a TIM6 interrupt at CAN_NODE_BASE_FREQ (CanNode::baseTick).

    EBB42: CAN on PB0 (RX) / PB1 (TX) = FDCAN2, 8 MHz crystal, no bootloader,
    flashed over USB in DFU mode at 0x08000000, 120R termination by jumper.

    Debug output (printf) goes to the USB-C port as a serial port (UsbCdc),
    the same cable as for flashing: open the COM port / /dev/ttyACM0 with
    any terminal, any baud rate.
************************************************************************/

#include "mbed.h"
#include "configuration.h"

#ifdef REMORA_CAN_NODE

#include "CanNode.h"
#include "UsbCdc.h"

#define NODE_CAN_BITRATE    1000000         // fixed in phase 1, must match "CAN" "Bitrate" on the hub
#define NODE_STATUS_MS      2000            // debug status line period

static const char* stateName(uint8_t state)
{
    switch (state)
    {
        case NODE_STATE_UNASSIGNED: return "unassigned";
        case NODE_STATE_PREOP:      return "pre-operational";
        case NODE_STATE_RUNNING:    return "running";
        case NODE_STATE_SAFE:       return "safe";
    }
    return "?";
}

static CanNode* baseNode = nullptr;

static void baseThreadIrq()
{
    if (TIM6->SR & TIM_SR_UIF)
    {
        TIM6->SR = ~TIM_SR_UIF;
        baseNode->baseTick();
    }
}

// TIM6 (basic timer, no pins; TIM2 is the mbed us ticker), highest priority: steady step timing
static void startBaseThread(CanNode* node)
{
    baseNode = node;
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->CR1 = 0;
    TIM6->PSC = 0;
    TIM6->ARR = SystemCoreClock / CAN_NODE_BASE_FREQ - 1;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0;
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_SetVector(TIM6_DAC_LPTIM1_IRQn, (uint32_t)baseThreadIrq);
    NVIC_SetPriority(TIM6_DAC_LPTIM1_IRQn, 0);
    NVIC_EnableIRQ(TIM6_DAC_LPTIM1_IRQn);
    TIM6->CR1 = TIM_CR1_CEN;
}

int main()
{
    Watchdog& watchdog = Watchdog::get_instance();
    watchdog.start(2000);

    RemoraCAN can(FDCAN2, GPIOB, 0, 1, GPIO_AF3_FDCAN2, NODE_CAN_BITRATE);
    CanNode node(&can);

    // USB serial number = node UUID, also visible in the Windows device manager / lsusb -v
    char uuid[13];
    for (int i = 0; i < 6; i++) snprintf(&uuid[2 * i], 3, "%02x", node.getUuid()[i]);
    usbCdc.start(uuid);

    printf("\nRemora CAN node, BTT EBB42\n");
    printf("UUID %s, SystemCoreClock %lu Hz\n", uuid, (unsigned long)SystemCoreClock);
    can.start();                            // prints the result
    startBaseThread(&node);
    printf("base thread %d Hz (step generators)\n", CAN_NODE_BASE_FREQ);

    uint32_t lastStatus = 0;

    while (1)
    {
        watchdog.kick();
        uint32_t now = HAL_GetTick();

        node.service(now);
        usbCdc.service();

        if (now - lastStatus >= NODE_STATUS_MS)
        {
            lastStatus = now;
            printf("node: UUID %s, number %d, %s, uptime %lu s\n", uuid, node.getNode(),
                   stateName(node.getState()), (unsigned long)(now / 1000));
            printf("node: %d module(s), %lu COMMAND frame(s), outputs 0x%04x, inputs 0x%04x\n",
                   node.getModuleCount(), (unsigned long)node.getCommandCount(),
                   node.getOutputs(), node.getInputs());
            for (int j = 0; j < node.getJointCount(); j++)
            {
                printf("node: joint %d %s, position %ld steps, %lu JOINT frame(s)\n", j,
                       node.getJointEnabled(j) ? "enabled" : "disabled", (long)node.getJointPosition(j),
                       (unsigned long)node.getJointFrames());
            }
            for (int p = 0; p < node.getPwmCount(); p++)
            {
                char name[6];
                printf("node: PWM %d %s, %u of %u ticks high, %lu PWM frame(s)\n", p, canPinName(node.getPwmPin(p), name),
                       node.getPwmOnTicks(p), node.getPwmPeriod(p), (unsigned long)node.getPwmFrames());
            }
            for (int a = 0; a < node.getAnalogCount(); a++)
            {
                char name[6];
                printf("node: analog %d %s, ADC %u\n", a, canPinName(node.getAnalogPin(a), name),
                       node.getAnalogValue(a));
            }
            for (int t = 0; t < node.getTmcCount(); t++)
            {
                NodeTmc2209& tmc = node.getTmc(t);
                char name[6];
                printf("node: TMC2209 %s %s, %lu configuration(s)\n", canPinName(tmc.getPin(), name),
                       tmcStatusText(tmc.getStatus()), (unsigned long)tmc.getConfigs());
            }
            can.printStatus();
        }
    }
}

#endif
