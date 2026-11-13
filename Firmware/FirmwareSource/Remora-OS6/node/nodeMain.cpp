/***********************************************************************
    Remora CAN node firmware (targets with the macro REMORA_CAN_NODE:
    BTT EBB42, BTT EBB SB2209 CAN)

    Takes its node number and its modules (digital pins, step generators,
    TMC2209 drivers, thermistors, PWM, RGB chains) from the hub and
    exchanges the process data with it (CAN_PROTOCOL.md, CanNode). The
    step pulses and the PWM are made in the tick timer's interrupt at
    CAN_NODE_BASE_FREQ (CanNode::baseTick).

    Nothing here knows the board: its name and CAN pins are in board.h,
    the CAN controller, the tick timer, the GPIO and the console are its
    family's (PORTING.md). Debug output (printf) goes to the USB port as
    a serial port, the same cable as for flashing: open the COM port /
    /dev/ttyACM0 with any terminal, any baud rate.
************************************************************************/

#include "mbed.h"
#include "configuration.h"

// NODE_SERIAL_TEST: the board's serialTestMain.cpp has the main() of the USB serial test firmware
#if defined(REMORA_CAN_NODE) && !defined(NODE_SERIAL_TEST)

#include "CanNode.h"
#include "tickTimer.h"
#include "nodeConsole.h"

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

static void baseTick()
{
    baseNode->baseTick();
}

int main()
{
    Watchdog& watchdog = Watchdog::get_instance();
    watchdog.start(2000);

    // static: in RAM (.bss), not on the stack. Bare metal has one small stack for main() and every
    // interrupt (and newlib's printf); with the RGB chain buffers CanNode grew past 1 KB, main()'s
    // frame to 1.4 KB, and the node no longer started (no USB console, no CAN ACK). Function-local
    // statics are still constructed here, after the mbed / HAL start-up, on the first pass.
    CanBus* can = canBusCreate(NODE_CAN_BITRATE);   // the board's controller (board.h: BOARD_CAN)
    static CanNode node(can);

    // USB serial number = node UUID, also visible in the Windows device manager / lsusb -v
    char uuid[13];
    for (int i = 0; i < 6; i++) snprintf(&uuid[2 * i], 3, "%02x", node.getUuid()[i]);
    nodeConsoleStart(uuid);

    printf("\nRemora CAN node, %s\n", BOARD_NAME);
    printf("UUID %s, SystemCoreClock %lu Hz\n", uuid, (unsigned long)SystemCoreClock);
    can->start();                           // prints the result
    baseNode = &node;
    if (tickTimerStart(CAN_NODE_BASE_FREQ, baseTick)) tickTimerRun(true);
    printf("base thread %d Hz (step generators)\n", CAN_NODE_BASE_FREQ);

    uint32_t lastStatus = 0;

    while (1)
    {
        watchdog.kick();
        uint32_t now = remoraMillis();

        node.service(now);
        nodeConsoleService();

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
            can->printStatus();
        }
    }
}

#endif
