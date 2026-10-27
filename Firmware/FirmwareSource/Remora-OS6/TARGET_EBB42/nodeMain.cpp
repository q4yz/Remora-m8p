/***********************************************************************
    Remora CAN node firmware for the BTT EBB42 V1.1 / V1.2 (STM32G0B1CBT6)

    Phase 1: takes its node number from the hub and sends heartbeats
    (CAN_PROTOCOL.md). Extruder, heater, thermistor and fans follow with the
    configuration transfer and the process data in the next phases.

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
            can.printStatus();
        }
    }
}

#endif
