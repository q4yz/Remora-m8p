/***********************************************************************
    Remora CAN node firmware for the BTT EBB42 V1.1 / V1.2 (STM32G0B1CBT6)

    Phase 1: takes its node number from the hub and sends heartbeats
    (CAN_PROTOCOL.md). Extruder, heater, thermistor and fans follow with the
    configuration transfer and the process data in the next phases.

    EBB42: CAN on PB0 (RX) / PB1 (TX) = FDCAN2, 8 MHz crystal, no bootloader,
    flashed over USB in DFU mode at 0x08000000, 120R termination by jumper.
************************************************************************/

#include "mbed.h"
#include "configuration.h"

#ifdef REMORA_CAN_NODE

#include "CanNode.h"

#define NODE_CAN_BITRATE    1000000         // fixed in phase 1, must match "CAN" "Bitrate" on the hub

int main()
{
    Watchdog& watchdog = Watchdog::get_instance();
    watchdog.start(2000);

    RemoraCAN can(FDCAN2, GPIOB, 0, 1, GPIO_AF3_FDCAN2, NODE_CAN_BITRATE);
    can.start();

    CanNode node(&can);

    while (1)
    {
        watchdog.kick();
        node.service(HAL_GetTick());
    }
}

#endif
