#ifndef CANBUS_H
#define CANBUS_H

/***********************************************************************
    A classic CAN controller, as the CAN hub / node logic (CanHub,
    CanNode, this folder) uses it; no MCU dependencies. A board family
    implements it on its peripheral (STM32G0: RemoraCAN on FDCAN, polled),
    the PC tests with a simulated bus (Firmware/Tests/can/stubs).
************************************************************************/

#include <stdint.h>

typedef struct
{
    uint32_t id;
    bool     extended;
    uint8_t  len;
    uint8_t  data[8];
} canFrame_t;

class CanBus
{
    public:

        virtual bool start(void) = 0;
        virtual bool send(const canFrame_t& frame) = 0;     // false: TX queue full
        virtual bool receive(canFrame_t& frame) = 0;        // false: nothing received
        virtual void service(void) = 0;                     // call regularly: e.g. restarts after bus-off
        virtual uint16_t getErrorCount(void) = 0;           // transmit + receive error counters
        virtual void printStatus(void) = 0;                 // one line for the debug console
};

#endif
