#ifndef REMORACAN_H
#define REMORACAN_H

#include "mbed.h"
#include "stm32f4xx_hal.h"

#include "CanBus.h"

/***********************************************************************
    Classic CAN on the STM32F4 bxCAN peripheral (CAN1 / CAN2), polled
    (no interrupts, nothing may disturb the base thread), register level.
    Accepts every standard and extended data frame into RX FIFO 0 (one
    filter bank, mask 0), sends through the three TX mailboxes in the
    order queued. A bus-off is left automatically (ABOM).

    Clock: PCLK1 (F446 at 180 MHz: 45 MHz, F429 at 168 MHz: 42 MHz). The
    bit timing is searched at start(): 8 ... 25 time quanta per bit,
    sample point near 87.5 %; 1 Mbit/s, 500, 250 and 125 kbit/s work on
    both.
************************************************************************/

class RemoraCAN : public CanBus
{
    private:

        CAN_TypeDef*        can;
        uint32_t            bitrate;
        bool                running;
        bool                wasBusOff;
        uint32_t            busOffCount;
        uint32_t            txCount;            // frames queued
        uint32_t            txDropped;          // all mailboxes full (nobody acknowledges)
        uint32_t            rxCount;
        uint32_t            lastError;          // last protocol error code (LEC), 0 = none

        bool bitTiming(uint32_t* btr);

    public:

        // instance CAN1 or CAN2; rx/tx: GPIO port, pin number and alternate function
        RemoraCAN(CAN_TypeDef* instance, GPIO_TypeDef* port, uint16_t rxPin, uint16_t txPin,
                  uint8_t alternate, uint32_t bitrate);

        bool start(void) override;
        bool send(const canFrame_t& frame) override;
        bool receive(canFrame_t& frame) override;
        void service(void) override;            // statistics: bus-off, last error

        uint16_t getErrorCount(void) override;  // transmit + receive error counters
        void printStatus(void) override;        // one line for the debug console
};

#endif
