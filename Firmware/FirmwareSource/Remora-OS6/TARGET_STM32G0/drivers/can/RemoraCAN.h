#ifndef REMORACAN_H
#define REMORACAN_H

#include "mbed.h"
#include "stm32g0xx_hal.h"

#include "CanBus.h"

/***********************************************************************
    Classic CAN on the STM32G0 FDCAN peripheral, polled (no interrupts,
    nothing may disturb the base thread). Accepts every standard and extended
    data frame into RX FIFO 0, sends through the TX FIFO.

    Kernel clock: PCLK (64 MHz). Bit timing: 16 time quanta per bit,
    sample point 87.5 %, prescaler = 64 MHz / (16 * bitrate):
    1 Mbit/s, 500, 250 and 125 kbit/s.
************************************************************************/

class RemoraCAN : public CanBus
{
    private:

        FDCAN_HandleTypeDef hfdcan;
        uint32_t            bitrate;
        bool                running;
        uint32_t            busOffCount;
        uint32_t            txCount;            // frames queued
        uint32_t            txDropped;          // TX FIFO full (nobody acknowledges)
        uint32_t            rxCount;
        uint32_t            lastError;          // last protocol error code (LEC), 0 = none

    public:

        // instance FDCAN1 or FDCAN2; rx/tx: GPIO port, pin number and alternate function
        RemoraCAN(FDCAN_GlobalTypeDef* instance, GPIO_TypeDef* port, uint16_t rxPin, uint16_t txPin,
                  uint8_t alternate, uint32_t bitrate);

        bool start(void) override;
        bool send(const canFrame_t& frame) override;
        bool receive(canFrame_t& frame) override;
        void service(void) override;        // call regularly: restarts after bus-off

        bool isRunning(void) { return this->running; }
        uint32_t getBusOffCount(void) { return this->busOffCount; }
        uint16_t getErrorCount(void) override;  // transmit + receive error counters
        void printStatus(void) override;        // one line for the debug console
};

#endif
