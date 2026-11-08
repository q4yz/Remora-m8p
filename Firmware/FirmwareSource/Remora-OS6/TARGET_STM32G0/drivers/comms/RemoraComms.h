#ifndef REMORASPI_H
#define REMORASPI_H

#include "mbed.h"
#include "configuration.h"
#include "remora.h"

#include "stm32g0xx_hal.h"

#include "modules/module.h"
#include "drivers/remoraLink/RemoraLink.h"

/***********************************************************************
    SPI slave link to the host for STM32G0 boards: the hardware side.

    The frame handling (queue, torn frames, re-align, link status,
    statistics) is RemoraLink, shared by every MCU family; this class is
    the SPI2 / DMA1 / NSS pin part of it (LinkHardware) and the servo
    thread module that updates the link status.
************************************************************************/

class RemoraComms : public Module, public LinkHardware, public RemoraLink
{
    private:

        SPI_TypeDef*        spiType;
        SPI_HandleTypeDef   spiHandle;
        DMA_HandleTypeDef   hdma_spi_tx;
        DMA_HandleTypeDef   hdma_spi_rx;

        volatile txData_t*  ptrTxData;
        rxData_t            spiRxBuffer;        // DMA target, rewritten by every transfer

        bool                uploadMode;         // SD upload in progress
        bool                initialised;        // init() ran once, re-inits (re-align) are silent

        PinName             interruptPin;
        InterruptIn         slaveSelect;

        void NSSinterrupt(void);
        void startTransfer(void);

    public:

        RemoraComms(volatile rxData_t*, volatile txData_t*, SPI_TypeDef*, PinName);

        virtual void update(void);      // servo thread

        void init(void);
        void start(void);
        void SPItasks(void) { this->tasks(); }
        void setUploadMode(bool mode) { this->uploadMode = mode; }
        void stopDMA(void) { this->stopLink(); }
        void restartDMA(void) { this->restartLink(); }

        // LinkHardware
        virtual uint32_t dmaRemaining(void);
        virtual bool nssHigh(void);
        virtual void rearm(void);
        virtual void restart(void);
        virtual void stop(void);
        virtual uint32_t millis(void);
};

#endif
