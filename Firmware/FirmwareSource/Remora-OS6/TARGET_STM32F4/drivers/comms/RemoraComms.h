#ifndef REMORASPI_H
#define REMORASPI_H

#include "mbed.h"
#include "configuration.h"
#include "remora.h"

#include "stm32f4xx_hal.h"

#include "modules/module.h"
#include "drivers/remoraLink/RemoraLink.h"

/***********************************************************************
    SPI slave link to the host for STM32F4 boards: the hardware side.

    The frame handling (commands to rxData in the NSS interrupt, queue to
    the main loop, torn frames, re-align, link status, statistics) is
    RemoraLink, shared by every MCU family; this class is the SPI / DMA /
    NSS pin part of it (LinkHardware) and the servo thread module that
    updates the link status.

    SPI1 (PA5 SCK, PA6 MISO, PA7 MOSI, DMA2 streams 0 / 3) or SPI2 (PB13,
    PB14, PB15, DMA1 streams 3 / 4). The slave select is the SPI's NSS pin
    (PA4 / PB12, hardware NSS) or, on boards that share the SPI with the
    SD card, another pin (software NSS, the SPI always selected).
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
        bool                sharedSPI;          // software NSS: the slave select is not the SPI's NSS pin

        PinName             interruptPin;
        InterruptIn         slaveSelect;

        void NSSinterrupt(void);
        void startTransfer(void);
        void spiReset(void);
        static void streamOff(DMA_HandleTypeDef* h);

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
