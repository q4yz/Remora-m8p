#ifndef REMORASPI_H
#define REMORASPI_H

#include "mbed.h"
#include "configuration.h"
#include "remora.h"

#include "stm32g0xx_hal.h"

#include "modules/module.h"

class RemoraComms : public Module
{
    private:

        SPI_TypeDef*        spiType;
        SPI_HandleTypeDef   spiHandle;
        DMA_HandleTypeDef   hdma_spi_tx;
        DMA_HandleTypeDef   hdma_spi_rx;

        volatile rxData_t*  ptrRxData;
        volatile txData_t*  ptrTxData;
        rxData_t            spiRxBuffer;

        uint8_t             noDataCount;
        uint8_t             rejectCnt;

        uint8_t             DMArxCnt;
        uint32_t            ticksStart;
        uint32_t            ticks;

        volatile bool       NSS;
        bool                resetSPI;

        volatile bool       data;
        bool                CommsStatus;
        bool                SPIdataError;

        rxData_t            uploadRxBuffer;     // copy of the last SD upload frame
        bool                uploadFrame;        // a new SD upload frame is waiting in uploadRxBuffer
        bool                uploadMode;         // SD upload in progress
        bool                initialised;        // init() ran once, re-inits (re-align) are silent

        PinName             interruptPin;
        InterruptIn         slaveSelect;

        void NSSinterrupt(void);

    public:

        RemoraComms(volatile rxData_t*, volatile txData_t*, SPI_TypeDef*, PinName);

        virtual void update(void);

        void init(void);
        void start(void);
        void SPItasks(void);
        bool getStatus(void);
        void setStatus(bool);
        bool getError(void);
        void setError(bool);

        bool getUploadFrame(uint8_t*);
        void setUploadMode(bool);
        void stopDMA(void);
        void restartDMA(void);

        // link statistics, printed by main() while the link is down (REMORA_DEBUG)
        volatile uint32_t   dbgFrames;          // NSS rising edges handled
        volatile uint32_t   dbgReads;           // good PRU_READ frames
        volatile uint32_t   dbgWrites;          // good PRU_WRITE frames
        volatile uint32_t   dbgRejects;         // frames with an unknown header
        volatile uint32_t   dbgTimeouts;        // DMA did not complete a frame in time
        volatile int32_t    dbgLastBad;         // header of the last rejected frame
};

#endif
