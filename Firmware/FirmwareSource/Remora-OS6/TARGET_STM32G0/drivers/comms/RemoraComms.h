#ifndef REMORASPI_H
#define REMORASPI_H

#include "mbed.h"
#include "configuration.h"
#include "remora.h"

#include "stm32g0xx_hal.h"

#include "modules/module.h"

#define REJECT_REALIGN      16      // consecutive bad headers before the DMA is re-aligned
#define EDGE_REALIGN        3       // consecutive NSS edges in the middle of a DMA frame before re-aligning
#define IDLE_REARM_MS       20      // bus quiet this long: SPI and DMA are re-armed for the next frame

class RemoraComms : public Module
{
    private:

        SPI_TypeDef*        spiType;
        SPI_HandleTypeDef   spiHandle;
        DMA_HandleTypeDef   hdma_spi_tx;
        DMA_HandleTypeDef   hdma_spi_rx;

        volatile rxData_t*  ptrRxData;
        volatile txData_t*  ptrTxData;
        rxData_t            spiRxBuffer;        // DMA target, rewritten by every transfer
        rxData_t            frameBuffer;        // copy taken in the NSS interrupt, read by SPItasks()
        volatile bool       frameTorn;          // the NSS edge did not land on a DMA frame boundary
        volatile uint32_t   lastEdgeTick;       // HAL tick of the last NSS edge
        volatile bool       rearmed;            // re-armed since the last NSS edge
        uint8_t             tornCnt;

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
        volatile uint32_t   dbgRearms;          // re-arms after a quiet bus
        volatile uint32_t   dbgTorn;            // frames dropped: NSS edge off the frame boundary or
                                                // the next frame already started during the copy
};

#endif
