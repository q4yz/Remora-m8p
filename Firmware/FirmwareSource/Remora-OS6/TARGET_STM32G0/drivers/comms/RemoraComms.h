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
#define FRAME_QUEUE         4       // frames the NSS interrupt can hold for SPItasks() (one slot stays free)

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
        // frames copied in the NSS interrupt, taken by SPItasks(): READ and WRITE arrive back to back,
        // a single slot lost the READ whenever the main loop was slower than the gap between them
        rxData_t            frameQueue[FRAME_QUEUE];
        volatile bool       frameTorn[FRAME_QUEUE]; // the NSS edge did not land on a DMA frame boundary
        volatile uint8_t    qHead;              // next slot the interrupt fills
        volatile uint8_t    qTail;              // next slot SPItasks() takes
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
        volatile uint32_t   dbgLost;            // frames dropped because the queue was full
        volatile uint32_t   dbgTorn;            // frames dropped: NSS edge off the frame boundary or
                                                // the next frame already started during the copy
};

#endif
