#ifndef REMORALINK_H
#define REMORALINK_H

/***********************************************************************
    Remora SPI link logic, independent of the MCU

    The frame handling every SPI slave driver needs, apart from the
    registers: the NSS interrupt takes each finished 64 byte frame,
    moves WRITE data to rxData at once (the commands never wait for the
    main loop) and hands the frame to a small queue; the main loop takes
    it from there for the statistics, bad headers and the SD upload.
    Frames whose NSS edge did not land on a DMA frame boundary ("torn")
    are dropped, and when the frames stay torn or keep a bad header the
    link is re-aligned. While the bus is
    quiet the SPI is re-armed, so the next frame starts at byte 0 in
    both directions.

    The hardware is behind LinkHardware: a board family implements
    those few calls (SPI, DMA, NSS pin, millisecond tick), everything
    here stays the same, and Firmware/Tests/link runs it on a PC.
************************************************************************/

#include <cstdint>

#include "configuration.h"
#include "remora.h"

#define REJECT_REALIGN      16      // consecutive bad headers before the link is re-aligned
#define EDGE_REALIGN        3       // consecutive torn frames before the link is re-aligned
#define IDLE_REARM_MS       20      // bus quiet this long: SPI and DMA are re-armed for the next frame
#define FRAME_QUEUE         4       // frames the NSS interrupt can hold for tasks() (one slot stays free)
#define DMA_SETTLE_READS    64      // reads of the DMA counter while the last byte reaches memory


class LinkHardware
{
    public:

        // bytes the RX DMA still has to receive in the current frame (SPI_BUFF_SIZE: frame complete)
        virtual uint32_t dmaRemaining(void) = 0;
        // NSS / chip select is high: the bus is idle, no frame in progress
        virtual bool nssHigh(void) = 0;
        // bus idle: SPI and both DMA channels start over at byte 0 (FIFOs emptied), a few us,
        // called from the NSS interrupt
        virtual void rearm(void) = 0;
        // full restart from the main loop: stop, wait for the bus to go idle, set SPI and DMA up again
        virtual void restart(void) = 0;
        // stop the DMA (SD upload), restart() resumes
        virtual void stop(void) = 0;
        virtual uint32_t millis(void) = 0;
};


class RemoraLink
{
    private:

        LinkHardware&       hw;
        volatile rxData_t*  ptrRxData;
        const volatile uint8_t* dmaRxBuffer;    // the RX DMA target, rewritten by every transfer

        // frames copied in the NSS interrupt, taken by tasks(): with two frames per servo period
        // (READ and WRITE back to back) a single slot lost the READ when the main loop was slow
        rxData_t            isrFrame;           // the frame being taken, the interrupt's
        rxData_t            frameQueue[FRAME_QUEUE];
        volatile bool       frameTorn[FRAME_QUEUE];
        volatile uint8_t    qHead;              // next slot the interrupt fills
        volatile uint8_t    qTail;              // next slot tasks() takes
        volatile uint32_t   lastEdgeTick;       // millis() of the last NSS edge
        volatile bool       rearmed;            // re-armed since the last NSS edge
        volatile bool       restarting;         // stop() / restart(): the interrupt leaves the SPI alone
        uint8_t             tornCnt;
        uint8_t             rejectCnt;
        uint8_t             noDataCount;

        volatile bool       data;               // a good frame since the last servoUpdate()
        volatile bool       status;
        bool                dataError;

        rxData_t            uploadRxBuffer;     // copy of the last SD upload frame
        bool                uploadFrame;        // a new SD upload frame is waiting in uploadRxBuffer

        void realign(void);

    public:

        RemoraLink(LinkHardware& hw, volatile rxData_t* ptrRxData, const volatile uint8_t* dmaRxBuffer);

        void frameEnd(void);        // NSS rising edge interrupt
        void tasks(void);           // main loop
        void servoUpdate(void);     // servo thread: link status from the frames since the last call

        bool getStatus(void) const { return this->status; }
        void setStatus(bool s) { this->status = s; }
        bool getError(void) const { return this->dataError; }
        void setError(bool e) { this->dataError = e; }
        bool getUploadFrame(uint8_t* frame);

        void stopLink(void);        // SD upload: stop the DMA, the interrupt queues nothing
        void restartLink(void);     // resume after stopLink(), or re-align

        // link statistics, printed by main() while the link is down (REMORA_DEBUG)
        volatile uint32_t   dbgFrames;          // frames taken from the queue
        volatile uint32_t   dbgReads;           // good PRU_READ frames
        volatile uint32_t   dbgWrites;          // good PRU_WRITE frames
        volatile uint32_t   dbgRejects;         // frames with an unknown header
        volatile uint32_t   dbgTimeouts;        // re-aligns after torn frames
        volatile int32_t    dbgLastBad;         // header of the last rejected frame
        volatile uint32_t   dbgRearms;          // re-arms after a quiet bus
        volatile uint32_t   dbgLost;            // frames the main loop did not see: the queue was full (their data was used)
        volatile uint32_t   dbgLate;            // frame ends where the next frame had already started (no re-arm)
        volatile uint32_t   dbgTorn;            // frames dropped: NSS edge off the frame boundary or
                                                // the next frame already started during the copy
};

#endif
