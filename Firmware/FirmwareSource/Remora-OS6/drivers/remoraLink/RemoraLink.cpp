#include "RemoraLink.h"

#if defined(__arm__)
// Cortex-M, as __disable_irq() / __enable_irq() (this file stays free of the MCU headers)
#define LINK_IRQ_OFF()  __asm volatile ("cpsid i" ::: "memory")
#define LINK_IRQ_ON()   __asm volatile ("cpsie i" ::: "memory")
#else
#define LINK_IRQ_OFF()
#define LINK_IRQ_ON()
#endif


RemoraLink::RemoraLink(LinkHardware& hw, volatile rxData_t* ptrRxData, const volatile uint8_t* dmaRxBuffer) :
    hw(hw),
    ptrRxData(ptrRxData),
    dmaRxBuffer(dmaRxBuffer),
    qHead(0),
    qTail(0),
    lastEdgeTick(0),
    rearmed(true),
    restarting(false),
    tornCnt(0),
    rejectCnt(0),
    noDataCount(0),
    data(false),
    status(false),
    dataError(false),
    uploadFrame(false),
    dbgFrames(0), dbgReads(0), dbgWrites(0), dbgRejects(0), dbgTimeouts(0), dbgLastBad(0),
    dbgRearms(0), dbgLost(0), dbgLate(0), dbgTorn(0)
{
}


void RemoraLink::frameEnd()
{
    // NSS / CS has gone high, a frame is complete. The frame is copied here, not in the main loop:
    // the host may start the next frame right away and the circular DMA then overwrites the
    // buffer while the main loop is still reading it.

    this->lastEdgeTick = this->hw.millis();
    this->rearmed = false;

    // stop() / restart(): the DMA buffer is not a frame, and the SPI is the main loop's
    if (this->restarting) return;

    // the last byte may still be on its way from the SPI FIFO to memory, give the DMA a moment
    uint32_t remaining = this->hw.dmaRemaining();
    for (int i = 0; i < DMA_SETTLE_READS && remaining != SPI_BUFF_SIZE; i++)
    {
        remaining = this->hw.dmaRemaining();
    }

    uint8_t slot = this->qHead;
    uint8_t next = (uint8_t)((slot + 1) % FRAME_QUEUE);
    if (next == this->qTail)
    {
        // the main loop is far behind: drop this frame, the ones queued stay intact
        this->dbgLost++;
        if (this->hw.nssHigh()) this->hw.rearm();
        return;
    }

    bool torn = (remaining != SPI_BUFF_SIZE);

    if (!torn)
    {
        const volatile uint32_t* src = (const volatile uint32_t*)this->dmaRxBuffer;
        uint32_t* dst = (uint32_t*)this->frameQueue[slot].rxBuffer;
        for (int i = 0; i < SPI_BUFF_SIZE / 4; i++)
        {
            dst[i] = src[i];
        }
        // the next frame started during the copy: the copy may hold bytes of both frames
        torn = (this->hw.dmaRemaining() != SPI_BUFF_SIZE);
    }

    this->frameTorn[slot] = torn;
    this->qHead = next;                     // publish the slot after it is complete

    // re-arm for the next frame, unless it has already started (NSS low again): then the circular
    // DMA simply carries on, as without the re-arm
    if (this->hw.nssHigh()) this->hw.rearm();
    else this->dbgLate++;
}


void RemoraLink::realign()
{
    this->restarting = true;
    this->hw.restart();
    this->qTail = this->qHead;              // frames queued before the re-align are stale
    this->rearmed = true;
    this->restarting = false;
}


void RemoraLink::tasks()
{
    // The reply (TX) side can lose its byte alignment without the receive side noticing: the
    // host then reads a header shifted by a byte or a few bits (00646174, bcbcbcbc, ...), drops
    // SPI-status and stops clocking, and every E-stop reset would meet the same shifted reply.
    // Whenever the bus has been quiet for IDLE_REARM_MS the SPI (bit counter, FIFOs) and both
    // DMA channels start over from byte 0, so the next frame is aligned in both directions.
    // LinuxCNC clocks every servo period (1 ms), this never runs while the link is up.
    if (this->qTail == this->qHead && !this->rearmed && (this->hw.millis() - this->lastEdgeTick) > IDLE_REARM_MS)
    {
        this->dbgRearms++;
        this->realign();
    }

    while (this->qTail != this->qHead)
    {
        // the interrupt never writes a queued slot, it drops a frame when the queue is full
        uint8_t slot = this->qTail;
        bool torn = this->frameTorn[slot];
        const rxData_t& frame = this->frameQueue[slot];

        this->dbgFrames++;

        if (torn)
        {
            // the NSS edge came in the middle of a DMA frame: once in a while a higher priority
            // interrupt was late, the frame is dropped. Edge after edge means the DMA lost the
            // frame boundary (started mid frame at boot, a byte missing), every following frame
            // would be shifted: re-align. No printf here, it blocks the main loop
            this->qTail = (uint8_t)((slot + 1) % FRAME_QUEUE);
            this->dbgTorn++;
            if (++this->tornCnt >= EDGE_REALIGN)
            {
                this->tornCnt = 0;
                this->dbgTimeouts++;
                this->realign();
            }
            continue;
        }
        this->tornCnt = 0;

        switch (frame.header)
        {
            case PRU_READ:
                this->dbgReads++;
                this->data = true;
                this->rejectCnt = 0;
                break;

            case PRU_WRITE:
            {
                this->dbgWrites++;
                this->data = true;
                this->rejectCnt = 0;

                // a whole frame for the threads: they never see half of it (16 word copies)
                const uint32_t* src = (const uint32_t*)frame.rxBuffer;
                volatile uint32_t* dst = (volatile uint32_t*)this->ptrRxData->rxBuffer;
                LINK_IRQ_OFF();
                for (int i = 0; i < SPI_BUFF_SIZE / 4; i++)
                {
                    dst[i] = src[i];
                }
                LINK_IRQ_ON();
                break;
            }

            case PRU_UPLOAD:
                this->rejectCnt = 0;
                for (int i = 0; i < SPI_BUFF_SIZE; i++)
                {
                    this->uploadRxBuffer.rxBuffer[i] = frame.rxBuffer[i];
                }
                this->uploadFrame = true;
                break;

            default:
                this->rejectCnt++;
                this->dbgRejects++;
                this->dbgLastBad = frame.header;
                if (this->rejectCnt > 5)
                {
                    this->dataError = true;
                }
                // a shifted DMA rejects every frame, a single bad header is harmless: re-align only
                // when they persist. A re-align in the middle of traffic costs the host a bad reply
                if (this->rejectCnt >= REJECT_REALIGN)
                {
                    this->rejectCnt = 0;
                    this->realign();
                }
        }

        // the slot is free only now: the interrupt may fill it while the frame is read above
        if (this->qTail == slot) this->qTail = (uint8_t)((slot + 1) % FRAME_QUEUE);
    }
}


void RemoraLink::servoUpdate()
{
    if (this->data)
    {
        this->noDataCount = 0;
        this->status = true;
    }
    else
    {
        this->noDataCount++;
    }

    if (this->noDataCount > DATA_ERR_MAX)
    {
        this->noDataCount = 0;
        this->status = false;
    }

    this->data = false;
}


bool RemoraLink::getUploadFrame(uint8_t* frame)
{
    if (!this->uploadFrame) return false;

    for (int i = 0; i < SPI_BUFF_SIZE; i++)
    {
        frame[i] = this->uploadRxBuffer.rxBuffer[i];
    }
    this->uploadFrame = false;
    return true;
}


void RemoraLink::stopLink()
{
    // restartLink() resumes; until then the NSS interrupt must not touch the SPI
    this->restarting = true;
    this->hw.stop();
}


void RemoraLink::restartLink()
{
    this->realign();
}
