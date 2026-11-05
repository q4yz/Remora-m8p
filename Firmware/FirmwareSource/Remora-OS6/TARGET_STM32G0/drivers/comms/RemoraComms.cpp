#include "mbed.h"
#include "RemoraComms.h"

#include "stm32g0xx_hal.h"

/***********************************************************************
    SPI slave link to the Raspberry Pi for STM32G0 boards

    SPI2 with hardware NSS on PB12 (SCK PB13, MISO PB14, MOSI PB11), on the
    BTT Manta M8P these are the EXP2 header pins 4, 2, 1 and 6.

    Same scheme as the STM32F4 driver: both DMA channels run in circular mode
    over the 64 byte frame, the rising edge of NSS flags the end of a frame
    and SPItasks(), called from the main loop, checks the header and copies
    the data. On the G0 the DMA channels are connected to the SPI through
    the DMAMUX, so the requests have to be set explicitly.
************************************************************************/

RemoraComms::RemoraComms(volatile rxData_t* ptrRxData, volatile txData_t* ptrTxData, SPI_TypeDef* spiType, PinName interruptPin) :
    spiType(spiType),
    ptrRxData(ptrRxData),
    ptrTxData(ptrTxData),
    interruptPin(interruptPin),
    slaveSelect(interruptPin)
{
    this->spiHandle.Instance = this->spiType;
    this->noDataCount = 0;
    this->rejectCnt = 0;
    this->NSS = false;
    this->resetSPI = false;
    this->data = false;
    this->CommsStatus = false;
    this->SPIdataError = false;
    this->uploadFrame = false;
    this->uploadMode = false;
    this->initialised = false;
    this->dbgFrames = this->dbgReads = this->dbgWrites = this->dbgRejects = this->dbgTimeouts = 0;
    this->dbgLastBad = 0;
    this->dbgTorn = 0;
    this->dbgRearms = 0;
    this->lastEdgeTick = 0;
    this->rearmed = true;
    this->qHead = this->qTail = 0;
    this->dbgLost = 0;
    this->tornCnt = 0;

    if (this->spiType != SPI2 || this->interruptPin != PB_12)
    {
        printf("Error - STM32G0 Remora comms only supports SPI2 with NSS on PB_12\n");
    }

    // Cortex-M0+ has 4 priority levels, keep the NSS edge below the base thread
    slaveSelect.rise(callback(this, &RemoraComms::NSSinterrupt));
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
}


void RemoraComms::update()
{
    if (this->data)
    {
        this->noDataCount = 0;
        this->CommsStatus = true;
    }
    else
    {
        this->noDataCount++;
    }

    if (this->noDataCount > DATA_ERR_MAX)
    {
        this->noDataCount = 0;
        this->CommsStatus = false;
    }

    this->data = false;
}


void RemoraComms::init()
{
    if (this->spiHandle.Instance != SPI2) return;

    // also called by restartDMA to re-align, print only the first time
    bool verbose = !this->initialised;
    this->initialised = true;

    if (verbose) printf("Initialising SPI2 slave\n");

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB11     ------> SPI2_MOSI
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_SPI2_CLK_ENABLE();

    this->spiHandle.Init.Mode              = SPI_MODE_SLAVE;
    this->spiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    this->spiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    this->spiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    this->spiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    this->spiHandle.Init.NSS               = SPI_NSS_HARD_INPUT;
    this->spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    this->spiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    this->spiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    this->spiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    this->spiHandle.Init.CRCPolynomial     = 7;
    this->spiHandle.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    this->spiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

    HAL_SPI_Init(&this->spiHandle);

    if (verbose) printf("Initialising DMA for SPI2\n");

    __HAL_RCC_DMA1_CLK_ENABLE();

    this->hdma_spi_tx.Instance                 = DMA1_Channel3;
    this->hdma_spi_tx.Init.Request             = DMA_REQUEST_SPI2_TX;
    this->hdma_spi_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    this->hdma_spi_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    this->hdma_spi_tx.Init.MemInc              = DMA_MINC_ENABLE;
    this->hdma_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    this->hdma_spi_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    this->hdma_spi_tx.Init.Mode                = DMA_CIRCULAR;
    this->hdma_spi_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

    HAL_DMA_Init(&this->hdma_spi_tx);
    __HAL_LINKDMA(&this->spiHandle, hdmatx, this->hdma_spi_tx);

    this->hdma_spi_rx.Instance                 = DMA1_Channel2;
    this->hdma_spi_rx.Init.Request             = DMA_REQUEST_SPI2_RX;
    this->hdma_spi_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    this->hdma_spi_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    this->hdma_spi_rx.Init.MemInc              = DMA_MINC_ENABLE;
    this->hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    this->hdma_spi_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    this->hdma_spi_rx.Init.Mode                = DMA_CIRCULAR;
    this->hdma_spi_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

    HAL_DMA_Init(&this->hdma_spi_rx);
    __HAL_LINKDMA(&this->spiHandle, hdmarx, this->hdma_spi_rx);

    // the DMA runs without interrupts, the frame end is detected with the NSS edge
}


void RemoraComms::start()
{
    this->ptrTxData->header = PRU_DATA;
    HAL_SPI_TransmitReceive_DMA(&this->spiHandle, (uint8_t *)this->ptrTxData->txBuffer, (uint8_t *)this->spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);

    // HAL enables the DMA interrupts for its callbacks, they are not used and the NVIC lines stay off
    HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
}


void RemoraComms::NSSinterrupt()
{
    // NSS / CS has gone high, a frame is complete. The frame is copied here, not in the main loop:
    // the host may start the next frame right away (two frames per servo period, at a low SPI clock
    // there is hardly a gap between them) and the circular DMA then overwrites the buffer while the
    // main loop is still reading it - the torn READ / WRITE headers.

    // the last byte may still be on its way from the SPI FIFO to memory, give the DMA a moment
    uint32_t remaining = __HAL_DMA_GET_COUNTER(&this->hdma_spi_rx);
    for (int i = 0; i < 64 && remaining != SPI_BUFF_SIZE; i++)
    {
        remaining = __HAL_DMA_GET_COUNTER(&this->hdma_spi_rx);
    }

    this->lastEdgeTick = HAL_GetTick();
    this->rearmed = false;

    uint8_t slot = this->qHead;
    uint8_t next = (uint8_t)((slot + 1) % FRAME_QUEUE);
    if (next == this->qTail)
    {
        // the main loop is far behind: drop this frame, the ones queued stay intact
        this->dbgLost++;
        return;
    }

    bool torn = (remaining != SPI_BUFF_SIZE);

    if (!torn)
    {
        const volatile uint32_t* src = (const volatile uint32_t*)this->spiRxBuffer.rxBuffer;
        uint32_t* dst = (uint32_t*)this->frameQueue[slot].rxBuffer;
        for (int i = 0; i < SPI_BUFF_SIZE / 4; i++)
        {
            dst[i] = src[i];
        }
        // the next frame started during the copy: the copy may hold bytes of both frames
        torn = (__HAL_DMA_GET_COUNTER(&this->hdma_spi_rx) != SPI_BUFF_SIZE);
    }

    this->frameTorn[slot] = torn;
    this->qHead = next;                     // publish the slot after it is complete
}


void RemoraComms::SPItasks()
{
    // The reply (TX) side can lose its byte alignment without the receive side noticing: the
    // host then reads a header shifted by a byte or a few bits (00646174, bcbcbcbc, ...), drops
    // SPI-status and stops clocking, and every E-stop reset would meet the same shifted reply.
    // Whenever the bus has been quiet for IDLE_REARM_MS the SPI (bit counter, FIFOs) and both
    // DMA channels start over from byte 0, so the next frame is aligned in both directions.
    // LinuxCNC clocks every servo period (1 ms), this never runs while the link is up.
    if (this->qTail == this->qHead && !this->rearmed && (HAL_GetTick() - this->lastEdgeTick) > IDLE_REARM_MS)
    {
        this->rearmed = true;
        this->dbgRearms++;
        this->restartDMA();
    }

    while (this->qTail != this->qHead)
    {
        // the interrupt never writes a queued slot, it drops a frame when the queue is full
        uint8_t slot = this->qTail;
        bool torn = this->frameTorn[slot];
        rxData_t frame = this->frameQueue[slot];
        this->qTail = (uint8_t)((slot + 1) % FRAME_QUEUE);

        this->dbgFrames++;

        if (torn)
        {
            // the NSS edge came in the middle of a DMA frame: once in a while the main loop or a
            // higher priority interrupt was late, the frame is dropped. Edge after edge means the
            // DMA lost the frame boundary (started mid frame at boot, a byte missing), every
            // following frame would be shifted: re-align. Counted in dbgTorn / dbgTimeouts,
            // no printf (it blocks the main loop and costs more frames)
            this->dbgTorn++;
            if (++this->tornCnt >= EDGE_REALIGN)
            {
                this->tornCnt = 0;
                this->dbgTimeouts++;
                this->restartDMA();
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
                // READ so do nothing with the received data
                break;

            case PRU_WRITE:
                this->dbgWrites++;
                this->data = true;
                this->rejectCnt = 0;

                // we've got a good WRITE header, move the data to rxData
                // ensure an atomic access to the rxBuffer
                __disable_irq();

                for (int i = 0; i < SPI_BUFF_SIZE; i++)
                {
                    this->ptrRxData->rxBuffer[i] = frame.rxBuffer[i];
                }

                __enable_irq();
                break;

            case PRU_UPLOAD:
                this->rejectCnt = 0;
                // copy the frame now, before the next transfer overwrites the DMA buffer
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
                    this->SPIdataError = true;
                }
                // shifted frames when the host sends faster than the DMA timeout above (LinuxCNC:
                // every 1 ms), the next frame completes the count: re-align as well. Only when the
                // bad headers persist: a shifted DMA rejects every frame, while a single torn header
                // (READ and WRITE back to back) is harmless. Restarting the DMA mid traffic sends the
                // host a bad payload, remora-spi then drops SPI-status and the machine goes to E-stop
                if (this->rejectCnt >= REJECT_REALIGN)
                {
                    this->restartDMA();
                }
        }
    }
}


bool RemoraComms::getStatus(void)
{
    return this->CommsStatus;
}

void RemoraComms::setStatus(bool status)
{
    this->CommsStatus = status;
}

bool RemoraComms::getError(void)
{
    return this->SPIdataError;
}

void RemoraComms::setError(bool error)
{
    this->SPIdataError = error;
}

bool RemoraComms::getUploadFrame(uint8_t* frame)
{
    if (!this->uploadFrame) return false;

    for (int i = 0; i < SPI_BUFF_SIZE; i++)
    {
        frame[i] = this->uploadRxBuffer.rxBuffer[i];
    }
    this->uploadFrame = false;
    return true;
}

void RemoraComms::setUploadMode(bool mode)
{
    this->uploadMode = mode;
}

void RemoraComms::stopDMA()
{
    // the SD card has its own SPI bus here, pausing just keeps the frames in step with the main loop
    HAL_SPI_DMAStop(&this->spiHandle);
}

void RemoraComms::restartDMA()
{
    HAL_SPI_DMAStop(&this->spiHandle);

    // wait for the master to finish any transfer in progress so the next frame starts on a boundary
    uint32_t start = HAL_GetTick();
    while (!this->slaveSelect.read() && (HAL_GetTick() - start) < 10) {}

    // reset the SPI peripheral to empty its FIFOs and clear the overrun flag
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();

    // re-initialise the SPI and DMA handles without resetting the TX buffer header like start() does
    this->init();
    this->qTail = this->qHead;              // frames queued before the re-align are stale
    this->rearmed = true;
    HAL_SPI_TransmitReceive_DMA(&this->spiHandle, (uint8_t *)this->ptrTxData->txBuffer, (uint8_t *)this->spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);
    HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
}
