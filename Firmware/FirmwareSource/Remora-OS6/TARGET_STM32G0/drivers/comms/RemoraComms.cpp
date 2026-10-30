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
    // NSS / CS has gone high, packet received
    this->NSS = true;
}


void RemoraComms::SPItasks()
{
    if (this->NSS)
    {
        this->NSS = false;
        this->dbgFrames++;

        this->DMArxCnt = 0;
        this->ticksStart = HAL_GetTick();

        // wait for DMA to complete and break if DMA is not complete in time
        while (this->DMArxCnt != SPI_BUFF_SIZE)
        {
            this->DMArxCnt = __HAL_DMA_GET_COUNTER(&this->hdma_spi_rx);
            this->ticks = HAL_GetTick() - this->ticksStart;

            if (this->ticks > 2)
            {
                this->resetSPI = true;
                this->dbgTimeouts++;
                break;
            }
        }

        if (this->resetSPI)
        {
            // the frame boundary is lost: the DMA was started in the middle of a frame (the host
            // was already clocking when the board booted) or a byte went missing. The DMA is
            // circular, every following frame would be shifted, so drop this one and re-align on
            // the next NSS edge, in normal operation as in upload mode. Counted in dbgTimeouts,
            // no printf (it blocks the main loop and costs more frames)
            // While LinuxCNC is running (CommsStatus) a restart would send it a bad payload and E-stop
            // the machine, there a shifted DMA shows up as bad headers and is handled below
            this->resetSPI = false;
            if (this->uploadMode || !this->CommsStatus)
            {
                this->restartDMA();
            }
            return;
        }

        switch (this->spiRxBuffer.header)
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
                    this->ptrRxData->rxBuffer[i] = this->spiRxBuffer.rxBuffer[i];
                }

                __enable_irq();
                break;

            case PRU_UPLOAD:
                this->rejectCnt = 0;
                // copy the frame now, before the next transfer overwrites the DMA buffer
                for (int i = 0; i < SPI_BUFF_SIZE; i++)
                {
                    this->uploadRxBuffer.rxBuffer[i] = this->spiRxBuffer.rxBuffer[i];
                }
                this->uploadFrame = true;
                break;

            default:
                this->rejectCnt++;
                this->dbgRejects++;
                this->dbgLastBad = this->spiRxBuffer.header;
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
    this->NSS = false;
    HAL_SPI_TransmitReceive_DMA(&this->spiHandle, (uint8_t *)this->ptrTxData->txBuffer, (uint8_t *)this->spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);
    HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
}
