#include "mbed.h"
#include "RemoraComms.h"

#include "stm32f4xx_hal.h"

/***********************************************************************
    SPI slave link to the host for STM32F4 boards, the hardware side of
    RemoraLink (drivers/remoraLink: commands to rxData in the NSS
    interrupt, frame queue, torn frames, re-align, link status).

    Both DMA streams run in circular mode over the 64 byte frame without
    interrupts, the rising edge of the slave select marks the end of a
    frame. On the SDIO boards (Octopus, SKR 2) the SD card's DMA shares
    DMA2 stream 3 with the SPI1 TX: the SD upload stops the link
    (stop()) and restarts it (restart()) around every card access.
************************************************************************/

RemoraComms::RemoraComms(volatile rxData_t* ptrRxData, volatile txData_t* ptrTxData, SPI_TypeDef* spiType, PinName interruptPin) :
    RemoraLink(*this, ptrRxData, spiRxBuffer.rxBuffer),
    spiType(spiType),
    ptrTxData(ptrTxData),
    uploadMode(false),
    initialised(false),
    sharedSPI(false),
    interruptPin(interruptPin),
    slaveSelect(interruptPin)
{
    this->spiHandle.Instance = this->spiType;

    // the slave select is the SPI's own NSS pin, or another one when the SPI is shared with the SD card
    this->sharedSPI = !((spiType == SPI1 && interruptPin == PA_4) || (spiType == SPI2 && interruptPin == PB_12));

    // The NSS edge at the highest priority, above the threads (base 2, servo 3): the frame end copies
    // LinuxCNC's commands to rxData and re-arms the SPI before the next frame; held back by a thread
    // it would miss the gap to the next frame. A few us: a 64 byte copy and the re-arm.
    slaveSelect.rise(callback(this, &RemoraComms::NSSinterrupt));
    IRQn_Type irq;
    switch (STM_PIN(interruptPin))
    {
        case 0: irq = EXTI0_IRQn; break;
        case 1: irq = EXTI1_IRQn; break;
        case 2: irq = EXTI2_IRQn; break;
        case 3: irq = EXTI3_IRQn; break;
        case 4: irq = EXTI4_IRQn; break;
        case 5: case 6: case 7: case 8: case 9: irq = EXTI9_5_IRQn; break;
        default: irq = EXTI15_10_IRQn; break;
    }
    HAL_NVIC_SetPriority(irq, 0, 0);
}


void RemoraComms::update()
{
    this->servoUpdate();
}


void RemoraComms::NSSinterrupt()
{
    this->frameEnd();
}


void RemoraComms::init()
{
    if (this->spiHandle.Instance != SPI1 && this->spiHandle.Instance != SPI2)
    {
        printf("Error - STM32F4 Remora comms supports SPI1 and SPI2 only\n");
        return;
    }
    bool spi1 = this->spiHandle.Instance == SPI1;

    // also called by restart() to re-align, print only the first time
    bool verbose = !this->initialised;
    this->initialised = true;

    if (verbose) printf("Initialising SPI%d slave%s\n", spi1 ? 1 : 2, this->sharedSPI ? ", shared with the SD card" : "");

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if (spi1)
    {
        // PA4 NSS (unless shared), PA5 SCK, PA6 MISO, PA7 MOSI
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | (this->sharedSPI ? 0 : GPIO_PIN_4);
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        __HAL_RCC_SPI1_CLK_ENABLE();
    }
    else
    {
        // PB12 NSS (unless shared), PB13 SCK, PB14 MISO, PB15 MOSI
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | (this->sharedSPI ? 0 : GPIO_PIN_12);
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        __HAL_RCC_SPI2_CLK_ENABLE();
    }

    this->spiHandle.Init.Mode           = SPI_MODE_SLAVE;
    this->spiHandle.Init.Direction      = SPI_DIRECTION_2LINES;
    this->spiHandle.Init.DataSize       = SPI_DATASIZE_8BIT;
    this->spiHandle.Init.CLKPolarity    = SPI_POLARITY_LOW;
    this->spiHandle.Init.CLKPhase       = SPI_PHASE_1EDGE;
    this->spiHandle.Init.NSS            = this->sharedSPI ? SPI_NSS_SOFT : SPI_NSS_HARD_INPUT;
    this->spiHandle.Init.FirstBit       = SPI_FIRSTBIT_MSB;
    this->spiHandle.Init.TIMode         = SPI_TIMODE_DISABLE;
    this->spiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    this->spiHandle.Init.CRCPolynomial  = 10;
    HAL_SPI_Init(&this->spiHandle);

    // software NSS: SSI low, as NSS low - always selected, the frame end comes from the other pin
    if (this->sharedSPI) CLEAR_BIT(this->spiHandle.Instance->CR1, SPI_CR1_SSI);

    if (verbose) printf("Initialising DMA for SPI%d\n", spi1 ? 1 : 2);

    if (spi1)
    {
        __HAL_RCC_DMA2_CLK_ENABLE();
        this->hdma_spi_tx.Instance      = DMA2_Stream3;
        this->hdma_spi_tx.Init.Channel  = DMA_CHANNEL_3;
        this->hdma_spi_rx.Instance      = DMA2_Stream0;
        this->hdma_spi_rx.Init.Channel  = DMA_CHANNEL_3;
    }
    else
    {
        __HAL_RCC_DMA1_CLK_ENABLE();
        this->hdma_spi_tx.Instance      = DMA1_Stream4;
        this->hdma_spi_tx.Init.Channel  = DMA_CHANNEL_0;
        this->hdma_spi_rx.Instance      = DMA1_Stream3;
        this->hdma_spi_rx.Init.Channel  = DMA_CHANNEL_0;
    }

    DMA_HandleTypeDef* streams[2] = { &this->hdma_spi_tx, &this->hdma_spi_rx };
    for (DMA_HandleTypeDef* h : streams)
    {
        h->Init.Direction           = (h == &this->hdma_spi_tx) ? DMA_MEMORY_TO_PERIPH : DMA_PERIPH_TO_MEMORY;
        h->Init.PeriphInc           = DMA_PINC_DISABLE;
        h->Init.MemInc              = DMA_MINC_ENABLE;
        h->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        h->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        h->Init.Mode                = DMA_CIRCULAR;
        h->Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        h->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(h);
    }
    __HAL_LINKDMA(&this->spiHandle, hdmatx, this->hdma_spi_tx);
    __HAL_LINKDMA(&this->spiHandle, hdmarx, this->hdma_spi_rx);

    // the DMA runs without interrupts, the frame end is detected with the slave select edge
}


void RemoraComms::startTransfer()
{
    HAL_SPI_TransmitReceive_DMA(&this->spiHandle, (uint8_t *)this->ptrTxData->txBuffer, (uint8_t *)this->spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);

    // HAL enables the stream interrupts for its callbacks: not used, and the SD card's driver may have
    // left the shared stream's NVIC line on with its own handler
    if (this->spiHandle.Instance == SPI1)
    {
        HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
        HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    }
}


void RemoraComms::start()
{
    this->ptrTxData->header = PRU_DATA;
    this->startTransfer();
}


uint32_t RemoraComms::dmaRemaining()
{
    return __HAL_DMA_GET_COUNTER(&this->hdma_spi_rx);
}


bool RemoraComms::nssHigh()
{
    return this->slaveSelect.read() != 0;
}


uint32_t RemoraComms::millis()
{
    return HAL_GetTick();
}


void RemoraComms::spiReset()
{
    if (this->spiHandle.Instance == SPI1)
    {
        __HAL_RCC_SPI1_FORCE_RESET();
        __HAL_RCC_SPI1_RELEASE_RESET();
    }
    else
    {
        __HAL_RCC_SPI2_FORCE_RESET();
        __HAL_RCC_SPI2_RELEASE_RESET();
    }
}


// a stream off (EN reads back 0 once the current transfer has stopped) and all its flags cleared
void RemoraComms::streamOff(DMA_HandleTypeDef* h)
{
    h->Instance->CR &= ~DMA_SxCR_EN;
    for (int i = 0; i < 1000 && (h->Instance->CR & DMA_SxCR_EN); i++) {}
    __HAL_DMA_CLEAR_FLAG(h, __HAL_DMA_GET_TC_FLAG_INDEX(h) | __HAL_DMA_GET_HT_FLAG_INDEX(h) |
                            __HAL_DMA_GET_TE_FLAG_INDEX(h) | __HAL_DMA_GET_DME_FLAG_INDEX(h) |
                            __HAL_DMA_GET_FE_FLAG_INDEX(h));
}


// Start the SPI and both DMA streams over at byte 0, register level (a few us). Run at every frame
// end while the bus is idle: whatever state the reply side was left in (a byte in the data register
// too many, a stray clock edge in the bit counter) is gone before the next frame, in both
// directions. The streams keep their HAL set up (addresses, channel, circular).
void RemoraComms::rearm()
{
    streamOff(&this->hdma_spi_rx);
    streamOff(&this->hdma_spi_tx);

    // the only way to empty the data register and the bit counter: reset the peripheral
    this->spiReset();
    SPI_TypeDef* spi = this->spiHandle.Instance;

    // as HAL_SPI_Init left it: CR1 0 = slave, mode 0, MSB first, 8 bit, hardware NSS input;
    // shared: software NSS, SSI low (selected)
    spi->CR1 = this->sharedSPI ? SPI_CR1_SSM : 0;
    spi->CR2 = SPI_CR2_RXDMAEN;

    this->hdma_spi_rx.Instance->NDTR = SPI_BUFF_SIZE;
    this->hdma_spi_tx.Instance->NDTR = SPI_BUFF_SIZE;
    this->hdma_spi_rx.Instance->CR |= DMA_SxCR_EN;
    this->hdma_spi_tx.Instance->CR |= DMA_SxCR_EN;

    // TX DMA before SPE (RM0390 SPI DMA sequence): the data register holds the first reply byte
    // before the master's next clock edge
    spi->CR2 |= SPI_CR2_TXDMAEN;
    spi->CR1 |= SPI_CR1_SPE;
}


void RemoraComms::stop()
{
    // release the DMA streams: the SDIO's DMA uses DMA2 stream 3, the SPI1 TX stream
    HAL_SPI_DMAStop(&this->spiHandle);
}


void RemoraComms::restart()
{
    HAL_SPI_DMAStop(&this->spiHandle);

    // wait for the master to finish any transfer in progress so the next frame starts on a boundary
    uint32_t start = HAL_GetTick();
    while (!this->slaveSelect.read() && (HAL_GetTick() - start) < 10) {}

    // reset the SPI peripheral to discard any stale byte in the data register and the overrun flag
    this->spiReset();

    // set SPI and DMA up again (the SD card may have reconfigured the shared stream), without
    // resetting the TX buffer header like start() does
    this->init();
    this->startTransfer();
}
