#include "mbed.h"
#include "RemoraComms.h"

#include "stm32g0xx_hal.h"

/***********************************************************************
    SPI slave link to the host for STM32G0 boards, the hardware side of
    RemoraLink (drivers/remoraLink: frame queue, torn frames, re-align,
    link status).

    SPI2 with hardware NSS on PB12 (SCK PB13, MISO PB14, MOSI PB11), on the
    BTT Manta M8P these are the EXP2 header pins 4, 2, 1 and 6. Both DMA
    channels run in circular mode over the 64 byte frame without
    interrupts, the rising edge of NSS marks the end of a frame. On the G0
    the DMA channels are connected to the SPI through the DMAMUX, so the
    requests are set explicitly.
************************************************************************/

RemoraComms::RemoraComms(volatile rxData_t* ptrRxData, volatile txData_t* ptrTxData, SPI_TypeDef* spiType, PinName interruptPin) :
    RemoraLink(*this, ptrRxData, spiRxBuffer.rxBuffer),
    spiType(spiType),
    ptrTxData(ptrTxData),
    uploadMode(false),
    initialised(false),
    interruptPin(interruptPin),
    slaveSelect(interruptPin)
{
    this->spiHandle.Instance = this->spiType;

    if (this->spiType != SPI2 || this->interruptPin != PB_12)
    {
        printf("Error - STM32G0 Remora comms only supports SPI2 with NSS on PB_12\n");
    }

    // Cortex-M0+ has 4 priority levels (us ticker 0, base thread 1, servo 2, comms 3). The NSS edge
    // goes to 0: below the threads the servo thread (ADC reads, all its modules) held it back by
    // more than the gap to the next frame, and the base thread could preempt it in the middle of
    // rearm() - with the SPI in reset when the next frame started (0xffffffff, shifted replies).
    // The handler is a few us: a 64 byte copy and the re-arm, the base thread jitters by that much.
    slaveSelect.rise(callback(this, &RemoraComms::NSSinterrupt));
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
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


void RemoraComms::startTransfer()
{
    HAL_SPI_TransmitReceive_DMA(&this->spiHandle, (uint8_t *)this->ptrTxData->txBuffer, (uint8_t *)this->spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);

    // HAL enables the DMA interrupts for its callbacks, they are not used and the NVIC lines stay off
    HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
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
    return (GPIOB->IDR & GPIO_PIN_12) != 0;
}


uint32_t RemoraComms::millis()
{
    return HAL_GetTick();
}


// Start the SPI and both DMA channels over at byte 0, register level (a few us). Run at every frame
// end while the bus is idle: whatever state the reply side was left in (a byte in the TX FIFO too
// many or too few, a stray clock edge in the bit counter) is gone before the next frame, in both
// directions. The DMA channels keep their HAL set up (addresses, circular, DMAMUX request).
void RemoraComms::rearm()
{
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;

    // the only way to empty the TX FIFO: reset the peripheral
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();

    // as HAL_SPI_Init left it: CR1 0 = slave, mode 0, MSB first, hardware NSS input
    SPI2->CR1 = 0;
    SPI2->CR2 = SPI_DATASIZE_8BIT | SPI_CR2_FRXTH | SPI_CR2_RXDMAEN;

    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    DMA1_Channel2->CNDTR = SPI_BUFF_SIZE;
    DMA1_Channel3->CNDTR = SPI_BUFF_SIZE;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    // TX DMA before SPE (RM0444 SPI DMA sequence): the FIFO holds the first reply bytes before
    // the master's next clock edge
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
    SPI2->CR1 |= SPI_CR1_SPE;
}


void RemoraComms::stop()
{
    // the SD card has its own SPI bus here, pausing just keeps the frames in step with the main loop
    HAL_SPI_DMAStop(&this->spiHandle);
}


void RemoraComms::restart()
{
    HAL_SPI_DMAStop(&this->spiHandle);

    // wait for the master to finish any transfer in progress so the next frame starts on a boundary
    uint32_t start = HAL_GetTick();
    while (!this->slaveSelect.read() && (HAL_GetTick() - start) < 10) {}

    // reset the SPI peripheral to empty its FIFOs and clear the overrun flag
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();

    // set SPI and DMA up again without resetting the TX buffer header like start() does
    this->init();
    this->startTransfer();
}
