#include "RemoraCAN.h"

#define CAN_KERNEL_CLOCK    64000000u
#define CAN_TQ_PER_BIT      16u             // 1 sync + 13 seg1 + 2 seg2: sample point 87.5 %

RemoraCAN::RemoraCAN(FDCAN_GlobalTypeDef* instance, GPIO_TypeDef* port, uint16_t rxPin, uint16_t txPin,
                     uint8_t alternate, uint32_t bitrate) :
    bitrate(bitrate),
    running(false),
    busOffCount(0)
{
    // FDCAN kernel clock from PCLK (64 MHz), FDCAN1 and FDCAN2 share the clock enable
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&clk);
    __HAL_RCC_FDCAN_CLK_ENABLE();

    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = (1u << rxPin) | (1u << txPin);
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = alternate;
    HAL_GPIO_Init(port, &gpio);

    this->hfdcan.Instance = instance;
    this->hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    this->hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    this->hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
    this->hfdcan.Init.AutoRetransmission = ENABLE;
    this->hfdcan.Init.TransmitPause = DISABLE;
    this->hfdcan.Init.ProtocolException = DISABLE;
    this->hfdcan.Init.NominalPrescaler = CAN_KERNEL_CLOCK / (CAN_TQ_PER_BIT * bitrate);
    this->hfdcan.Init.NominalSyncJumpWidth = 2;
    this->hfdcan.Init.NominalTimeSeg1 = 13;
    this->hfdcan.Init.NominalTimeSeg2 = 2;
    this->hfdcan.Init.DataPrescaler = this->hfdcan.Init.NominalPrescaler;      // unused, classic CAN
    this->hfdcan.Init.DataSyncJumpWidth = 2;
    this->hfdcan.Init.DataTimeSeg1 = 13;
    this->hfdcan.Init.DataTimeSeg2 = 2;
    this->hfdcan.Init.StdFiltersNbr = 0;
    this->hfdcan.Init.ExtFiltersNbr = 0;
    this->hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
}


bool RemoraCAN::start()
{
    if (CAN_KERNEL_CLOCK % (CAN_TQ_PER_BIT * this->bitrate) != 0)
    {
        printf("CAN: bit rate %lu not possible, use 1000000, 500000, 250000 or 125000\n", (unsigned long)this->bitrate);
        return false;
    }

    if (HAL_FDCAN_Init(&this->hfdcan) != HAL_OK)
    {
        printf("CAN: init failed\n");
        return false;
    }

    // no filters: every data frame into RX FIFO 0, remote frames rejected
    if (HAL_FDCAN_ConfigGlobalFilter(&this->hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&this->hfdcan) != HAL_OK)
    {
        printf("CAN: start failed\n");
        return false;
    }

    this->running = true;
    printf("CAN: running at %lu bit/s\n", (unsigned long)this->bitrate);
    return true;
}


bool RemoraCAN::send(const canFrame_t& frame)
{
    if (!this->running || frame.len > 8) return false;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&this->hfdcan) == 0) return false;     // never block

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = frame.id;
    header.IdType = frame.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = (uint32_t)frame.len << 16;      // FDCAN_DLC_BYTES_n for n <= 8
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&this->hfdcan, &header, (uint8_t*)frame.data) == HAL_OK;
}


bool RemoraCAN::receive(canFrame_t& frame)
{
    if (!this->running) return false;
    if (HAL_FDCAN_GetRxFifoFillLevel(&this->hfdcan, FDCAN_RX_FIFO0) == 0) return false;

    FDCAN_RxHeaderTypeDef header;
    if (HAL_FDCAN_GetRxMessage(&this->hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK) return false;

    frame.id = header.Identifier;
    frame.extended = (header.IdType == FDCAN_EXTENDED_ID);
    frame.len = (uint8_t)((header.DataLength >> 16) & 0xF);
    if (frame.len > 8) frame.len = 8;                   // classic CAN only
    return true;
}


void RemoraCAN::service()
{
    if (!this->running) return;

    // bus-off (e.g. bus unplugged, missing termination): the peripheral stops, restart it
    FDCAN_ProtocolStatusTypeDef status;
    if (HAL_FDCAN_GetProtocolStatus(&this->hfdcan, &status) == HAL_OK && status.BusOff)
    {
        this->busOffCount++;
        HAL_FDCAN_Stop(&this->hfdcan);
        HAL_FDCAN_Start(&this->hfdcan);
    }
}


uint16_t RemoraCAN::getErrorCount()
{
    FDCAN_ErrorCountersTypeDef counters;
    if (HAL_FDCAN_GetErrorCounters(&this->hfdcan, &counters) != HAL_OK) return 0xFFFF;
    return (uint16_t)(counters.TxErrorCnt + counters.RxErrorCnt);
}
