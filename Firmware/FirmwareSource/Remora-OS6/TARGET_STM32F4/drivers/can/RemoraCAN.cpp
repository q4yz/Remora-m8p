#include "RemoraCAN.h"
#include "configuration.h"                  // board.h: BOARD_CAN

#define CAN_INIT_TIMEOUT_MS     10          // leaving init mode needs 11 recessive bits on the bus

// the board's controller: board.h, BOARD_CAN = instance, port, RX pin, TX pin, alternate function
CanBus* canBusCreate(uint32_t bitrate)
{
#ifdef BOARD_CAN
    return new RemoraCAN(BOARD_CAN, bitrate);
#else
    (void)bitrate;
    return nullptr;
#endif
}


RemoraCAN::RemoraCAN(CAN_TypeDef* instance, GPIO_TypeDef* port, uint16_t rxPin, uint16_t txPin,
                     uint8_t alternate, uint32_t bitrate) :
    can(instance),
    bitrate(bitrate),
    running(false),
    wasBusOff(false),
    busOffCount(0),
    txCount(0),
    txDropped(0),
    rxCount(0),
    lastError(0)
{
    // CAN2 is a slave of CAN1: its filters are in CAN1, which needs its clock too
    __HAL_RCC_CAN1_CLK_ENABLE();
#ifdef CAN2
    if (instance == CAN2) __HAL_RCC_CAN2_CLK_ENABLE();
#endif

    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
#ifdef GPIOH
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
#endif

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = (1u << rxPin) | (1u << txPin);
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;                // RX recessive while no transceiver drives it
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = alternate;
    HAL_GPIO_Init(port, &gpio);
}


// prescaler and segments for the bit rate from PCLK1, exact; false: not possible
bool RemoraCAN::bitTiming(uint32_t* btr)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    for (uint32_t tq = 25; tq >= 8; tq--)
    {
        if (pclk % (this->bitrate * tq) != 0) continue;
        uint32_t prescaler = pclk / (this->bitrate * tq);
        if (prescaler < 1 || prescaler > 1024) continue;
        uint32_t ts2 = (tq + 4) / 8;            // 12.5 %: sample point 87.5 %
        if (ts2 < 1) ts2 = 1;
        uint32_t ts1 = tq - 1 - ts2;
        if (ts1 > 16 || ts2 > 8) continue;
        uint32_t sjw = ts2 < 2 ? ts2 : 2;
        *btr = ((sjw - 1) << CAN_BTR_SJW_Pos) | ((ts2 - 1) << CAN_BTR_TS2_Pos) |
               ((ts1 - 1) << CAN_BTR_TS1_Pos) | (prescaler - 1);
        return true;
    }
    return false;
}


bool RemoraCAN::start()
{
    uint32_t btr;
    if (!this->bitTiming(&btr))
    {
        printf("CAN: bit rate %lu not possible, use 1000000, 500000, 250000 or 125000\n", (unsigned long)this->bitrate);
        return false;
    }

    // out of sleep, into init mode
    this->can->MCR = (this->can->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ;
    uint32_t t0 = HAL_GetTick();
    while (!(this->can->MSR & CAN_MSR_INAK))
    {
        if (HAL_GetTick() - t0 > CAN_INIT_TIMEOUT_MS)
        {
            printf("CAN: init failed\n");
            return false;
        }
    }

    // automatic bus-off recovery, automatic retransmission, TX mailboxes in the order queued
    this->can->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_TXFP;
    this->can->BTR = btr;

    // one filter bank, 32 bit mask 0: every frame into FIFO 0. The filters are CAN1's; CAN2 starts
    // at bank 14 (CAN2SB)
    uint32_t bank = 0;
#ifdef CAN2
    if (this->can == CAN2) bank = 14;
#endif
    CAN_TypeDef* f = CAN1;
    f->FMR |= CAN_FMR_FINIT;
#ifdef CAN2
    f->FMR = (f->FMR & ~CAN_FMR_CAN2SB) | (14u << CAN_FMR_CAN2SB_Pos);
#endif
    f->FA1R &= ~(1u << bank);
    f->FS1R |= (1u << bank);                // 32 bit
    f->FM1R &= ~(1u << bank);               // mask mode
    f->FFA1R &= ~(1u << bank);              // FIFO 0
    f->sFilterRegister[bank].FR1 = 0;       // id
    f->sFilterRegister[bank].FR2 = 0;       // mask: no bit compared
    f->FA1R |= (1u << bank);
    f->FMR &= ~CAN_FMR_FINIT;

    // leave init mode: needs 11 recessive bits, a bus without a transceiver never gives them
    this->can->MCR &= ~CAN_MCR_INRQ;
    t0 = HAL_GetTick();
    while (this->can->MSR & CAN_MSR_INAK)
    {
        if (HAL_GetTick() - t0 > CAN_INIT_TIMEOUT_MS)
        {
            printf("CAN: start failed (no recessive bus level: transceiver, wiring?)\n");
            return false;
        }
    }

    this->running = true;
    printf("CAN: running at %lu bit/s\n", (unsigned long)this->bitrate);
    return true;
}


bool RemoraCAN::send(const canFrame_t& frame)
{
    if (!this->running || frame.len > 8) return false;

    // a free mailbox, never block
    uint32_t tsr = this->can->TSR;
    int box = (tsr & CAN_TSR_TME0) ? 0 : (tsr & CAN_TSR_TME1) ? 1 : (tsr & CAN_TSR_TME2) ? 2 : -1;
    if (box < 0)
    {
        this->txDropped++;
        return false;
    }

    CAN_TxMailBox_TypeDef& m = this->can->sTxMailBox[box];
    m.TIR = frame.extended ? ((frame.id << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE) : (frame.id << CAN_TI0R_STID_Pos);
    m.TDTR = frame.len;
    m.TDLR = (uint32_t)frame.data[0] | ((uint32_t)frame.data[1] << 8) | ((uint32_t)frame.data[2] << 16) |
             ((uint32_t)frame.data[3] << 24);
    m.TDHR = (uint32_t)frame.data[4] | ((uint32_t)frame.data[5] << 8) | ((uint32_t)frame.data[6] << 16) |
             ((uint32_t)frame.data[7] << 24);
    m.TIR |= CAN_TI0R_TXRQ;
    this->txCount++;
    return true;
}


bool RemoraCAN::receive(canFrame_t& frame)
{
    if (!this->running || !(this->can->RF0R & CAN_RF0R_FMP0)) return false;

    const CAN_FIFOMailBox_TypeDef& m = this->can->sFIFOMailBox[0];
    uint32_t rir = m.RIR;
    frame.extended = (rir & CAN_RI0R_IDE) != 0;
    frame.id = frame.extended ? (rir >> CAN_RI0R_EXID_Pos) : (rir >> CAN_RI0R_STID_Pos);
    frame.len = (uint8_t)(m.RDTR & CAN_RDT0R_DLC);
    if (frame.len > 8) frame.len = 8;
    uint32_t lo = m.RDLR, hi = m.RDHR;
    for (int i = 0; i < 4; i++)
    {
        frame.data[i] = (uint8_t)(lo >> (8 * i));
        frame.data[4 + i] = (uint8_t)(hi >> (8 * i));
    }
    this->can->RF0R = CAN_RF0R_RFOM0;       // release the FIFO slot
    bool remote = (rir & CAN_RI0R_RTR) != 0;
    if (remote) return false;               // data frames only
    this->rxCount++;
    return true;
}


void RemoraCAN::service()
{
    if (!this->running) return;

    uint32_t esr = this->can->ESR;
    uint32_t lec = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
    if (lec != 0 && lec != 7)
    {
        this->lastError = lec;
        this->can->ESR = CAN_ESR_LEC;       // "user" code 7: a new error shows as a change
    }

    // bus-off (bus unplugged, missing termination): ABOM brings it back, here it is only counted
    bool busOff = (esr & CAN_ESR_BOFF) != 0;
    if (busOff && !this->wasBusOff) this->busOffCount++;
    this->wasBusOff = busOff;
}


uint16_t RemoraCAN::getErrorCount()
{
    uint32_t esr = this->can->ESR;
    return (uint16_t)(((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos) + ((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos));
}


void RemoraCAN::printStatus()
{
    static const char* const errorNames[8] = { "none", "stuff", "form", "ACK (nobody answers)",
                                               "bit1", "bit0", "CRC", "none" };

    uint32_t esr = this->can->ESR, tsr = this->can->TSR;
    int waiting = !(tsr & CAN_TSR_TME0) + !(tsr & CAN_TSR_TME1) + !(tsr & CAN_TSR_TME2);

    printf("CAN: tx %lu (dropped %lu, waiting %d), rx %lu, tx-errors %lu, rx-errors %lu%s, last error %s, bus-off %lu\n",
           (unsigned long)this->txCount, (unsigned long)this->txDropped, waiting, (unsigned long)this->rxCount,
           (unsigned long)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos), (unsigned long)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos),
           (esr & CAN_ESR_EPVF) ? " (passive)" : "", errorNames[this->lastError & 7], (unsigned long)this->busOffCount);
}
