#include "NodeTmc2209.h"
#include "canNodeIo.h"
#include "canProtocol.h"

#include <stdio.h>

// registers
#define REG_GCONF               0x00
#define REG_GSTAT               0x01
#define REG_IFCNT               0x02
#define REG_IOIN                0x06
#define REG_IHOLD_IRUN          0x10
#define REG_TPOWERDOWN          0x11
#define REG_TCOOLTHRS           0x14
#define REG_SGTHRS              0x40
#define REG_CHOPCONF            0x6C
#define REG_PWMCONF             0x70

#define TMC_SYNC                0x05
#define TMC_WRITE               0x80
#define TMC_MASTER              0xFF        // address in a reply

// UART state
#define UART_IDLE               0
#define UART_TX                 1
#define UART_RX                 2
#define UART_DONE               3           // sent, and the reply received if one was expected
#define UART_TIMEOUT            4           // reply incomplete

#define RX_FIRST_TIMEOUT        200         // ticks to the reply (it starts 8 bit times = 24 ticks after the request)
#define RX_NEXT_TIMEOUT         60          // ticks between two reply bytes (none: they follow each other)

// main loop steps
#define STEP_OFF                0
#define STEP_IOIN               1           // driver there? its version
#define STEP_IFCNT_BEFORE       2
#define STEP_WRITE              3
#define STEP_IFCNT_AFTER        4
#define STEP_GSTAT              5           // configured: reset since?


uint8_t tmcCrc(const uint8_t* data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (b & 0x01)) crc = (uint8_t)((crc << 1) ^ 0x07);
            else crc = (uint8_t)(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}


const char* tmcStatusText(uint8_t status)
{
    switch (status)
    {
        case TMC_STATUS_NONE:           return "not configured yet";
        case TMC_STATUS_OK:             return "ok";
        case TMC_STATUS_NO_ANSWER:      return "NOT answering (motor power, UART pin, address?)";
        case TMC_STATUS_WRITE_FAILED:   return "writes NOT taken (UART errors)";
    }
    return "?";
}


void NodeTmc2209::setup(uint8_t pin, uint8_t address, uint16_t rsense, uint16_t current, uint16_t microsteps,
                        bool stealth, uint8_t stall)
{
    this->pin = pin;
    this->address = address;
    this->active = false;
    this->uart = UART_IDLE;
    this->step = STEP_OFF;
    this->status = TMC_STATUS_NONE;
    this->version = 0;
    this->configs = 0;

    // current scale as TMCStepper::rms_current: CS + 1 = 32 * sqrt(2) * I * (Rsense + 20 mOhm) / Vfs,
    // Vfs 325 mV, or 180 mV (vsense) for small currents; integer, no floating point on the node
    uint64_t x = 32ull * 141421ull * current * (rsense + 20u);
    uint32_t cs = (uint32_t)(x / (325000ull * 100000ull));
    bool vsense = false;
    if (cs < 17)
    {
        vsense = true;
        cs = (uint32_t)(x / (180000ull * 100000ull));
    }
    cs = cs ? cs - 1 : 0;
    if (cs > 31) cs = 31;

    uint32_t mres = 0;                      // 0: 256 microsteps ... 8: full steps
    while (mres < 8 && (256u >> mres) > microsteps) mres++;

    uint32_t gconf = (1u << 6) | (1u << 7) | (1u << 8);            // pdn_disable, mstep_reg_select, multistep_filt
    if (!stealth) gconf |= 1u << 2;                                 // en_SpreadCycle

    // toff 3, hstrt 5, hend 0, tbl 2 (36 clocks), interpolation to 256 microsteps
    uint32_t chopconf = 3u | (5u << 4) | (2u << 15) | (vsense ? 1u << 17 : 0) | (mres << 24) | (1u << 28);

    int i = 0;
    this->regAddr[i] = REG_GSTAT;      this->regValue[i++] = 0x07;                  // clear reset, drv_err, uv_cp
    this->regAddr[i] = REG_GCONF;      this->regValue[i++] = gconf;
    this->regAddr[i] = REG_CHOPCONF;   this->regValue[i++] = chopconf;
    this->regAddr[i] = REG_IHOLD_IRUN; this->regValue[i++] = (cs / 2) | (cs << 8) | (10u << 16);   // hold: half, iholddelay 10
    this->regAddr[i] = REG_TPOWERDOWN; this->regValue[i++] = 128;                   // ~2 s to the hold current
    this->regAddr[i] = REG_PWMCONF;    this->regValue[i++] = 0xC80D0E24;            // StealthChop, autoscale and autograd
    this->regAddr[i] = REG_TCOOLTHRS;  this->regValue[i++] = stall ? 0xFFFFF : 0;   // StallGuard to DIAG at all speeds
    this->regAddr[i] = REG_SGTHRS;     this->regValue[i++] = stall;
}


void NodeTmc2209::start()
{
    this->uart = UART_IDLE;
    this->step = STEP_IOIN;
    this->retries = 0;
    this->wait = 0;
    this->status = TMC_STATUS_NONE;
    this->firstService = true;
    this->active = true;                    // last: the interrupt uses the pin from now on
}


void NodeTmc2209::stop()
{
    this->active = false;
    this->step = STEP_OFF;
    this->uart = UART_IDLE;
}


/***********************************************************************
    UART, base thread interrupt: 1 start bit, 8 data bits LSB first, 1 stop bit
************************************************************************/

void NodeTmc2209::startTransfer(uint8_t len, uint8_t replyLen)
{
    this->txLen = len;
    this->rxLen = replyLen;
    this->rxCount = 0;
    this->byteIndex = 0;
    this->bitIndex = 0;
    this->shift = (uint16_t)((this->txBuf[0] << 1) | 0x200);
    this->tickCount = 1;                    // first bit on the next tick
    this->uart = UART_TX;                   // last: hands the transfer to the interrupt
}


void NodeTmc2209::sendWrite(uint8_t reg, uint32_t value)
{
    uint8_t* b = this->txBuf;
    b[0] = TMC_SYNC;
    b[1] = this->address;
    b[2] = (uint8_t)(reg | TMC_WRITE);
    b[3] = (uint8_t)(value >> 24);
    b[4] = (uint8_t)(value >> 16);
    b[5] = (uint8_t)(value >> 8);
    b[6] = (uint8_t)value;
    b[7] = tmcCrc(b, 7);
    this->startTransfer(8, 0);
}


void NodeTmc2209::sendRead(uint8_t reg)
{
    uint8_t* b = this->txBuf;
    b[0] = TMC_SYNC;
    b[1] = this->address;
    b[2] = reg;
    b[3] = tmcCrc(b, 3);
    this->startTransfer(4, 8);
}


void NodeTmc2209::tick()
{
    if (!this->active) return;

    uint8_t state = this->uart;
    if (state == UART_TX)
    {
        if (--this->tickCount) return;
        if (this->bitIndex >= 10)
        {
            // stop bit done: next byte, or listen for the reply
            if (++this->byteIndex < this->txLen)
            {
                this->shift = (uint16_t)((this->txBuf[this->byteIndex] << 1) | 0x200);
                this->bitIndex = 0;
            }
            else if (this->rxLen)
            {
                canIoSetOutput(this->pin, false);       // input with pull up, the driver answers
                this->bitIndex = -1;
                this->timeout = RX_FIRST_TIMEOUT;
                this->uart = UART_RX;
                return;
            }
            else
            {
                this->uart = UART_DONE;
                return;
            }
        }
        canIoWrite(this->pin, (this->shift >> this->bitIndex) & 1);
        this->bitIndex++;
        this->tickCount = TMC_OVERSAMPLE;
        return;
    }

    if (state != UART_RX) return;

    bool level = canIoRead(this->pin);
    if (this->bitIndex < 0)
    {
        // waiting for a start bit; the data bits are sampled in their middle from here
        if (!level)
        {
            this->bitIndex = 0;
            this->shift = 0;
            this->tickCount = TMC_OVERSAMPLE + 1;
        }
        else if (--this->timeout == 0)
        {
            canIoSetOutput(this->pin, true);
            this->uart = UART_TIMEOUT;
        }
        return;
    }

    if (--this->tickCount) return;
    if (this->bitIndex < 8)
    {
        this->shift = (uint16_t)((this->shift >> 1) | (level ? 0x80 : 0));
        if (++this->bitIndex < 8)
        {
            this->tickCount = TMC_OVERSAMPLE;
            return;
        }
        this->rxBuf[this->rxCount] = (uint8_t)this->shift;
        if (++this->rxCount >= this->rxLen)
        {
            canIoSetOutput(this->pin, true);            // line idle high again
            this->uart = UART_DONE;
            return;
        }
        this->tickCount = TMC_OVERSAMPLE;               // to the middle of the stop bit
        return;
    }
    this->bitIndex = -1;                                // stop bit: wait for the next byte
    this->timeout = RX_NEXT_TIMEOUT;
}


/***********************************************************************
    Main loop
************************************************************************/

// the reply to a read of reg, value in readValue
bool NodeTmc2209::readReply(uint8_t reg)
{
    const uint8_t* b = this->rxBuf;
    if (this->uart != UART_DONE || this->rxCount != 8) return false;
    if (b[0] != TMC_SYNC || b[1] != TMC_MASTER || b[2] != reg || b[7] != tmcCrc(b, 7)) return false;
    this->readValue = ((uint32_t)b[3] << 24) | ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 8) | b[6];
    return true;
}


void NodeTmc2209::done(uint8_t result)
{
    this->status = result;
    this->step = result == TMC_STATUS_OK ? STEP_GSTAT : STEP_IOIN;     // check, or configure again
    this->retries = 0;
    this->wait = this->now + TMC_CHECK_MS;
}


bool NodeTmc2209::service(uint32_t now)
{
    this->now = now;
    if (!this->active || this->step == STEP_OFF) return false;
    if (this->firstService)
    {
        this->firstService = false;
        this->wait = now + 2;               // the pin was just set up: idle high for a moment
    }

    uint8_t state = this->uart;
    if (state == UART_TX || state == UART_RX) return false;
    if ((int32_t)(now - this->wait) < 0) return false;

    if (state == UART_IDLE)
    {
        switch (this->step)
        {
            case STEP_IOIN:         this->sendRead(REG_IOIN); break;
            case STEP_IFCNT_BEFORE: this->sendRead(REG_IFCNT); break;
            case STEP_WRITE:        this->sendWrite(this->regAddr[this->regIndex], this->regValue[this->regIndex]); break;
            case STEP_IFCNT_AFTER:  this->sendRead(REG_IFCNT); break;
            case STEP_GSTAT:        this->sendRead(REG_GSTAT); break;
        }
        return false;
    }

    // transfer finished: a gap before the next one, the driver releases the line after its stop bit
    uint8_t before = this->status;
    this->wait = now + 1;

    if (this->step != STEP_WRITE)
    {
        static const uint8_t reg[] = { 0, REG_IOIN, REG_IFCNT, 0, REG_IFCNT, REG_GSTAT };
        bool ok = this->readReply(reg[this->step]);
        this->uart = UART_IDLE;
        if (!ok)
        {
            if (++this->retries < TMC_RETRIES) return false;          // read again
            this->done(TMC_STATUS_NO_ANSWER);
            return this->status != before;
        }
        this->retries = 0;
    }
    this->uart = UART_IDLE;

    switch (this->step)
    {
        case STEP_IOIN:
            this->version = (uint8_t)(this->readValue >> 24);
            this->step = STEP_IFCNT_BEFORE;
            break;

        case STEP_IFCNT_BEFORE:
            this->ifcnt = (uint8_t)this->readValue;
            this->regIndex = 0;
            this->step = STEP_WRITE;
            break;

        case STEP_WRITE:
            if (++this->regIndex >= TMC_CONFIG_REGS) this->step = STEP_IFCNT_AFTER;
            break;

        case STEP_IFCNT_AFTER:
            // every write the driver accepted (address and CRC right) counts
            if ((uint8_t)(this->readValue - this->ifcnt) == TMC_CONFIG_REGS)
            {
                this->configs++;
                this->done(TMC_STATUS_OK);
            }
            else this->done(TMC_STATUS_WRITE_FAILED);
            break;

        case STEP_GSTAT:
            if (this->readValue & 0x01)
            {
                this->step = STEP_IOIN;     // reset since the configuration (motor power was off): again
                this->status = TMC_STATUS_NONE;
            }
            else this->wait = now + TMC_CHECK_MS;
            break;
    }
    return this->status != before;
}
