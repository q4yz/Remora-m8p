#include "NodeAdxl345.h"
#include "canNodeIo.h"
#include "canProtocol.h"

#include <string.h>

// ADXL345 registers
#define ADXL_DEVID              0x00
#define ADXL_BW_RATE            0x2C
#define ADXL_POWER_CTL          0x2D
#define ADXL_DATA_FORMAT        0x31
#define ADXL_DATAX0             0x32        // X, Y, Z, int16 little endian
#define ADXL_FIFO_CTL           0x38
#define ADXL_FIFO_STATUS        0x39

#define ADXL_READ               0x80
#define ADXL_MULTI              0x40
#define ADXL_ID                 0xE5
#define ADXL_MEASURE            0x08        // POWER_CTL
#define ADXL_FULL_RES           0x08        // DATA_FORMAT; 4 wire SPI, interrupts active high
#define ADXL_FIFO_STREAM        0x80        // FIFO_CTL: stream mode, the oldest sample is dropped when full
#define ADXL_FIFO_SIZE          32

#define ADXL_SPI_HZ             5000000     // the sensor's maximum
#define ADXL_SPI_MODE           3           // CPOL 1, CPHA 1
#define ADXL_CHECK_MS           1000        // device ID check, retry of a missing sensor
#define ADXL_FIFO_GAP_US        5           // datasheet: 5 us between reading the data and the next FIFO read


NodeAdxl345::NodeAdxl345() :
    cs(CAN_PIN_NONE), sck(CAN_PIN_NONE), mosi(CAN_PIN_NONE), miso(CAN_PIN_NONE),
    flags(0),
    active(false),
    status(ACCEL_STATUS_NONE),
    deviceId(0),
    lastCheck(0),
    lastRead(0),
    samples(0),
    overrun(false),
    totalSamples(0),
    overruns(0)
{
    memset(this->sum, 0, sizeof(this->sum));
    memset(this->last, 0, sizeof(this->last));
}


void NodeAdxl345::setup(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t flags)
{
    this->cs = cs;
    this->sck = sck;
    this->mosi = mosi;
    this->miso = miso;
    this->flags = flags;
}


void NodeAdxl345::writeReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value }, rx[2];
    canIoWrite(this->cs, false);
    canIoSpiTransfer(tx, rx, 2);
    canIoWrite(this->cs, true);
}


uint8_t NodeAdxl345::readReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(ADXL_READ | reg), 0 }, rx[2] = { 0, 0 };
    canIoWrite(this->cs, false);
    canIoSpiTransfer(tx, rx, 2);
    canIoWrite(this->cs, true);
    return rx[1];
}


bool NodeAdxl345::init()
{
    this->writeReg(ADXL_POWER_CTL, 0);                  // standby while it is set up
    this->deviceId = this->readReg(ADXL_DEVID);
    if (this->deviceId != ADXL_ID) return false;

    this->writeReg(ADXL_DATA_FORMAT, (uint8_t)(ADXL_FULL_RES | (this->flags & ACCEL_RANGE_MASK)));
    this->writeReg(ADXL_BW_RATE, (uint8_t)(this->flags >> ACCEL_RATE_SHIFT));
    this->writeReg(ADXL_FIFO_CTL, 0);                   // bypass: the FIFO is cleared
    this->writeReg(ADXL_FIFO_CTL, ADXL_FIFO_STREAM);
    this->writeReg(ADXL_POWER_CTL, ADXL_MEASURE);
    return true;
}


bool NodeAdxl345::start(uint32_t now)
{
    if (!canIoSpiStart(this->sck, this->mosi, this->miso, ADXL_SPI_HZ, ADXL_SPI_MODE)) return false;
    canIoConfigure(this->cs, true, DPIN_MOD_NONE, true);   // CS idle high
    this->active = true;
    this->samples = 0;
    memset(this->sum, 0, sizeof(this->sum));
    this->overrun = false;
    this->lastCheck = now;
    this->lastRead = now;
    this->status = this->init() ? ACCEL_STATUS_OK : ACCEL_STATUS_MISSING;
    return true;
}


void NodeAdxl345::stop()
{
    if (!this->active) return;
    if (this->status == ACCEL_STATUS_OK) this->writeReg(ADXL_POWER_CTL, 0);
    this->active = false;
    this->status = ACCEL_STATUS_NONE;
    canIoSpiStop();
    canIoRelease(this->cs);
}


bool NodeAdxl345::service(uint32_t now)
{
    if (!this->active) return false;
    uint8_t before = this->status;

    if ((now - this->lastCheck) >= ADXL_CHECK_MS)
    {
        this->lastCheck = now;
        if (this->status == ACCEL_STATUS_OK)
        {
            this->deviceId = this->readReg(ADXL_DEVID);
            if (this->deviceId != ADXL_ID) this->status = ACCEL_STATUS_MISSING;
        }
        else if (this->init()) this->status = ACCEL_STATUS_OK;      // back (plugged in, power): set up again
    }

    if (this->status == ACCEL_STATUS_OK && now != this->lastRead)
    {
        this->lastRead = now;
        int entries = this->readReg(ADXL_FIFO_STATUS) & 0x3F;
        if (entries > ADXL_FIFO_SIZE)
        {
            // impossible: no sensor drives MISO (0xFF), unplugged; no garbage samples until the ID check
            this->status = ACCEL_STATUS_MISSING;
            this->deviceId = 0xFF;
            this->lastCheck = now;
            return true;
        }
        if (entries == ADXL_FIFO_SIZE)
        {
            this->overrun = true;           // full: the sensor dropped samples
            this->overruns++;
        }
        for (int i = 0; i < entries && i < ADXL_FIFO_SIZE; i++)
        {
            canIoDelayUs(ADXL_FIFO_GAP_US);
            uint8_t tx[7] = { ADXL_READ | ADXL_MULTI | ADXL_DATAX0, 0, 0, 0, 0, 0, 0 }, rx[7];
            canIoWrite(this->cs, false);
            canIoSpiTransfer(tx, rx, 7);
            canIoWrite(this->cs, true);
            for (int a = 0; a < 3; a++) this->sum[a] += (int16_t)(rx[1 + 2 * a] | (rx[2 + 2 * a] << 8));
            if (this->samples < 0xFFFF) this->samples++;
            this->totalSamples++;
        }
    }
    return this->status != before;
}


bool NodeAdxl345::take(int16_t xyz[3], uint8_t* count, uint8_t* frameFlags)
{
    if (!this->samples) return false;
    int32_t n = this->samples;
    for (int a = 0; a < 3; a++)
    {
        int32_t s = this->sum[a];
        int32_t v = (s >= 0 ? s + n / 2 : s - n / 2) / n;     // rounded
        xyz[a] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
        this->last[a] = xyz[a];
        this->sum[a] = 0;
    }
    *count = (uint8_t)(n > 255 ? 255 : n);
    *frameFlags = this->overrun ? ACCEL_FRAME_OVERRUN : 0;
    this->samples = 0;
    this->overrun = false;
    return true;
}
