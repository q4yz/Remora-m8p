#include "NodeAdxl345.h"
#include "canNodeIo.h"
#include "canProtocol.h"

#include <string.h>
#include <math.h>

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
    overruns(0),
    impactOn(false),
    impactThreshold2(0),
    impactHoldMs(0),
    hpB0(0), hpB1(0), hpA1(0), hpA2(0),
    hpPrimed(false),
    impactActive(false),
    impactUntil(0),
    impactNew(false),
    impactCount(0),
    peak2Frame(0),
    peak2Impact(0),
    peak2Recent(0),
    peak2Max(0)
{
    memset(this->sum, 0, sizeof(this->sum));
    memset(this->last, 0, sizeof(this->last));
}


static uint16_t isqrt(uint32_t v)
{
    uint32_t r = 0, bit = 1u << 30;
    while (bit > v) bit >>= 2;
    while (bit)
    {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return (uint16_t)(r > 0xFFFF ? 0xFFFF : r);
}


void NodeAdxl345::setImpact(uint16_t thresholdCounts, uint16_t holdMs, uint16_t cutoffHz, uint16_t rateHz)
{
    // RBJ biquad high-pass, Q = 1 / sqrt(2), once at the configuration (soft float is fine here)
    float w0 = 6.2831853f * (float)cutoffHz / (float)rateHz;
    float c = cosf(w0), alpha = sinf(w0) * 0.70710678f;    // sin(w0) / (2 Q)
    float a0 = 1.0f + alpha;
    const float q = 16384.0f;
    this->hpB0 = (int32_t)lrintf((1.0f + c) / 2.0f / a0 * q);
    this->hpB1 = (int32_t)lrintf(-(1.0f + c) / a0 * q);
    this->hpA1 = (int32_t)lrintf(-2.0f * c / a0 * q);
    this->hpA2 = (int32_t)lrintf((1.0f - alpha) / a0 * q);
    this->impactThreshold2 = (uint32_t)thresholdCounts * thresholdCounts;
    this->impactHoldMs = holdMs;
    this->hpPrimed = false;
    this->impactActive = false;
    this->impactNew = false;
    this->impactCount = 0;
    this->peak2Frame = this->peak2Impact = this->peak2Recent = this->peak2Max = 0;
    this->impactOn = true;
}


// one sensor sample through the high-pass, the magnitude against the threshold
void NodeAdxl345::sample(const int16_t xyz[3], uint32_t now)
{
    if (!this->hpPrimed)
    {
        // start in the steady state of this sample: no step response from gravity
        for (int a = 0; a < 3; a++)
        {
            this->hpX1[a] = this->hpX2[a] = xyz[a];
            this->hpY1[a] = this->hpY2[a] = 0;
        }
        this->hpPrimed = true;
        return;
    }

    uint32_t mag2 = 0;
    for (int a = 0; a < 3; a++)
    {
        // |x| <= 4096 (13 bit full resolution), |y| a few thousand, coefficients < 2^15: < 2^30
        int32_t x = xyz[a];
        int32_t acc = this->hpB0 * (x + this->hpX2[a]) + this->hpB1 * this->hpX1[a]
                    - this->hpA1 * this->hpY1[a] - this->hpA2 * this->hpY2[a];
        int32_t y = (acc >= 0 ? acc + 8192 : acc - 8192) / 16384;
        if (y > 32767) y = 32767;
        else if (y < -32767) y = -32767;
        this->hpX2[a] = this->hpX1[a];
        this->hpX1[a] = x;
        this->hpY2[a] = this->hpY1[a];
        this->hpY1[a] = y;
        uint32_t m = (uint32_t)(y * y);
        mag2 = (mag2 + m < mag2) ? 0xFFFFFFFFu : mag2 + m;
    }

    if (mag2 > this->peak2Frame) this->peak2Frame = mag2;
    if (mag2 > this->peak2Recent) this->peak2Recent = mag2;
    if (mag2 > this->peak2Max) this->peak2Max = mag2;

    if (mag2 <= this->impactThreshold2) return;
    if (!this->impact(now))
    {
        // a new impact (the output was off): reported once, its peak from here on
        this->impactCount++;
        this->impactNew = true;
        this->peak2Impact = 0;
    }
    if (mag2 > this->peak2Impact) this->peak2Impact = mag2;
    this->impactActive = true;
    this->impactUntil = now + this->impactHoldMs;
}


bool NodeAdxl345::impact(uint32_t now)
{
    if (this->impactActive && (int32_t)(this->impactUntil - now) <= 0) this->impactActive = false;
    return this->impactActive;
}


bool NodeAdxl345::takeImpact()
{
    bool n = this->impactNew;
    this->impactNew = false;
    return n;
}


uint16_t NodeAdxl345::getImpactPeak() const
{
    return isqrt(this->peak2Impact);
}


uint16_t NodeAdxl345::takeRecentPeak()
{
    uint16_t p = isqrt(this->peak2Recent);
    this->peak2Recent = 0;
    return p;
}


uint16_t NodeAdxl345::getMaxPeak() const
{
    return isqrt(this->peak2Max);
}


void NodeAdxl345::setup(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t flags)
{
    this->cs = cs;
    this->sck = sck;
    this->mosi = mosi;
    this->miso = miso;
    this->flags = flags;
    this->impactOn = false;
    this->impactActive = false;
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
    this->hpPrimed = false;
    return true;
}


void NodeAdxl345::stop()
{
    if (!this->active) return;
    if (this->status == ACCEL_STATUS_OK) this->writeReg(ADXL_POWER_CTL, 0);
    this->active = false;
    this->status = ACCEL_STATUS_NONE;
    this->impactOn = false;
    this->impactActive = false;
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
        else if (this->init())
        {
            this->status = ACCEL_STATUS_OK;     // back (plugged in, power): set up again
            this->hpPrimed = false;             // the filter starts over
        }
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
            int16_t xyz[3];
            for (int a = 0; a < 3; a++)
            {
                xyz[a] = (int16_t)(rx[1 + 2 * a] | (rx[2 + 2 * a] << 8));
                this->sum[a] += xyz[a];
            }
            if (this->impactOn) this->sample(xyz, now);
            if (this->samples < 0xFFFF) this->samples++;
            this->totalSamples++;
        }
    }
    return this->status != before;
}


bool NodeAdxl345::take(int16_t xyz[3], uint16_t* peak, uint8_t* frameFlags)
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
    *peak = isqrt(this->peak2Frame);
    this->peak2Frame = 0;
    *frameFlags = this->overrun ? ACCEL_FRAME_OVERRUN : 0;
    this->samples = 0;
    this->overrun = false;
    return true;
}
