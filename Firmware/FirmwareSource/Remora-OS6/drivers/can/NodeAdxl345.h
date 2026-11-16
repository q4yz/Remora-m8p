#ifndef NODEADXL345_H
#define NODEADXL345_H

#include <stdint.h>

/***********************************************************************
    ADXL345 accelerometer of a CAN node (BTT EBB42 / SB2209: SPI2, CS PB12,
    SCK PB10, MOSI PB11, MISO PB2), 4 wire SPI mode 3 at up to 5 MHz,
    through canIoSpi* (canNodeIo.h), main loop only.

    Full resolution (3.9 mg per count in every range), the FIFO in stream
    mode: the sensor samples at its own output data rate, service() empties
    the FIFO once per ms and sums the samples up, take() gives their
    average (a box filter against aliasing: the hub passes one value per ms
    to LinuxCNC). The FIFO holds 32 samples, 10 ms at 3200 Hz: a main loop
    that stalls longer loses samples, take() reports it.

    Impact detection (setImpact): every sample through a 2nd order
    Butterworth high-pass per axis (integer, Q14 coefficients: the
    Cortex-M0+ has no FPU), the squared magnitude of the three against the
    threshold. Above it the impact output is on until the hold time after
    the last such sample. The filter starts from the first sample (no
    spike from gravity at the start).

    The device ID (0xE5) is checked at the start and every second: a sensor
    that does not answer (cable, wrong pins) is ACCEL_STATUS_MISSING and is
    set up again once it answers.
************************************************************************/

class NodeAdxl345
{
    private:

        uint8_t     cs, sck, mosi, miso;
        uint8_t     flags;              // ACCEL_* (canProtocol.h): range, rate
        bool        active;
        uint8_t     status;             // ACCEL_STATUS_*
        uint8_t     deviceId;           // last read
        uint32_t    lastCheck;          // ms, device ID check / retry
        uint32_t    lastRead;           // ms, FIFO read
        int32_t     sum[3];
        uint16_t    samples;            // in sum
        bool        overrun;            // since the last take()
        uint32_t    totalSamples;
        uint32_t    overruns;
        int16_t     last[3];            // the last average taken

        // impact detection
        bool        impactOn;
        uint32_t    impactThreshold2;   // counts^2
        uint16_t    impactHoldMs;
        int32_t     hpB0, hpB1, hpA1, hpA2;     // Q14, b2 = b0
        int32_t     hpX1[3], hpX2[3], hpY1[3], hpY2[3];
        bool        hpPrimed;
        bool        impactActive;
        uint32_t    impactUntil;        // ms
        bool        impactNew;          // a new impact, not reported yet
        uint32_t    impactCount;
        uint32_t    peak2Frame;         // highest magnitude^2 since the last take()
        uint32_t    peak2Impact;        // of the last impact
        uint32_t    peak2Recent;        // since the last takeRecentPeak() (console)
        uint32_t    peak2Max;           // since the configuration

        void writeReg(uint8_t reg, uint8_t value);
        uint8_t readReg(uint8_t reg);
        bool init(void);                // false: no sensor
        void sample(const int16_t xyz[3], uint32_t now);

    public:

        NodeAdxl345(void);

        void setup(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t flags);
        bool start(uint32_t now);       // SPI on, sensor set up; false: no SPI on these pins
        void stop(void);                // sensor to standby, pins released

        // impact detection: threshold in counts, hold in ms, cutoff of the high-pass in Hz; rateHz the
        // sensor's output data rate. Call after setup(); setup() turns it off
        void setImpact(uint16_t thresholdCounts, uint16_t holdMs, uint16_t cutoffHz, uint16_t rateHz);
        bool impact(uint32_t now);      // the impact output: an impact within the hold time
        bool takeImpact(void);          // true once per new impact (EMCY)
        uint32_t getImpactCount(void) const { return this->impactCount; }
        uint16_t getImpactPeak(void) const;         // of the last impact, counts
        uint16_t takeRecentPeak(void);  // the highest magnitude since the last call, counts (console)
        uint16_t getMaxPeak(void) const;            // since the configuration, counts

        bool service(uint32_t now);     // true: the status changed
        // the average of the samples since the last call, the highest impact magnitude among them
        // (counts, 0 without impact detection), flags ACCEL_FRAME_*; false: no new sample
        bool take(int16_t xyz[3], uint16_t* peak, uint8_t* frameFlags);

        bool isActive(void) const { return this->active; }
        uint8_t getStatus(void) const { return this->status; }
        uint8_t getDeviceId(void) const { return this->deviceId; }
        uint8_t getCs(void) const { return this->cs; }
        uint8_t getFlags(void) const { return this->flags; }
        uint32_t getSamples(void) const { return this->totalSamples; }
        uint32_t getOverruns(void) const { return this->overruns; }
        const int16_t* getLast(void) const { return this->last; }
};

#endif
