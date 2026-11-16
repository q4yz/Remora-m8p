/* simulated node GPIO: each simulated node has its own SimIo, selected before its service() */
#include "canNodeIoSim.h"
#include "canProtocol.h"
#include <string.h>
#include <math.h>

SimIo* simIo = nullptr;

extern uint32_t sim_uid[3];
void canIoUniqueId(uint32_t id[3]) { id[0] = sim_uid[0]; id[1] = sim_uid[1]; id[2] = sim_uid[2]; }

bool canIoPinExists(uint8_t pin) { return CAN_PIN_PORT(pin) <= 5; }
bool canIoPinReserved(uint8_t pin) { return pin == CAN_PIN(1, 0) || pin == CAN_PIN(1, 1) || pin == CAN_PIN(0, 11) || pin == CAN_PIN(0, 12); }

void canIoConfigure(uint8_t pin, bool output, uint8_t modifier, bool level)
{
    simIo->configured[pin] = true;
    simIo->output[pin] = output;
    simIo->modifier[pin] = modifier;
    if (output) simIo->level[pin] = level;
    simIo->latch[pin] = level;
    simIo->configures++;
}

void canIoRelease(uint8_t pin) { simIo->configured[pin] = false; simIo->output[pin] = false; simIo->analogOn[pin] = false; }
void canIoWrite(uint8_t pin, bool level)
{
    if (!simIo->configured[pin]) return;
    simIo->latch[pin] = level;
    if (!simIo->output[pin]) return;
    if (level && !simIo->level[pin]) simIo->rises[pin]++;
    simIo->level[pin] = level;
}
bool canIoRead(uint8_t pin) { return simIo->level[pin]; }
/* ADC on PA_0 ... PA_7 */
bool canIoAnalogCapable(uint8_t pin) { return CAN_PIN_PORT(pin) == 0 && CAN_PIN_NUMBER(pin) <= 7; }
void canIoAnalogConfigure(uint8_t pin) { simIo->configured[pin] = true; simIo->output[pin] = false; simIo->analogOn[pin] = true; }
uint16_t canIoAnalogRead(uint8_t pin) { return simIo->analogOn[pin] ? simIo->analog[pin] : 0; }

void canIoSetOutput(uint8_t pin, bool output)
{
    if (!simIo->configured[pin]) return;
    simIo->output[pin] = output;
    if (output) simIo->level[pin] = simIo->latch[pin];
}

/* LED chains on the timer channel pins of the EBB42 / SB2209 / M8P RGB headers: PD_3, PA_9, PB_15, PC_6 */
bool canIoRgbCapable(uint8_t pin)
{
    return pin == CAN_PIN(3, 3) || pin == CAN_PIN(0, 9) || pin == CAN_PIN(1, 15) || pin == CAN_PIN(2, 6);
}
bool canIoRgbConfigure(uint8_t pin, int leds, int bytesPerLed)
{
    if (!canIoRgbCapable(pin) || leds < 1 || leds > 64) return false;
    simIo->rgbLeds[pin] = leds;
    simIo->rgbBpl[pin] = bytesPerLed;
    simIo->rgbSends[pin] = 0;
    memset(simIo->rgb[pin], 0xEE, sizeof(simIo->rgb[pin]));     /* not sent yet */
    return true;
}
bool canIoRgbSend(uint8_t pin, const uint8_t* bytes)
{
    if (!simIo->rgbLeds[pin]) return false;
    memcpy(simIo->rgb[pin], bytes, simIo->rgbLeds[pin] * simIo->rgbBpl[pin]);
    simIo->rgbSends[pin]++;
    return true;
}
void canIoRgbRelease(uint8_t pin) { simIo->rgbLeds[pin] = 0; }

/* SPI: SCK, MOSI, MISO of the EBB42 / SB2209 SPI2 (PB_10, PB_11, PB_2) or of SPI1 (PA_5, PA_7, PA_6) */
bool canIoSpiCapable(uint8_t sck, uint8_t mosi, uint8_t miso)
{
    return (sck == CAN_PIN(1, 10) && mosi == CAN_PIN(1, 11) && miso == CAN_PIN(1, 2)) ||
           (sck == CAN_PIN(0, 5) && mosi == CAN_PIN(0, 7) && miso == CAN_PIN(0, 6));
}
bool canIoSpiStart(uint8_t sck, uint8_t mosi, uint8_t miso, uint32_t, int mode)
{
    if (!canIoSpiCapable(sck, mosi, miso) || mode != 3) return false;
    simIo->spiOn = true;
    simIo->spiPins[0] = sck; simIo->spiPins[1] = mosi; simIo->spiPins[2] = miso;
    for (int k = 0; k < 3; k++) simIo->configured[simIo->spiPins[k]] = true;
    return true;
}
void canIoSpiStop()
{
    if (!simIo->spiOn) return;
    simIo->spiOn = false;
    for (int k = 0; k < 3; k++) simIo->configured[simIo->spiPins[k]] = false;
}
/* the ADXL345: byte 0 read bit 7, multi byte bit 6, register; a missing one leaves MISO high */
void canIoSpiTransfer(const uint8_t* tx, uint8_t* rx, int len)
{
    memset(rx, 0xFF, len);
    if (!simIo->spiOn || !simIo->adxlPresent || len < 2) return;
    uint8_t reg = tx[0] & 0x3F;
    if (!(tx[0] & 0x80)) { simIo->adxlReg[reg] = tx[1]; if (reg == 0x38 && !(tx[1] & 0xC0)) simIo->adxlFifo = 0; return; }
    rx[0] = 0;
    if (reg == 0x00) rx[1] = 0xE5;
    else if (reg == 0x39) rx[1] = (uint8_t)simIo->adxlFifo;
    else if (reg == 0x32 && len == 7 && simIo->adxlFifo > 0)
    {
        simIo->adxlFifo--;
        simIo->adxlReads++;
        int hz = 3200 >> (0x0F - (simIo->adxlReg[0x2C] & 0x0F));
        for (int a = 0; a < 3; a++)
        {
            int v = simIo->adxlXyz[a];
            if (a == 0 && simIo->adxlSineAmp != 0)
                v += (int)lrintf(simIo->adxlSineAmp * sinf(6.2831853f * simIo->adxlSineHz * simIo->adxlReads / hz));
            if (simIo->adxlImpulseSamples > 0) v += simIo->adxlImpulse[a];
            rx[1 + 2 * a] = (uint8_t)v; rx[2 + 2 * a] = (uint8_t)((uint16_t)v >> 8);
        }
        if (simIo->adxlImpulseSamples > 0) simIo->adxlImpulseSamples--;
    }
    else rx[1] = simIo->adxlReg[reg];
}
void canIoDelayUs(uint32_t) {}
