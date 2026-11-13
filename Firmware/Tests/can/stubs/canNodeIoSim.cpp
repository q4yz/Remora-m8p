/* simulated node GPIO: each simulated node has its own SimIo, selected before its service() */
#include "canNodeIoSim.h"
#include "canProtocol.h"
#include <string.h>

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
