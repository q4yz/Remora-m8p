/* simulated node GPIO: each simulated node has its own SimIo, selected before its service() */
#include "canNodeIoSim.h"
#include "canProtocol.h"

SimIo* simIo = nullptr;

bool canIoPinExists(uint8_t pin) { return CAN_PIN_PORT(pin) <= 5; }
bool canIoPinReserved(uint8_t pin) { return pin == CAN_PIN(1, 0) || pin == CAN_PIN(1, 1) || pin == CAN_PIN(0, 11) || pin == CAN_PIN(0, 12); }

void canIoConfigure(uint8_t pin, bool output, uint8_t modifier, bool level)
{
    simIo->configured[pin] = true;
    simIo->output[pin] = output;
    simIo->modifier[pin] = modifier;
    if (output) simIo->level[pin] = level;
    simIo->configures++;
}

void canIoRelease(uint8_t pin) { simIo->configured[pin] = false; simIo->output[pin] = false; }
void canIoWrite(uint8_t pin, bool level)
{
    if (!simIo->configured[pin] || !simIo->output[pin]) return;
    if (level && !simIo->level[pin]) simIo->rises[pin]++;
    simIo->level[pin] = level;
}
bool canIoRead(uint8_t pin) { return simIo->level[pin]; }
