#ifndef CANNODEIO_H
#define CANNODEIO_H

#include <stdint.h>

/***********************************************************************
    GPIO access of a CAN node, pins as one byte (CAN_PIN in canProtocol.h).
    canNodeIo.cpp implements it with the STM32G0 HAL, the PC test of the
    protocol (Firmware/Tests/can) with a simulation.
************************************************************************/

bool canIoPinExists(uint8_t pin);
bool canIoPinReserved(uint8_t pin);         // used by the node itself: CAN, USB

// modifier: DPIN_MOD_*; an output starts at level
void canIoConfigure(uint8_t pin, bool output, uint8_t modifier, bool level);
void canIoRelease(uint8_t pin);             // back to the reset state (analog, high impedance)
void canIoWrite(uint8_t pin, bool level);
bool canIoRead(uint8_t pin);
void canIoSetOutput(uint8_t pin, bool output);  // direction only, level and pull stay: half duplex UART, interrupt safe

bool canIoAnalogCapable(uint8_t pin);       // the pin has an ADC input
void canIoAnalogConfigure(uint8_t pin);
uint16_t canIoAnalogRead(uint8_t pin);      // 0 ... 65535, main loop only (a conversion takes a few us)

// WS2812 LED chains on a timer channel pin (timer + DMA, no interrupts held)
bool canIoRgbCapable(uint8_t pin);          // the pin is a channel of a timer that can drive a chain
bool canIoRgbConfigure(uint8_t pin, int leds, int bytesPerLed);  // false: no free timer / DMA channel
bool canIoRgbSend(uint8_t pin, const uint8_t* bytes);   // leds * bytesPerLed bytes; false: busy, try again
void canIoRgbRelease(uint8_t pin);

#endif
