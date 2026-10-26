#ifndef RAWCONSOLE_H
#define RAWCONSOLE_H

// Minimal register level console on USART4 TX = PE8 (Manta M8P: EXP2 pin 7), 115200 8N1.
// Used by REMORA_DEBUG builds instead of the mbed serial console, so boot output does not
// depend on the mbed serial driver.

void rawConsoleInit(void);
void rawConsolePuts(const char *s);

#endif
