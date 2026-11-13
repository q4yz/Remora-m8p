#ifndef NODECONSOLE_H
#define NODECONSOLE_H

/***********************************************************************
    printf of a CAN node, which has no UART to spare: the family's USB
    serial port (REMORA_HAS_USB_CONSOLE; STM32G0:
    TARGET_STM32G0/drivers/usbCdc, polled from the main loop).
************************************************************************/

void nodeConsoleStart(const char* serialNumber);    // the USB serial number, up to 12 characters
void nodeConsoleService(void);                      // main loop

#endif
