#ifndef USBCDC_H
#define USBCDC_H

#include "mbed.h"
#include "stm32g0xx_hal.h"

/***********************************************************************
    Minimal USB serial port (CDC ACM) for the STM32G0 USB device, polled
    (call service() from the main loop, no interrupts). Debug output of the
    CAN nodes over the same USB-C cable that is used for DFU flashing.

    mbed-os has no USB device support for the STM32G0, so this sits directly
    on the ST HAL PCD driver. Output only: what the host sends is discarded.
    USB clock from HSI48, trimmed to the host's start of frame by the CRS.

    Shows up as "USB Serial Device (COMx)" on Windows, /dev/ttyACM0 on Linux,
    any baud rate. stdout (printf) is redirected to it, see UsbCdc.cpp.
    Output written before a terminal is open is kept up to the buffer size.
************************************************************************/

#define USBCDC_TX_BUFFER    2048

class UsbCdc
{
    private:

        PCD_HandleTypeDef hpcd;

        // control endpoint transfer in progress
        const uint8_t*  ep0Data;
        uint16_t        ep0Remaining;
        bool            ep0Zlp;             // end the data stage with a zero length packet
        bool            ep0InData;          // IN data stage running, status OUT follows
        bool            ep0LineCoding;      // SET_LINE_CODING data stage expected

        uint8_t         lineCoding[7];
        uint8_t         ep0Buffer[8];
        uint8_t         rxPacket[64];
        uint8_t         txPacket[64];
        uint8_t         stringBuffer[64];

        char            serial[13];         // iSerialNumber, e.g. the node UUID

        volatile bool   configured;
        bool            txBusy;

        uint8_t         txBuffer[USBCDC_TX_BUFFER];
        uint16_t        txHead, txTail;

        void ep0Send(const uint8_t* data, uint16_t len, uint16_t requested);
        void ep0SendChunk(void);
        void ep0Status(void);
        void ep0Stall(void);
        bool getDescriptor(uint8_t type, uint8_t index, uint16_t requested);
        void openEndpoints(void);

    public:

        UsbCdc();

        void start(const char* serialNumber);  // up to 12 characters
        void service(void);                     // poll the USB peripheral, send buffered output
        int  write(const void* data, int len);  // never blocks, drops what does not fit
        bool isConfigured(void) { return this->configured; }

        // HAL callbacks
        void onSetup(void);
        void onDataIn(uint8_t epnum);
        void onDataOut(uint8_t epnum);
        void onReset(void);
};

extern UsbCdc usbCdc;

#endif
