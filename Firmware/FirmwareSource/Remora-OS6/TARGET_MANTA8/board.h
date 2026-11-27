#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    BTT Manta M8P V1.0 / V1.1

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "BTT Manta M8P V1.0 / V1.1"

#define BOARD_SD_SPI_PINS       PA_7, PA_6, PA_5, PA_8    // SD card: mosi, miso, sclk, cs
#define BOARD_LINK_SPI          , SPI2, PB_12          // SPI link to the host: SPI, slave select pin

// The SD card's SPI1 also runs to the stepper driver sockets: released after config.txt is read and
// between the accesses of an upload (sdBusRelease / sdBusAcquire in board.cpp)
#define BOARD_SD_RELEASE

// config.txt upload over the SPI link while LinuxCNC is offline (drivers/sdUpload)
#define BOARD_SD_UPLOAD

// CAN hub for expansion nodes (CAN_PROTOCOL.md): FDCAN1 on PD12 (RX) / PD13 (TX), alternate
// function 3, on-board transceiver. The arguments of the family's CAN driver (canBusCreate)
#define BOARD_CAN               FDCAN1, GPIOD, 12, 13, GPIO_AF3_FDCAN1

// debug builds (REMORA_DEBUG): register level console on the UART instead of mbed's
#define BOARD_RAW_CONSOLE

// Pins the board itself uses: a module on one of them would make the MCU drive a line another
// driver already drives (CB1 / Pi SPI, CAN transceiver, USB hub, crystal) - two outputs against
// each other is how a GPIO, or the whole MCU, burns out - or break the firmware's own link and
// console. config.txt is refused when a module names one.
#define BOARD_RESERVED_PINS \
    { "PB_11", "SPI link to the CB1 / Pi (MOSI)" }, { "PB_12", "SPI link to the CB1 / Pi (CS)" }, \
    { "PB_13", "SPI link to the CB1 / Pi (SCK)" },  { "PB_14", "SPI link to the CB1 / Pi (MISO)" }, \
    { "PD_12", "CAN transceiver (RX)" },            { "PD_13", "CAN transceiver (TX)" }, \
    { "PA_5",  "SD card (SCK)" },  { "PA_6", "SD card (MISO)" }, { "PA_7", "SD card (MOSI)" }, { "PA_8", "SD card (CS)" }, \
    { "PA_11", "USB" },            { "PA_12", "USB" }, \
    { "PA_13", "SWD" },            { "PA_14", "SWD" }, \
    { "PF_0",  "8 MHz crystal" },  { "PF_1",  "8 MHz crystal" }, \
    { "PE_8",  "console UART (TX)" }, { "PE_9", "console UART (RX)" },

#endif
