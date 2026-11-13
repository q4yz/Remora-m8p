#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    BTT Octopus V1.0 / V1.1 and Octopus Pro V1.0 / V1.1 with the STM32F446ZE
    (12 MHz crystal): the same MCU pins, one firmware

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "BTT Octopus / Octopus Pro (STM32F446)"

#define BOARD_SD_SDIO                               // SD card on SDIO
// SPI link to the host on SPI1, hardware NSS: PA4 CS, PA5 SCK, PA6 MISO, PA7 MOSI (EXP2 / SPI header)
#define BOARD_LINK_SPI          , SPI1, PA_4

// the SD card is released after config.txt is read: its DMA disturbs the SPI slave DMA on the STM32F4
#define BOARD_SD_RELEASE

// config.txt upload over the SPI link while LinuxCNC is offline (drivers/sdUpload)
#define BOARD_SD_UPLOAD

// Pins the board itself uses: config.txt is refused when a module names one (two outputs driving
// each other, or the firmware's own link and console broken)
#define BOARD_RESERVED_PINS     { "PA_4",  "SPI link to the CB1 / Pi (CS)" },   { "PA_5", "SPI link to the CB1 / Pi (SCK)" },     { "PA_6",  "SPI link to the CB1 / Pi (MISO)" }, { "PA_7", "SPI link to the CB1 / Pi (MOSI)" },     { "PC_8",  "SD card (D0)" },  { "PC_9",  "SD card (D1)" },  { "PC_10", "SD card (D2)" },     { "PC_11", "SD card (D3)" },  { "PC_12", "SD card (CLK)" }, { "PD_2",  "SD card (CMD)" },     { "PA_11", "USB" },           { "PA_12", "USB" },     { "PA_13", "SWD" },           { "PA_14", "SWD" },     { "PH_0",  "12 MHz crystal" }, { "PH_1", "12 MHz crystal" },     { "PA_9",  "console UART (TX)" }, { "PA_10", "console UART (RX)" },   /* USART1, the TFT header */

#endif
