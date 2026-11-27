#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    Makerbase Robin Nano 3

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "Makerbase Robin Nano 3"

#define BOARD_SD_SPI_PINS       PC_12, PC_11, PC_10, PC_9    // SD card: mosi, miso, sclk, cs
#define BOARD_LINK_SPI          , SPI1, PE_10          // SPI link to the host: SPI, slave select pin

// the SD card is released after config.txt is read: its DMA disturbs the SPI slave DMA on the STM32F4
#define BOARD_SD_RELEASE

#endif
