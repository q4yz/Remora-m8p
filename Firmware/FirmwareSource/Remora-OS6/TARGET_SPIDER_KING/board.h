#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    Fysetc Spider King

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "Fysetc Spider King"

#define BOARD_SD_SPI_PINS       PA_7, PA_6, PA_5, PA_4    // SD card: mosi, miso, sclk, cs
#define BOARD_LINK_SPI          , SPI2, PB_12          // SPI link to the host: SPI, slave select pin

// the SD card is released after config.txt is read: its DMA disturbs the SPI slave DMA on the STM32F4
#define BOARD_SD_RELEASE

#endif
