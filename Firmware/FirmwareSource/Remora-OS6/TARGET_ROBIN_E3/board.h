#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    Makerbase Robin E3

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "Makerbase Robin E3"

#define BOARD_SD_SPI_PINS       PB_15, PB_14, PB_13, PA_15    // SD card: mosi, miso, sclk, cs
#define BOARD_LINK_SPI          , SPI1, PA_4          // SPI link to the host: SPI, slave select pin

#endif
