#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    BTT SKR Mini E3

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "BTT SKR Mini E3"

#define BOARD_SD_SPI_PINS       PA_7, PA_6, PA_5, PA_4    // SD card: mosi, miso, sclk, cs
#define BOARD_LINK_SPI          , SPI1, PC_1          // SPI link to the host: SPI, slave select pin

// the SD card shares its SPI with the link: released after config.txt is read
#define BOARD_SD_RELEASE

#endif
