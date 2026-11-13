#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    BTT SKR 3

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "BTT SKR 3"

#define BOARD_SD_SDIO                               // SD card on SDIO
#define BOARD_LINK_SPI          , SPI1, PA_4          // SPI link to the host: SPI, slave select pin

#endif
