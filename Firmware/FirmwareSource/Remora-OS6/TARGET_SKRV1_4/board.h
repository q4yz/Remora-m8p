#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    BTT SKR V1.4

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "BTT SKR V1.4"

#define BOARD_SD_SPI_PINS       P0_9, P0_8, P0_7, P0_6    // SD card: mosi, miso, sclk, cs
#define BOARD_LINK_SPI                              // SPI link to the host: fixed in the comms driver

#endif
