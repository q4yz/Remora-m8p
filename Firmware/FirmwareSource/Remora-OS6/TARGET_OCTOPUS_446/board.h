#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    BTT Octopus (STM32F446)

    What main.cpp needs to know about this board (PORTING.md): the SD
    card, the SPI link to the LinuxCNC host and what else the board
    has. The MCU family's part is TARGET_<FAMILY>/family.h.
************************************************************************/

#define BOARD_NAME              "BTT Octopus (STM32F446)"

#define BOARD_SD_SDIO                               // SD card on SDIO
#define BOARD_LINK_SPI          , SPI1, PA_4          // SPI link to the host: SPI, slave select pin

// the SD card is released after config.txt is read: its DMA disturbs the SPI slave DMA on the STM32F4
#define BOARD_SD_RELEASE

// config.txt upload over the SPI link while LinuxCNC is offline (drivers/sdUpload)
#define BOARD_SD_UPLOAD

#endif
