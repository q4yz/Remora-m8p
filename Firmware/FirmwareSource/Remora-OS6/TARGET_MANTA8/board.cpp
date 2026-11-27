/***********************************************************************
    BTT Manta M8P: what only this board needs (board.h has its pins)
************************************************************************/

#include "mbed.h"
#include "pinmap.h"
#include "PeripheralPins.h"

#include "sdBus.h"

// The SD card's SPI1 lines (PA5 SCK, PA6 MISO, PA7 MOSI, PA8 CS) also run to the stepper driver
// sockets. mbed's SDBlockDevice::deinit() leaves them driven for good, so between card accesses they
// go to analog (high impedance, the reset state): the MCU never drives a line a driver module or a
// jumper may hold. The card has its own pull-up on CS, it stays deselected.
void sdBusRelease(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}

void sdBusAcquire(void)
{
    // CS high (deselected) before SCK and MOSI come back
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
    pinmap_pinout(PA_5, PinMap_SPI_SCLK);
    pinmap_pinout(PA_6, PinMap_SPI_MISO);
    pinmap_pinout(PA_7, PinMap_SPI_MOSI);
}
