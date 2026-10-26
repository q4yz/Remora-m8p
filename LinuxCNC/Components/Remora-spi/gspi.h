/********************************************************************
* Description:  gspi.h
*               Generic Linux SPI backend for remora-spi: spidev for the
*               SPI link and the GPIO character device for the PRU reset
*               pin. Used on boards without a register level driver, e.g.
*               the BigTreeTech CB1 (Allwinner H616). The Raspberry Pi keeps
*               its bcm2835 / RP1 drivers.
*
* License: GPL Version 3
********************************************************************/

#ifndef GSPI_H
#define GSPI_H

#include <stdint.h>

#define GSPI_DEFAULT_GPIOCHIP   "/dev/gpiochip0"
#define GSPI_DEFAULT_FREQ       8000000         // 8 MHz, safe with jumper wires

// SPI link
const char *gspi_find_device(void);     // first /dev/spidevB.C that exists, NULL if none
int  gspi_init(const char *device, uint32_t speed_hz);
int  gspi_transfer(const void *txbuf, void *rxbuf, uint32_t len);
void gspi_close(void);

// PRU reset pin. pin is a GPIO line offset ("233") or an Allwinner style
// port name ("PH9" = (H - A) * 32 + 9) on the given gpiochip.
int  gspi_reset_init(const char *chip, const char *pin);
void gspi_reset_set(int value);
void gspi_reset_close(void);

// Optional chip select on a GPIO, driven low around every frame. For boards whose SPI chip
// select is not on the header (BigTreeTech CB1: the CE0 position is not connected).
int  gspi_cs_init(const char *chip, const char *pin);
void gspi_cs_close(void);

// parse a pin name or number into a line offset, -1 if invalid
int  gspi_parse_pin(const char *pin);

#endif
