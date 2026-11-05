/********************************************************************
* Description:  sunxi_spi.h
*               Register level SPI for Allwinner SoCs with the sun6i / H3
*               style SPI controller (H3, H5, H6, H616 / H618: BigTreeTech
*               CB1, Orange Pi Zero 2 ...), used by the generic driver
*               (gspi.c) in place of the spidev ioctl.
*
*               The kernel still owns the controller: its spi-sun6i driver
*               sets up the pins, clocks, reset and the clock divider through
*               spidev at load time. The servo thread then runs the frames
*               straight on the registers (mmap of /dev/mem), polled, without
*               a system call, an interrupt or a context switch.
*
* License: GPL Version 3
********************************************************************/

#ifndef SUNXI_SPI_H
#define SUNXI_SPI_H

#include <stdint.h>

// Take over the controller behind an open, configured spidev device. cs is the controller's
// chip select (spidevB.C). Returns 1 on success, 0 if the controller is not a supported
// Allwinner one or cannot be mapped (spidev stays usable).
int  sunxi_spi_init(const char *device, int spidev_fd, uint32_t speed_hz, int cs);
int  sunxi_spi_active(void);
int  sunxi_spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len);
void sunxi_spi_close(void);

// The Remora board is selected by a GPIO: stop driving the controller's chip select and, if the
// line is on an Allwinner PIO, drive it straight on the PIO data register (the line must already
// be claimed as an output). chip_fd: the gpiochip, -1 if unknown; line: the line offset on it.
// Returns 1 if the register chip select is used, 0: keep the GPIO ioctl.
int  sunxi_spi_cs_init(int chip_fd, int line);
void sunxi_spi_cs_close(void);

#endif
