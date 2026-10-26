/********************************************************************
* Description:  gspi.c
*               Generic Linux SPI backend for remora-spi, see gspi.h
*
*               Everything that can block or allocate (open, configure,
*               request the GPIO line) happens at load time. The servo
*               thread only issues one SPI_IOC_MESSAGE ioctl per frame and
*               one GPIO_V2_LINE_SET_VALUES ioctl per reset pin update.
*
* License: GPL Version 3
********************************************************************/

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>

// spi-dw.h (Raspberry Pi 5 driver) defines its own copies of the SPI mode flags, drop the
// kernel's so the Pi drivers included after this file see exactly their own definitions
#undef SPI_CPHA
#undef SPI_CPOL
#undef SPI_MODE_0
#undef SPI_MODE_1
#undef SPI_MODE_2
#undef SPI_MODE_3
#undef SPI_CS_HIGH
#undef SPI_LSB_FIRST
#undef SPI_3WIRE
#undef SPI_LOOP
#undef SPI_NO_CS
#undef SPI_READY

#include "gspi.h"

static int gspi_fd = -1;
static uint32_t gspi_speed;
static int gspi_error_reported = 0;

static int gspi_reset_fd = -1;
static int gspi_reset_value = -1;

static int gspi_cs_fd = -1;             // optional GPIO chip select, -1: the controller's own CS

static int gspi_line_set(int fd, int value);


// Keep the SPI controller powered between transfers. With runtime PM the
// controller may otherwise be suspended between frames and resumed by the
// next transfer, which adds latency. Best effort, needs root.
static void gspi_keep_powered(const char *device)
{
    const char *name = strrchr(device, '/');
    char path[256];
    int fd;

    name = name ? name + 1 : device;

    // spidevX.Y/device is the spi device, its controller is two levels up
    // (spi_master/spiX), the platform device three levels up
    snprintf(path, sizeof(path), "/sys/class/spidev/%s/device/../../../power/control", name);

    fd = rtapi_open_as_root(path, O_WRONLY);
    if (fd < 0)
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "gspi: cannot keep SPI controller powered (%s)\n", path);
        return;
    }

    if (write(fd, "on", 2) != 2)
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "gspi: writing %s failed\n", path);
    }
    close(fd);
}


const char *gspi_find_device(void)
{
    static char path[32];
    int bus, cs;

    // the CB1 only has SPI1 on its header, so don't assume bus 0
    for (bus = 0; bus < 8; bus++)
    {
        for (cs = 0; cs < 4; cs++)
        {
            snprintf(path, sizeof(path), "/dev/spidev%d.%d", bus, cs);
            if (access(path, F_OK) == 0) return path;
        }
    }
    return NULL;
}


int gspi_init(const char *device, uint32_t speed_hz)
{
    uint8_t mode = 0;           // SPI mode 0: CPOL 0, CPHA 0
    uint8_t bits = 8;
    uint8_t lsb = 0;

    gspi_fd = rtapi_open_as_root(device, O_RDWR);
    if (gspi_fd < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "gspi: cannot open %s: %s\n", device, strerror(errno));
        rtapi_print_msg(RTAPI_MSG_ERR, "gspi: enable the SPI controller and spidev with a device tree overlay, or set spi_dev\n");
        return 0;
    }

    if (ioctl(gspi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(gspi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(gspi_fd, SPI_IOC_WR_LSB_FIRST, &lsb) < 0 ||
        ioctl(gspi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "gspi: cannot configure %s: %s\n", device, strerror(errno));
        gspi_close();
        return 0;
    }

    // read back what the controller could actually do
    if (ioctl(gspi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &gspi_speed) < 0)
    {
        gspi_speed = speed_hz;
    }

    gspi_keep_powered(device);

    rtapi_print_msg(RTAPI_MSG_ERR, "gspi: using %s, mode 0, %u Hz\n", device, gspi_speed);
    return 1;
}


int gspi_transfer(const void *txbuf, void *rxbuf, uint32_t len)
{
    struct spi_ioc_transfer tr;

    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)txbuf;
    tr.rx_buf = (unsigned long)rxbuf;
    tr.len = len;
    tr.speed_hz = gspi_speed;
    tr.bits_per_word = 8;

    // GPIO chip select (CB1: the CE0 position of the header is not connected): low for the
    // whole frame, the rising edge after it tells the Remora firmware the frame is complete
    if (gspi_cs_fd >= 0) gspi_line_set(gspi_cs_fd, 0);

    int result = ioctl(gspi_fd, SPI_IOC_MESSAGE(1), &tr);

    if (gspi_cs_fd >= 0) gspi_line_set(gspi_cs_fd, 1);

    if (result < 0)
    {
        // never return a stale frame, the caller rejects an invalid header
        memset(rxbuf, 0, len);

        if (!gspi_error_reported)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "gspi: SPI transfer failed: %s\n", strerror(errno));
            gspi_error_reported = 1;
        }
        return 0;
    }

    return 1;
}


void gspi_close(void)
{
    if (gspi_fd >= 0)
    {
        close(gspi_fd);
        gspi_fd = -1;
    }
}


int gspi_parse_pin(const char *pin)
{
    char *end;
    long n;

    if (!pin || !*pin) return -1;

    // Allwinner style port name, e.g. PH9 or PH_9
    if ((pin[0] == 'P' || pin[0] == 'p') && isalpha((unsigned char)pin[1]))
    {
        int port = toupper((unsigned char)pin[1]) - 'A';
        const char *num = pin + 2;
        if (*num == '_') num++;

        n = strtol(num, &end, 10);
        if (end == num || *end != '\0' || n < 0 || n > 31) return -1;
        return port * 32 + (int)n;
    }

    // plain line offset
    n = strtol(pin, &end, 10);
    if (end == pin || *end != '\0' || n < 0 || n > 1023) return -1;
    return (int)n;
}


// Claim one GPIO line as an output with the given initial level, returns the line fd or -1
static int gspi_claim_output(const char *chip, const char *pin, const char *what, int initial)
{
#ifdef GPIO_V2_GET_LINE_IOCTL
    struct gpio_v2_line_request req;
    int line, chip_fd;

    line = gspi_parse_pin(pin);
    if (line < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "gspi: invalid %s '%s'\n", what, pin);
        return -1;
    }

    chip_fd = rtapi_open_as_root(chip, O_RDWR);
    if (chip_fd < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "gspi: cannot open %s: %s\n", chip, strerror(errno));
        return -1;
    }

    memset(&req, 0, sizeof(req));
    req.offsets[0] = line;
    req.num_lines = 1;
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    // set the initial level in the same request, no glitch when the line becomes an output
    req.config.num_attrs = 1;
    req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    req.config.attrs[0].attr.values = initial ? 1 : 0;
    req.config.attrs[0].mask = 1;
    snprintf(req.consumer, sizeof(req.consumer), "remora-spi %s", what);

    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "gspi: cannot claim line %d on %s for %s: %s\n", line, chip, what, strerror(errno));
        close(chip_fd);
        return -1;
    }
    close(chip_fd);

    rtapi_print_msg(RTAPI_MSG_ERR, "gspi: %s on %s line %d (%s)\n", what, chip, line, pin);
    return req.fd;
#else
    (void)chip; (void)pin; (void)initial;
    rtapi_print_msg(RTAPI_MSG_ERR, "gspi: built without GPIO v2 support, %s is not available\n", what);
    return -1;
#endif
}


static int gspi_line_set(int fd, int value)
{
#ifdef GPIO_V2_GET_LINE_IOCTL
    struct gpio_v2_line_values values;

    values.bits = value ? 1 : 0;
    values.mask = 1;
    return ioctl(fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values);
#else
    (void)fd; (void)value;
    return -1;
#endif
}


int gspi_reset_init(const char *chip, const char *pin)
{
    gspi_reset_fd = gspi_claim_output(chip, pin, "PRU reset", 0);
    gspi_reset_value = 0;
    return gspi_reset_fd >= 0;
}


void gspi_reset_set(int value)
{
    value = value ? 1 : 0;

    // the reset pin rarely changes, only call into the kernel on a change
    if (gspi_reset_fd < 0 || value == gspi_reset_value) return;

    if (gspi_line_set(gspi_reset_fd, value) == 0)
    {
        gspi_reset_value = value;
    }
}


int gspi_cs_init(const char *chip, const char *pin)
{
    // chip select is active low, idle high until the first frame
    gspi_cs_fd = gspi_claim_output(chip, pin, "chip select", 1);
    return gspi_cs_fd >= 0;
}


void gspi_cs_close(void)
{
    if (gspi_cs_fd >= 0)
    {
        gspi_line_set(gspi_cs_fd, 1);
        close(gspi_cs_fd);
        gspi_cs_fd = -1;
    }
}


void gspi_reset_close(void)
{
    if (gspi_reset_fd >= 0)
    {
        close(gspi_reset_fd);
        gspi_reset_fd = -1;
    }
}
