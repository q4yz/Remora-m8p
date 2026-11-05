/********************************************************************
* Description:  sunxi_spi.c
*               Register level SPI for Allwinner SoCs, see sunxi_spi.h
*
*               Registers as in the kernel's drivers/spi/spi-sun6i.c. The
*               kernel driver leaves the controller enabled, in master mode,
*               with the clock divider of the last transfer and all its
*               interrupts disabled, so a polled transfer here never meets
*               the kernel's interrupt handler. Runtime PM must stay off
*               (gspi_keep_powered), the suspend asserts the controller reset.
*
* License: GPL Version 3
********************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>

#include "sunxi_spi.h"

#define SUNXI_GBL_CTL           0x04
#define SUNXI_GBL_BUS_ENABLE    (1u << 0)
#define SUNXI_GBL_MASTER        (1u << 1)

#define SUNXI_TFR_CTL           0x08
#define SUNXI_TFR_CS_MASK       0x30
#define SUNXI_TFR_CS(cs)        (((cs) << 4) & SUNXI_TFR_CS_MASK)
#define SUNXI_TFR_CS_MANUAL     (1u << 6)
#define SUNXI_TFR_CS_LEVEL      (1u << 7)
#define SUNXI_TFR_DHB           (1u << 8)
#define SUNXI_TFR_XCH           (1u << 31)

#define SUNXI_INT_CTL           0x10
#define SUNXI_INT_STA           0x14
#define SUNXI_FIFO_CTL          0x18
#define SUNXI_FIFO_RF_RST       (1u << 15)
#define SUNXI_FIFO_TF_RST       (1u << 31)
#define SUNXI_FIFO_STA          0x1c
#define SUNXI_CLK_CTL           0x24
#define SUNXI_BURST_CNT         0x30
#define SUNXI_XMIT_CNT          0x34
#define SUNXI_BURST_CTL_CNT     0x38
#define SUNXI_TXDATA            0x200
#define SUNXI_RXDATA            0x300

#define SUNXI_FIFO_DEPTH        64      // H3 / H616; the A31 / H6 FIFO is 128, 64 works on both

// PIO (pinctrl@300b000, r_pio@7022000): 0x24 bytes per port, data register at 0x10
#define SUNXI_PIO_BANK_SIZE     0x24
#define SUNXI_PIO_DAT           0x10

static volatile uint8_t *sunxi_regs = NULL;
static uint32_t sunxi_speed;            // SPI clock, for the timeout
static int sunxi_cs = -1;               // controller chip select, -1: GPIO chip select
static int sunxi_error_reported = 0;

static volatile uint8_t *sunxi_pio_map = NULL;
static volatile uint32_t *sunxi_cs_dat = NULL;     // PIO data register of the GPIO chip select
static uint32_t sunxi_cs_bit;

static inline uint32_t sunxi_read(uint32_t reg)
{
    return *(volatile uint32_t *)(sunxi_regs + reg);
}

static inline void sunxi_write(uint32_t reg, uint32_t value)
{
    *(volatile uint32_t *)(sunxi_regs + reg) = value;
}


// Map one page of physical memory containing base, returns a pointer to base or NULL
static volatile uint8_t *sunxi_map(unsigned long base, volatile uint8_t **page)
{
    unsigned long page_base = base & ~0xffful;
    void *map;
    int fd;

    fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: cannot open /dev/mem: %s\n", strerror(errno));
        return NULL;
    }

    map = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
    close(fd);

    if (map == MAP_FAILED)
    {
        // e.g. a kernel with CONFIG_IO_STRICT_DEVMEM, which hides the registers of bound drivers
        rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: cannot map 0x%08lx: %s\n", base, strerror(errno));
        return NULL;
    }

    *page = map;
    return (volatile uint8_t *)map + (base - page_base);
}


// Read a small sysfs file below /sys/class/spidev/<name>/device/../../../ (the controller)
static ssize_t sunxi_read_controller_file(const char *name, const char *file, char *buf, size_t size)
{
    char path[PATH_MAX];
    ssize_t len;
    int fd;

    snprintf(path, sizeof(path), "/sys/class/spidev/%s/device/../../../%s", name, file);
    fd = open(path, O_RDONLY);
    if (fd < 0) return -1;

    len = read(fd, buf, size - 1);
    close(fd);
    if (len < 0) return -1;

    buf[len] = '\0';
    return len;
}


// Does the NUL separated compatible list contain one of the supported controllers?
static int sunxi_compatible(const char *list, ssize_t len)
{
    static const char *supported[] = { "allwinner,sun8i-h3-spi", "allwinner,sun6i-a31-spi", NULL };
    const char *p;
    int i;

    for (p = list; p < list + len; p += strlen(p) + 1)
    {
        for (i = 0; supported[i]; i++)
        {
            if (!strcmp(p, supported[i])) return 1;
        }
    }
    return 0;
}


int sunxi_spi_init(const char *device, int spidev_fd, uint32_t speed_hz, int cs)
{
    const char *name = strrchr(device, '/');
    char path[PATH_MAX], real[PATH_MAX], buf[256];
    struct spi_ioc_transfer tr;
    uint8_t dummy = 0;
    unsigned long base = 0;
    volatile uint8_t *page;
    char *component;
    ssize_t len;

    name = name ? name + 1 : device;

    // the controller must be one the sun6i driver drives with a clock divider (not the R329 / D1 one)
    len = sunxi_read_controller_file(name, "of_node/compatible", buf, sizeof(buf));
    if (len <= 0 || !sunxi_compatible(buf, len))
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "sunxi_spi: %s is not on an Allwinner H3 / H6 / H616 SPI controller\n", device);
        return 0;
    }

    // its register base is in the platform device name: .../soc/5011000.spi/spi_master/spi1/spi1.0
    snprintf(path, sizeof(path), "/sys/class/spidev/%s/device", name);
    if (!realpath(path, real))
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: cannot resolve %s\n", path);
        return 0;
    }
    for (component = strtok(real, "/"); component; component = strtok(NULL, "/"))
    {
        char suffix[8];
        if (sscanf(component, "%lx.%7s", &base, suffix) == 2 && !strcmp(suffix, "spi")) break;
        base = 0;
    }
    if (!base)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: no controller address in %s\n", path);
        return 0;
    }

    // the controller is only usable while it is runtime resumed: its suspend asserts the reset and
    // gates the clocks. gspi_keep_powered wrote "on" to power/control, without it the kernel would
    // suspend the controller 2 s after its last own transfer.
    len = sunxi_read_controller_file(name, "power/control", buf, sizeof(buf));
    if (len <= 0 || strncmp(buf, "on", 2) != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: the SPI controller is not kept powered (power/control is not 'on')\n");
        return 0;
    }

    // one transfer through the kernel at our clock: it resumes the controller and sets the clock
    // divider, mode 0, master and manual chip select. The registers keep these afterwards.
    sunxi_speed = speed_hz ? speed_hz : 1000000;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)&dummy;
    tr.len = 1;
    tr.speed_hz = sunxi_speed;
    tr.bits_per_word = 8;
    if (ioctl(spidev_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: priming transfer failed: %s\n", strerror(errno));
        return 0;
    }

    sunxi_regs = sunxi_map(base, &page);
    if (!sunxi_regs) return 0;

    if ((sunxi_read(SUNXI_GBL_CTL) & (SUNXI_GBL_BUS_ENABLE | SUNXI_GBL_MASTER)) != (SUNXI_GBL_BUS_ENABLE | SUNXI_GBL_MASTER) ||
        sunxi_read(SUNXI_CLK_CTL) == 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: controller at 0x%08lx not set up by the kernel (GBL_CTL 0x%08x)\n",
                        base, sunxi_read(SUNXI_GBL_CTL));
        munmap((void *)page, 0x1000);
        sunxi_regs = NULL;
        return 0;
    }

    sunxi_write(SUNXI_INT_CTL, 0);          // polled, the kernel's interrupt handler never runs
    sunxi_cs = cs;

    rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: register access to the SPI controller at 0x%08lx, CLK_CTL 0x%08x\n",
                    base, sunxi_read(SUNXI_CLK_CTL));
    return 1;
}


int sunxi_spi_active(void)
{
    return sunxi_regs != NULL;
}


int sunxi_spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    volatile uint8_t *txd = sunxi_regs + SUNXI_TXDATA;
    volatile uint8_t *rxd = sunxi_regs + SUNXI_RXDATA;
    uint32_t sent = 0, received = 0, tcr;
    long long deadline;

    sunxi_write(SUNXI_INT_STA, ~0u);
    sunxi_write(SUNXI_FIFO_CTL, SUNXI_FIFO_RF_RST | SUNXI_FIFO_TF_RST);

    // full duplex, manual chip select
    tcr = sunxi_read(SUNXI_TFR_CTL) & ~(SUNXI_TFR_DHB | SUNXI_TFR_XCH);
    tcr |= SUNXI_TFR_CS_MANUAL;
    if (sunxi_cs >= 0)
    {
        tcr = (tcr & ~(SUNXI_TFR_CS_MASK | SUNXI_TFR_CS_LEVEL)) | SUNXI_TFR_CS(sunxi_cs);
    }
    if (sunxi_cs_dat) *sunxi_cs_dat &= ~sunxi_cs_bit;       // GPIO chip select low
    sunxi_write(SUNXI_TFR_CTL, tcr);

    sunxi_write(SUNXI_BURST_CTL_CNT, len & 0xffffff);
    sunxi_write(SUNXI_BURST_CNT, len);
    sunxi_write(SUNXI_XMIT_CNT, len);

    while (sent < len && sent < SUNXI_FIFO_DEPTH) *txd = tx[sent++];

    sunxi_write(SUNXI_TFR_CTL, tcr | SUNXI_TFR_XCH);

    // 4 times the time on the wire plus 100 us
    deadline = rtapi_get_time() + (long long)len * 8 * 4000000000LL / sunxi_speed + 100000;

    while (received < len)
    {
        uint32_t fsr = sunxi_read(SUNXI_FIFO_STA);
        uint32_t rx_count = fsr & 0xff;
        uint32_t tx_free = SUNXI_FIFO_DEPTH - ((fsr >> 16) & 0xff);

        while (rx_count-- && received < len) rx[received++] = *rxd;
        while (tx_free-- && sent < len) *txd = tx[sent++];

        if (rtapi_get_time() > deadline) break;
    }

    if (sunxi_cs >= 0) sunxi_write(SUNXI_TFR_CTL, tcr | SUNXI_TFR_CS_LEVEL);
    if (sunxi_cs_dat) *sunxi_cs_dat |= sunxi_cs_bit;        // rising edge: frame complete

    if (received < len)
    {
        // never return a stale frame, the caller rejects an invalid header
        memset(rx, 0, len);

        if (!sunxi_error_reported)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: SPI transfer timeout, %u of %u bytes\n", received, len);
            sunxi_error_reported = 1;
        }
        return 0;
    }

    return 1;
}


void sunxi_spi_close(void)
{
    if (sunxi_regs)
    {
        munmap((void *)((uintptr_t)sunxi_regs & ~(uintptr_t)0xfff), 0x1000);
        sunxi_regs = NULL;
    }
    sunxi_spi_cs_close();
}


int sunxi_spi_cs_init(int chip_fd, int line)
{
    struct gpiochip_info info;
    unsigned long base;
    volatile uint8_t *page;
    char suffix[16];
    volatile uint8_t *pio;

    if (!sunxi_regs) return 0;
    sunxi_cs = -1;      // the GPIO selects the Remora board, leave the controller's chip select alone

    if (chip_fd < 0) return 0;

    // the gpiochip label is the pinctrl device name, e.g. 300b000.pinctrl (PA ... PI) or 7022000.pinctrl (PL)
    memset(&info, 0, sizeof(info));
    if (ioctl(chip_fd, GPIO_GET_CHIPINFO_IOCTL, &info) < 0 ||
        sscanf(info.label, "%lx.%15s", &base, suffix) != 2 || strcmp(suffix, "pinctrl") != 0)
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "sunxi_spi: gpiochip '%s' is not an Allwinner PIO, chip select stays on the GPIO ioctl\n",
                        info.label);
        return 0;
    }

    pio = sunxi_map(base, &page);
    if (!pio) return 0;

    sunxi_pio_map = page;
    sunxi_cs_dat = (volatile uint32_t *)(pio + (line / 32) * SUNXI_PIO_BANK_SIZE + SUNXI_PIO_DAT);
    sunxi_cs_bit = 1u << (line % 32);
    *sunxi_cs_dat |= sunxi_cs_bit;

    rtapi_print_msg(RTAPI_MSG_ERR, "sunxi_spi: chip select on the PIO data register at 0x%08lx\n",
                    base + (line / 32) * SUNXI_PIO_BANK_SIZE + SUNXI_PIO_DAT);
    return 1;
}


void sunxi_spi_cs_close(void)
{
    if (sunxi_pio_map)
    {
        *sunxi_cs_dat |= sunxi_cs_bit;
        munmap((void *)sunxi_pio_map, 0x1000);
        sunxi_pio_map = NULL;
        sunxi_cs_dat = NULL;
    }
}
