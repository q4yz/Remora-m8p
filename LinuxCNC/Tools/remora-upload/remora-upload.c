/********************************************************************
* Description:  remora-upload.c
*               Writes firmware.bin or config.txt to the SD card of a
*               Remora board over the SPI link, while LinuxCNC is not
*               running. Uses the Linux spidev driver, optionally with a
*               chip select on a GPIO (BTT CB1, like remora-spi).
*
*               Build:  gcc -O2 -Wall -o remora-upload remora-upload.c
*               Use:    sudo ./remora-upload firmware.bin
*                       sudo ./remora-upload --sync config.txt
*
*               The board side is Firmware/FirmwareSource/Remora-OS6/
*               drivers/sdUpload, the frame layout below must match
*               sdUpload.h.
*
********************************************************************/

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>

#define SPI_BUFF_SIZE       64
#define PRU_DATA            0x64617461      // "data" SPI payload, the board runs normally
#define PRU_UPLOAD          0x75706c64      // "upld" SPI payload

#define UPLOAD_DATA_MAX     44

#define UPLOAD_CMD_POLL     0
#define UPLOAD_CMD_BEGIN    1
#define UPLOAD_CMD_DATA     2
#define UPLOAD_CMD_END      3
#define UPLOAD_CMD_REBOOT   4
#define UPLOAD_CMD_INFO     6

#define UPLOAD_ST_OK        1
#define UPLOAD_ST_ERROR     2

#define UPLOAD_ERR_OFFSET   5
#define UPLOAD_ERR_CMD      10

#define UPLOAD_INFO_CONFIG  0x01            // config.txt was found at boot
#define UPLOAD_INFO_ERROR   0x02            // it did not load

#define POLL_INTERVAL_US    200             // gap between transfers, lets the board prepare its reply
#define RESEND_MS           250             // re-send a command that got no reply in this time
#define STALE_REPLIES       3               // or once this many valid replies to an older command came back
#define BEGIN_TIMEOUT_MS    15000           // covers a board that is still booting / watchdog resetting
#define DATA_TIMEOUT_MS     10000
#define END_TIMEOUT_MS      30000           // write, read back and rename on the board
#define BOOT_TIMEOUT_MS     20000           // reboot (and flashing a firmware) until the board answers again
#define BOOT_SETTLE_MS      1500            // the board's START state, before LinuxCNC may connect

#define DEFAULT_GPIOCHIP    "/dev/gpiochip0"

#pragma pack(push, 1)

typedef struct
{
    uint32_t header;
    uint8_t  cmd;
    uint8_t  seq;
    uint16_t len;
    uint32_t arg0;
    uint32_t arg1;
    uint8_t  data[UPLOAD_DATA_MAX];
    uint32_t crc;
} uploadCmd_t;

typedef struct
{
    uint32_t header;
    uint8_t  status;
    uint8_t  seq;
    uint8_t  error;
    uint8_t  flags;                         // UPLOAD_INFO_* in an INFO reply
    uint32_t nextOffset;                    // INFO: size of config.txt
    uint32_t fileCrc;                       // INFO: its CRC-32
    uint8_t  spare1[UPLOAD_DATA_MAX];
    uint32_t crc;
} uploadReply_t;

#pragma pack(pop)

_Static_assert(sizeof(uploadCmd_t) == SPI_BUFF_SIZE, "command frame size");
_Static_assert(sizeof(uploadReply_t) == SPI_BUFF_SIZE, "reply frame size");


static int spi_fd = -1;
static int cs_fd = -1;                      // GPIO chip select, -1: the controller's own CS
static uint32_t spi_speed = 4000000;
static uint8_t seq;


static const char* error_text(uint8_t error)
{
    switch (error)
    {
        case 1:  return "LinuxCNC is communicating with the board";
        case 2:  return "file name not allowed (firmware.bin or config.txt only)";
        case 3:  return "file too big or size mismatch";
        case 4:  return "no upload in progress";
        case 5:  return "data offset out of sequence";
        case 6:  return "SD card could not be mounted";
        case 7:  return "SD card write failed";
        case 8:  return "verification failed, file on the SD card does not match";
        case 9:  return "could not replace the file on the SD card";
        case 10: return "unknown command (firmware too old for this, flash a new firmware.bin)";
        default: return "unknown error";
    }
}


static uint32_t crc32(uint32_t crc, const uint8_t* data, size_t len)
{
    // standard CRC-32 (IEEE 802.3), same as the firmware
    crc = ~crc;
    while (len--)
    {
        crc ^= *data++;
        for (int k = 0; k < 8; k++)
        {
            crc = (crc >> 1) ^ (0xEDB88320 & (0 - (crc & 1)));
        }
    }
    return ~crc;
}


static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}


// first /dev/spidevB.C that exists, the same choice as remora-spi without spi_dev
static const char* find_spi_device(void)
{
    static char path[32];
    for (int bus = 0; bus < 8; bus++)
    {
        for (int cs = 0; cs < 4; cs++)
        {
            snprintf(path, sizeof(path), "/dev/spidev%d.%d", bus, cs);
            if (access(path, F_OK) == 0) return path;
        }
    }
    return NULL;
}


static int spi_open(const char* device)
{
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;

    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0)
    {
        fprintf(stderr, "cannot open %s: %s\n", device, strerror(errno));
        fprintf(stderr, "enable spidev (Raspberry Pi: \"dtparam=spi=on\" in /boot/firmware/config.txt, "
                        "CB1: the spidev overlay) and reboot\n");
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
    {
        fprintf(stderr, "cannot configure %s: %s\n", device, strerror(errno));
        return -1;
    }

    return 0;
}


// GPIO line offset ("74") or Allwinner style port name ("PC10" = (C - A) * 32 + 10), as remora-spi
static int parse_pin(const char* pin)
{
    char* end;
    long n;

    if (!pin || !*pin) return -1;

    if ((pin[0] == 'P' || pin[0] == 'p') && isalpha((unsigned char)pin[1]))
    {
        int port = toupper((unsigned char)pin[1]) - 'A';
        const char* num = pin + 2;
        if (*num == '_') num++;

        n = strtol(num, &end, 10);
        if (end == num || *end != '\0' || n < 0 || n > 31) return -1;
        return port * 32 + (int)n;
    }

    n = strtol(pin, &end, 10);
    if (end == pin || *end != '\0' || n < 0 || n > 1023) return -1;
    return (int)n;
}


static int cs_set(int value)
{
    struct gpio_v2_line_values values;
    values.bits = value ? 1 : 0;
    values.mask = 1;
    return ioctl(cs_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values);
}


// chip select on a GPIO (CB1: the CE0 position of the header is not connected), idle high
static int cs_open(const char* chip, const char* pin)
{
    struct gpio_v2_line_request req;

    int line = parse_pin(pin);
    if (line < 0)
    {
        fprintf(stderr, "invalid chip select pin '%s'\n", pin);
        return -1;
    }

    int chip_fd = open(chip, O_RDWR);
    if (chip_fd < 0)
    {
        fprintf(stderr, "cannot open %s: %s\n", chip, strerror(errno));
        return -1;
    }

    memset(&req, 0, sizeof(req));
    req.offsets[0] = line;
    req.num_lines = 1;
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    // the initial level in the same request, no glitch when the line becomes an output
    req.config.num_attrs = 1;
    req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    req.config.attrs[0].attr.values = 1;
    req.config.attrs[0].mask = 1;
    snprintf(req.consumer, sizeof(req.consumer), "remora-upload cs");

    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0)
    {
        fprintf(stderr, "cannot claim line %d on %s for the chip select: %s\n", line, chip, strerror(errno));
        if (errno == EBUSY) fprintf(stderr, "the line is in use, is LinuxCNC (remora-spi) still running?\n");
        close(chip_fd);
        return -1;
    }
    close(chip_fd);

    cs_fd = req.fd;
    return 0;
}


static int spi_transfer(const void* tx, void* rx)
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = SPI_BUFF_SIZE;
    tr.speed_hz = spi_speed;
    tr.bits_per_word = 8;

    // low for the whole frame, the rising edge after it tells the board the frame is complete
    if (cs_fd >= 0) cs_set(0);
    int result = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (cs_fd >= 0) cs_set(1);

    if (result < 0)
    {
        fprintf(stderr, "SPI transfer failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}


static void seal(uploadCmd_t* cmd)
{
    cmd->header = PRU_UPLOAD;
    cmd->crc = crc32(0, (uint8_t*)cmd, SPI_BUFF_SIZE - 4);
}


static int reply_valid(const uploadReply_t* reply)
{
    return reply->header == PRU_UPLOAD && reply->crc == crc32(0, (const uint8_t*)reply, SPI_BUFF_SIZE - 4);
}


/* Send a command and poll until the board replies to it. The reply to a
 * frame can only be clocked out with the next frame, so every command is
 * followed by POLL frames. Commands without a reply (lost or corrupted
 * frames, or the board busy writing to the SD card with its SPI paused)
 * are re-sent; the board treats repeated commands as no-ops. */
static int command(uploadCmd_t* cmd, uploadReply_t* reply, int timeout_ms)
{
    uploadCmd_t poll;
    uploadReply_t rx;

    memset(&poll, 0, sizeof(poll));
    poll.cmd = UPLOAD_CMD_POLL;
    seal(&poll);

    cmd->seq = ++seq;
    seal(cmd);

    uint64_t deadline = now_ms() + timeout_ms;
    while (now_ms() < deadline)
    {
        if (spi_transfer(cmd, &rx) < 0) return -1;

        uint64_t resend = now_ms() + RESEND_MS;
        int stale = 0;
        while (now_ms() < resend)
        {
            usleep(POLL_INTERVAL_US);
            if (spi_transfer(&poll, &rx) < 0) return -1;

            if (reply_valid(&rx))
            {
                if (rx.seq == cmd->seq)
                {
                    *reply = rx;
                    return 0;
                }

                // the board is answering but still shows an older reply, the command was lost
                if (++stale >= STALE_REPLIES) break;
            }
        }
    }

    return -1;
}


static int linuxcnc_running(void)
{
    DIR* proc = opendir("/proc");
    struct dirent* entry;
    int running = 0;

    if (!proc) return 0;

    while (!running && (entry = readdir(proc)) != NULL)
    {
        if (!isdigit((unsigned char)entry->d_name[0])) continue;

        char path[300];
        char comm[64] = "";
        snprintf(path, sizeof(path), "/proc/%s/comm", entry->d_name);

        FILE* f = fopen(path, "r");
        if (!f) continue;
        if (fgets(comm, sizeof(comm), f))
        {
            comm[strcspn(comm, "\n")] = '\0';
            running = !strcmp(comm, "rtapi_app") || !strcmp(comm, "linuxcnc") || !strcmp(comm, "milltask");
        }
        fclose(f);
    }

    closedir(proc);
    return running;
}


static uint8_t* read_file(const char* path, long* size)
{
    // firmware images are a few hundred kB, read the whole file
    FILE* f = fopen(path, "rb");
    if (!f)
    {
        fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno));
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    *size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t* data = malloc(*size > 0 ? *size : 1);
    if (!data || fread(data, 1, *size, f) != (size_t)*size)
    {
        fprintf(stderr, "cannot read %s\n", path);
        fclose(f);
        free(data);
        return NULL;
    }
    fclose(f);
    return data;
}


static int upload(const uint8_t* data, long size, const char* file_path, const char* name)
{
    uploadCmd_t cmd;
    uploadReply_t reply;
    uint32_t file_crc = crc32(0, data, size);

    printf("Uploading %s as %s (%ld bytes, CRC %08x)\n", file_path, name, size, file_crc);

    // start the transfer
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd = UPLOAD_CMD_BEGIN;
    cmd.arg0 = size;
    cmd.arg1 = file_crc;
    cmd.len = strlen(name);
    memcpy(cmd.data, name, cmd.len);

    if (command(&cmd, &reply, BEGIN_TIMEOUT_MS) < 0)
    {
        fprintf(stderr, "no reply from the board - check the SPI wiring, the chip select (--spi-cs-pin) "
                        "and that the firmware supports SD upload\n");
        return -1;
    }
    if (reply.status != UPLOAD_ST_OK)
    {
        fprintf(stderr, "board refused the upload: %s\n", error_text(reply.error));
        return -1;
    }

    // send the data, the board's next offset is authoritative
    uint32_t offset = 0;
    int last_percent = -1;
    while (offset < (uint32_t)size)
    {
        uint32_t len = size - offset;
        if (len > UPLOAD_DATA_MAX) len = UPLOAD_DATA_MAX;

        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = UPLOAD_CMD_DATA;
        cmd.arg0 = offset;
        cmd.len = len;
        memcpy(cmd.data, data + offset, len);

        if (command(&cmd, &reply, DATA_TIMEOUT_MS) < 0)
        {
            fprintf(stderr, "\nno reply from the board at offset %u\n", offset);
            return -1;
        }
        if (reply.status != UPLOAD_ST_OK && reply.error != UPLOAD_ERR_OFFSET)
        {
            fprintf(stderr, "\nupload failed at offset %u: %s\n", offset, error_text(reply.error));
            return -1;
        }
        offset = reply.nextOffset;

        int percent = (int)((uint64_t)offset * 100 / size);
        if (percent != last_percent)
        {
            printf("\r  %3d%%", percent);
            fflush(stdout);
            last_percent = percent;
        }
    }
    printf("\n");

    // write the rest, verify on the board and rename into place
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd = UPLOAD_CMD_END;

    if (command(&cmd, &reply, END_TIMEOUT_MS) < 0)
    {
        fprintf(stderr, "no reply from the board while finishing the upload\n");
        return -1;
    }
    if (reply.status != UPLOAD_ST_OK)
    {
        fprintf(stderr, "upload failed: %s\n", error_text(reply.error));
        return -1;
    }
    printf("%s written and verified on the SD card\n", name);
    return 0;
}


// config.txt the board loaded at boot; puts the board into upload mode, reboot it afterwards
static int board_info(uint32_t* size, uint32_t* crc, uint8_t* flags)
{
    uploadCmd_t cmd;
    uploadReply_t reply;

    // LinuxCNC just stopped: the board sees the link as running for a moment longer
    for (int attempt = 0; ; attempt++)
    {
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = UPLOAD_CMD_INFO;

        if (command(&cmd, &reply, BEGIN_TIMEOUT_MS) < 0)
        {
            fprintf(stderr, "no reply from the board - check the SPI wiring, the chip select (--spi-cs-pin) "
                            "and that the firmware supports SD upload\n");
            return -1;
        }
        if (reply.status == UPLOAD_ST_OK || reply.error != 1 || attempt >= 20) break;
        usleep(100 * 1000);
    }
    if (reply.status != UPLOAD_ST_OK)
    {
        fprintf(stderr, "board refused the info request: %s\n", error_text(reply.error));
        return -1;
    }

    *size = reply.nextOffset;
    *crc = reply.fileCrc;
    *flags = reply.flags;
    return 0;
}


static void reboot_board(void)
{
    uploadCmd_t cmd;
    uploadReply_t reply;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd = UPLOAD_CMD_REBOOT;
    command(&cmd, &reply, 1000);
}


// after a reboot: wait until the board sends normal "data" frames again
static int wait_for_board(void)
{
    uploadCmd_t poll;
    uploadReply_t rx;

    memset(&poll, 0, sizeof(poll));
    poll.cmd = UPLOAD_CMD_POLL;             // ignored by the board
    seal(&poll);

    usleep(500 * 1000);                     // the board is still in upload mode until it resets

    uint64_t deadline = now_ms() + BOOT_TIMEOUT_MS;
    while (now_ms() < deadline)
    {
        if (spi_transfer(&poll, &rx) < 0) return -1;
        if (rx.header == PRU_DATA)
        {
            usleep(BOOT_SETTLE_MS * 1000);
            return 0;
        }
        usleep(20 * 1000);
    }

    fprintf(stderr, "the board did not come back within %d s after the reboot\n", BOOT_TIMEOUT_MS / 1000);
    return -1;
}


/* --sync: make the board run FILE as its config.txt. Compares the CRC of the
 * config the board loaded at boot with FILE and uploads only when they
 * differ. Either way the board reboots (answering INFO stops its threads),
 * so it starts fresh just before LinuxCNC connects. */
static int sync_config(const char* file_path)
{
    long size;
    uint8_t* data = read_file(file_path, &size);
    if (!data) return 1;

    uint32_t local_crc = crc32(0, data, size);
    uint32_t board_size, board_crc;
    uint8_t flags;
    int result = 1;

    if (board_info(&board_size, &board_crc, &flags) < 0) goto out;

    if ((flags & UPLOAD_INFO_CONFIG) && board_size == (uint32_t)size && board_crc == local_crc)
    {
        printf("config.txt on the board matches %s (%ld bytes, CRC %08x)\n", file_path, size, local_crc);
        reboot_board();
        if (wait_for_board() < 0) goto out;

        if (flags & UPLOAD_INFO_ERROR)
        {
            fprintf(stderr, "but the board could not load it (JSON or module error), see its debug console\n");
            goto out;
        }
        result = 0;
        goto out;
    }

    if (flags & UPLOAD_INFO_CONFIG)
        printf("config.txt on the board differs (%u bytes, CRC %08x), updating\n", board_size, board_crc);
    else
        printf("no config.txt on the board, uploading\n");

    if (upload(data, size, file_path, "config.txt") < 0) goto out;

    printf("Board rebooting with the new configuration\n");
    reboot_board();
    if (wait_for_board() < 0) goto out;

    // what the board loaded now
    if (board_info(&board_size, &board_crc, &flags) < 0) goto out;
    reboot_board();
    if (wait_for_board() < 0) goto out;

    if (!(flags & UPLOAD_INFO_CONFIG) || board_size != (uint32_t)size || board_crc != local_crc)
    {
        fprintf(stderr, "the board does not report the new config after the reboot (%u bytes, CRC %08x)\n",
                board_size, board_crc);
        goto out;
    }
    if (flags & UPLOAD_INFO_ERROR)
    {
        fprintf(stderr, "the board could not load the new config (JSON or module error), see its debug console\n");
        goto out;
    }
    printf("Board runs the new configuration\n");
    result = 0;

out:
    free(data);
    return result;
}


static void usage(const char* prog)
{
    fprintf(stderr,
        "Usage: %s [options] FILE        upload FILE\n"
        "       %s [options] --sync FILE make the board's config.txt match FILE\n"
        "       %s [options] --info      show the config.txt the board loaded\n"
        "Write to the SD card of a Remora board over SPI while LinuxCNC is offline.\n"
        "\n"
        "  -n NAME              name on the SD card: firmware.bin or config.txt\n"
        "                       (default: the name of FILE)\n"
        "  -d DEV               spidev device (default: the first /dev/spidevB.C, as remora-spi)\n"
        "  -s HZ                SPI clock (default 4000000)\n"
        "  -c, --spi-cs-pin PIN chip select on a GPIO, e.g. PC10 (BTT CB1), as remora-spi spi_cs_pin\n"
        "  -C, --spi-cs-chip DEV gpiochip of the chip select (default " DEFAULT_GPIOCHIP ")\n"
        "  -k                   do not reboot the board afterwards (to upload another file first)\n"
        "  -f                   skip the check that LinuxCNC is not running\n"
        "  --sync               upload FILE as config.txt only if the board has another one, then\n"
        "                       reboot and wait for the board; for the HAL file before remora-spi:\n"
        "                       loadusr -w remora-upload --sync --spi-cs-pin PC10 -s 1000000 config.txt\n"
        "  --info               print size and CRC of the board's config.txt (reboots the board)\n"
        "\n"
        "After a firmware.bin upload the board reboots and its SD bootloader flashes it.\n",
        prog, prog, prog);
}


int main(int argc, char** argv)
{
    const char* device = NULL;
    const char* name = NULL;
    const char* cs_pin = NULL;
    const char* cs_chip = DEFAULT_GPIOCHIP;
    int reboot = 1;
    int force = 0;
    int sync = 0;
    int info = 0;
    int opt;

    static const struct option options[] =
    {
        { "spi-cs-pin",  required_argument, NULL, 'c' },
        { "spi-cs-chip", required_argument, NULL, 'C' },
        { "sync",        no_argument,       NULL, 'S' },
        { "info",        no_argument,       NULL, 'I' },
        { "help",        no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    while ((opt = getopt_long(argc, argv, "n:d:s:c:C:kfh", options, NULL)) != -1)
    {
        switch (opt)
        {
            case 'n': name = optarg; break;
            case 'd': device = optarg; break;
            case 's': spi_speed = strtoul(optarg, NULL, 0); break;
            case 'c': cs_pin = optarg; break;
            case 'C': cs_chip = optarg; break;
            case 'k': reboot = 0; break;
            case 'f': force = 1; break;
            case 'S': sync = 1; break;
            case 'I': info = 1; break;
            default:  usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }

    if (sync && info)
    {
        usage(argv[0]);
        return 2;
    }
    if (optind != argc - (info ? 0 : 1))
    {
        usage(argv[0]);
        return 2;
    }

    const char* file_path = info ? NULL : argv[optind];
    if (!info && !sync)
    {
        if (!name)
        {
            const char* slash = strrchr(file_path, '/');
            name = slash ? slash + 1 : file_path;
        }
        if (strcmp(name, "firmware.bin") && strcmp(name, "config.txt"))
        {
            fprintf(stderr, "the board only accepts firmware.bin or config.txt, use -n to set the name\n");
            return 2;
        }
    }

    // --sync runs from the HAL file, LinuxCNC is already starting: rtapi_app runs, but remora-spi
    // is not loaded yet; the board refuses anyway while LinuxCNC is talking to it
    if (!force && !sync && linuxcnc_running())
    {
        fprintf(stderr, "LinuxCNC is running, stop it first (it owns the SPI link)\n");
        return 1;
    }

    if (!device) device = find_spi_device();
    if (!device)
    {
        fprintf(stderr, "no /dev/spidev device found, enable spidev or use -d\n");
        return 1;
    }
    if (spi_open(device) < 0) return 1;
    if (cs_pin && cs_open(cs_chip, cs_pin) < 0) return 1;

    srand((unsigned)time(NULL) ^ (unsigned)getpid());
    seq = (uint8_t)rand();

    int result = 0;

    if (sync)
    {
        result = sync_config(file_path);
    }
    else if (info)
    {
        uint32_t size, crc;
        uint8_t flags;
        if (board_info(&size, &crc, &flags) < 0) result = 1;
        else if (!(flags & UPLOAD_INFO_CONFIG)) printf("no config.txt on the board\n");
        else printf("config.txt: %u bytes, CRC %08x%s\n", size, crc,
                    (flags & UPLOAD_INFO_ERROR) ? ", did NOT load (JSON or module error)" : ", loaded");
        if (reboot)
        {
            reboot_board();
            if (wait_for_board() < 0) result = 1;
        }
    }
    else
    {
        long size;
        uint8_t* data = read_file(file_path, &size);
        if (!data) return 1;

        if (upload(data, size, file_path, name) < 0)
        {
            result = 1;
        }
        else if (reboot)
        {
            reboot_board();
            if (!strcmp(name, "firmware.bin"))
                printf("Board rebooting, the bootloader flashes the new firmware (LEDs blink for a few seconds)\n");
            else
                printf("Board rebooting with the new configuration\n");
        }
        else
        {
            printf("Board stays in upload mode, it reboots by itself after 30 s without SPI traffic\n");
        }
        free(data);
    }

    if (cs_fd >= 0)
    {
        cs_set(1);
        close(cs_fd);
    }
    close(spi_fd);
    return result;
}
