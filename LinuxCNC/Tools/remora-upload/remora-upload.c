/********************************************************************
* Description:  remora-upload.c
*               Writes firmware.bin or config.txt to the SD card of a
*               Remora board over the SPI link, while LinuxCNC is not
*               running. Uses the Linux spidev driver.
*
*               Build:  gcc -O2 -Wall -o remora-upload remora-upload.c
*               Use:    sudo ./remora-upload firmware.bin
*
*               The board side is Firmware/FirmwareSource/Remora-OS6/
*               TARGET_STM32F4/drivers/sdUpload, the frame layout below
*               must match sdUpload.h.
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
#include <linux/spi/spidev.h>

#define SPI_BUFF_SIZE       64
#define PRU_UPLOAD          0x75706c64      // "upld" SPI payload

#define UPLOAD_DATA_MAX     44

#define UPLOAD_CMD_POLL     0
#define UPLOAD_CMD_BEGIN    1
#define UPLOAD_CMD_DATA     2
#define UPLOAD_CMD_END      3
#define UPLOAD_CMD_REBOOT   4

#define UPLOAD_ST_OK        1
#define UPLOAD_ST_ERROR     2

#define UPLOAD_ERR_OFFSET   5

#define POLL_INTERVAL_US    200             // gap between transfers, lets the board prepare its reply
#define RESEND_MS           250             // re-send a command that got no reply in this time
#define STALE_REPLIES       3               // or once this many valid replies to an older command came back
#define BEGIN_TIMEOUT_MS    15000           // covers a board that is still booting / watchdog resetting
#define DATA_TIMEOUT_MS     10000
#define END_TIMEOUT_MS      30000           // write, read back and rename on the board

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
    uint8_t  spare0;
    uint32_t nextOffset;
    uint32_t fileCrc;
    uint8_t  spare1[UPLOAD_DATA_MAX];
    uint32_t crc;
} uploadReply_t;

#pragma pack(pop)

_Static_assert(sizeof(uploadCmd_t) == SPI_BUFF_SIZE, "command frame size");
_Static_assert(sizeof(uploadReply_t) == SPI_BUFF_SIZE, "reply frame size");


static int spi_fd = -1;
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
        case 10: return "unknown command (firmware without SD upload support?)";
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


static int spi_open(const char* device)
{
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;

    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0)
    {
        fprintf(stderr, "cannot open %s: %s\n", device, strerror(errno));
        fprintf(stderr, "enable the SPI driver with \"dtparam=spi=on\" in /boot/firmware/config.txt and reboot\n");
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


static int spi_transfer(const void* tx, void* rx)
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = SPI_BUFF_SIZE;
    tr.speed_hz = spi_speed;
    tr.bits_per_word = 8;

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
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


static void usage(const char* prog)
{
    fprintf(stderr,
        "Usage: %s [options] FILE\n"
        "Write FILE to the SD card of a Remora board over SPI while LinuxCNC is offline.\n"
        "\n"
        "  -n NAME   name on the SD card: firmware.bin or config.txt\n"
        "            (default: the name of FILE)\n"
        "  -d DEV    spidev device (default /dev/spidev0.0)\n"
        "  -s HZ     SPI clock (default 4000000)\n"
        "  -k        do not reboot the board afterwards (to upload another file first)\n"
        "  -f        skip the check that LinuxCNC is not running\n"
        "\n"
        "After a firmware.bin upload the board reboots and its SD bootloader flashes it.\n",
        prog);
}


int main(int argc, char** argv)
{
    const char* device = "/dev/spidev0.0";
    const char* name = NULL;
    int reboot = 1;
    int force = 0;
    int opt;

    while ((opt = getopt(argc, argv, "n:d:s:kfh")) != -1)
    {
        switch (opt)
        {
            case 'n': name = optarg; break;
            case 'd': device = optarg; break;
            case 's': spi_speed = strtoul(optarg, NULL, 0); break;
            case 'k': reboot = 0; break;
            case 'f': force = 1; break;
            default:  usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }

    if (optind != argc - 1)
    {
        usage(argv[0]);
        return 2;
    }

    const char* file_path = argv[optind];
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

    if (!force && linuxcnc_running())
    {
        fprintf(stderr, "LinuxCNC is running, stop it first (it owns the SPI link)\n");
        return 1;
    }

    // read the whole file, firmware images are a few hundred kB
    FILE* f = fopen(file_path, "rb");
    if (!f)
    {
        fprintf(stderr, "cannot open %s: %s\n", file_path, strerror(errno));
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t* data = malloc(size > 0 ? size : 1);
    if (!data || fread(data, 1, size, f) != (size_t)size)
    {
        fprintf(stderr, "cannot read %s\n", file_path);
        return 1;
    }
    fclose(f);

    uint32_t file_crc = crc32(0, data, size);

    if (spi_open(device) < 0) return 1;

    srand((unsigned)time(NULL) ^ (unsigned)getpid());
    seq = (uint8_t)rand();

    uploadCmd_t cmd;
    uploadReply_t reply;

    // start the transfer
    printf("Uploading %s as %s (%ld bytes, CRC %08x)\n", file_path, name, size, file_crc);

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd = UPLOAD_CMD_BEGIN;
    cmd.arg0 = size;
    cmd.arg1 = file_crc;
    cmd.len = strlen(name);
    memcpy(cmd.data, name, cmd.len);

    if (command(&cmd, &reply, BEGIN_TIMEOUT_MS) < 0)
    {
        fprintf(stderr, "no reply from the board - check the SPI wiring and that the firmware supports SD upload\n");
        return 1;
    }
    if (reply.status != UPLOAD_ST_OK)
    {
        fprintf(stderr, "board refused the upload: %s\n", error_text(reply.error));
        return 1;
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
            return 1;
        }
        if (reply.status != UPLOAD_ST_OK && reply.error != UPLOAD_ERR_OFFSET)
        {
            fprintf(stderr, "\nupload failed at offset %u: %s\n", offset, error_text(reply.error));
            return 1;
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
        return 1;
    }
    if (reply.status != UPLOAD_ST_OK)
    {
        fprintf(stderr, "upload failed: %s\n", error_text(reply.error));
        return 1;
    }
    printf("%s written and verified on the SD card\n", name);

    if (reboot)
    {
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = UPLOAD_CMD_REBOOT;
        command(&cmd, &reply, 1000);

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
    close(spi_fd);
    return 0;
}
