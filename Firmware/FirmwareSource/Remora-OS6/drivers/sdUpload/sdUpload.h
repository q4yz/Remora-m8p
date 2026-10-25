#ifndef SDUPLOAD_H
#define SDUPLOAD_H

#include "configuration.h"

#ifdef SD_UPLOAD

#include "mbed.h"
#include "FATFileSystem.h"

#include "configuration.h"
#include "remora.h"
#include "RemoraComms.h"

/***********************************************************************
    SD card file upload over the Remora SPI link

    Lets the Raspberry Pi write firmware.bin or config.txt to the board's
    SD card while LinuxCNC is not running (see LinuxCNC/Tools/remora-upload).

    The host sends a command frame, then polls until a reply with the same
    sequence number comes back. All frames are 64 bytes and end with a
    CRC-32 of the first 60 bytes. Multi-byte fields are little endian.

    The comms DMA is stopped while the card is accessed and restarted on a
    frame boundary afterwards (RemoraComms::stopDMA / restartDMA). On the
    STM32F4 SDIO boards this is required, the SDIO shares DMA2 stream 3 with
    the SPI1 TX DMA; on the Manta M8P the card has its own SPI bus and the
    pause only keeps the main loop and the comms in step. Data is buffered in
    RAM and written in UPLOAD_BUF_SIZE blocks to keep these pauses rare; the
    host simply re-sends a command it got no reply for.

    Each target's RemoraComms provides the PRU_UPLOAD frame hand-off
    (getUploadFrame), setUploadMode, stopDMA and restartDMA.

    The file is written to upload.tmp, read back and CRC checked, and only
    then renamed to its final name. Leaving upload mode always reboots the
    board, which also lets the SD bootloader flash a new firmware.bin.
************************************************************************/

#define UPLOAD_DATA_MAX     44              // data bytes per frame
#define UPLOAD_BUF_SIZE     8192            // RAM buffer flushed to the SD card in one go
#define UPLOAD_MAX_SIZE     (1024 * 1024)   // sanity limit on the file size
#define UPLOAD_TIMEOUT_MS   30000           // reboot if the host goes silent during an upload
#define UPLOAD_REBOOT_MS    200             // time for the host to read the reboot reply

// host commands
#define UPLOAD_CMD_POLL     0               // no-op, clocks out the current reply
#define UPLOAD_CMD_BEGIN    1               // arg0 = file size, arg1 = file CRC-32, data = file name
#define UPLOAD_CMD_DATA     2               // arg0 = file offset, len = data bytes
#define UPLOAD_CMD_END      3               // write, verify and rename the file
#define UPLOAD_CMD_REBOOT   4               // reboot the board
#define UPLOAD_CMD_ABORT    5               // abandon the upload and reboot

// reply status
#define UPLOAD_ST_OK        1
#define UPLOAD_ST_ERROR     2

// reply error codes
#define UPLOAD_ERR_NONE         0
#define UPLOAD_ERR_LINUXCNC     1           // LinuxCNC is communicating with the board
#define UPLOAD_ERR_NAME         2           // file name not allowed
#define UPLOAD_ERR_SIZE         3           // file too big or data past the end of the file
#define UPLOAD_ERR_STATE        4           // command not valid now (no upload in progress)
#define UPLOAD_ERR_OFFSET       5           // data offset ahead of the next expected offset
#define UPLOAD_ERR_MOUNT        6           // SD card could not be mounted
#define UPLOAD_ERR_WRITE        7           // SD card write failed
#define UPLOAD_ERR_VERIFY       8           // read back CRC does not match
#define UPLOAD_ERR_RENAME       9           // could not replace the target file
#define UPLOAD_ERR_CMD          10          // unknown command

#pragma pack(push, 1)

typedef struct
{
    uint32_t header;                        // PRU_UPLOAD
    uint8_t  cmd;
    uint8_t  seq;
    uint16_t len;
    uint32_t arg0;
    uint32_t arg1;
    uint8_t  data[UPLOAD_DATA_MAX];
    uint32_t crc;                           // CRC-32 of the preceding 60 bytes
} uploadCmd_t;

typedef struct
{
    uint32_t header;                        // PRU_UPLOAD
    uint8_t  status;
    uint8_t  seq;                           // sequence number of the command this replies to
    uint8_t  error;
    uint8_t  spare0;
    uint32_t nextOffset;                    // number of file bytes accepted so far
    uint32_t fileCrc;                       // CRC-32 of the bytes accepted so far
    uint8_t  spare1[UPLOAD_DATA_MAX];
    uint32_t crc;                           // CRC-32 of the preceding 60 bytes
} uploadReply_t;

#pragma pack(pop)

static_assert(sizeof(uploadCmd_t) == SPI_BUFF_SIZE, "upload command must fill an SPI frame");
static_assert(sizeof(uploadReply_t) == SPI_BUFF_SIZE, "upload reply must fill an SPI frame");


class SDUpload
{
    private:

        RemoraComms*        comms;
        FATFileSystem*      fileSystem;
        BlockDevice*        blockDevice;
        volatile txData_t*  ptrTxData;
        void                (*onStart)(void);

        bool                active;         // upload mode entered, normal operation suspended
        bool                receiving;      // file transfer in progress
        bool                fileCreated;    // upload.tmp has been created for this transfer
        bool                rebootPending;

        char                target[16];
        uint32_t            fileSize;
        uint32_t            expectedCrc;
        uint32_t            receivedCrc;
        uint32_t            nextOffset;

        uint32_t            lastFrameTick;
        uint32_t            rebootTick;

        uint32_t            bufLen;
        uint8_t             buffer[UPLOAD_BUF_SIZE];

        void handleFrame(uploadCmd_t*);
        void enterUploadMode(void);
        uint8_t writeToCard(bool finish);
        uint8_t verifyAndRename(void);
        void reply(uint8_t seq, uint8_t status, uint8_t error);

    public:

        SDUpload(RemoraComms*, FATFileSystem*, BlockDevice*, volatile txData_t*, void (*onStart)(void));

        void service(void);
        bool isActive(void);

        static uint32_t crc32(uint32_t crc, const uint8_t* data, uint32_t len);
};

#endif // SD_UPLOAD

#endif
