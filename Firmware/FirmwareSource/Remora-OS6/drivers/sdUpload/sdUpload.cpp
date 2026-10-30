#include "sdUpload.h"

#ifdef SD_UPLOAD

#include <string.h>

#define UPLOAD_TMP_PATH     "/fs/upload.tmp"


SDUpload::SDUpload(RemoraComms* comms, FATFileSystem* fileSystem, BlockDevice* blockDevice, volatile txData_t* ptrTxData, void (*onStart)(void)) :
    comms(comms),
    fileSystem(fileSystem),
    blockDevice(blockDevice),
    ptrTxData(ptrTxData),
    onStart(onStart)
{
    this->active = false;
    this->receiving = false;
    this->fileCreated = false;
    this->rebootPending = false;
    this->target[0] = '\0';
    this->fileSize = 0;
    this->expectedCrc = 0;
    this->receivedCrc = 0;
    this->nextOffset = 0;
    this->lastFrameTick = 0;
    this->rebootTick = 0;
    this->bufLen = 0;
    this->configFlags = 0;
    this->configSize = 0;
    this->configCrc = 0;
}


void SDUpload::setConfigInfo(bool found, bool error, uint32_t size, uint32_t crc)
{
    this->configFlags = (found ? UPLOAD_INFO_CONFIG : 0) | (error ? UPLOAD_INFO_ERROR : 0);
    this->configSize = size;
    this->configCrc = crc;
}


bool SDUpload::isActive()
{
    return this->active;
}


void SDUpload::service()
{
    uploadCmd_t frame;

    if (this->comms->getUploadFrame((uint8_t*)&frame))
    {
        this->lastFrameTick = HAL_GetTick();
        this->handleFrame(&frame);
    }

    if (this->rebootPending && (HAL_GetTick() - this->rebootTick) > UPLOAD_REBOOT_MS)
    {
        printf("SD upload: rebooting\n");
        NVIC_SystemReset();
    }

    if (this->active && (HAL_GetTick() - this->lastFrameTick) > UPLOAD_TIMEOUT_MS)
    {
        printf("SD upload: host timeout, rebooting\n");
        NVIC_SystemReset();
    }
}


void SDUpload::handleFrame(uploadCmd_t* cmd)
{
    // ignore corrupted frames, the host re-sends commands it gets no reply for
    if (crc32(0, (uint8_t*)cmd, SPI_BUFF_SIZE - 4) != cmd->crc) return;

    if (cmd->cmd == UPLOAD_CMD_POLL) return;

    // never interfere with a running machine
    if (!this->active && this->comms->getStatus())
    {
        this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_LINUXCNC);
        return;
    }

    switch (cmd->cmd)
    {
        case UPLOAD_CMD_BEGIN:
        {
            char name[sizeof(this->target)];
            uint16_t len = cmd->len < sizeof(name) - 1 ? cmd->len : sizeof(name) - 1;
            memcpy(name, cmd->data, len);
            name[len] = '\0';

            if (strcmp(name, "firmware.bin") && strcmp(name, "config.txt"))
            {
                this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_NAME);
                return;
            }

            if (cmd->arg0 > UPLOAD_MAX_SIZE)
            {
                this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_SIZE);
                return;
            }

            if (!this->active) this->enterUploadMode();

            // (re)start the transfer, a repeated BEGIN simply starts over
            strcpy(this->target, name);
            this->fileSize = cmd->arg0;
            this->expectedCrc = cmd->arg1;
            this->receivedCrc = 0;
            this->nextOffset = 0;
            this->bufLen = 0;
            this->fileCreated = false;
            this->receiving = true;

            printf("SD upload: receiving %s (%lu bytes)\n", this->target, (unsigned long)this->fileSize);
            this->reply(cmd->seq, UPLOAD_ST_OK, UPLOAD_ERR_NONE);
            break;
        }

        case UPLOAD_CMD_DATA:
        {
            if (!this->receiving)
            {
                this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_STATE);
                return;
            }

            if (cmd->arg0 < this->nextOffset)
            {
                // repeated frame, already stored
                this->reply(cmd->seq, UPLOAD_ST_OK, UPLOAD_ERR_NONE);
                return;
            }

            if (cmd->arg0 > this->nextOffset)
            {
                // a frame was lost, the reply tells the host where to resume
                this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_OFFSET);
                return;
            }

            if (cmd->len > UPLOAD_DATA_MAX || this->nextOffset + cmd->len > this->fileSize)
            {
                this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_SIZE);
                return;
            }

            memcpy(&this->buffer[this->bufLen], cmd->data, cmd->len);
            this->bufLen += cmd->len;
            this->nextOffset += cmd->len;
            this->receivedCrc = crc32(this->receivedCrc, cmd->data, cmd->len);

            uint8_t error = UPLOAD_ERR_NONE;
            if (this->bufLen > UPLOAD_BUF_SIZE - UPLOAD_DATA_MAX)
            {
                error = this->writeToCard(false);
            }

            if (error)
            {
                this->receiving = false;
                this->reply(cmd->seq, UPLOAD_ST_ERROR, error);
            }
            else
            {
                this->reply(cmd->seq, UPLOAD_ST_OK, UPLOAD_ERR_NONE);
            }
            break;
        }

        case UPLOAD_CMD_END:
        {
            if (!this->receiving)
            {
                // a repeated END after the file was stored successfully
                bool done = this->active && this->target[0] != '\0' && this->nextOffset == this->fileSize;
                this->reply(cmd->seq, done ? UPLOAD_ST_OK : UPLOAD_ST_ERROR, done ? UPLOAD_ERR_NONE : UPLOAD_ERR_STATE);
                return;
            }

            if (this->nextOffset != this->fileSize)
            {
                this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_SIZE);
                return;
            }

            uint8_t error = UPLOAD_ERR_NONE;
            if (this->receivedCrc != this->expectedCrc)
            {
                error = UPLOAD_ERR_VERIFY;
            }
            else
            {
                error = this->writeToCard(true);
            }

            this->receiving = false;
            if (error)
            {
                this->target[0] = '\0';
                printf("SD upload: failed, error %d\n", error);
                this->reply(cmd->seq, UPLOAD_ST_ERROR, error);
            }
            else
            {
                printf("SD upload: %s written\n", this->target);
                this->reply(cmd->seq, UPLOAD_ST_OK, UPLOAD_ERR_NONE);
            }
            break;
        }

        case UPLOAD_CMD_INFO:
        {
            if (!this->active) this->enterUploadMode();

            // reported in the offset / CRC fields, they are not used outside a transfer
            uint32_t offset = this->nextOffset, crc = this->receivedCrc;
            this->nextOffset = this->configSize;
            this->receivedCrc = this->configCrc;
            this->reply(cmd->seq, UPLOAD_ST_OK, UPLOAD_ERR_NONE, this->configFlags);
            this->nextOffset = offset;
            this->receivedCrc = crc;
            break;
        }

        case UPLOAD_CMD_REBOOT:
        case UPLOAD_CMD_ABORT:
            // reboot once the host has had time to read the reply
            this->reply(cmd->seq, UPLOAD_ST_OK, UPLOAD_ERR_NONE);
            this->rebootPending = true;
            this->rebootTick = HAL_GetTick();
            break;

        default:
            this->reply(cmd->seq, UPLOAD_ST_ERROR, UPLOAD_ERR_CMD);
    }
}


void SDUpload::enterUploadMode()
{
    printf("\n## Entering SD UPLOAD mode\n");

    // stop the threads so they don't write over the reply in the TX buffer
    if (this->onStart) this->onStart();

    // the file system was left mounted after reading the config, with the block device deinitialised
    this->fileSystem->unmount();

    this->comms->setUploadMode(true);
    this->active = true;
}


uint8_t SDUpload::writeToCard(bool finish)
{
    Watchdog& watchdog = Watchdog::get_instance();
    uint8_t error = UPLOAD_ERR_NONE;

    // pause the comms DMA while the card is in use (on the F4 the SDIO RX DMA shares a stream with the SPI TX DMA)
    this->comms->stopDMA();
    watchdog.kick();

    if (this->fileSystem->mount(this->blockDevice))
    {
        error = UPLOAD_ERR_MOUNT;
    }
    else
    {
        watchdog.kick();

        if (this->bufLen > 0 || !this->fileCreated)
        {
            FILE* file = fopen(UPLOAD_TMP_PATH, this->fileCreated ? "ab" : "wb");
            if (!file)
            {
                error = UPLOAD_ERR_WRITE;
            }
            else
            {
                if (fwrite(this->buffer, 1, this->bufLen, file) != this->bufLen) error = UPLOAD_ERR_WRITE;
                if (fclose(file)) error = UPLOAD_ERR_WRITE;
                this->fileCreated = true;
                this->bufLen = 0;
            }
        }

        watchdog.kick();

        if (!error && finish)
        {
            error = this->verifyAndRename();
        }

        this->fileSystem->unmount();
    }

    this->comms->restartDMA();
    return error;
}


uint8_t SDUpload::verifyAndRename()
{
    Watchdog& watchdog = Watchdog::get_instance();

    // read the file back to make sure what is on the card is what the host sent
    FILE* file = fopen(UPLOAD_TMP_PATH, "rb");
    if (!file) return UPLOAD_ERR_VERIFY;

    uint32_t crc = 0;
    uint32_t total = 0;
    size_t n;
    while ((n = fread(this->buffer, 1, UPLOAD_BUF_SIZE, file)) > 0)
    {
        crc = crc32(crc, this->buffer, n);
        total += n;
        watchdog.kick();
    }
    fclose(file);

    if (total != this->fileSize || crc != this->expectedCrc)
    {
        remove(UPLOAD_TMP_PATH);
        return UPLOAD_ERR_VERIFY;
    }

    char path[24];
    snprintf(path, sizeof(path), "/fs/%s", this->target);
    remove(path);
    if (rename(UPLOAD_TMP_PATH, path)) return UPLOAD_ERR_RENAME;

    return UPLOAD_ERR_NONE;
}


void SDUpload::reply(uint8_t seq, uint8_t status, uint8_t error, uint8_t flags)
{
    uploadReply_t reply;
    memset(&reply, 0, sizeof(reply));

    reply.header = PRU_UPLOAD;
    reply.status = status;
    reply.seq = seq;
    reply.error = error;
    reply.spare0 = flags;
    reply.nextOffset = this->nextOffset;
    reply.fileCrc = this->receivedCrc;
    reply.crc = crc32(0, (uint8_t*)&reply, SPI_BUFF_SIZE - 4);

    // the TX DMA is circular, the new reply goes out with the next transfer
    uint8_t* src = (uint8_t*)&reply;
    for (int i = 0; i < SPI_BUFF_SIZE; i++)
    {
        this->ptrTxData->txBuffer[i] = src[i];
    }
}


uint32_t SDUpload::crc32(uint32_t crc, const uint8_t* data, uint32_t len)
{
    // standard CRC-32 (IEEE 802.3), chainable: crc32(crc32(0, a), b) == crc32(0, a + b)
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

#endif // SD_UPLOAD
