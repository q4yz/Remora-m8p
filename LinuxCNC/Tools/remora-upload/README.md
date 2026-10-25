# remora-upload

Writes `firmware.bin` or `config.txt` to the SD card of a Remora board over the
existing Raspberry Pi SPI link, so the card does not have to be taken out of the
board. Only works while LinuxCNC is **not** running, since LinuxCNC owns the
SPI link.

Supported boards: Remora-OS6 firmware built with SD upload support (`SD_UPLOAD`
in `configuration.h`): the STM32F4 boards with an SDIO card slot (BTT Octopus /
Octopus Pro F446 and F429, BTT SKR v2, Black F407VE) and the BTT Manta M8P
V1.0 / V1.1 (STM32G0B1). The first firmware with this feature has to be put
on the card the usual way.

## Build

On the Raspberry Pi:

```
gcc -O2 -Wall -o remora-upload remora-upload.c
```

The tool uses the kernel spidev driver. Enable it with `dtparam=spi=on` in
`/boot/firmware/config.txt` (`/boot/config.txt` on older images) and reboot;
`/dev/spidev0.0` must exist.

## Use

```
sudo ./remora-upload firmware.bin          # upload, board reboots and flashes it
sudo ./remora-upload config.txt            # upload a new configuration
sudo ./remora-upload -n config.txt my.json # upload a file under another name
sudo ./remora-upload -k config.txt && sudo ./remora-upload firmware.bin
                                           # several files, one reboot
```

Options: `-d DEV` spidev device (default `/dev/spidev0.0`), `-s HZ` SPI clock
(default 4 MHz), `-k` do not reboot afterwards, `-f` skip the LinuxCNC check.

## Example script

`reflash.sh` wraps the tool for the usual update: it builds `remora-upload` if
needed, checks that LinuxCNC is stopped, the SPI device exists and a config is
valid JSON, uploads the files with a single reboot and waits for the bootloader.

```
./reflash.sh firmware.bin              # new firmware
./reflash.sh config.txt                # new configuration
./reflash.sh firmware.bin config.txt   # both
./reflash.sh -n config.txt my.json     # upload my.json as config.txt
```

`SPI_DEV`, `SPI_SPEED` and `REMORA_UPLOAD` override the device, clock and tool path.

## How it works

- The board accepts upload frames (`"upld"` header) only while LinuxCNC is not
  exchanging data with it, then stops its base and servo threads.
- Data is sent in 44 byte chunks, every frame is CRC-32 protected and the host
  re-sends anything that is not acknowledged, so dropped or corrupted frames
  only cost time.
- The board buffers 8 kB in RAM and writes it to `upload.tmp` on the card. The
  SPI DMA is paused while the card is accessed, because the SDIO DMA shares a
  stream with it.
- At the end the board reads `upload.tmp` back, checks the CRC-32 of the whole
  file and only then replaces the target file. A failed upload never leaves a
  broken `firmware.bin` or `config.txt` on the card.
- The board then reboots. For `firmware.bin` the SD bootloader flashes the new
  image. If the host goes silent during an upload, the board reboots by itself
  after 30 s.
