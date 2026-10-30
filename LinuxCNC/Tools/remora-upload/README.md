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
`/dev/spidev0.0` must exist. On a BTT CB1 enable spidev with its overlay
(`/dev/spidev1.0`). Without `-d` the tool takes the first `/dev/spidevB.C`, the
same rule as remora-spi without `spi_dev`.

## Use

```
sudo ./remora-upload firmware.bin          # upload, board reboots and flashes it
sudo ./remora-upload config.txt            # upload a new configuration
sudo ./remora-upload -n config.txt my.json # upload a file under another name
sudo ./remora-upload -k config.txt && sudo ./remora-upload firmware.bin
                                           # several files, one reboot
```

Options: `-d DEV` spidev device (default: the first `/dev/spidevB.C`), `-s HZ`
SPI clock (default 4 MHz), `-c` / `--spi-cs-pin PIN` and `-C` / `--spi-cs-chip
DEV` chip select on a GPIO, `-k` do not reboot afterwards, `-f` skip the
LinuxCNC check, `--info` show the board's `config.txt`, `--sync` see below.

### BTT CB1

The CB1 does not connect the chip select position of the header, the board
gets its chip select from a GPIO, as with remora-spi `spi_cs_pin`. Pass the
same pin and use the same SPI clock as in the `loadrt remora-spi` line:

```
sudo ./remora-upload --spi-cs-pin PC10 -s 1000000 config.txt
```

`reset_pin` is not needed for the upload.

## Config from the LinuxCNC folder at every start (`--sync`)

Keep `config.txt` next to the HAL and INI files and let LinuxCNC put it on
the board when it starts, before remora-spi takes over the SPI link. In the
HAL file, before `loadrt remora-spi`:

```
loadusr -w remora-upload --sync --spi-cs-pin PC10 -s 1000000 config.txt
loadrt remora-spi PRU_base_freq=40000 SPI_freq=1000000 spi_cs_pin=PC10 reset_pin=PC12
```

A relative path is relative to the config folder, LinuxCNC runs the HAL file
there. `--sync` asks the board for the size and CRC-32 of the `config.txt` it
loaded at boot:

- the same: nothing is written;
- different or missing: the file is uploaded and verified;
- either way the board reboots (answering stops its threads) and the tool
  waits until it runs again; after an upload it checks that the board now
  reports the new config.

It fails, and with `loadusr -w` LinuxCNC does not start, if the board does
not answer, the upload fails, or the board reports that it could not load
the config (JSON or module error, see its debug console). A start costs about
2 s for the reboot, an update a few seconds more. Needs firmware with the
INFO command (Remora-OS6 with the CAN expansion, phase 2); older firmware
answers "unknown command".

`loadusr` runs it as the LinuxCNC user, without sudo. Install it setuid root
(as LinuxCNC's `rtapi_app` is), or give the user access to the spidev and
gpiochip devices with a udev rule:

```
gcc -O2 -Wall -o remora-upload remora-upload.c
sudo install -o root -m 4755 remora-upload /usr/local/bin/
```

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

`SPI_DEV`, `SPI_SPEED`, `SPI_CS_PIN`, `SPI_CS_CHIP` and `REMORA_UPLOAD` override
the device, clock, GPIO chip select and tool path, e.g. on a CB1:

```
SPI_CS_PIN=PC10 SPI_SPEED=1000000 ./reflash.sh config.txt
```

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
