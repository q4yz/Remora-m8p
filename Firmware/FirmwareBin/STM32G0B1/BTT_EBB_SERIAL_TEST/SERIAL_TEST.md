# USB serial test firmware (BTT EBB42 / EBB SB2209 CAN)

`firmware.bin` here: only the USB serial port, no CAN, no node logic.
Source `Firmware/FirmwareSource/Remora-OS6/TARGET_EBB42/serialTestMain.cpp`,
build `bash Firmware/build.sh EBB_SERIAL_TEST`. Runs on both boards.

## Flash

Hold BOOT, plug in USB (or press RESET), then:

```
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
```

Then power the board off completely (24 V and USB, about 10 s) and on
again: after a DFU flash the STM32G0 keeps starting its ROM bootloader
until a real power-on.

## Read

Windows: "USB Serial Device (COMx)" in the device manager, any terminal
(PuTTY: Serial, COMx, any speed). Linux: `screen /dev/ttyACM0`. Every
0.5 s a counter line, every 5 s the full report:

```
=== Remora USB serial test ... ===
UID ..., DEV_ID 0x467 rev ..., flash 128 KB
SystemCoreClock 64000000 Hz, system clock: PLL from HSE (8 MHz crystal), HSE running
reset cause: power-on reset-pin (CSR ...)
option bytes OPTR 0x........: nBOOT_SEL 1, nBOOT1 1, nBOOT0 1, IWDG_SW 1 (software watchdog), RDP 0xaa
  boot: BOOT0 pin (PA14) decides, low = this firmware
#12 uptime 6.0 s, USB configured, HSI48 trim 64, CRS in sync
```

Compare the two boards line by line. Telling lines:

- `PLL from HSI16 (crystal NOT used)`: the 8 MHz crystal did not start.
- `boot: ... SYSTEM BOOTLOADER`: the option bytes make the chip start the
  ROM bootloader, never the firmware.
- `HARDWARE watchdog`: the watchdog runs from reset, a firmware that is
  slow to kick it resets in a loop.
- `CRS SYNC MISSED` / `not configured`: USB clock or data lines.

## No output at all on one board

First check the flash itself: flash with STM32CubeProgrammer and
"Verify programming" ticked. A failed verify means the flash holds a
half-written program, which crashes and gets reset by the watchdog in a
loop (`reset cause: WATCHDOG`, USB enumeration errors such as -71 on
Linux). Do a full chip erase, then flash again. This was the case on an
SB2209 on 2026-11-10; the EBB42 / SB2209 hardware and firmware were fine.
That SB2209 had `OPTR 0xfedffeaa`: single bank flash (DUAL_BANK 0), the
EBB42 dual bank; harmless for Remora, which does not write its flash.

`lsusb` / device manager:

- `0483:df11` (STM32 BOOTLOADER): the chip sits in the ROM bootloader,
  the firmware does not start. Read its option bytes over DFU and compare
  them with the working board:

  ```
  dfu-util -a 1 -s 0x1FFF7800:128 -U options.bin
  ```

  The first 4 bytes are OPTR (little endian): bit 24 nBOOT_SEL, bit 25
  nBOOT1, bit 26 nBOOT0, bit 16 IWDG_SW. A working EBB42 (2026-11-10):
  `OPTR 0xfefffeaa`, nBOOT_SEL 0 / nBOOT0 1 = always main flash, the BOOT0
  pin is ignored (DFU only through the empty-flash check). With nBOOT_SEL 0
  and nBOOT0 0 the chip never starts the firmware; with nBOOT_SEL 1 the
  BOOT0 pin (PA14) decides, a high level on it starts the ROM bootloader.
- nothing: no power, or the USB data lines of that board do not reach
  the MCU (cable, connector).
