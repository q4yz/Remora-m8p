# BTT Manta M8P V1.0 / V1.1 (STM32G0B1VET6)

Firmware target `MANTA8`:

```
mbed compile -m MANTA8 -t GCC_ARM --profile release
```

Copy `BUILD/MANTA8/GCC_ARM-RELEASE/firmware.bin` to the root of the **MCU** micro
SD card (the slot next to the MCU, not the CM4 card) together with `config.txt`,
and press reset. The BTT bootloader flashes it and renames it to `FIRMWARE.CUR`.

The M8P **V2.0** uses an STM32H723 and is a different board.

## Board facts used by the firmware

| | |
|---|---|
| MCU | STM32G0B1VET6, Cortex-M0+ 64 MHz, no FPU |
| Clock | 8 MHz crystal (HSE), PLL to 64 MHz |
| Bootloader | 8 KiB, application at `0x08002000` |
| MCU SD card | SPI1 PA5 / PA6 / PA7, CS PA8 (shared with the TMC SPI bus) |
| Remora SPI link | SPI2 on the EXP2 header, hardware NSS PB12 |
| Debug console | 115200 baud TX on EXP2 pin 7 (PE8) |
| Thread timers | base TIM6, servo TIM7 (TIM2 is the mbed microsecond ticker) |

## Wiring the Raspberry Pi / CM4 to EXP2

The CM4 on the M8P is not connected to the MCU by SPI on the board, so the
link is made with jumper wires from the CM4 40-pin GPIO header to EXP2.

| Signal | CM4 40-pin header | M8P EXP2 | MCU pin |
|---|---|---|---|
| MOSI (GPIO10) | 19 | 6 | PB11 |
| MISO (GPIO9) | 21 | 1 | PB14 |
| SCLK (GPIO11) | 23 | 2 | PB13 |
| CE0 (GPIO8) | 24 | 4 | PB12 |
| Reset (GPIO25) | 22 | 3 | PF7 |
| GND | 25 | 9 | GND |

EXP2 pin 8 is the MCU hardware reset, leave it unconnected. Remove any
display from EXP1/EXP2.

Keep the wires short and lower the SPI clock: the default divider of the
`remora-spi` component gives more than 15 MHz, which is too fast for the G0
SPI slave over jumper wires. Start with

```
loadrt remora-spi SPI_clk_div=64 PRU_base_freq=40000 ...
```

`remora-spi` drives the Raspberry Pi SPI hardware directly, so it needs a CM4
(or another Raspberry Pi), a BTT CB1 is not supported.

## Stepper drivers

| Driver | Step | Dir | Enable | UART / CS | Endstop |
|---|---|---|---|---|---|
| M1 | PE2 | PB4 | PC11 | PC10 | MIN1 PF3 |
| M2 | PF12 | PF11 | PB3 | PF13 | MIN2 PF4 |
| M3 | PD7 | PD6 | PF10 | PF9 | MIN3 PF5 |
| M4 | PD3 | PD2 | PD5 | PD4 | MIN4 PC0 |
| M5 | PC9 | PC8 | PD1 | PD0 | MIN5 PC1 |
| M6 V1.1 | PA10 | PA14 * | PA15 | PF8 | MIN6 PC2 |
| M7 V1.1 | PD11 | PD9 | PD15 | PD14 | |
| M8 V1.1 | PD8 | PC6 | PC7 | PD10 | |
| M6 V1.0 | PA10 | PD15 | PA15 | PF8 | MIN6 PC2 |
| M7 V1.0 | PD12 | PD11 | PD14 | PD13 | |
| M8 V1.0 | PD10 | PD8 | PD9 | PC7 | |

\* On V1.1 the M6 DIR signal comes from PA14, which is also the SWD clock.
Using M6 disables SWD debugging.

Enable pins are active low, which is what the Remora stepgen drives.

## Heaters, thermistors and fans

| | Pin | | Pin |
|---|---|---|---|
| BED-OUT | PB7 | TB | PA0 |
| HE0 | PE3 | TH0 | PA1 |
| HE1 | PB5 | TH1 | PA2 |
| HE2 | PB6 | TH2 | PA3 |
| HE3 | PE1 | TH3 | PA4 |
| FAN0 | PE6 | FAN1 | PE0 |
| FAN2 | PC12 | FAN3 | PE5 |
| FAN4 | PE4 | FAN5 | PB8 |
| FAN6 | PB9 | | |

Use software PWM (`"Hardware PWM": "False"`) for heaters and fans. The pin
tables of this target only list hardware PWM for the pins of the 64-pin
package, port E pins are not among them.

## Performance

The Cortex-M0+ has no FPU and runs at 64 MHz, about a third of the
STM32F446. The stepgen uses integer maths only, but with all eight drivers
the base thread still takes a large part of each 25 us period at 40 kHz.
With more than four or five joints, check the step pulses with a scope and
lower the base frequency (`PRU_base_freq` and the "Base" thread frequency) if
pulses go missing. The minimum step pulse width is `STEP_PULSE_MIN_NS` in
`configuration.h` (3 us), timed with the base thread timer.

## Updating over SPI

`firmware.bin` and `config.txt` can be written to the MCU SD card from the
CM4 while LinuxCNC is stopped, with `LinuxCNC/Tools/remora-upload/reflash.sh`.
The card runs at the mbed default SPI clock of 1 MHz, so a firmware upload
including the read back check takes around ten seconds.

Pin data: BTT Manta M8P V1.0 and V1.1 schematics
(github.com/bigtreetech/Manta-M8P) and the Klipper sample configs
`generic-bigtreetech-manta-m8p-v1.0.cfg` / `-v1.1.cfg`.
