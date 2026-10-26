# remora-spi

HAL component that talks to a Remora board (STM32 running Remora firmware) over
SPI. The host side has three drivers, chosen when the component loads:

| Host | Driver | How |
|---|---|---|
| Raspberry Pi 3 / 4 / CM4 | bcm2835 | SPI registers directly |
| Raspberry Pi 5 / CM5 | rp1 | SPI registers directly |
| Anything else, e.g. BigTreeTech CB1 | generic (`gspi.c`) | Linux `spidev` + GPIO character device |

On a Raspberry Pi nothing changes: the Pi drivers are still used. Every other
board gets the generic driver. `spi_backend=spidev` forces the generic driver on
a Pi too, which is handy for comparing.

```
sudo halcompile --install remora-spi.c
```

`gspi.c`, `bcm2835.c` and the RP1 files are pulled in by `remora-spi.c`, keep
them in the same folder.

## Parameters

| Parameter | Drivers | Default | |
|---|---|---|---|
| `PRU_base_freq` | all | 40000 | base thread frequency of the firmware |
| `SPI_clk_div` | bcm2835 | 16 | SPI clock divider (power of 2) |
| `SPI_num`, `CS_num` | rp1 | 0, 0 | SPI controller and chip select |
| `SPI_freq` | rp1, generic | 20 MHz (rp1), 8 MHz (generic) | SPI clock in Hz |
| `spi_backend` | | `auto` | `auto` or `spidev` |
| `spi_dev` | generic | first `/dev/spidevB.C` found | spidev device |
| `reset_pin` | generic | none | PRU reset GPIO, line number or Allwinner name like `PH9` |
| `reset_chip` | generic | `/dev/gpiochip0` | gpiochip of `reset_pin` |
| `spi_cs_pin` | generic | none (controller CS) | chip select on a GPIO, driven low around each frame |
| `spi_cs_chip` | generic | `/dev/gpiochip0` | gpiochip of `spi_cs_pin` |

On the Pi drivers the PRU reset is fixed on GPIO25.

## BigTreeTech CB1 (Allwinner H616)

1. A PREEMPT_RT kernel and LinuxCNC (uspace).
2. Enable SPI1 with spidev through a device tree overlay, so `/dev/spidev1.0`
   exists. The overlay name depends on the image, see
   `ls /boot/dtb/allwinner/overlay | grep -i spi` and the `overlays=` line in
   `/boot/armbianEnv.txt` (Armbian) or `/boot/BoardEnv.txt` (BTT image).
3. Pick a free 3.3 V GPIO for the PRU reset and wire it to the board's reset
   pin input (Manta M8P: EXP2 pin 3). Allwinner GPIOs are numbered
   `(port - 'A') * 32 + pin`, so `reset_pin=PH9` is line 233 on gpiochip0. Port C
   runs at 1.8 V on CB1 V2.1, don't use it there.
4. The CB1 puts SPI1 on the same header pins as a Raspberry Pi (MOSI 19 = PH7,
   MISO 21 = PH8, SCLK 23 = PH6), but **the CE0 (pin 24) and GPIO25 (pin 22)
   positions are not connected on the CB1**. Use free GPIOs for chip select and
   reset instead, e.g. header pin 15 (PC10) and pin 13 (PC12), wired to the
   board's chip select and reset inputs (Manta M8P: EXP2 pin 4 and pin 3).
5. Load the component, for example:

```
loadrt remora-spi PRU_base_freq=40000 SPI_freq=1000000 spi_cs_pin=PC10 reset_pin=PC12
```

The driver logs which device, clock, chip select and reset lines it uses.
Without `spi_cs_pin` the Remora board never sees a chip select on a CB1.

### Latency

The generic driver does one `ioctl` per 64 byte frame, two frames per servo
period. That adds some tens of µs per frame compared to the register drivers,
well within a 1 ms servo period. To keep it predictable:

- Run the CPU at a fixed clock: `GOVERNOR=performance` and `MIN_SPEED` =
  `MAX_SPEED` in `/etc/default/cpufrequtils` on Armbian.
- Isolate a core for LinuxCNC (`isolcpus=3 nohz_full=3 rcu_nocbs=3`) and keep
  interrupts off it (`irqaffinity=0-2`).
- The driver keeps the SPI controller powered (writes `on` to its
  `power/control` in sysfs) so it is not suspended between frames.
- If transfers show jitter, raise the priority of the SPI controller's interrupt
  thread, e.g. `chrt -f -p 80 $(pgrep -f 'irq/.*spi')`.

Measure before tuning: `halshow` shows the time the `remora.read` and
`remora.write` functions take, and `latency-histogram --nobase --speriod 1000000`
shows the servo thread jitter.

`remora-upload` uses the same spidev device. It only works while LinuxCNC is
stopped.
