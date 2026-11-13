# Porting Remora to another board or MCU family

Remora is split in two: **logic** that is the same on every board, and a
small **hardware layer** per MCU family (`TARGET_<FAMILY>/`). A new board
of an existing family is mostly a pin map; a new family implements the
hardware layer below. The STM32G0 (`TARGET_STM32G0`, BTT Manta M8P, EBB42,
SB2209) is the reference implementation.

## Portable logic (no MCU headers, PC tested)

| Folder | What | PC test |
|---|---|---|
| `drivers/remoraLink/` | SPI link frames: queue, torn frames, re-align, link status | `Firmware/Tests/link` |
| `drivers/stepScheduler/` | event driven step generation (`"Step Generator": "Interrupt"`) | `Firmware/Tests/stepsched` |
| `drivers/can/` | CAN hub / node protocol (`CanHub`, `CanNode`, `CAN_PROTOCOL.md`), the TMC2209 UART driver `NodeTmc2209` | `Firmware/Tests/can` |
| `drivers/rgb/` | WS2812 colour orders and set point coding | (in the CAN test) |

Run all of them with `sh Firmware/Tests/<name>/run.sh` (Linux / WSL, g++),
the CI runs them on every push.

## Hardware layer of a family

Required:

| Interface | Header | STM32G0 implementation |
|---|---|---|
| SPI slave link: DMA counter, NSS level, re-arm, restart, stop | `LinkHardware` in `drivers/remoraLink/RemoraLink.h` | `TARGET_STM32G0/drivers/comms/RemoraComms` |
| Base and servo thread timers (`pruThread`, `pruTimer`, interrupt vectors, priorities) | `TARGET_<FAMILY>/thread/` | `TARGET_STM32G0/thread/` (TIM6 base, TIM7 servo) |
| GPIO by name with fast `set()` / `get()` | `TARGET_<FAMILY>/drivers/pin/pin.h` | `TARGET_STM32G0/drivers/pin` (BSRR / BRR / IDR) |

Optional, each switches a feature on; without it the feature is absent or
falls back:

| Feature | Interface | Without it | STM32G0 implementation |
|---|---|---|---|
| Interrupt step generation | `drivers/stepScheduler/stepTimer.h` (`stepTimerAvailable/Hz/Start`) | weak defaults: Stepgen in the base thread | `TARGET_STM32G0/drivers/stepTimer` (TIM3) |
| CAN hub / nodes | `CanBus` in `drivers/can/CanBus.h` | no CAN | `TARGET_STM32G0/drivers/can/RemoraCAN` (FDCAN, polled) |
| GPIO by pin byte (TMC2209 UART, CAN node modules) | `drivers/can/canNodeIo.h` | nodes impossible, hub TMC2209 on TMCStepper | `TARGET_STM32G0/drivers/can/canNodeIo.cpp` |
| TMC2209 over the node driver on the hub | a tick timer, `modules/tmc/tmc2209G0.cpp` | `TMCStepper` + `SoftwareSerial` | TIM14 at 40 kHz |
| WS2812 LED chains | `TARGET_<FAMILY>/drivers/ws2812/Ws2812.h` | "RGB" module prints "not supported" | timer + DMA2 |
| MCU over temperature guard | `MCU_TEMP_SUPPORTED`, `modules/mcuTemp` | off | internal sensor, register reads |
| USB serial console (nodes) | `TARGET_<FAMILY>/drivers/usbCdc` | UART console | `TARGET_STM32G0/drivers/usbCdc` |

Family code that is still guarded by `#if defined(TARGET_STM32G0)` in shared
files, to become interfaces when a second family needs them: `modules/rgb`
(Ws2812), `modules/mcuTemp` (ADC registers), `modules/tmc/tmc2209G0.cpp`
(TIM14), the CAN hub set-up in `main.cpp`.

## A new board of an existing family

1. `custom_targets.json`: the target, inheriting the family's MCU.
2. `TARGET_<BOARD>/PinNames.h`, `PeripheralPins.c` (from mbed or the
   board's schematic), the SPI link pins in `main.cpp` (the
   `RemoraComms` line of the board).
3. `Firmware/build.sh`: a row in `TARGETS` (and its `FirmwareBin` folder),
   `.github/workflows/firmware.yml`: the name in the matrix.
4. LinuxCNC_UI: the board's pins and its SPI / reset pins in the profile.

## A new family

1. The required hardware layer above, starting from a copy of
   `TARGET_STM32G0` and replacing the register code. Keep the logic in
   the portable folders; if the new family needs a change there, change
   it for all families and extend the PC test.
2. The optional interfaces one by one, each with a hardware test.
3. `Firmware/build.sh` and the CI matrix.

Check on hardware: the debug build (`-D REMORA_DEBUG=1`) prints the link
statistics, the thread load (`[debug] load:`) and with the interrupt step
generation `[debug] stepgen:` while LinuxCNC is not connected.
