# Porting Remora to another board or MCU family

Remora is split in three:

- **logic** that is the same everywhere (`main.cpp`, `modules/`, `drivers/`,
  `node/`): it never names a board or a family,
- a **family layer** per MCU family (`TARGET_<FAMILY>/`): the registers,
- a **board file** per board (`TARGET_<BOARD>/board.h`): the pins.

`configuration.h` includes the family's `family.h` and the board's
`board.h`; mbed puts the folders of the target being built on the include
path, so the right two are found by name. Shared code asks
`#ifdef REMORA_HAS_...` (the family implements an interface) or
`#ifdef BOARD_...` (the board has something), never `TARGET_...`.

The STM32G0 (`TARGET_STM32G0`: BTT Manta M8P, EBB42, SB2209) implements
everything and is the reference.

`TARGET_STM/` (mbed's label for every STM32) holds the drivers that are the
same on every STM32 family once the family names its timers in `family.h`:
the step timer (`STEP_TIMER`, ...), the tick timer (`TICK_TIM`, ...) and
the node IO (GPIO by pin byte). Each is compiled only where the family
declares the interface.

## Portable logic (no MCU headers, PC tested)

| Folder | What | PC test |
|---|---|---|
| `drivers/remoraLink/` | SPI link frames: commands to `rxData` in the NSS interrupt, queue to the main loop, torn frames, re-align, link status | `Firmware/Tests/link` |
| `drivers/stepScheduler/` | event driven step generation (the default with a step timer; `"Step Generator": "Base Thread"` opts out) | `Firmware/Tests/stepsched` |
| `drivers/can/` | CAN hub / node protocol (`CanHub`, `CanNode`, `CAN_PROTOCOL.md`), the TMC2209 UART driver `NodeTmc2209` | `Firmware/Tests/can` |
| `drivers/rgb/` | WS2812 colour orders and set point coding | (in the CAN test) |

Run them with `sh Firmware/Tests/<name>/run.sh` (Linux / WSL, g++), the CI
runs them on every push. A change to this logic for a new family is a
change for all families: extend the test with it.

## The family layer: `TARGET_<FAMILY>/`

`family.h` names the family (`FAMILY_NAME`), gives `remoraMillis()`, the
pin name range (`CONFIG_PIN_LAST_PORT`), optionally its own
`PRU_BASEFREQ` and temperature limits, and lists the optional interfaces
it implements.

Required:

| Interface | Header | STM32G0 implementation |
|---|---|---|
| SPI slave link: DMA counter, NSS level, re-arm, restart, stop | `LinkHardware` in `drivers/remoraLink/RemoraLink.h` | `drivers/comms/RemoraComms` |
| Base and servo thread timers (`pruThread`, `pruTimer`, vectors, priorities) | `TARGET_<FAMILY>/thread/` | `thread/` (TIM6 base, TIM7 servo) |
| GPIO by name with fast `set()` / `get()` | `TARGET_<FAMILY>/drivers/pin/pin.h` | `drivers/pin` (BSRR / BRR / IDR) |

Interrupt priorities, highest first: the SPI NSS edge, the step timer and
the tick timer; then the base thread; then the servo thread. The link
writes `rxData` in the NSS interrupt, so it must be above the threads.

Optional, each declared in `family.h`; without it the feature is absent or
falls back as the table says:

| `family.h` | Interface | Without it | STM32G0 implementation |
|---|---|---|---|
| `REMORA_HAS_STEP_TIMER` | `drivers/stepScheduler/stepTimer.h`: a one-shot timer interrupt and a busy wait | Stepgen modules in the base thread | `TARGET_STM/drivers/stepTimer` (TIM3, output compare) |
| `REMORA_HAS_TICK_TIMER` | `drivers/tickTimer/tickTimer.h`: a periodic highest priority interrupt | no CAN node firmware; hub TMC2209 on `TMCStepper` | `TARGET_STM/drivers/tickTimer` (TIM14 hub, TIM6 node) |
| `REMORA_HAS_NODE_IO` | `drivers/can/canNodeIo.h`: GPIO, ADC, LED chains by pin byte, the unique ID | no CAN node firmware; hub TMC2209 on `TMCStepper` | `TARGET_STM/drivers/nodeIo` |
| `REMORA_HAS_CAN` | `drivers/can/CanBus.h`: `CanBus` and `canBusCreate()` | no CAN hub, no node | `drivers/can/RemoraCAN` (FDCAN, polled) |
| `REMORA_HAS_LED_CHAIN` | `drivers/rgb/ledChain.h`: `LedChain`, `ledChainCreate()` | "RGB" module prints "not supported" | `drivers/ws2812` (timer + DMA) |
| `REMORA_HAS_MCU_TEMP` | `drivers/mcuTemp/mcuTempSensor.h` | no over temperature shutdown | `drivers/mcuTemp` (ADC registers) |
| `REMORA_HAS_USB_CONSOLE` | `drivers/console/nodeConsole.h` | a node has no console | `drivers/usbCdc` |
| `REMORA_HAS_THREAD_STATS` | `pruThread`: `maxTicks`, `overruns`, `takeAverage()`, `periodTicks()` | no `[debug] load:` line | `thread/pruThread` |
| `REMORA_HAS_BOOT_INFO` | `drivers/debug/bootInfo.h` | no reset cause in debug builds | `drivers/debug` |

The hub's TMC2209s use the CAN nodes' UART driver (`modules/tmc/tmc2209Uart.cpp`)
when the family has the tick timer and the node IO, else `TMCStepper` with
`SoftwareSerial` (`modules/tmc/tmc2209.cpp`).

## The board file: `TARGET_<BOARD>/board.h`

| Macro | Meaning |
|---|---|
| `BOARD_NAME` | printed at boot |
| `BOARD_SD_SPI_PINS` or `BOARD_SD_SDIO` | the SD card with `config.txt` |
| `BOARD_LINK_SPI` | the SPI and slave select pin of the link to the LinuxCNC host (arguments of the family's `RemoraComms`) |
| `BOARD_SD_RELEASE` | the card is let go after `config.txt` is read |
| `BOARD_SD_UPLOAD` | `config.txt` / firmware upload over the link (`drivers/sdUpload`) |
| `BOARD_CAN` | the CAN controller and its pins (arguments of the family's CAN driver): a hub board is a CAN hub, a node board needs it |
| `BOARD_RESERVED_PINS` | pins the board uses itself, `config.txt` is refused when a module names one |
| `BOARD_NODE_RESERVED_PINS` | the same for a CAN node board |
| `BOARD_RAW_CONSOLE` | debug builds: the family's register level UART console |

Code only one board needs goes in `TARGET_<BOARD>/board.cpp` (the Manta
M8P: `sdBusRelease()` / `sdBusAcquire()`, its SD card shares lines with the
driver sockets).

## A new board of an existing family

1. `custom_targets.json`: the target, inheriting the family's MCU; a CAN
   node gets the macro `REMORA_CAN_NODE` (its `main()` is `node/nodeMain.cpp`).
2. `TARGET_<BOARD>/`: `PinNames.h`, `PeripheralPins.c` (from mbed or the
   board's schematic) and `board.h`.
3. `Firmware/build.sh`: a row in `TARGETS` (and its `FirmwareBin` folder),
   `.github/workflows/firmware.yml`: the name in the matrix.
4. LinuxCNC_UI: the board's pins and its SPI / reset pins in the profile.

## A new family

1. `TARGET_<FAMILY>/family.h` and the required layer above, starting from
   a copy of `TARGET_STM32G0` and replacing the register code.
2. The optional interfaces one by one: implement, add its
   `REMORA_HAS_...` to `family.h`, test on hardware.
3. `Firmware/build.sh` and the CI matrix.

Check on hardware with the debug build (`-D REMORA_DEBUG=1`): while
LinuxCNC is not connected it prints the link statistics, the thread load
(`[debug] load:`) and for the interrupt step generation `[debug] stepgen:`
with the latest step edge and the longest step interrupt. Measure the CPU
at the highest step rate the machine uses on all its motors at once: the
step interrupts are above the threads, and what they leave must still run
the base thread.

## State of the families

| Family | Boards | State |
|---|---|---|
| STM32G0 | Manta M8P, EBB42, EBB SB2209 | everything, built by the CI, run on hardware |
| STM32F4 | Octopus / Octopus Pro F446 and F429 (built by the CI), SKR 2, Monster8, Robin Nano 3, Spider, Spider King | link on `RemoraLink`, step timer (TIM12), tick timer (TIM7, hub TMC2209 UART), MCU temperature, thread statistics, boot info; not yet: CAN (bxCAN), LED chains; not yet run on hardware |
| STM32F1, STM32H7, LPC176x | Robin E3, SKR Mini E3, SKR 3, SKR V1.4 | required layer only, not built by the CI |
