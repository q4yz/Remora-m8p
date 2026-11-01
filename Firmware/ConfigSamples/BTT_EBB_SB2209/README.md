# BTT EBB SB2209 CAN V1.0 as a Remora CAN node

Stealthburner toolhead board for the Manta M8P hub. It is a CAN node exactly
like the EBB42: same MCU, crystal, CAN and USB pins, same node firmware (target
`EBB_SB2209` inherits `EBB42` and only changes the board name in the boot
message). Everything in `../BTT_EBB42/README.md` (debug output over USB, wiring,
hub configuration, E-stop when the node is missing) applies unchanged; this
file lists what is different: the pins. Protocol:
`FirmwareSource/Remora-OS6/TARGET_STM32G0/drivers/can/CAN_PROTOCOL.md`.

Supported on the node: Digital Pin (inputs and outputs), Stepgen (a LinuxCNC
joint, e.g. the extruder motor), TMC2209 (the onboard driver over its UART),
Temperature (thermistors) and PWM (fans, heater).

## Board facts

| | |
|---|---|
| MCU | STM32G0B1CBT6, Cortex-M0+ 64 MHz, 128 KB flash |
| Clock | 8 MHz crystal |
| Bootloader | none, flashed over USB-C in DFU mode at `0x08000000` |
| CAN | PB0 (RX) / PB1 (TX) = FDCAN2, 1 Mbit/s, 120R termination by jumper |
| Power | 12 ... 24 V, heater output max 5 A, fans max 1 A (1.5 A peak) |

## Pins

From the Klipper sample config (`sample-bigtreetech-ebb-sb-canbus-v1.0.cfg`).
Remora writes the pins as `PD_0`; `PD0` is accepted too.

| Function | Pin | Remora module |
|---|---|---|
| Extruder step / dir / enable | PD0 / PD1 / PD2 (enable active low) | Stepgen |
| TMC2209 UART | PA15 (RSense 0.11) | TMC2209 |
| Heater E0 | PB13 | Digital Pin or PWM |
| Hotend thermistor | PA3 (4.7k pullup) | Temperature |
| Board thermistor (NTC 3950) | PA2 | Temperature |
| FAN0 (part fan) | PA0 | PWM |
| FAN1 (hotend fan) | PA1 | PWM |
| 4-wire fan PWM / tachometer | PB14 / PB15 | PWM / Digital Pin (no RPM) |
| Probe (NPN / PNP by jumper) | PC13 (pull up) | Digital Pin input |
| BLTouch sensor / control | PB8 / PB9 | Digital Pin |
| IO header | PB5, PB6, PB7 | Digital Pin |
| RGB (neopixel) | PD3 | not supported |
| ADXL345 | CS PB12, SPI2 PB2 / PB11 / PB10 | not supported |
| PT100 / PT1000 (MAX31865) | CS PA4, SPI1 | not supported (only NTC thermistors) |

Reserved and rejected by the node: PB0 / PB1 (CAN), PA11 / PA12 (USB).

## Build and flash

```
mbed compile -m EBB_SB2209 -t GCC_ARM --profile release
```

or take `FirmwareBin/STM32G0B1/BTT_EBB_SB2209/firmware.bin`. Hold BOOT, press
RESET (or plug in USB while holding BOOT), then from the CB1 / PC:

```
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
```

After flashing press RESET (without BOOT). The USB console then starts with
`Remora CAN node, BTT EBB SB2209 CAN` and the node UUID.

## Hub configuration: a complete toolhead

The node entry in the M8P `config.txt` (`"CAN"` `"Nodes"`, see the EBB42
README for finding the UUID). Joint, data bit, SP and PV numbers are examples
and must not be used by modules of the M8P itself:

```json
{ "UUID": "0e0d81e4210c", "Node": 1, "Comment": "toolhead",
  "Modules": [
    { "Type": "Stepgen", "Comment": "extruder", "Joint Number": 3,
      "Step Pin": "PD_0", "Direction Pin": "PD_1", "Enable Pin": "PD_2" },
    { "Type": "TMC2209", "Comment": "extruder driver", "RX pin": "PA_15",
      "RSense": 0.11, "Address": 0, "Current": 650, "Microsteps": 16,
      "Stealth chop": "off", "Stall sensitivity": 0 },
    { "Type": "Temperature", "Comment": "hotend", "PV[i]": 2, "Sensor": "Thermistor",
      "Thermistor": { "Pin": "PA_3", "beta": 3950, "r0": 100000, "t0": 25 } },
    { "Type": "Temperature", "Comment": "board", "PV[i]": 3, "Sensor": "Thermistor",
      "Thermistor": { "Pin": "PA_2", "beta": 3950, "r0": 100000, "t0": 25 } },
    { "Type": "PWM", "Comment": "hotend heater", "SP[i]": 0, "PWM Pin": "PB_13",
      "Period us": 100000, "Safe Duty": 0 },
    { "Type": "PWM", "Comment": "part fan", "SP[i]": 1, "PWM Pin": "PA_0",
      "Period us": 10000, "Safe Duty": 0 },
    { "Type": "PWM", "Comment": "hotend fan", "SP[i]": 2, "PWM Pin": "PA_1",
      "Period us": 10000, "Safe Duty": 100 },
    { "Type": "Digital Pin", "Comment": "probe", "Pin": "PC_13", "Mode": "Input",
      "Modifier": "Pull Up", "Data Bit": 4 } ] }
```

The node takes up to 16 modules: up to 2 joints / TMC2209, 4 thermistors and 4
PWM outputs, and up to 16 digital inputs and 16 outputs. In HAL they appear like local ones:
`remora.joint.3.*`, `remora.PV.2`, `remora.SP.1`, `remora.input.04`.

The heater can also be a Digital Pin output instead of PWM. The hotend fan's
`"Safe Duty": 100` keeps the heat break cooled while LinuxCNC or the hub is gone.
The 4-wire fan works on PB14 as a PWM output (100 Hz software PWM, not the
25 kHz of the fan spec); its tachometer on PB15 can only be read as a digital
input, there is no RPM measurement.
