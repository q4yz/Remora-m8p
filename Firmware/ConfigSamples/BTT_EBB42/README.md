# BTT EBB42 CAN V1.1 / V1.2 as a Remora CAN node

Expansion board for the Manta M8P hub, see
`FirmwareSource/Remora-OS6/TARGET_STM32G0/drivers/can/CAN_PROTOCOL.md`.

**State: phase 2 + joints + TMC2209 + temperature + PWM.** The node joins
the bus, gets its node number and its modules from the hub. Supported on the
node: Digital Pin (inputs and outputs), Stepgen (a LinuxCNC joint, e.g. the
extruder motor), TMC2209 (its driver over the UART), Temperature (thermistor)
and PWM (fans, heater).

## Board facts

| | |
|---|---|
| MCU | STM32G0B1CBT6, Cortex-M0+ 64 MHz, 128 KB flash |
| Clock | 8 MHz crystal |
| Bootloader | none, flashed over USB-C in DFU mode at `0x08000000` |
| CAN | PB0 (RX) / PB1 (TX) = FDCAN2, 1 Mbit/s, 120R termination by jumper |

Pins for the next phases (Klipper sample config v1.2): extruder step PD0 /
dir PD1 / enable PD2 (active low), TMC2209 UART PA15, heater PB13,
thermistor PA3, part fan PA0, hotend fan PA1, RGB PD3, probe PB8 / PB9,
endstops PB4 / PB3.

## Build and flash

```
mbed compile -m EBB42 -t GCC_ARM --profile release
```

Hold BOOT, press RESET (or plug in USB while holding BOOT), then from the CB1 / PC:

```
dfu-util -a 0 -s 0x08000000:leave -D BUILD/EBB42/GCC_ARM-RELEASE/firmware.bin
```

After flashing press RESET (without BOOT).

## Debug output over USB

The node firmware is a USB serial port on the same USB-C connector: "USB
Serial Device (COMx)" on Windows, `/dev/ttyACM0` on Linux, any baud rate
(e.g. PuTTY, or `picocom /dev/ttyACM0`). The USB serial number is the node
UUID. Every 2 s:

```
node: UUID 0e0d81e4210c, number 0, unassigned, uptime 12 s
CAN: tx 0 (dropped 0, waiting 0), rx 0, tx-errors 0, rx-errors 0, last error none, bus-off 0
```

plus `LSS: IDENTIFY from the hub ...` and `LSS: ASSIGN ...` when the hub
talks to the node. `rx 0` means nothing from the hub arrives; `last error
ACK` means nobody acknowledges the node's frames (wiring, termination, the
hub not running).

## Wiring

CAN H / CAN L / GND between the M8P CAN connector and the EBB42. The bus needs
120R at both ends: the termination jumper on the EBB42, and on the M8P side.

## Hub configuration

Add to the M8P `config.txt` (top level, next to `"Modules"`):

```json
"CAN": { "Bitrate": 1000000, "Nodes": [] }
```

With an empty node list the M8P prints the UUID of every node it finds on its
debug console (EXP2 pin 7, 115200 baud):

```
CAN: unknown node UUID 0e0d81e4210c, add it to "CAN" "Nodes" in config.txt
```

Add it with a node number:

```json
"CAN": { "Bitrate": 1000000,
         "Nodes": [ { "UUID": "0e0d81e4210c", "Node": 1, "Comment": "toolhead" } ] }
```

After a reset the M8P reports `CAN: node 1 (toolhead) online, state 1`.

Modules of the node go into its entry, with the same keys as local modules.
`"Data Bit"` is the LinuxCNC bit and must not be used by a Digital Pin of the
M8P itself:

```json
"Nodes": [ { "UUID": "0e0d81e4210c", "Node": 1, "Comment": "toolhead",
             "Modules": [
               { "Type": "Digital Pin", "Comment": "probe connector PB8",
                 "Pin": "PB_8", "Mode": "Output", "Data Bit": 1, "Safe State": "Low" } ] } ]
```

The M8P console then shows `CAN: node 1 (toolhead) configured, 1 module(s),
running`, the EBB42 USB console the modules it received. `halcmd setp
remora.output.01 1` switches PB8. Pins PB0 / PB1 (CAN) and PA11 / PA12 (USB)
are reserved and rejected.

## Stepper (joint)

The EBB42 stepper (TMC2209: step PD0, direction PD1, enable PD2) as LinuxCNC
joint 3, same keys as a Stepgen on the M8P:

```json
"Modules": [
  { "Type": "Stepgen", "Comment": "extruder", "Joint Number": 3,
    "Step Pin": "PD_0", "Direction Pin": "PD_1", "Enable Pin": "PD_2" },
  { "Type": "TMC2209", "Comment": "extruder driver", "RX pin": "PA_15",
    "RSense": 0.11, "Address": 0, "Current": 650, "Microsteps": 16,
    "Stealth chop": "off", "Stall sensitivity": 0 } ]
```

The joint number must not be used by a Stepgen of the M8P. In HAL it is
`remora.joint.3.*` like a local joint. The node makes the pulses itself at up
to 20 kHz; the feedback reaches LinuxCNC about 1 ms later than for a local
joint, so allow for a little more following error at speed. Without commands
from the hub for 10 ms the motor stops and the driver is disabled.

The TMC2209 module sets current (mA RMS), microsteps and StealthChop /
SpreadCycle over the UART on PA15, same keys as the local TMC2209 module
(RSense 0.11 on the EBB42). The node configures the driver right after its
configuration and again whenever the driver lost its motor power. The M8P
console shows the result in the boot summary:

```
  node 1  8e85cfce1e6c  toolhead   mandatory online, configured, 2 module(s), joint(s) 3
           TMC2209 PA_15 ok, version 0x21
```

`NOT answering`: no motor power (24 V) or a wrong pin / address; the node
keeps trying every second. Without the TMC2209 module the driver runs on its
power-on defaults (microsteps from its MS pins, current from VREF).

## Hotend: thermistor and heater

Thermistor on PA3 (4.7k pullup on the EBB42) as LinuxCNC process variable 2,
heater MOSFET on PB13 as LinuxCNC output bit 0, same keys as on the M8P:

```json
{ "Type": "Temperature", "Comment": "hotend", "PV[i]": 2, "Sensor": "Thermistor",
  "Thermistor": { "Pin": "PA_3", "beta": 3950, "r0": 100000, "t0": 25 } },
{ "Type": "Digital Pin", "Comment": "hotend heater", "Pin": "PB_13",
  "Mode": "Output", "Data Bit": 0 }
```

The PV must not be used by a module of the M8P. In HAL the temperature is
`remora.PV.2` like a local one, updated every 100 ms; 999 while the sensor is
open / shorted or the node is lost (the heater PID turns the heater off). The
boot summary on the M8P console shows the first reading:

```
           thermistor PV[2]: 23.4 C, ADC 62410
```

The heater can also be a PWM output instead of the Digital Pin (PID output
to the SP): `{ "Type": "PWM", "SP[i]": 0, "PWM Pin": "PB_13", "Period us":
100000, "Safe Duty": 0 }`.

`"Pullup"` in `"Thermistor"` sets another pullup (Ohm). Only NTC thermistors
(Beta formula), no PT1000. The heater output goes low when the node loses the
hub or LinuxCNC.

## Fans (PWM)

Part fan PA0 on LinuxCNC set point 1, hotend fan PA1 on set point 2, same keys
as a software PWM on the M8P:

```json
{ "Type": "PWM", "Comment": "part fan", "SP[i]": 1, "PWM Pin": "PA_0",
  "Period us": 10000, "Safe Duty": 0 },
{ "Type": "PWM", "Comment": "hotend fan", "SP[i]": 2, "PWM Pin": "PA_1",
  "Period us": 10000, "Safe Duty": 100 }
```

In HAL `remora.SP.1` = 0 ... 100 (%). The node makes the PWM itself (100 Hz by
default, `"Period us"` 1000 or more); `"PWM Max"` 1 ... 255 limits the range
(e.g. 128 for a 12 V fan on the 24 V supply). `"Safe Duty"` runs while
LinuxCNC or the hub is gone: 100 keeps the hotend fan on, so the heat break
stays cool. The SP must not be used by a module of the M8P.

## E-stop when the node is missing

Nodes are mandatory by default (`"Mandatory": "False"` makes one optional).
Give the hub a status bit, a free LinuxCNC input bit:

```json
"CAN": { "Bitrate": 1000000, "Status Bit": 15, "Nodes": [ ... ] }
```

and put it into the E-stop chain in the HAL file:

```
net can-ok remora.input.15 => iocontrol.0.emc-enable-in
```

LinuxCNC can then not leave E-stop without the toolhead and goes to E-stop
when it is lost. The M8P prints the whole network once at boot:

```
CAN network at boot: 1 of 1 configured node(s) online
  node 1  8e85cfce1e6c  toolhead   mandatory online, configured, 1 module(s)
  status bit remora.input.15 = 1, all mandatory nodes running
```
