# BTT EBB42 CAN V1.1 / V1.2 as a Remora CAN node

Expansion board for the Manta M8P hub, see
`FirmwareSource/Remora-OS6/TARGET_STM32G0/drivers/can/CAN_PROTOCOL.md`.

**State: phase 2.** The node joins the bus, gets its node number and its
modules from the hub. Supported on the node so far: Digital Pin (inputs and
outputs). Extruder, heater, thermistor and fans follow in the next phases.

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
