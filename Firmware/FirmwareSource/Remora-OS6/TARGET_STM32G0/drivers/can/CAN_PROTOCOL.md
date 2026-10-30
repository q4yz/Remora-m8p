# Remora CAN expansion protocol

The Remora board that talks SPI to LinuxCNC is the **hub** (Manta M8P), expansion
boards are **nodes** (BTT EBB42 toolhead). LinuxCNC only sees the hub: remote
modules map onto the same joint, SP / PV and digital I/O indices as local ones,
so the SPI frame and `remora-spi` do not change.

Classic CAN, 1 Mbit/s, 8 byte frames, polled on both sides (no interrupts that
could disturb the base thread).

## Identifiers

11-bit identifiers, node number in the low bits (CANopen layout):

| Identifier | Name | Direction | Phase |
|---|---|---|---|
| `0x080` | SYNC | hub → all | reserved |
| `0x080 + n` | EMCY (TMC2209 status) | node → hub | TMC2209 |
| `0x100 + n` | JOINT (step generator commands) | hub → node | joints |
| `0x180 + n` | JOINT FEEDBACK (step counts) | node → hub | joints |
| `0x200 + n` | COMMAND (process data) | hub → node | 3/4 |
| `0x280 + n` | STATUS (process data) | node → hub | 3/4 |
| `0x300 + n` | PWM (duties) | hub → node | PWM |
| `0x380 + n` | ANALOG (ADC readings) | node → hub | temperature |
| `0x580 + n` | CONFIG REPLY | node → hub | 2 |
| `0x600 + n` | CONFIG | hub → node | 2 |
| `0x700 + n` | HEARTBEAT | node → hub | 1 |
| `0x7E5` | LSS (identify, assign) | hub → all | 1 |
| ext `0x1E000000` \| UUID bits 0-23 | LSS REPLY | node → hub | 1 |

Node numbers 1 ... 15. Lower identifier = higher priority on the bus.

## Node identity and numbers (phase 1)

- Every node derives a 48-bit **UUID** from the STM32 96-bit unique ID
  (FNV-1a 64, lower 48 bits), written as 12 hex digits, e.g. `0e0d81e4210c`.
- The hub's `config.txt` maps UUIDs to node numbers:

  ```json
  "CAN": {
      "Bitrate": 1000000,
      "Nodes": [
          { "UUID": "0e0d81e4210c", "Node": 1, "Comment": "left toolhead" },
          { "UUID": "4a17c3d29be0", "Node": 2, "Comment": "right toolhead" }
      ]
  }
  ```

- While a configured node is missing, the hub sends `LSS IDENTIFY` (`0x7E5`,
  byte 0 = `0x01`) once per second; with all nodes online every 5 s while
  LinuxCNC is not running, so nodes added later are found too. Every node answers with an LSS REPLY:
  UUID (6 bytes), state, current node number. The replies use an extended
  identifier containing 24 bits of the UUID, so simultaneous replies are
  serialised by CAN arbitration instead of colliding.
- A node with a configured UUID but the wrong number gets `LSS ASSIGN`
  (byte 0 = `0x02`, bytes 1-6 UUID, byte 7 node number) and confirms with a
  new LSS REPLY. Unknown UUIDs are printed on the hub's debug console when
  they appear and in the boot summary, copy them into `config.txt`; they are
  assigned number 0 (unassigned).
- An assigned node sends a HEARTBEAT every 100 ms: state, protocol version,
  CAN error counter, uptime. The hub marks a node lost after 350 ms without
  one and starts identifying again; a node that rebooted gets its number back.

Node states: 0 unassigned, 1 pre-operational (number assigned), 2 running
(configured), 3 safe (lost the hub, outputs in their safe state).

## Configuration transfer (phase 2)

Nodes have no SD card: their modules are in the node's entry of the hub's
`config.txt`, with the same keys as local modules:

```json
{ "UUID": "8e85cfce1e6c", "Node": 1, "Comment": "toolhead",
  "Modules": [
    { "Type": "Digital Pin", "Pin": "PB_8", "Mode": "Output", "Data Bit": 1,
      "Invert": "False", "Modifier": "None", "Safe State": "Low" },
    { "Type": "Digital Pin", "Pin": "PB_9", "Mode": "Input", "Data Bit": 3,
      "Invert": "True", "Modifier": "Pull Up", "Lost State": "High" } ] }
```

`"Data Bit"` is the LinuxCNC bit (`remora.output.NN` / `remora.input.NN`), it
must not be used by a local Digital Pin or another node. `"Safe State"`: output
level while LinuxCNC or the hub is gone. `"Lost State"`: input value LinuxCNC
sees while the node is lost or not configured.

Once a node is online the hub sends, on CONFIG `0x600 + n`, one frame at a
time, each acknowledged on CONFIG REPLY `0x580 + n` (byte 0 command, 1 index,
2 status, 3 detail):

| Byte 0 | Frame | Bytes 1 ... 7 |
|---|---|---|
| `0x01` | BEGIN | module count |
| `0x02` | MODULE | index, type, 5 data bytes |
| `0x03` | END | module count, 16-bit sum over all type + data bytes |

Without a reply within 50 ms a frame is sent again (3 times, then the
transfer starts over after 1 s); the node answers a repeated frame again
without applying it twice. The node checks every module (pin exists, not
reserved for CAN / USB, not used twice, bit free) and only applies the
configuration at END, then reports state RUNNING. A rejected configuration is
printed on the hub console and not retried until the node reboots. The hub
sends the configuration again whenever a node comes back or reports a state
other than RUNNING / SAFE (it rebooted).

Module types (pins as `port << 4 | number`, port 0 = A, `0xFF` none):

- `0x01` digital pin, data: pin, flags (bit 0 output, 1 invert, 2 safe high,
  4-5 modifier: none, pull up, pull down, open drain), node local data bit.
- `0x02` stepgen, data: step pin, 0, node local joint (0 or 1), direction
  pin, enable pin (active low, `0xFF`: none).
- `0x03` TMC2209, data: UART pin, flags (bits 0-1 address, bit 2
  StealthChop, bits 4-7 microsteps as power of 2), current / 10 mA, Rsense
  mOhm, stall threshold (SGTHRS). Up to 2 per node, one per UART pin.
- `0x04` analog input, data: ADC pin, 0, node local analog index (0 ... 3).
- `0x05` PWM output, data: pin, safe duty (%), node local PWM index (0 ... 3),
  period in base ticks (16-bit little endian, 25 µs each, at least 40 = 1 ms).

## Process data (phase 3)

- COMMAND `0x200 + n`, hub → node: output bits (16-bit, node local), flags
  (bit 0: LinuxCNC gone, outputs safe). Sent on a change and every 20 ms.
- STATUS `0x280 + n`, node → hub: input bits (16-bit, node local). Sent on a
  change and every 20 ms.
- Node without COMMAND for 100 ms: outputs to their safe state, state SAFE,
  back to RUNNING with the next COMMAND.
- The hub maps the node local bits onto the LinuxCNC bits; latency is a few
  milliseconds (main loop, not the servo thread), fine for switches, LEDs and
  relays, not for a probe that needs the exact trigger position.

## Mandatory nodes and E-stop

Every node is mandatory unless its entry says `"Mandatory": "False"`.
`"CAN" "Status Bit"` is a LinuxCNC input bit that is 1 only while every
mandatory node is online and configured:

```json
"CAN": { "Bitrate": 1000000, "Status Bit": 15, "Nodes": [ ... ] }
```

It goes into the E-stop chain in HAL, e.g.

```
net can-ok remora.input.15 => iocontrol.0.emc-enable-in
```

(or into the existing `and2` / estop_latch chain). LinuxCNC then cannot leave
E-stop while a mandatory node is missing, and goes to E-stop when one is lost
(no heartbeat for 350 ms), rejects its configuration or reboots (bit 0 until
it is configured again). An optional node going away only switches its inputs
to their `"Lost State"`. Without a status bit LinuxCNC does not see node
failures; the boot summary warns about it.

At boot (2.5 s after the start) the hub prints the network once: every
configured node with mandatory / optional and online / configured /
REJECTED / MISSING, the unknown UUIDs on the bus, the status bit and the bus
counters. After that only changes are printed (and nothing while LinuxCNC
runs); reset the board to see the summary again.

## Joints (remote step generators)

A `"Stepgen"` module in a node entry is a LinuxCNC joint like a local one,
same keys; `"Enable Pin"` may be left out:

```json
{ "Type": "Stepgen", "Comment": "extruder", "Joint Number": 3,
  "Step Pin": "PD_0", "Direction Pin": "PD_1", "Enable Pin": "PD_2" }
```

The joint must not be used by a Stepgen of the hub or another node; up to 2
joints per node. LinuxCNC sees no difference: `remora.joint.3.*` as usual.

- JOINT `0x100 + n`, hub → node, right after every SPI WRITE from LinuxCNC
  (once per servo period), at least every 3 ms without LinuxCNC: byte 0
  enable bits (node local joints), byte 1 sequence, then per joint the
  frequency command, 24-bit signed, steps / s.
- JOINT FEEDBACK `0x180 + n`, node → hub, the answer to every JOINT: byte 0
  the sequence answered, byte 1 the enabled joints, then per joint the low 24
  bits of the step count. The hub extends them to 32 bits into the joint
  feedback; after a node reboot the count LinuxCNC sees carries on, no jump.
- The node runs the same DDS as the Remora stepgen in its own base thread at
  40 kHz (max 20 kHz step rate, pulses one base period = 25 µs long) and takes
  each command as it arrives.
- Node without JOINT for 10 ms: joints stop, drivers disabled, until JOINT
  comes back. Also disabled in state SAFE and while LinuxCNC is gone.

Latency: the feedback is one servo period older than for a local joint
(about 1 ms), LinuxCNC sees a slightly larger following error at speed
(velocity × 1 ms); `FERROR` / `MIN_FERROR` must allow for it. Bus load: one
JOINT and one JOINT FEEDBACK (~130 µs each at 1 Mbit/s) per node and servo
period, about 26 % of the bus per node with joints.

## TMC2209 drivers

A `"TMC2209"` module in a node entry, same keys as the local module:

```json
{ "Type": "TMC2209", "Comment": "extruder", "RX pin": "PA_15", "RSense": 0.11,
  "Address": 0, "Current": 800, "Microsteps": 16, "Stealth chop": "on",
  "Stall sensitivity": 0 }
```

`"Current"` mA RMS (up to 2000), `"Microsteps"` 1, 2, 4 ... 256,
`"Stealth chop"` `"on"` / `"off"` (SpreadCycle, the default), `"Stall
sensitivity"` 0 (off) ... 255. The hub checks the values when it reads
config.txt.

The node configures the driver itself over its single wire UART, bit-banged
in the base thread at 13333 baud (3 base ticks per bit), without blocking its
main loop (about 100 ms per configuration):

- read IOIN (version), read IFCNT, write GSTAT (clear), GCONF, CHOPCONF,
  IHOLD_IRUN, TPOWERDOWN, PWMCONF, TCOOLTHRS, SGTHRS, read IFCNT again: it
  must have gone up by 8, else "writes not taken", retried every second
- then GSTAT every second: reset flag set (the driver lost its motor power)
  or no answer → configure again
- current as the Remora TMC module (TMCStepper `rms_current`, hold current
  half, vsense for small currents), but with the internal reference
  (`I_scale_analog` 0), VREF is not used; chopper toff 3, hstrt 5, hend 0,
  tbl 2, interpolation on

EMCY `0x080 + n`, node → hub: byte 0 `0x01` (TMC), 1 module index, 2 status
(0 not configured yet, 1 ok, 2 not answering, 3 writes not taken), 3 driver
version (`0x21`); on a change and every second. The hub prints it in the boot
summary and on changes. A driver that does not answer does not stop the node.

## Temperatures (thermistors)

A `"Temperature"` module in a node entry, same keys as the local module, plus
an optional `"Pullup"` (Ohm, default 4700 as the local module):

```json
{ "Type": "Temperature", "Comment": "hotend", "PV[i]": 2, "Sensor": "Thermistor",
  "Thermistor": { "Pin": "PA_3", "beta": 3950, "r0": 100000, "t0": 25 } }
```

The PV must not be used by a module of the hub (any `"PV[i]"`) or another
node; up to 4 per node. LinuxCNC sees it as a local one: `remora.PV.2`.

- The node reads the ADC every 10 ms and sends the average every 100 ms raw in
  ANALOG `0x380 + n`: per local input 2 bytes, 0 ... 65535, little endian.
- The hub turns it into °C with the formula of the local module (Beta, the
  thermistor between the ADC pin and ground, the pullup to the reference) and
  writes it into the process variable.
- 999 (as the local module's sensor error) while the node is lost or not
  configured, and with an open (ADC 65535) or shorted (ADC 0) sensor, or a
  result ≤ 0 °C. The heater PID in LinuxCNC then turns the heater off.

The heater is a `"Digital Pin"` output on the node (e.g. PB13 on the EBB42),
driven from HAL; it goes off when the node loses the hub or LinuxCNC (safe
state low).

## PWM outputs (fans, heaters)

A `"PWM"` module in a node entry, same keys as the local software PWM:

```json
{ "Type": "PWM", "Comment": "part fan", "SP[i]": 1, "PWM Pin": "PA_0",
  "PWM Max": 0, "Period us": 10000, "Safe Duty": 0 }
```

`"SP[i]"` the LinuxCNC set point in % (`remora.SP.1`, limited to 0 ... 100,
a negative PID output is 0), `"PWM Max"` 1 ... 255 scales it as on the hub
(0 or left out: 255, the full range, e.g. 128 for a 12 V fan on 24 V),
`"Period us"` 1000 ... 1638375 (default 10000 = 100 Hz), `"Safe Duty"` % while
the hub or LinuxCNC is gone (e.g. 100 for the hotend fan, 0 for a heater).
`"Hardware PWM"` / `"Variable Freq"` are not supported on nodes. The SP must not
be used by a module of the hub (any `"SP[i]"`, `"Period SP[i]"`) or another
node; up to 4 per node.

- The node makes the PWM in its base thread (40 kHz): resolution one base
  tick, 0.25 % at 100 Hz, 2.5 % at 1 kHz. Not for 4-pin fans (25 kHz).
- PWM `0x300 + n`, hub → node: per local output 2 bytes, duty 0 ... 10000
  (0.01 %), little endian. On a change, at most every 10 ms (a PID changes
  the SP every servo period), and every 100 ms.
- The node runs the safe duty after the configuration until LinuxCNC is
  there, while LinuxCNC is gone (COMMAND safe flag) and in state SAFE (no
  COMMAND from the hub for 100 ms).

A heater on PWM: PID output to the SP, `"Safe Duty": 0`, and a period of
about 100 ms (`"Period us": 100000`) for the MOSFET.

## Next phases

3. **Remote modules**: the hotend fan rule (on above a temperature) runs
   locally on the node.
5. **Safety**: node faults (e.g. TMC2209 not answering) and status bits to
   LinuxCNC in the 2 spare bytes of the SPI frame.
