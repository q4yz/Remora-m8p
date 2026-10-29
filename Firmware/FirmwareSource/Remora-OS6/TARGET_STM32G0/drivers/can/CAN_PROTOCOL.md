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
| `0x080` | SYNC | hub → all | 4 |
| `0x080 + n` | EMCY | node → hub | 5 |
| `0x200 + n` | COMMAND (process data) | hub → node | 3/4 |
| `0x280 + n` | STATUS (process data) | node → hub | 3/4 |
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
  they appear and again every 5 s, copy them into `config.txt`; they are
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

Module types: `0x01` digital pin, data: pin (`port << 4 | number`, port 0 =
A), flags (bit 0 output, 1 invert, 2 safe high, 4-5 modifier: none, pull up,
pull down, open drain), node local data bit.

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

## Next phases

3. **Remote modules**: temperature, heater, fans via SP / PV indices; the
   hotend fan rule runs locally on the node.
4. **Remote stepgen**: SYNC once per servo period, the node applies the speed
   command at SYNC and runs its own stepgen; the step count goes back in STATUS.
5. **Safety**: node watchdog (no SYNC for 5 ms → stop, outputs safe), node
   status bits to LinuxCNC in the 2 spare bytes of the SPI frame.
