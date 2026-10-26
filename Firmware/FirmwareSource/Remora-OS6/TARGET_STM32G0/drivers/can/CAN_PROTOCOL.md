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
  byte 0 = `0x01`) once per second. Every node answers with an LSS REPLY:
  UUID (6 bytes), state, current node number. The replies use an extended
  identifier containing 24 bits of the UUID, so simultaneous replies are
  serialised by CAN arbitration instead of colliding.
- A node with a configured UUID but the wrong number gets `LSS ASSIGN`
  (byte 0 = `0x02`, bytes 1-6 UUID, byte 7 node number) and confirms with a
  new LSS REPLY. Unknown UUIDs are printed once on the hub's debug console,
  copy them into `config.txt`; they are assigned number 0 (unassigned).
- An assigned node sends a HEARTBEAT every 100 ms: state, protocol version,
  CAN error counter, uptime. The hub marks a node lost after 350 ms without
  one and starts identifying again; a node that rebooted gets its number back.

Node states: 0 unassigned, 1 pre-operational (number assigned), 2 running
(configured, phase 2+), 3 safe (lost the hub, outputs in their safe state).

## Next phases

2. **Configuration**: the hub sends each node its modules (from the node's
   entry in `config.txt`) over CONFIG / CONFIG REPLY, segmented, CRC checked.
   Nodes have no SD card.
3. **Remote modules**: temperature, heater, fans via SP / PV indices; the
   hotend fan rule runs locally on the node.
4. **Remote stepgen**: SYNC once per servo period, the node applies the speed
   command at SYNC and runs its own stepgen; the step count goes back in STATUS.
5. **Safety**: node watchdog (no SYNC for 5 ms → stop, outputs safe), node
   status bits to LinuxCNC in the 2 spare bytes of the SPI frame.
