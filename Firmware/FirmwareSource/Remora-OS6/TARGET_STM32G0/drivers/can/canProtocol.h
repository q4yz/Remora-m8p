#ifndef CANPROTOCOL_H
#define CANPROTOCOL_H

/***********************************************************************
    Remora CAN expansion protocol, version 1 (see CAN_PROTOCOL.md)

    The Remora board that talks SPI to LinuxCNC is the hub, expansion boards
    (e.g. BTT EBB42 toolhead boards) are nodes. Classic CAN, 8 byte frames,
    11-bit identifiers with the node number in the low bits, CANopen style:

        0x080           SYNC            hub  -> all     reserved
        0x080 + node    EMCY            node -> hub     status / fault report, e.g. TMC driver
        0x100 + node    JOINT           hub  -> node    step generator commands, every servo period
        0x180 + node    JOINT FEEDBACK  node -> hub     step counts, answer to every JOINT
        0x200 + node    COMMAND         hub  -> node    process data (phase 3/4)
        0x280 + node    STATUS          node -> hub     process data (phase 3/4)
        0x580 + node    CONFIG REPLY    node -> hub     configuration transfer (phase 2)
        0x600 + node    CONFIG          hub  -> node    configuration transfer (phase 2)
        0x700 + node    HEARTBEAT       node -> hub     node alive, state
        0x7E5           LSS             hub  -> all     identify / assign node numbers
        0x1Exxxxxx ext  LSS REPLY       node -> hub     identity, extended id 0x1E000000 | UUID bits 0-23

    Nodes are identified by a 48-bit UUID derived from the STM32 96-bit unique
    ID. Unassigned nodes all answer IDENTIFY at the same time, so the replies
    use an extended identifier containing 24 bits of the UUID: CAN arbitration
    then serialises them instead of corrupting each other.
************************************************************************/

#include <stdint.h>

#define CAN_PROTOCOL_VERSION    4           // 2: configuration transfer and digital I/O, 3: joints, 4: TMC2209

#define CAN_NODE_MAX            15          // node numbers 1 ... 15

// standard identifiers
#define CAN_ID_SYNC             0x080
#define CAN_ID_EMCY             0x080       // + node
#define CAN_ID_JOINT            0x100       // + node
#define CAN_ID_JOINT_FEEDBACK   0x180       // + node
#define CAN_ID_COMMAND          0x200       // + node
#define CAN_ID_STATUS           0x280       // + node
#define CAN_ID_CONFIG_REPLY     0x580       // + node
#define CAN_ID_CONFIG           0x600       // + node
#define CAN_ID_HEARTBEAT        0x700       // + node
#define CAN_ID_LSS              0x7E5

// extended identifier for LSS replies: base | low 24 bits of the UUID
#define CAN_EXTID_LSS_REPLY     0x1E000000u     // bits 25-28, the UUID uses bits 0-23
#define CAN_EXTID_LSS_MASK      0x1F000000u

// LSS commands, byte 0 of a CAN_ID_LSS frame
#define LSS_IDENTIFY            0x01        // all nodes reply with their identity
#define LSS_ASSIGN              0x02        // bytes 1-6 UUID, byte 7 node number (0 = unassign)

// node states, byte 0 of HEARTBEAT and byte 6 of an LSS reply
#define NODE_STATE_UNASSIGNED   0x00
#define NODE_STATE_PREOP        0x01        // node number assigned, not configured
#define NODE_STATE_RUNNING      0x02        // configured, process data running (phase 2+)
#define NODE_STATE_SAFE         0x03        // lost the hub, outputs in their safe state

#define CAN_HEARTBEAT_MS        100         // node heartbeat period
#define CAN_NODE_TIMEOUT_MS     350         // hub: node lost after this long without heartbeat
#define CAN_IDENTIFY_MS         1000        // hub: IDENTIFY period while nodes are missing

// configuration transfer (phase 2), CONFIG frames hub -> node, byte 0:
#define CFG_BEGIN               0x01        // byte 1 module count; outputs go to their safe state, pins released
#define CFG_MODULE              0x02        // byte 1 index, byte 2 module type, bytes 3-7 module data
#define CFG_END                 0x03        // byte 1 module count, bytes 2-3 checksum: node applies, state RUNNING
// CONFIG REPLY node -> hub: byte 0 command, byte 1 index (count for BEGIN / END), byte 2 status, byte 3 detail

// CONFIG REPLY status
#define CFG_OK                  0
#define CFG_ERR_TYPE            1           // unknown module type
#define CFG_ERR_PIN             2           // pin does not exist on this board
#define CFG_ERR_RESERVED        3           // pin used by the node itself (CAN, USB)
#define CFG_ERR_PIN_USED        4           // pin used by two modules
#define CFG_ERR_FULL            5           // too many modules
#define CFG_ERR_SEQUENCE        6           // frame out of order
#define CFG_ERR_CHECKSUM        7           // END: count or checksum does not match
#define CFG_ERR_BIT             8           // local data bit out of range or used twice
#define CFG_ERR_JOINT           9           // local joint out of range or used twice
#define CFG_ERR_VALUE           10          // module setting out of range (e.g. TMC2209 microsteps)

#define CAN_MODULES_MAX         16          // modules per node
#define CAN_CONFIG_REPLY_MS     50          // hub: resend a config frame without reply after this
#define CAN_CONFIG_RETRIES      3           // then start over after CAN_CONFIG_RESTART_MS
#define CAN_CONFIG_RESTART_MS   1000

// module types, a module record is type + 5 data bytes
#define MODULE_DIGITAL_PIN      0x01        // data: pin, flags (DPIN_*), local data bit, 0, 0
#define MODULE_STEPGEN          0x02        // data: step pin, 0, local joint, direction pin, enable pin
#define MODULE_TMC2209          0x03        // data: UART pin, TMC_* flags, current / 10 mA, Rsense mOhm, stall threshold

// TMC2209 flags: bits 0-1 UART address, bit 2 StealthChop, bits 4-7 microsteps as power of 2 (0 ... 8)
#define TMC_ADDR_MASK           0x03
#define TMC_STEALTH             0x04
#define TMC_MSTEP_SHIFT         4
#define CAN_NODE_TMC            2           // TMC2209 drivers per node
#define TMC_CURRENT_MAX         2000        // mA RMS

// EMCY node -> hub: byte 0 EMCY_*, then
//   EMCY_TMC   byte 1 module index (0 ...), 2 status (TMC_STATUS_* in NodeTmc2209.h),
//              3 driver version (0x21), on a change and every CAN_EMCY_REPEAT_MS
#define EMCY_TMC                0x01
#define CAN_EMCY_REPEAT_MS      1000

#define CAN_PIN_NONE            0xFF        // e.g. a stepgen without enable pin

// digital pin flags
#define DPIN_OUTPUT             0x01
#define DPIN_INVERT             0x02
#define DPIN_SAFE_HIGH          0x04        // output level while the hub or LinuxCNC is gone
#define DPIN_MOD_SHIFT          4           // bits 4-5: pin modifier
#define DPIN_MOD_MASK           0x30
#define DPIN_MOD_NONE           0
#define DPIN_MOD_PULLUP         1
#define DPIN_MOD_PULLDOWN       2
#define DPIN_MOD_OPENDRAIN      3

// process data (phase 3): COMMAND hub -> node, STATUS node -> hub
#define CAN_PROCESS_MS          20          // COMMAND / STATUS sent on a change and at least this often
#define CAN_COMMAND_TIMEOUT_MS  100         // node: no COMMAND for this long -> state SAFE
#define CMD_FLAG_SAFE           0x01        // LinuxCNC gone: outputs to their safe state

// joints (step generators on the node), JOINT hub -> node, JOINT FEEDBACK node -> hub:
//   JOINT           byte 0 enable bits (node local joints), 1 sequence, then per joint
//                   a 24-bit signed frequency command in Hz (steps / s), little endian
//   JOINT FEEDBACK  byte 0 sequence of the JOINT answered, 1 enabled joints, then per
//                   joint the low 24 bits of the step count; the hub extends them to 32
//                   bits (a node moves far less than 2^23 steps between two frames)
#define CAN_NODE_JOINTS         2           // joints per node, fits one 8 byte frame
#define CAN_JOINT_REFRESH_MS    3           // hub: JOINT at least this often without LinuxCNC frames
#define CAN_JOINT_TIMEOUT_MS    10          // node: no JOINT for this long -> stop, drivers disabled
#define CAN_NODE_BASE_FREQ      40000       // node step generator thread, max step rate is half of it
#define CAN_FREQ_MAX            8388607     // 24-bit frequency command

static inline void canPutInt24(uint8_t* p, int32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

static inline uint32_t canGetUint24(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static inline int32_t canGetInt24(const uint8_t* p)
{
    uint32_t v = canGetUint24(p);
    return (v & 0x800000u) ? (int32_t)(v | 0xFF000000u) : (int32_t)v;
}

// difference of two 24-bit counts, -2^23 ... 2^23 - 1
static inline int32_t canDelta24(uint32_t now, uint32_t before)
{
    uint32_t d = (now - before) & 0xFFFFFFu;
    return (d & 0x800000u) ? (int32_t)(d | 0xFF000000u) : (int32_t)d;
}

#pragma pack(push, 1)

// CONFIG frame (8 bytes)
typedef struct
{
    uint8_t  cmd;                           // CFG_*
    uint8_t  index;                         // module index, or module count for BEGIN / END
    uint8_t  type;                          // MODULE_*; END: checksum low byte
    uint8_t  data[5];                       // END: data[0] checksum high byte
} canConfigFrame_t;

// CONFIG REPLY payload (4 bytes)
typedef struct
{
    uint8_t  cmd;                           // the command answered
    uint8_t  index;
    uint8_t  status;                        // CFG_OK, CFG_ERR_*
    uint8_t  detail;                        // e.g. the pin that was rejected
} canConfigReply_t;

// COMMAND payload (3 bytes)
typedef struct
{
    uint16_t outputs;                       // node local output bits
    uint8_t  flags;                         // CMD_FLAG_*
} canCommand_t;

// STATUS payload (2 bytes)
typedef struct
{
    uint16_t inputs;                        // node local input bits
} canStatus_t;

// LSS reply payload (8 bytes)
typedef struct
{
    uint8_t  uuid[6];
    uint8_t  state;                         // NODE_STATE_*
    uint8_t  node;                          // assigned node number, 0 if none
} canLssReply_t;

// HEARTBEAT payload (8 bytes)
typedef struct
{
    uint8_t  state;                         // NODE_STATE_*
    uint8_t  protocol;                      // CAN_PROTOCOL_VERSION
    uint16_t errors;                        // CAN error counter of the node
    uint32_t uptime;                        // ms since boot
} canHeartbeat_t;

#pragma pack(pop)

static_assert(sizeof(canLssReply_t) == 8, "LSS reply must fit a CAN frame");
static_assert(sizeof(canHeartbeat_t) == 8, "heartbeat must fit a CAN frame");
static_assert(sizeof(canConfigFrame_t) == 8, "config frame must fit a CAN frame");


// pins travel as one byte: port << 4 | number, port 0 = A ... 5 = F
#define CAN_PIN(port, number)   ((uint8_t)(((port) << 4) | (number)))
#define CAN_PIN_PORT(pin)       ((pin) >> 4)
#define CAN_PIN_NUMBER(pin)     ((pin) & 0x0F)

// "PB_8", "PB8", "pb_8" -> pin byte
static inline bool canParsePin(const char* s, uint8_t* pin)
{
    if (!s || (s[0] != 'P' && s[0] != 'p')) return false;
    char p = s[1];
    if (p >= 'a' && p <= 'f') p = (char)(p - 'a' + 'A');
    if (p < 'A' || p > 'F') return false;
    s += 2;
    if (*s == '_') s++;
    if (*s < '0' || *s > '9') return false;
    int n = 0;
    while (*s >= '0' && *s <= '9') n = n * 10 + (*s++ - '0');
    if (*s || n > 15) return false;
    *pin = CAN_PIN(p - 'A', n);
    return true;
}

// pin byte -> "PB_8"
static inline const char* canPinName(uint8_t pin, char buf[6])
{
    int n = CAN_PIN_NUMBER(pin);
    buf[0] = 'P';
    buf[1] = (char)('A' + CAN_PIN_PORT(pin));
    buf[2] = '_';
    if (n >= 10) { buf[3] = '1'; buf[4] = (char)('0' + n - 10); buf[5] = 0; }
    else         { buf[3] = (char)('0' + n); buf[4] = 0; }
    return buf;
}

// END checksum: 16-bit sum over type and data of every module record
static inline uint16_t canConfigChecksum(uint16_t sum, const uint8_t record[6])
{
    for (int i = 0; i < 6; i++) sum = (uint16_t)(sum + record[i]);
    return sum;
}

static inline const char* canConfigError(uint8_t status)
{
    switch (status)
    {
        case CFG_OK:            return "ok";
        case CFG_ERR_TYPE:      return "unknown module type";
        case CFG_ERR_PIN:       return "pin does not exist on the node";
        case CFG_ERR_RESERVED:  return "pin reserved on the node (CAN / USB)";
        case CFG_ERR_PIN_USED:  return "pin used twice";
        case CFG_ERR_FULL:      return "too many modules";
        case CFG_ERR_SEQUENCE:  return "sequence error";
        case CFG_ERR_CHECKSUM:  return "checksum error";
        case CFG_ERR_BIT:       return "data bit out of range or used twice";
        case CFG_ERR_JOINT:     return "joint out of range or used twice";
        case CFG_ERR_VALUE:     return "setting out of range";
    }
    return "?";
}


// 48-bit node UUID from the 96-bit STM32 unique ID (FNV-1a 64, lower 48 bits),
// written as 12 hex digits in config.txt
static inline void canUuidFromUid(const uint8_t uid[12], uint8_t uuid[6])
{
    uint64_t h = 0xcbf29ce484222325ULL;
    for (int i = 0; i < 12; i++)
    {
        h ^= uid[i];
        h *= 0x100000001b3ULL;
    }
    for (int i = 0; i < 6; i++)
    {
        uuid[i] = (uint8_t)(h >> (8 * (5 - i)));
    }
}

static inline uint32_t canUuidLow24(const uint8_t uuid[6])
{
    return ((uint32_t)uuid[3] << 16) | ((uint32_t)uuid[4] << 8) | uuid[5];
}

#endif
