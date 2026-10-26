#ifndef CANPROTOCOL_H
#define CANPROTOCOL_H

/***********************************************************************
    Remora CAN expansion protocol, version 1 (see CAN_PROTOCOL.md)

    The Remora board that talks SPI to LinuxCNC is the hub, expansion boards
    (e.g. BTT EBB42 toolhead boards) are nodes. Classic CAN, 8 byte frames,
    11-bit identifiers with the node number in the low bits, CANopen style:

        0x080           SYNC            hub  -> all     once per servo period (phase 4)
        0x080 + node    EMCY            node -> hub     fault report (phase 5)
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

#define CAN_PROTOCOL_VERSION    1

#define CAN_NODE_MAX            15          // node numbers 1 ... 15

// standard identifiers
#define CAN_ID_SYNC             0x080
#define CAN_ID_EMCY             0x080       // + node
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

#pragma pack(push, 1)

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
