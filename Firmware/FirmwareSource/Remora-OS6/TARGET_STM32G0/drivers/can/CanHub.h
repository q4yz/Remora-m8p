#ifndef CANHUB_H
#define CANHUB_H

#include "RemoraCAN.h"
#include "canProtocol.h"

/***********************************************************************
    CAN hub (the Remora board talking SPI to LinuxCNC), phase 1:
    finds the expansion nodes, assigns their node numbers from config.txt
    by UUID and monitors their heartbeats.

        "CAN": { "Bitrate": 1000000,
                 "Nodes": [ { "UUID": "0e0d81e4210c", "Node": 1, "Comment": "toolhead" } ] }

    Unknown nodes are reported with their UUID so it can be copied into the
    config. service() runs in the main loop; while quiet (SPI link running)
    nothing is printed, a blocking printf would make the board miss frames.
************************************************************************/

typedef struct
{
    uint8_t  uuid[6];
    uint8_t  node;              // configured node number
    char     comment[24];
    bool     alive;
    bool     reported;          // state change waiting to be printed
    uint8_t  state;             // NODE_STATE_* from the last heartbeat
    uint16_t errors;            // CAN error counter reported by the node
    uint32_t lastSeen;          // ms
    uint32_t lastAssign;        // ms, ASSIGN rate limit
} canNodeEntry_t;

class CanHub
{
    private:

        RemoraCAN*      can;
        canNodeEntry_t  nodes[CAN_NODE_MAX];
        int             nodeCount;
        uint32_t        lastIdentify;

        uint8_t         unknownUuid[6];     // unknown node waiting to be printed
        bool            unknownPending;
        uint8_t         unknownPrinted[6];  // last unknown node printed, each one is printed once
        bool            unknownPrintedAny;

        void handleLssReply(const canFrame_t& frame, uint32_t now);
        void handleHeartbeat(const canFrame_t& frame, uint32_t now);
        void sendIdentify(void);
        void sendAssign(const uint8_t uuid[6], uint8_t node);
        void report(void);

    public:

        CanHub(RemoraCAN* can);

        bool addNode(const char* uuidHex, int node, const char* comment);
        void service(uint32_t now, bool quiet);

        uint16_t aliveMask(void);           // bit n-1 set: node n alive
        int getNodeCount(void) { return this->nodeCount; }
};

// "0e0d81e4210c" -> 6 bytes, false if not 12 hex digits
bool canParseUuid(const char* text, uint8_t uuid[6]);

#endif
