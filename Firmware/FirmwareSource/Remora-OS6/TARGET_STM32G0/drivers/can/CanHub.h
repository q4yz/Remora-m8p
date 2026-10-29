#ifndef CANHUB_H
#define CANHUB_H

#include "RemoraCAN.h"
#include "canProtocol.h"

/***********************************************************************
    CAN hub (the Remora board talking SPI to LinuxCNC):

    phase 1  finds the expansion nodes, assigns their node numbers from
             config.txt by UUID and monitors their heartbeats
    phase 2  sends each node its modules (CONFIG, one module per frame,
             every frame acknowledged), again after a node reboot
    phase 3  digital outputs to the nodes (COMMAND), digital inputs from
             them (STATUS), mapped onto the LinuxCNC data bits

        "CAN": { "Bitrate": 1000000,
                 "Nodes": [ { "UUID": "0e0d81e4210c", "Node": 1, "Comment": "toolhead",
                              "Modules": [ { "Type": "Digital Pin", "Pin": "PB_8",
                                             "Mode": "Output", "Data Bit": 1 } ] } ] }

    Unknown nodes are reported with their UUID every few seconds so it can be
    copied into the config. service() runs in the main loop; while quiet (SPI link running)
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

    // modules from config.txt, as sent in CONFIG frames: type + 5 data bytes
    uint8_t  modules[CAN_MODULES_MAX][6];
    uint8_t  moduleCount;
    uint8_t  outBit[16];        // node local output bit -> LinuxCNC output bit
    uint8_t  outCount;
    uint8_t  inBit[16];         // node local input bit -> LinuxCNC input bit
    uint8_t  inCount;
    uint16_t inMask;            // LinuxCNC input bits served by this node
    uint16_t inLost;            // their values while the node is lost
    uint16_t inValues;          // their current values

    // configuration transfer
    uint8_t  cfgState;          // CFG_STATE_*
    uint8_t  cfgStep;           // 0 BEGIN, 1 ... moduleCount MODULE, moduleCount + 1 END
    uint8_t  cfgRetries;
    uint32_t cfgSentAt;         // ms
    uint8_t  cfgReport;         // CFG_REPORT_*, waiting to be printed
    uint8_t  cfgErrIndex, cfgErrStatus, cfgErrDetail;

    // process data
    uint16_t lastOutputs;       // node local bits in the last COMMAND
    uint8_t  lastFlags;
    uint32_t lastCommand;       // ms
    bool     commandSent;       // since the last configuration (debug print)
} canNodeEntry_t;

#define CFG_STATE_NEEDED        0       // send the configuration when the node is online
#define CFG_STATE_SENDING       1
#define CFG_STATE_DONE          2       // node running, process data
#define CFG_STATE_FAILED        3       // rejected, wait for a node reboot / config change

#define CFG_REPORT_NONE         0
#define CFG_REPORT_DONE         1
#define CFG_REPORT_REJECTED     2
#define CFG_REPORT_TIMEOUT      3

// pin modifiers for addDigitalPin: DPIN_MOD_* (canProtocol.h)

#define CAN_UNKNOWN_MAX         4
#define CAN_IDLE_IDENTIFY_MS    5000        // IDENTIFY while all nodes are online (not while running)
#define CAN_UNKNOWN_PRINT_MS    5000        // repeat the unknown UUIDs on the console
#define CAN_UNKNOWN_FORGET_MS   12000       // unknown node gone: no reply to two IDENTIFYs

typedef struct
{
    uint8_t  uuid[6];
    bool     used;
    uint32_t lastSeen;          // ms
} canUnknownEntry_t;

class CanHub
{
    private:

        RemoraCAN*      can;
        canNodeEntry_t  nodes[CAN_NODE_MAX];
        int             nodeCount;
        uint32_t        lastIdentify;
        uint32_t        lastStatus;         // ms, bus status print while nodes are missing
        uint32_t        lastUnknownPrint;   // ms

        canUnknownEntry_t unknown[CAN_UNKNOWN_MAX];     // nodes on the bus that are not in config.txt
        bool            unknownNew;         // a new one, print right away

        uint16_t        usedOutputs;        // LinuxCNC data bits already taken (local pins and nodes)
        uint16_t        usedInputs;

        canNodeEntry_t* findNode(uint8_t node);
        void handleLssReply(const canFrame_t& frame, uint32_t now);
        void handleHeartbeat(const canFrame_t& frame, uint32_t now);
        void handleConfigReply(const canFrame_t& frame, uint32_t now);
        void handleStatus(const canFrame_t& frame);
        void sendConfigStep(canNodeEntry_t* n, uint32_t now);
        void serviceConfig(canNodeEntry_t* n, uint32_t now);
        void sendCommand(canNodeEntry_t* n, uint16_t outputs, bool safe, uint32_t now);
        void sendIdentify(void);
        void sendAssign(const uint8_t uuid[6], uint8_t node);
        void report(uint32_t now);
        void rememberUnknown(const uint8_t uuid[6], uint32_t now);

    public:

        CanHub(RemoraCAN* can);

        bool addNode(const char* uuidHex, int node, const char* comment);

        // LinuxCNC data bits used by the hub's own Digital Pin modules, call before addDigitalPin
        void reserveBits(uint16_t outputs, uint16_t inputs);

        // a "Digital Pin" module of a node; lostHigh: input value LinuxCNC sees while the node is lost
        bool addDigitalPin(int node, const char* pin, bool output, bool invert, uint8_t modifier,
                           bool safeHigh, bool lostHigh, int dataBit);

        // outputs: rxData.outputs; safe: LinuxCNC gone, node outputs to their safe state
        void service(uint32_t now, bool quiet, bool safe = true, uint16_t outputs = 0);

        // LinuxCNC input bits served by nodes and their values: inputs = (inputs & ~mask) | values
        uint16_t inputMask(void);
        uint16_t inputValues(void);

        uint16_t aliveMask(void);           // bit n-1 set: node n alive
        uint16_t runningMask(void);         // bit n-1 set: node n configured and running
        int getNodeCount(void) { return this->nodeCount; }
};

// "0e0d81e4210c" -> 6 bytes, false if not 12 hex digits
bool canParseUuid(const char* text, uint8_t uuid[6]);

#endif
