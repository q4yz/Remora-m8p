#ifndef CANNODE_H
#define CANNODE_H

#include "RemoraCAN.h"
#include "canProtocol.h"

/***********************************************************************
    CAN node (expansion board, e.g. BTT EBB42):

    phase 1  identifies itself by UUID, takes the node number the hub
             assigns and sends heartbeats
    phase 2  receives its modules from the hub (CONFIG), checks them and
             sets them up, then state RUNNING
    phase 3  digital outputs from COMMAND, digital inputs in STATUS; without
             COMMAND for CAN_COMMAND_TIMEOUT_MS the outputs go to their safe
             state (state SAFE) until the hub is back
************************************************************************/

typedef struct
{
    uint8_t  type;              // MODULE_*
    uint8_t  pin;               // CAN_PIN
    uint8_t  flags;             // DPIN_*
    uint8_t  bit;               // local data bit
} canNodeModule_t;

class CanNode
{
    private:

        RemoraCAN*  can;
        uint8_t     uuid[6];
        uint8_t     node;           // 0: no number assigned yet
        uint8_t     state;          // NODE_STATE_*
        uint32_t    now;
        uint32_t    lastHeartbeat;

        // configuration: active modules, and the ones being received
        canNodeModule_t modules[CAN_MODULES_MAX];
        uint8_t     moduleCount;
        canNodeModule_t pending[CAN_MODULES_MAX];
        uint8_t     pendingCount;
        uint8_t     expectedCount;
        uint16_t    checksum;
        bool        receiving;      // between BEGIN and END

        // process data
        uint16_t    outputs;        // from the last COMMAND
        bool        safe;           // LinuxCNC gone (COMMAND flag) or hub gone (timeout)
        uint16_t    inputs;         // last sent in STATUS
        uint32_t    lastCommand;
        uint32_t    lastStatus;
        uint32_t    lastStatusTry;
        uint32_t    commandCount;   // COMMAND frames since the configuration (debug)

        bool sendLssReply(void);
        void sendHeartbeat(void);
        void sendConfigReply(uint8_t cmd, uint8_t index, uint8_t status, uint8_t detail);
        void sendStatus(uint16_t inputs);
        void handleLss(const canFrame_t& frame);
        void handleConfig(const canFrame_t& frame);
        void handleCommand(const canFrame_t& frame);
        uint8_t checkModule(const canFrame_t& frame, canNodeModule_t& m);
        void applyConfig(void);
        void releaseModules(void);
        void writeOutputs(void);
        uint16_t readInputs(void);

    public:

        void printOutputs(const char* why);         // debug: output bits and the pin levels

        CanNode(RemoraCAN* can);

        void service(uint32_t now);
        const uint8_t* getUuid(void) { return this->uuid; }
        uint8_t getNode(void) { return this->node; }
        uint8_t getState(void) { return this->state; }
        uint8_t getModuleCount(void) { return this->moduleCount; }
        uint16_t getOutputs(void) { return this->outputs; }
        uint16_t getInputs(void) { return this->inputs; }
        uint32_t getCommandCount(void) { return this->commandCount; }
};

#endif
