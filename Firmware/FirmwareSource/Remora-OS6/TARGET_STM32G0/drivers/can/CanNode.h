#ifndef CANNODE_H
#define CANNODE_H

#include "RemoraCAN.h"
#include "canProtocol.h"
#include "NodeTmc2209.h"

/***********************************************************************
    CAN node (expansion board, e.g. BTT EBB42):

    phase 1  identifies itself by UUID, takes the node number the hub
             assigns and sends heartbeats
    phase 2  receives its modules from the hub (CONFIG), checks them and
             sets them up, then state RUNNING
    phase 3  digital outputs from COMMAND, digital inputs in STATUS; without
             COMMAND for CAN_COMMAND_TIMEOUT_MS the outputs go to their safe
             state (state SAFE) until the hub is back
    joints   step generators: frequency commands from JOINT, step counts back
             in JOINT FEEDBACK; baseTick() makes the pulses, called by the node
             firmware from a timer interrupt at CAN_NODE_BASE_FREQ
    TMC2209  drivers configured over their UART (NodeTmc2209, bit-banged in
             baseTick), status to the hub in EMCY
    analog   ADC inputs (thermistors), averaged, raw to the hub in ANALOG
************************************************************************/

#define NODE_STEPBIT            22          // DDS accumulator bit that makes a step, as the Remora stepgen

// the module record as received (type + 5 data bytes)
typedef struct
{
    uint8_t  type;              // MODULE_*
    uint8_t  pin;               // CAN_PIN; stepgen: step pin; TMC2209: UART pin
    uint8_t  flags;             // DPIN_*; TMC2209: TMC_*
    uint8_t  bit;               // local data bit; stepgen: local joint; TMC2209: current / 10 mA; analog: local index
    uint8_t  dirPin;            // stepgen: direction pin; TMC2209: Rsense mOhm
    uint8_t  enablePin;         // stepgen: enable pin (active low), CAN_PIN_NONE: none; TMC2209: stall threshold
} canNodeModule_t;

// a step generator, the base thread interrupt (baseTick) and the main loop share it
typedef struct
{
    volatile bool     active;
    uint8_t           stepPin;
    uint8_t           dirPin;
    uint8_t           enablePin;
    bool              enabled;          // driver enabled (main loop)
    volatile int32_t  addValue;         // DDS add value per tick, 0: stopped
    uint32_t          accumulator;      // DDS accumulator (interrupt)
    volatile int32_t  count;            // position in steps (interrupt)
    bool              stepHigh;         // step pin high since the last tick (interrupt)
    bool              forward;          // level of the direction pin (interrupt)
} canNodeJoint_t;

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

        // joints
        canNodeJoint_t joints[CAN_NODE_JOINTS];
        uint8_t     jointCount;     // local joints 0 ... jointCount - 1 in JOINT frames
        int64_t     frequencyScale; // DDS add value per Hz, 32.32 fixed point
        uint32_t    lastJoint;      // ms, last JOINT frame
        bool        jointsStopped;  // JOINT timeout, reported once
        uint32_t    jointFrames;    // JOINT frames since the configuration (debug)

        // TMC2209 drivers, tmc[i] is module tmcModule[i]
        NodeTmc2209 tmc[CAN_NODE_TMC];
        volatile uint8_t tmcCount;
        uint8_t     tmcModule[CAN_NODE_TMC];
        uint32_t    lastEmcy[CAN_NODE_TMC];

        // analog inputs, local index i on analogPin[i]
        uint8_t     analogCount;    // local inputs 0 ... analogCount - 1 in ANALOG
        uint8_t     analogPin[CAN_NODE_ANALOG];
        uint32_t    analogSum[CAN_NODE_ANALOG];
        uint16_t    analogValue[CAN_NODE_ANALOG];   // last average sent
        uint16_t    analogSamples;
        uint32_t    lastAnalogSample;
        uint32_t    lastAnalog;

        bool sendLssReply(void);
        void sendHeartbeat(void);
        void sendConfigReply(uint8_t cmd, uint8_t index, uint8_t status, uint8_t detail);
        void sendStatus(uint16_t inputs);
        void handleLss(const canFrame_t& frame);
        void handleConfig(const canFrame_t& frame);
        void handleCommand(const canFrame_t& frame);
        void handleJoint(const canFrame_t& frame);
        void sendJointFeedback(uint8_t seq);
        bool sendTmcStatus(int i);
        void serviceTmc(void);
        void serviceAnalog(void);
        void setJointEnable(canNodeJoint_t& j, bool enabled);
        void stopJoints(void);
        uint8_t checkModule(const canFrame_t& frame, canNodeModule_t& m);
        void applyConfig(void);
        void releaseModules(void);
        void writeOutputs(void);
        uint16_t readInputs(void);

    public:

        void printOutputs(const char* why);         // debug: output bits and the pin levels

        CanNode(RemoraCAN* can, uint32_t baseFreq = CAN_NODE_BASE_FREQ);

        void service(uint32_t now);
        void baseTick(void);                        // step pulses, from the base thread interrupt
        const uint8_t* getUuid(void) { return this->uuid; }
        uint8_t getNode(void) { return this->node; }
        uint8_t getState(void) { return this->state; }
        uint8_t getModuleCount(void) { return this->moduleCount; }
        uint16_t getOutputs(void) { return this->outputs; }
        uint16_t getInputs(void) { return this->inputs; }
        uint32_t getCommandCount(void) { return this->commandCount; }
        uint8_t getJointCount(void) { return this->jointCount; }
        int32_t getJointPosition(int joint) { return this->joints[joint].count; }
        bool getJointEnabled(int joint) { return this->joints[joint].enabled; }
        uint32_t getJointFrames(void) { return this->jointFrames; }
        uint8_t getTmcCount(void) { return this->tmcCount; }
        NodeTmc2209& getTmc(int i) { return this->tmc[i]; }
        uint8_t getAnalogCount(void) { return this->analogCount; }
        uint8_t getAnalogPin(int i) { return this->analogPin[i]; }
        uint16_t getAnalogValue(int i) { return this->analogValue[i]; }
};

#endif
