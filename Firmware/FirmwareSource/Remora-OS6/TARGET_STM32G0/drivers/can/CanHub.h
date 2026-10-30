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
    joints   "Stepgen" modules on nodes: the LinuxCNC frequency commands go
             out in a JOINT frame right after each SPI WRITE, the step counts
             come back in JOINT FEEDBACK and go into the joint feedback
    TMC2209  "TMC2209" modules on nodes: the node configures the driver over
             its UART, the hub prints the status the node reports (EMCY)
    PWM      "PWM" modules on nodes: the LinuxCNC set point (%) as duty to the
             node (PWM frame), software PWM there, the safe duty without LinuxCNC
    temp     "Temperature" (thermistor) modules on nodes: the node sends its
             ADC readings (ANALOG), the hub turns them into the temperature,
             same Beta formula as the local module, into the process variable

        "CAN": { "Bitrate": 1000000,
                 "Nodes": [ { "UUID": "0e0d81e4210c", "Node": 1, "Comment": "toolhead",
                              "Modules": [ { "Type": "Digital Pin", "Pin": "PB_8",
                                             "Mode": "Output", "Data Bit": 1 } ] } ] }

    "Mandatory": "False" on a node makes it optional. "CAN" "Status Bit" is a
    LinuxCNC input bit that is 1 only while every mandatory node is online and
    configured; in HAL it goes into the E-stop chain, so LinuxCNC cannot leave
    E-stop while one is missing and goes to E-stop when one is lost.

    The network (configured and unknown nodes) is printed once at boot, then
    only changes. service() runs in the main loop; while quiet (SPI link
    running) nothing is printed, a blocking printf would make the board miss frames.
************************************************************************/

typedef struct
{
    uint8_t  uuid[6];
    uint8_t  node;              // configured node number
    char     comment[24];
    bool     mandatory;         // missing / not configured: status bit off (LinuxCNC E-stop)
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

    // joints: node local joint i is LinuxCNC joint jointNum[i]
    uint8_t  jointNum[CAN_NODE_JOINTS];
    uint8_t  jointCount;
    int32_t  jointPos[CAN_NODE_JOINTS];     // step count, extended from the 24 bits in JOINT FEEDBACK
    uint32_t jointRaw[CAN_NODE_JOINTS];     // last 24-bit count from the node
    uint8_t  jointSeq;
    bool     jointPending;      // new LinuxCNC commands not sent yet (TX queue full)
    uint32_t lastJoint;         // ms, last JOINT frame
    uint32_t jointFeedbacks;    // JOINT FEEDBACK frames received (debug)

    // TMC2209 modules: status and version from EMCY, by module index
    uint8_t  tmcStatus[CAN_MODULES_MAX];
    uint8_t  tmcVersion[CAN_MODULES_MAX];
    uint16_t tmcReport;         // modules whose status changed, waiting to be printed
    uint8_t  tmcCount;

    // thermistors: node local analog input i is LinuxCNC process variable analogPv[i]
    uint8_t  analogPv[CAN_NODE_ANALOG];
    uint8_t  analogCount;
    float    thermBeta[CAN_NODE_ANALOG];
    float    thermR0[CAN_NODE_ANALOG];
    float    thermT0[CAN_NODE_ANALOG];
    float    thermPullup[CAN_NODE_ANALOG];
    uint16_t analogRaw[CAN_NODE_ANALOG];
    float    temperature[CAN_NODE_ANALOG];  // from the last ANALOG frame
    bool     analogValid;       // an ANALOG frame since the configuration

    // PWM outputs: node local output i runs LinuxCNC set point pwmSp[i] (%)
    uint8_t  pwmSp[CAN_NODE_PWM];
    uint8_t  pwmMax[CAN_NODE_PWM];          // "PWM Max": 100 % SP gives pwmMax / 255 duty
    uint8_t  pwmCount;
    uint16_t pwmDuty[CAN_NODE_PWM];         // in the last PWM frame
    uint32_t lastPwm;           // ms
    bool     pwmSent;           // since the configuration
} canNodeEntry_t;

#define CAN_HUB_VARIABLES       6           // LinuxCNC process variables (VARIABLES in configuration.h)
#define CAN_TEMP_ERROR          999.0f      // sensor error or node lost, as the local Temperature module

#define CAN_HUB_JOINTS          8           // LinuxCNC joints (JOINTS in configuration.h)

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
#define CAN_BOOT_REPORT_MS      2500        // network summary this long after the start
#define CAN_UNKNOWN_FORGET_MS   12000       // unknown node gone: no reply to two IDENTIFYs

typedef struct
{
    uint8_t  uuid[6];
    bool     used;
    bool     printed;
    uint32_t lastSeen;          // ms
} canUnknownEntry_t;

class CanHub
{
    private:

        RemoraCAN*      can;
        canNodeEntry_t  nodes[CAN_NODE_MAX];
        int             nodeCount;
        uint32_t        lastIdentify;
        bool            quiet;          // LinuxCNC runs: no debug prints (the console blocks the main loop)
        uint32_t        bootStart;          // ms, first service() call
        bool            started;
        bool            bootReported;
        int             statusBit;          // LinuxCNC input bit: all mandatory nodes running, -1: none

        canUnknownEntry_t unknown[CAN_UNKNOWN_MAX];     // nodes on the bus that are not in config.txt
        bool            unknownNew;         // a new one, print right away

        uint16_t        usedOutputs;        // LinuxCNC data bits already taken (local pins and nodes)
        uint16_t        usedInputs;
        uint8_t         usedJoints;         // LinuxCNC joints already taken (local stepgens and nodes)
        uint8_t         usedVariables;      // LinuxCNC process variables already taken
        uint8_t         usedSetPoints;      // LinuxCNC set points already taken

        canNodeEntry_t* findNode(uint8_t node);
        bool pinUsed(const canNodeEntry_t* n, uint8_t pin);
        void handleLssReply(const canFrame_t& frame, uint32_t now);
        void handleHeartbeat(const canFrame_t& frame, uint32_t now);
        void handleConfigReply(const canFrame_t& frame, uint32_t now);
        void handleStatus(const canFrame_t& frame);
        void handleJointFeedback(const canFrame_t& frame);
        void handleEmcy(const canFrame_t& frame);
        void handleAnalog(const canFrame_t& frame);
        void sendPwm(canNodeEntry_t* n, const volatile float* setPoint, uint32_t now);
        void printTmc(const canNodeEntry_t* n, int module);
        bool sendJoint(canNodeEntry_t* n, const volatile int32_t* freq, uint8_t enable, bool safe);
        void sendConfigStep(canNodeEntry_t* n, uint32_t now);
        void serviceConfig(canNodeEntry_t* n, uint32_t now);
        void sendCommand(canNodeEntry_t* n, uint16_t outputs, bool safe, uint32_t now);
        void sendIdentify(void);
        void sendAssign(const uint8_t uuid[6], uint8_t node);
        void report(void);
        void bootReport(void);
        void rememberUnknown(const uint8_t uuid[6], uint32_t now);

    public:

        CanHub(RemoraCAN* can);

        // mandatory: LinuxCNC must not run without this node (status bit)
        bool addNode(const char* uuidHex, int node, const char* comment, bool mandatory = true);

        // LinuxCNC data bits used by the hub's own Digital Pin modules, call before addDigitalPin
        void reserveBits(uint16_t outputs, uint16_t inputs);

        // LinuxCNC input bit, 1 while every mandatory node is online and configured; wired
        // into the E-stop chain in HAL. Call after reserveBits, before addDigitalPin
        bool setStatusBit(int bit);
        bool networkOk(void);

        // a "Digital Pin" module of a node; lostHigh: input value LinuxCNC sees while the node is lost
        bool addDigitalPin(int node, const char* pin, bool output, bool invert, uint8_t modifier,
                           bool safeHigh, bool lostHigh, int dataBit);

        // LinuxCNC joints of the hub's own Stepgen modules, call before addStepgen
        void reserveJoints(uint8_t joints);

        // a "Stepgen" module of a node; enablePin "" or nullptr: none
        bool addStepgen(int node, int joint, const char* stepPin, const char* dirPin, const char* enablePin);

        // a "TMC2209" module of a node, keys as the local module: rsense in Ohm, current in mA (RMS),
        // microsteps 1 ... 256, stall: StallGuard threshold (SGTHRS), 0 off
        bool addTmc2209(int node, const char* pin, float rsense, int address, int current, int microsteps,
                        bool stealth, int stall);
        uint8_t getTmcStatus(int node, int driver);     // TMC_STATUS_* of the node's driver-th TMC2209 (test)

        // LinuxCNC process variables of the hub's own modules ("PV[i]"), call before addThermistor
        void reserveVariables(uint8_t variables);

        // a "Temperature" module with a thermistor on a node: Beta formula, pullup to the ADC
        // reference in Ohm (4700 as the local module); pv: LinuxCNC process variable ("PV[i]")
        bool addThermistor(int node, const char* pin, int pv, float beta, float r0, float t0, float pullup);

        // writes the node temperatures into txData.processVariable, main loop; CAN_TEMP_ERROR
        // while a node is lost or not configured, or its sensor is open / shorted
        void writeVariables(volatile float* processVariable);

        // LinuxCNC set points of the hub's own modules ("SP[i]", "Period SP[i]"), call before addPwm
        void reserveSetPoints(uint8_t setPoints);

        // a "PWM" module of a node: sp the LinuxCNC set point (%), pwmMax 1 ... 255 (255: full range),
        // periodUs the PWM period, safeDuty % while the hub or LinuxCNC is gone
        bool addPwm(int node, const char* pin, int sp, int pwmMax, int periodUs, int safeDuty);

        // PWM duties to the nodes, main loop: setPoint rxData.setPoint
        void servicePwm(uint32_t now, const volatile float* setPoint);

        // outputs: rxData.outputs; safe: LinuxCNC gone, node outputs to their safe state
        void service(uint32_t now, bool quiet, bool safe = true, uint16_t outputs = 0);

        // remote joints, call in the main loop after the SPI frames are handled. fresh: a new
        // WRITE frame (new frequency commands) since the last call; freq, enable: rxData.jointFreqCmd,
        // rxData.jointEnable; feedback: txData.jointFeedback, the remote joints are written
        void serviceJoints(uint32_t now, bool fresh, bool safe, const volatile int32_t* freq, uint8_t enable,
                           volatile int32_t* feedback);
        uint8_t jointMask(void);            // LinuxCNC joints served by nodes

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
