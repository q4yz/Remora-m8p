#include "CanHub.h"

#include <string.h>

static int hexDigit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool canParseUuid(const char* text, uint8_t uuid[6])
{
    if (!text || strlen(text) != 12) return false;
    for (int i = 0; i < 6; i++)
    {
        int hi = hexDigit(text[2 * i]), lo = hexDigit(text[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        uuid[i] = (uint8_t)(hi << 4 | lo);
    }
    return true;
}

static void printUuid(const uint8_t uuid[6])
{
    for (int i = 0; i < 6; i++) printf("%02x", uuid[i]);
}


CanHub::CanHub(RemoraCAN* can) :
    can(can),
    nodeCount(0),
    lastIdentify(0),
    quiet(false),
    bootStart(0),
    started(false),
    bootReported(false),
    statusBit(-1),
    unknownNew(false),
    usedOutputs(0),
    usedInputs(0),
    usedJoints(0)
{
    memset(this->nodes, 0, sizeof(this->nodes));
    memset(this->unknown, 0, sizeof(this->unknown));
}


bool CanHub::addNode(const char* uuidHex, int node, const char* comment, bool mandatory)
{
    if (this->nodeCount >= CAN_NODE_MAX) return false;

    canNodeEntry_t* n = &this->nodes[this->nodeCount];
    if (!canParseUuid(uuidHex, n->uuid))
    {
        printf("CAN: invalid UUID '%s', expected 12 hex digits\n", uuidHex ? uuidHex : "");
        return false;
    }
    if (node < 1 || node > CAN_NODE_MAX)
    {
        printf("CAN: invalid node number %d, 1 ... %d\n", node, CAN_NODE_MAX);
        return false;
    }
    for (int i = 0; i < this->nodeCount; i++)
    {
        if (this->nodes[i].node == node || !memcmp(this->nodes[i].uuid, n->uuid, 6))
        {
            printf("CAN: node %d or UUID %s configured twice\n", node, uuidHex);
            return false;
        }
    }

    n->node = (uint8_t)node;
    n->mandatory = mandatory;
    strncpy(n->comment, comment ? comment : "", sizeof(n->comment) - 1);
    this->nodeCount++;

    printf("CAN: node %d = %s (%s), %s\n", node, uuidHex, n->comment, mandatory ? "mandatory" : "optional");
    return true;
}


canNodeEntry_t* CanHub::findNode(uint8_t node)
{
    for (int i = 0; i < this->nodeCount; i++)
    {
        if (this->nodes[i].node == node) return &this->nodes[i];
    }
    return nullptr;
}


void CanHub::reserveBits(uint16_t outputs, uint16_t inputs)
{
    this->usedOutputs |= outputs;
    this->usedInputs |= inputs;
}


bool CanHub::setStatusBit(int bit)
{
    if (bit < 0 || bit > 15)
    {
        printf("CAN: status bit %d out of range 0 ... 15\n", bit);
        return false;
    }
    if (this->usedInputs & (1u << bit))
    {
        printf("CAN: status bit %d, input data bit %d is already used\n", bit, bit);
        return false;
    }
    this->usedInputs |= (uint16_t)(1u << bit);
    this->statusBit = bit;
    printf("CAN: status bit = remora.input.%02d, 1 while all mandatory nodes run\n", bit);
    return true;
}


bool CanHub::networkOk()
{
    for (int i = 0; i < this->nodeCount; i++)
    {
        const canNodeEntry_t* n = &this->nodes[i];
        if (n->mandatory && !(n->alive && n->cfgState == CFG_STATE_DONE)) return false;
    }
    return true;
}


bool CanHub::addDigitalPin(int node, const char* pinName, bool output, bool invert, uint8_t modifier,
                           bool safeHigh, bool lostHigh, int dataBit)
{
    canNodeEntry_t* n = (node >= 1 && node <= CAN_NODE_MAX) ? this->findNode((uint8_t)node) : nullptr;
    uint8_t pin;
    char name[6];

    if (!n)
    {
        printf("CAN: module for node %d, which is not configured\n", node);
        return false;
    }
    if (n->moduleCount >= CAN_MODULES_MAX)
    {
        printf("CAN: node %d: more than %d modules\n", node, CAN_MODULES_MAX);
        return false;
    }
    if (!canParsePin(pinName, &pin))
    {
        printf("CAN: node %d: invalid pin '%s'\n", node, pinName ? pinName : "");
        return false;
    }
    if (dataBit < 0 || dataBit > 15)
    {
        printf("CAN: node %d: %s, data bit %d out of range 0 ... 15\n", node, pinName, dataBit);
        return false;
    }
    uint16_t bit = (uint16_t)(1u << dataBit);
    if ((output ? this->usedOutputs : this->usedInputs) & bit)
    {
        printf("CAN: node %d: %s, %s data bit %d is already used\n", node, pinName, output ? "output" : "input", dataBit);
        return false;
    }
    if (this->pinUsed(n, pin))
    {
        printf("CAN: node %d: pin %s used twice\n", node, canPinName(pin, name));
        return false;
    }

    uint8_t flags = (uint8_t)((modifier << DPIN_MOD_SHIFT) & DPIN_MOD_MASK);
    if (output) flags |= DPIN_OUTPUT;
    if (invert) flags |= DPIN_INVERT;
    if (safeHigh) flags |= DPIN_SAFE_HIGH;

    // node local bits are numbered in order, the hub maps them onto the LinuxCNC bits
    uint8_t local;
    if (output)
    {
        local = n->outCount;
        n->outBit[n->outCount++] = (uint8_t)dataBit;
        this->usedOutputs |= bit;
    }
    else
    {
        local = n->inCount;
        n->inBit[n->inCount++] = (uint8_t)dataBit;
        n->inMask |= bit;
        if (lostHigh) n->inLost |= bit;
        n->inValues = n->inLost;
        this->usedInputs |= bit;
    }

    uint8_t* m = n->modules[n->moduleCount++];
    m[0] = MODULE_DIGITAL_PIN;
    m[1] = pin;
    m[2] = flags;
    m[3] = local;
    m[4] = 0;
    m[5] = 0;

    printf("CAN: node %d module %d: digital %s %s, LinuxCNC %s bit %d\n", node, n->moduleCount,
           output ? "output" : "input", canPinName(pin, name), output ? "output" : "input", dataBit);
    return true;
}


// a pin of a module record (type, pin, flags, bit, pin 2, pin 3) of the node
bool CanHub::pinUsed(const canNodeEntry_t* n, uint8_t pin)
{
    for (int i = 0; i < n->moduleCount; i++)
    {
        const uint8_t* m = n->modules[i];
        if (m[1] == pin) return true;
        if (m[0] == MODULE_STEPGEN && (m[4] == pin || m[5] == pin)) return true;
    }
    return false;
}


void CanHub::reserveJoints(uint8_t joints)
{
    this->usedJoints |= joints;
}


bool CanHub::addStepgen(int node, int joint, const char* stepName, const char* dirName, const char* enableName)
{
    canNodeEntry_t* n = (node >= 1 && node <= CAN_NODE_MAX) ? this->findNode((uint8_t)node) : nullptr;
    uint8_t step, dir, enable = CAN_PIN_NONE;
    char name[6];

    if (!n)
    {
        printf("CAN: module for node %d, which is not configured\n", node);
        return false;
    }
    if (n->moduleCount >= CAN_MODULES_MAX)
    {
        printf("CAN: node %d: more than %d modules\n", node, CAN_MODULES_MAX);
        return false;
    }
    if (n->jointCount >= CAN_NODE_JOINTS)
    {
        printf("CAN: node %d: more than %d joints\n", node, CAN_NODE_JOINTS);
        return false;
    }
    if (joint < 0 || joint >= CAN_HUB_JOINTS)
    {
        printf("CAN: node %d: joint %d out of range 0 ... %d\n", node, joint, CAN_HUB_JOINTS - 1);
        return false;
    }
    if (this->usedJoints & (1u << joint))
    {
        printf("CAN: node %d: joint %d is already used\n", node, joint);
        return false;
    }
    if (!canParsePin(stepName, &step) || !canParsePin(dirName, &dir) ||
        (enableName && *enableName && !canParsePin(enableName, &enable)))
    {
        printf("CAN: node %d: joint %d, invalid step / direction / enable pin\n", node, joint);
        return false;
    }
    uint8_t pins[3] = { step, dir, enable };
    for (int k = 0; k < 3; k++)
    {
        if (pins[k] == CAN_PIN_NONE) continue;
        bool twice = this->pinUsed(n, pins[k]);
        for (int l = 0; l < k; l++) if (pins[l] == pins[k]) twice = true;
        if (twice)
        {
            printf("CAN: node %d: pin %s used twice\n", node, canPinName(pins[k], name));
            return false;
        }
    }

    uint8_t local = n->jointCount;
    n->jointNum[n->jointCount++] = (uint8_t)joint;
    this->usedJoints |= (uint8_t)(1u << joint);

    uint8_t* m = n->modules[n->moduleCount++];
    m[0] = MODULE_STEPGEN;
    m[1] = step;
    m[2] = 0;
    m[3] = local;
    m[4] = dir;
    m[5] = enable;

    char d[6], e[6];
    printf("CAN: node %d module %d: stepgen, LinuxCNC joint %d, step %s, direction %s, enable %s\n", node,
           n->moduleCount, joint, canPinName(step, name), canPinName(dir, d),
           enable == CAN_PIN_NONE ? "none" : canPinName(enable, e));
    return true;
}


void CanHub::sendIdentify()
{
    canFrame_t f = {};
    f.id = CAN_ID_LSS;
    f.len = 1;
    f.data[0] = LSS_IDENTIFY;
    this->can->send(f);
}


void CanHub::sendAssign(const uint8_t uuid[6], uint8_t node)
{
    canFrame_t f = {};
    f.id = CAN_ID_LSS;
    f.len = 8;
    f.data[0] = LSS_ASSIGN;
    memcpy(&f.data[1], uuid, 6);
    f.data[7] = node;
    this->can->send(f);
}


void CanHub::handleLssReply(const canFrame_t& frame, uint32_t now)
{
    if (frame.len != 8) return;

    canLssReply_t reply;
    memcpy(&reply, frame.data, sizeof(reply));

    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        if (memcmp(n->uuid, reply.uuid, 6)) continue;

        // known node: make sure it carries the configured number
        if (reply.node != n->node && (now - n->lastAssign) > 100)
        {
            this->sendAssign(n->uuid, n->node);
            n->lastAssign = now;
        }
        return;
    }

    // a node that is not in config.txt: report its UUID, and make sure it has no number
    this->rememberUnknown(reply.uuid, now);
    if (reply.node != 0) this->sendAssign(reply.uuid, 0);
}


void CanHub::rememberUnknown(const uint8_t uuid[6], uint32_t now)
{
    canUnknownEntry_t* slot = nullptr;
    for (int i = 0; i < CAN_UNKNOWN_MAX; i++)
    {
        canUnknownEntry_t* u = &this->unknown[i];
        if (u->used && !memcmp(u->uuid, uuid, 6))
        {
            u->lastSeen = now;
            return;
        }
        // free slot, else the one not seen for the longest time
        if (!slot || (slot->used && (!u->used || (now - u->lastSeen) > (now - slot->lastSeen)))) slot = u;
    }

    memcpy(slot->uuid, uuid, 6);
    slot->used = true;
    slot->printed = false;
    slot->lastSeen = now;
    this->unknownNew = true;
}


void CanHub::handleHeartbeat(const canFrame_t& frame, uint32_t now)
{
    uint8_t node = (uint8_t)(frame.id - CAN_ID_HEARTBEAT);
    if (frame.len != 8) return;

    canHeartbeat_t hb;
    memcpy(&hb, frame.data, sizeof(hb));

    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        if (n->node != node) continue;

        if (!n->alive) n->reported = true;
        n->alive = true;
        n->lastSeen = now;
        n->state = hb.state;
        n->errors = hb.errors;

        // configured, but the node reports it is not: it rebooted, send the configuration again
        if (n->cfgState == CFG_STATE_DONE && hb.state != NODE_STATE_RUNNING && hb.state != NODE_STATE_SAFE)
        {
            n->cfgState = CFG_STATE_NEEDED;
            n->cfgRetries = 0;
        }
        return;
    }
}


/***********************************************************************
    Configuration transfer (phase 2): BEGIN, one MODULE frame per module,
    END with a checksum, each one acknowledged by the node (stop and wait)
************************************************************************/

void CanHub::sendConfigStep(canNodeEntry_t* n, uint32_t now)
{
    canConfigFrame_t c;
    memset(&c, 0, sizeof(c));

    if (n->cfgStep == 0)
    {
        c.cmd = CFG_BEGIN;
        c.index = n->moduleCount;
    }
    else if (n->cfgStep <= n->moduleCount)
    {
        c.cmd = CFG_MODULE;
        c.index = (uint8_t)(n->cfgStep - 1);
        memcpy(&c.type, n->modules[c.index], 6);
    }
    else
    {
        uint16_t sum = 0;
        for (int i = 0; i < n->moduleCount; i++) sum = canConfigChecksum(sum, n->modules[i]);
        c.cmd = CFG_END;
        c.index = n->moduleCount;
        c.type = (uint8_t)sum;
        c.data[0] = (uint8_t)(sum >> 8);
    }

    canFrame_t f;
    f.id = CAN_ID_CONFIG + n->node;
    f.extended = false;
    f.len = sizeof(c);
    memcpy(f.data, &c, sizeof(c));
    this->can->send(f);
    n->cfgSentAt = now;                 // a frame that could not be queued is resent after the timeout
}


void CanHub::serviceConfig(canNodeEntry_t* n, uint32_t now)
{
    if (n->cfgState == CFG_STATE_NEEDED)
    {
        // after a transfer that timed out, wait before starting over
        if (n->cfgRetries > CAN_CONFIG_RETRIES && (now - n->cfgSentAt) < CAN_CONFIG_RESTART_MS) return;

        n->cfgState = CFG_STATE_SENDING;
        n->cfgStep = 0;
        n->cfgRetries = 0;
        this->sendConfigStep(n, now);
    }
    else if (n->cfgState == CFG_STATE_SENDING && (now - n->cfgSentAt) >= CAN_CONFIG_REPLY_MS)
    {
        if (++n->cfgRetries > CAN_CONFIG_RETRIES)
        {
            n->cfgState = CFG_STATE_NEEDED;
            n->cfgReport = CFG_REPORT_TIMEOUT;
            return;
        }
        this->sendConfigStep(n, now);
    }
}


void CanHub::handleConfigReply(const canFrame_t& frame, uint32_t now)
{
    canNodeEntry_t* n = this->findNode((uint8_t)(frame.id - CAN_ID_CONFIG_REPLY));
    if (!n || n->cfgState != CFG_STATE_SENDING || frame.len < sizeof(canConfigReply_t)) return;

    canConfigReply_t r;
    memcpy(&r, frame.data, sizeof(r));

    // the reply to the frame in flight, anything else is a late duplicate
    uint8_t cmd, index;
    if (n->cfgStep == 0) { cmd = CFG_BEGIN; index = n->moduleCount; }
    else if (n->cfgStep <= n->moduleCount) { cmd = CFG_MODULE; index = (uint8_t)(n->cfgStep - 1); }
    else { cmd = CFG_END; index = n->moduleCount; }
    if (r.cmd != cmd || r.index != index) return;

    if (r.status != CFG_OK)
    {
        n->cfgState = CFG_STATE_FAILED;
        n->cfgErrIndex = n->cfgStep;
        n->cfgErrStatus = r.status;
        n->cfgErrDetail = r.detail;
        n->cfgReport = CFG_REPORT_REJECTED;
        return;
    }

    if (cmd == CFG_END)
    {
        n->cfgState = CFG_STATE_DONE;
        n->commandSent = false;
        n->cfgReport = CFG_REPORT_DONE;
        n->inValues = n->inLost;
        n->lastFlags = 0xFF;            // send the first COMMAND right away
        // the node counts from 0 again, the position LinuxCNC sees carries on from where it was
        for (int i = 0; i < CAN_NODE_JOINTS; i++) n->jointRaw[i] = 0;
        n->jointPending = true;
        return;
    }

    n->cfgStep++;
    n->cfgRetries = 0;
    this->sendConfigStep(n, now);
}


/***********************************************************************
    Process data (phase 3)
************************************************************************/

void CanHub::sendCommand(canNodeEntry_t* n, uint16_t outputs, bool safe, uint32_t now)
{
    canCommand_t cmd;
    cmd.outputs = 0;
    for (int i = 0; i < n->outCount; i++)
    {
        if (outputs & (1u << n->outBit[i])) cmd.outputs |= (uint16_t)(1u << i);
    }
    cmd.flags = safe ? CMD_FLAG_SAFE : 0;

    // on a change, and regularly: the node goes to its safe state without COMMAND
    if (cmd.outputs == n->lastOutputs && cmd.flags == n->lastFlags && (now - n->lastCommand) < CAN_PROCESS_MS) return;

    canFrame_t f;
    f.id = CAN_ID_COMMAND + n->node;
    f.extended = false;
    f.len = sizeof(cmd);
    memcpy(f.data, &cmd, sizeof(cmd));
    if (this->can->send(f))
    {
        // output changes only, and not while LinuxCNC runs: a line costs ~8 ms of main loop,
        // the SPI frames get lost and the link drops (the node prints every change itself)
        if (!this->quiet && (cmd.outputs != n->lastOutputs || !n->commandSent))
        {
            printf("CAN: node %d COMMAND outputs 0x%04x (LinuxCNC outputs 0x%04x), %s\n", n->node, cmd.outputs,
                   outputs, safe ? "safe (LinuxCNC not connected)" : "LinuxCNC connected");
        }
        n->commandSent = true;
        n->lastOutputs = cmd.outputs;
        n->lastFlags = cmd.flags;
        n->lastCommand = now;
    }
}


void CanHub::handleStatus(const canFrame_t& frame)
{
    canNodeEntry_t* n = this->findNode((uint8_t)(frame.id - CAN_ID_STATUS));
    if (!n || n->cfgState != CFG_STATE_DONE || frame.len < sizeof(canStatus_t)) return;

    canStatus_t s;
    memcpy(&s, frame.data, sizeof(s));

    uint16_t values = 0;
    for (int i = 0; i < n->inCount; i++)
    {
        if (s.inputs & (1u << i)) values |= (uint16_t)(1u << n->inBit[i]);
    }
    n->inValues = values;
}


/***********************************************************************
    Joints: JOINT after every LinuxCNC WRITE, JOINT FEEDBACK back
************************************************************************/

bool CanHub::sendJoint(canNodeEntry_t* n, const volatile int32_t* freq, uint8_t enable, bool safe)
{
    canFrame_t f;
    f.id = CAN_ID_JOINT + n->node;
    f.extended = false;
    f.len = (uint8_t)(2 + 3 * n->jointCount);
    f.data[0] = 0;
    f.data[1] = (uint8_t)(n->jointSeq + 1);
    for (int i = 0; i < n->jointCount; i++)
    {
        int joint = n->jointNum[i];
        int32_t v = safe ? 0 : freq[joint];
        if (v > CAN_FREQ_MAX) v = CAN_FREQ_MAX;
        else if (v < -CAN_FREQ_MAX) v = -CAN_FREQ_MAX;
        if (!safe && ((enable >> joint) & 1)) f.data[0] |= (uint8_t)(1u << i);
        canPutInt24(&f.data[2 + 3 * i], v);
    }
    if (!this->can->send(f)) return false;
    n->jointSeq++;
    return true;
}


void CanHub::handleJointFeedback(const canFrame_t& frame)
{
    canNodeEntry_t* n = this->findNode((uint8_t)(frame.id - CAN_ID_JOINT_FEEDBACK));
    if (!n || n->cfgState != CFG_STATE_DONE || frame.len < 2 + 3 * n->jointCount) return;

    for (int i = 0; i < n->jointCount; i++)
    {
        uint32_t raw = canGetUint24(&frame.data[2 + 3 * i]);
        n->jointPos[i] += canDelta24(raw, n->jointRaw[i]);
        n->jointRaw[i] = raw;
    }
    n->jointFeedbacks++;
}


void CanHub::serviceJoints(uint32_t now, bool fresh, bool safe, const volatile int32_t* freq, uint8_t enable,
                           volatile int32_t* feedback)
{
    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        if (!n->jointCount) continue;

        // the last position stays while the node is lost: LinuxCNC sees a following error
        for (int k = 0; k < n->jointCount; k++) feedback[n->jointNum[k]] = n->jointPos[k];

        if (!n->alive || n->cfgState != CFG_STATE_DONE) continue;

        // right after a WRITE frame; without LinuxCNC regularly, the node stops without JOINT
        if (fresh) n->jointPending = true;
        if (!n->jointPending && (now - n->lastJoint) < CAN_JOINT_REFRESH_MS) continue;

        if (this->sendJoint(n, freq, enable, safe))
        {
            n->jointPending = false;
            n->lastJoint = now;
        }
    }
}


uint8_t CanHub::jointMask()
{
    return this->usedJoints;
}


uint16_t CanHub::inputMask()
{
    uint16_t mask = 0;
    for (int i = 0; i < this->nodeCount; i++) mask |= this->nodes[i].inMask;
    if (this->statusBit >= 0) mask |= (uint16_t)(1u << this->statusBit);
    return mask;
}


uint16_t CanHub::inputValues()
{
    uint16_t values = 0;
    for (int i = 0; i < this->nodeCount; i++)
    {
        const canNodeEntry_t* n = &this->nodes[i];
        values |= (n->alive && n->cfgState == CFG_STATE_DONE) ? n->inValues : n->inLost;
    }
    if (this->statusBit >= 0 && this->networkOk()) values |= (uint16_t)(1u << this->statusBit);
    return values;
}


uint16_t CanHub::runningMask()
{
    uint16_t mask = 0;
    for (int i = 0; i < this->nodeCount; i++)
    {
        const canNodeEntry_t* n = &this->nodes[i];
        if (n->alive && n->cfgState == CFG_STATE_DONE) mask |= (uint16_t)(1u << (n->node - 1));
    }
    return mask;
}


void CanHub::service(uint32_t now, bool quiet, bool safe, uint16_t outputs)
{
    this->quiet = quiet;
    if (!this->started)
    {
        this->started = true;
        this->bootStart = now;
    }
    this->can->service();

    canFrame_t f;
    int budget = 16;                                    // bounded work per main loop pass
    while (budget-- && this->can->receive(f))
    {
        if (f.extended)
        {
            if ((f.id & CAN_EXTID_LSS_MASK) == CAN_EXTID_LSS_REPLY) this->handleLssReply(f, now);
        }
        else if (f.id > CAN_ID_JOINT_FEEDBACK && f.id <= CAN_ID_JOINT_FEEDBACK + CAN_NODE_MAX)
        {
            this->handleJointFeedback(f);
        }
        else if (f.id > CAN_ID_STATUS && f.id <= CAN_ID_STATUS + CAN_NODE_MAX)
        {
            this->handleStatus(f);
        }
        else if (f.id > CAN_ID_CONFIG_REPLY && f.id <= CAN_ID_CONFIG_REPLY + CAN_NODE_MAX)
        {
            this->handleConfigReply(f, now);
        }
        else if (f.id > CAN_ID_HEARTBEAT && f.id <= CAN_ID_HEARTBEAT + CAN_NODE_MAX)
        {
            this->handleHeartbeat(f, now);
        }
    }

    // heartbeat timeouts, configuration, process data
    bool missing = false;
    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        if (n->alive && (now - n->lastSeen) > CAN_NODE_TIMEOUT_MS)
        {
            n->alive = false;
            n->reported = true;
            n->cfgState = CFG_STATE_NEEDED;             // configure again when it is back
            n->cfgRetries = 0;
        }
        if (!n->alive)
        {
            missing = true;
            continue;
        }

        this->serviceConfig(n, now);
        if (n->cfgState == CFG_STATE_DONE) this->sendCommand(n, outputs, safe, now);
    }

    // fast while a configured node is missing; slow otherwise to find nodes added later
    // (unknown ones), but not while LinuxCNC runs: the bus is kept for process data then
    uint32_t identifyPeriod = (missing || this->nodeCount == 0) ? CAN_IDENTIFY_MS : CAN_IDLE_IDENTIFY_MS;
    if ((!quiet || missing) && (now - this->lastIdentify) >= identifyPeriod)
    {
        this->lastIdentify = now;
        this->sendIdentify();
    }

    for (int i = 0; i < CAN_UNKNOWN_MAX; i++)
    {
        canUnknownEntry_t* u = &this->unknown[i];
        if (u->used && (now - u->lastSeen) > CAN_UNKNOWN_FORGET_MS) u->used = false;
    }

    // once, after the nodes had time to answer; printed even if LinuxCNC already runs (hardly
    // this early): the user relies on it, and it costs the link one reconnect at most
    if (!this->bootReported && (now - this->bootStart) >= CAN_BOOT_REPORT_MS)
    {
        this->bootReported = true;
        this->bootReport();
    }

    if (!quiet && this->bootReported) this->report();
}


// the pin a rejected module is about: the node reports it (a stepgen has three)
static uint8_t errorPin(const canNodeEntry_t* n)
{
    switch (n->cfgErrStatus)
    {
        case CFG_ERR_PIN: case CFG_ERR_RESERVED: case CFG_ERR_PIN_USED: return n->cfgErrDetail;
    }
    return n->modules[n->cfgErrIndex - 1][1];
}


void CanHub::bootReport()
{
    int online = 0;
    for (int i = 0; i < this->nodeCount; i++) if (this->nodes[i].alive) online++;

    printf("\nCAN network at boot: %d of %d configured node(s) online\n", online, this->nodeCount);

    char name[6];
    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        printf("  node %-2d ", n->node);
        printUuid(n->uuid);
        printf("  %-10s %-9s ", n->comment, n->mandatory ? "mandatory" : "optional");

        if (!n->alive) printf("MISSING\n");
        else if (n->cfgState == CFG_STATE_DONE)
        {
            printf("online, configured, %d module(s)", n->moduleCount);
            for (int k = 0; k < n->jointCount; k++) printf("%s%d", k ? ", " : ", joint(s) ", n->jointNum[k]);
            printf("\n");
        }
        else if (n->cfgState == CFG_STATE_FAILED)
        {
            printf("online, configuration REJECTED");
            if (n->cfgErrIndex >= 1 && n->cfgErrIndex <= n->moduleCount)
            {
                printf(", module %d (%s)", n->cfgErrIndex, canPinName(errorPin(n), name));
            }
            printf(": %s\n", canConfigError(n->cfgErrStatus));
        }
        else printf("online, configuring\n");

        // summarised here, later changes are printed by report()
        n->reported = false;
        n->cfgReport = CFG_REPORT_NONE;
    }

    for (int i = 0; i < CAN_UNKNOWN_MAX; i++)
    {
        canUnknownEntry_t* u = &this->unknown[i];
        if (!u->used) continue;
        u->printed = true;
        printf("  unknown ");
        printUuid(u->uuid);
        printf("  not in config.txt, add it to \"CAN\" \"Nodes\"\n");
    }
    this->unknownNew = false;

    if (this->statusBit >= 0)
    {
        printf("  status bit remora.input.%02d = %s\n", this->statusBit,
               this->networkOk() ? "1, all mandatory nodes running" : "0, mandatory node(s) missing: LinuxCNC stays in E-stop");
    }
    else if (this->nodeCount)
    {
        printf("  no \"Status Bit\" in \"CAN\": LinuxCNC does NOT see a missing or lost node\n");
    }
    this->can->printStatus();
    printf("\n");
}


void CanHub::report()
{
    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        if (!n->reported) continue;
        n->reported = false;

        printf("CAN: node %d (%s) %s", n->node, n->comment, n->alive ? "online" : "LOST");
        if (n->alive) printf(", state %d, errors %d", n->state, n->errors);
        else if (n->mandatory && this->statusBit >= 0) printf(", mandatory: status bit 0, E-stop");
        printf("\n");
    }

    char name[6];
    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        uint8_t what = n->cfgReport;
        n->cfgReport = CFG_REPORT_NONE;

        switch (what)
        {
            case CFG_REPORT_DONE:
                printf("CAN: node %d (%s) configured, %d module(s), running\n", n->node, n->comment, n->moduleCount);
                break;

            case CFG_REPORT_REJECTED:
                printf("CAN: node %d (%s) rejected its configuration", n->node, n->comment);
                if (n->cfgErrIndex >= 1 && n->cfgErrIndex <= n->moduleCount)
                {
                    printf(", module %d (%s)", n->cfgErrIndex, canPinName(errorPin(n), name));
                }
                printf(": %s\n", canConfigError(n->cfgErrStatus));
                break;

            case CFG_REPORT_TIMEOUT:
                printf("CAN: node %d (%s) does not answer the configuration, retrying\n", n->node, n->comment);
                break;
        }
    }

    // nodes on the bus without an entry in config.txt, once each (the boot summary lists them all)
    if (this->unknownNew)
    {
        this->unknownNew = false;

        for (int i = 0; i < CAN_UNKNOWN_MAX; i++)
        {
            if (!this->unknown[i].used || this->unknown[i].printed) continue;
            this->unknown[i].printed = true;
            printf("CAN: unknown node UUID ");
            printUuid(this->unknown[i].uuid);
            printf(" on the bus, add it to \"CAN\" \"Nodes\" in config.txt\n");
        }
    }
}


uint16_t CanHub::aliveMask()
{
    uint16_t mask = 0;
    for (int i = 0; i < this->nodeCount; i++)
    {
        if (this->nodes[i].alive) mask |= (uint16_t)(1u << (this->nodes[i].node - 1));
    }
    return mask;
}
