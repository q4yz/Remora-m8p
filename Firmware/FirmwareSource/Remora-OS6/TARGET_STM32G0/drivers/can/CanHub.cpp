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
    lastStatus(0),
    lastUnknownPrint(0),
    unknownNew(false),
    usedOutputs(0),
    usedInputs(0)
{
    memset(this->nodes, 0, sizeof(this->nodes));
    memset(this->unknown, 0, sizeof(this->unknown));
}


bool CanHub::addNode(const char* uuidHex, int node, const char* comment)
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
    strncpy(n->comment, comment ? comment : "", sizeof(n->comment) - 1);
    this->nodeCount++;

    printf("CAN: node %d = %s (%s)\n", node, uuidHex, n->comment);
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
    for (int i = 0; i < n->moduleCount; i++)
    {
        if (n->modules[i][0] == MODULE_DIGITAL_PIN && n->modules[i][1] == pin)
        {
            printf("CAN: node %d: pin %s used twice\n", node, canPinName(pin, name));
            return false;
        }
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
        if (cmd.outputs != n->lastOutputs || cmd.flags != n->lastFlags || !n->commandSent)
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


uint16_t CanHub::inputMask()
{
    uint16_t mask = 0;
    for (int i = 0; i < this->nodeCount; i++) mask |= this->nodes[i].inMask;
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
    this->can->service();

    canFrame_t f;
    int budget = 16;                                    // bounded work per main loop pass
    while (budget-- && this->can->receive(f))
    {
        if (f.extended)
        {
            if ((f.id & CAN_EXTID_LSS_MASK) == CAN_EXTID_LSS_REPLY) this->handleLssReply(f, now);
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

    if (!quiet)
    {
        this->report(now);

        // bus diagnosis while a node is missing: no ACK = wiring / termination / node not running
        if ((missing || this->nodeCount == 0) && (now - this->lastStatus) >= 2000)
        {
            this->lastStatus = now;
            this->can->printStatus();
        }
    }
}


void CanHub::report(uint32_t now)
{
    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        if (!n->reported) continue;
        n->reported = false;

        printf("CAN: node %d (%s) %s", n->node, n->comment, n->alive ? "online" : "LOST");
        if (n->alive) printf(", state %d, errors %d", n->state, n->errors);
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
                    printf(", module %d (%s)", n->cfgErrIndex, canPinName(n->modules[n->cfgErrIndex - 1][1], name));
                }
                printf(": %s\n", canConfigError(n->cfgErrStatus));
                break;

            case CFG_REPORT_TIMEOUT:
                printf("CAN: node %d (%s) does not answer the configuration, retrying\n", n->node, n->comment);
                break;
        }
    }

    // nodes on the bus without an entry in config.txt: right away, then every few seconds
    if (this->unknownNew || (now - this->lastUnknownPrint) >= CAN_UNKNOWN_PRINT_MS)
    {
        this->unknownNew = false;
        this->lastUnknownPrint = now;

        for (int i = 0; i < CAN_UNKNOWN_MAX; i++)
        {
            if (!this->unknown[i].used) continue;
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
