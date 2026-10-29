#include "CanNode.h"
#include "canNodeIo.h"

#include <string.h>

CanNode::CanNode(RemoraCAN* can) :
    can(can),
    node(0),
    state(NODE_STATE_UNASSIGNED),
    now(0),
    lastHeartbeat(0),
    moduleCount(0),
    pendingCount(0),
    expectedCount(0),
    checksum(0),
    receiving(false),
    outputs(0),
    safe(true),
    inputs(0),
    lastCommand(0),
    lastStatus(0),
    lastStatusTry(0),
    commandCount(0)
{
    // 96-bit factory unique ID of the STM32 -> 48-bit UUID
    uint32_t w[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
    uint8_t uid[12];
    memcpy(uid, w, sizeof(uid));
    canUuidFromUid(uid, this->uuid);
}


bool CanNode::sendLssReply()
{
    canLssReply_t reply;
    memcpy(reply.uuid, this->uuid, 6);
    reply.state = this->state;
    reply.node = this->node;

    canFrame_t f;
    f.id = CAN_EXTID_LSS_REPLY | canUuidLow24(this->uuid);     // unique: replies arbitrate cleanly
    f.extended = true;
    f.len = sizeof(reply);
    memcpy(f.data, &reply, sizeof(reply));
    return this->can->send(f);
}


void CanNode::sendHeartbeat()
{
    canHeartbeat_t hb;
    hb.state = this->state;
    hb.protocol = CAN_PROTOCOL_VERSION;
    hb.errors = this->can->getErrorCount();
    hb.uptime = this->now;

    canFrame_t f;
    f.id = CAN_ID_HEARTBEAT + this->node;
    f.extended = false;
    f.len = sizeof(hb);
    memcpy(f.data, &hb, sizeof(hb));
    this->can->send(f);
}


void CanNode::sendConfigReply(uint8_t cmd, uint8_t index, uint8_t status, uint8_t detail)
{
    canConfigReply_t reply = { cmd, index, status, detail };

    canFrame_t f;
    f.id = CAN_ID_CONFIG_REPLY + this->node;
    f.extended = false;
    f.len = sizeof(reply);
    memcpy(f.data, &reply, sizeof(reply));
    this->can->send(f);
}


void CanNode::sendStatus(uint16_t in)
{
    canStatus_t status = { in };

    canFrame_t f;
    f.id = CAN_ID_STATUS + this->node;
    f.extended = false;
    f.len = sizeof(status);
    memcpy(f.data, &status, sizeof(status));
    if (this->can->send(f))
    {
        this->inputs = in;
        this->lastStatus = this->now;
    }
}


void CanNode::handleLss(const canFrame_t& frame)
{
    if (frame.len < 1) return;

    switch (frame.data[0])
    {
        case LSS_IDENTIFY:
        {
            bool sent = this->sendLssReply();
            printf("LSS: IDENTIFY from the hub, reply %s\n", sent ? "queued" : "NOT sent (TX queue full)");
            break;
        }

        case LSS_ASSIGN:
            if (frame.len == 8 && !memcmp(&frame.data[1], this->uuid, 6))
            {
                uint8_t n = frame.data[7];
                if (n > CAN_NODE_MAX) n = 0;

                // a new number: the configuration belonged to the old one
                if (n != this->node || this->state == NODE_STATE_UNASSIGNED)
                {
                    this->releaseModules();
                    this->node = n;
                    this->state = n ? NODE_STATE_PREOP : NODE_STATE_UNASSIGNED;
                }
                this->sendLssReply();                           // confirm
                printf("LSS: ASSIGN, node number %d\n", this->node);
            }
            break;
    }
}


/***********************************************************************
    Configuration transfer (phase 2)
************************************************************************/

uint8_t CanNode::checkModule(const canFrame_t& frame, canNodeModule_t& m)
{
    m.type = frame.data[2];
    m.pin = frame.data[3];
    m.flags = frame.data[4];
    m.bit = frame.data[5];

    if (m.type != MODULE_DIGITAL_PIN) return CFG_ERR_TYPE;
    if (!canIoPinExists(m.pin)) return CFG_ERR_PIN;
    if (canIoPinReserved(m.pin)) return CFG_ERR_RESERVED;
    if (m.bit > 15) return CFG_ERR_BIT;

    for (int i = 0; i < this->pendingCount; i++)
    {
        const canNodeModule_t& p = this->pending[i];
        if (p.pin == m.pin) return CFG_ERR_PIN_USED;
        if ((p.flags & DPIN_OUTPUT) == (m.flags & DPIN_OUTPUT) && p.bit == m.bit) return CFG_ERR_BIT;
    }
    return CFG_OK;
}


void CanNode::handleConfig(const canFrame_t& frame)
{
    if (frame.len != 8) return;

    canConfigFrame_t c;
    memcpy(&c, frame.data, sizeof(c));
    char name[6];

    switch (c.cmd)
    {
        case CFG_BEGIN:
            if (c.index > CAN_MODULES_MAX)
            {
                this->receiving = false;
                this->sendConfigReply(CFG_BEGIN, c.index, CFG_ERR_FULL, CAN_MODULES_MAX);
                return;
            }
            // a new configuration replaces the running one
            if (this->moduleCount || this->state == NODE_STATE_RUNNING || this->state == NODE_STATE_SAFE)
            {
                this->releaseModules();
                this->state = NODE_STATE_PREOP;
            }
            this->pendingCount = 0;
            this->expectedCount = c.index;
            this->checksum = 0;
            this->receiving = true;
            this->sendConfigReply(CFG_BEGIN, c.index, CFG_OK, 0);
            printf("CONFIG: %d module(s) from the hub\n", c.index);
            return;

        case CFG_MODULE:
        {
            if (this->receiving && this->pendingCount && c.index + 1 == this->pendingCount)
            {
                this->sendConfigReply(CFG_MODULE, c.index, CFG_OK, 0);     // repeated, the hub missed the reply
                return;
            }
            if (!this->receiving || c.index != this->pendingCount || this->pendingCount >= this->expectedCount)
            {
                this->receiving = false;
                this->sendConfigReply(CFG_MODULE, c.index, CFG_ERR_SEQUENCE, 0);
                return;
            }

            canNodeModule_t m;
            uint8_t status = this->checkModule(frame, m);
            if (status != CFG_OK)
            {
                this->receiving = false;
                this->sendConfigReply(CFG_MODULE, c.index, status, m.pin);
                printf("CONFIG: module %d (%s) rejected: %s\n", c.index + 1, canPinName(m.pin, name), canConfigError(status));
                return;
            }

            this->pending[this->pendingCount++] = m;
            this->checksum = canConfigChecksum(this->checksum, &frame.data[2]);
            this->sendConfigReply(CFG_MODULE, c.index, CFG_OK, 0);
            printf("CONFIG: module %d: digital %s %s, bit %d\n", c.index + 1,
                   (m.flags & DPIN_OUTPUT) ? "output" : "input", canPinName(m.pin, name), m.bit);
            return;
        }

        case CFG_END:
        {
            uint16_t sum = (uint16_t)(c.type | (c.data[0] << 8));

            if (!this->receiving)
            {
                // repeated END, the hub missed the reply
                bool done = (this->state == NODE_STATE_RUNNING || this->state == NODE_STATE_SAFE) && c.index == this->moduleCount;
                this->sendConfigReply(CFG_END, c.index, done ? CFG_OK : CFG_ERR_SEQUENCE, 0);
                return;
            }
            this->receiving = false;

            if (c.index != this->pendingCount || this->pendingCount != this->expectedCount || sum != this->checksum)
            {
                this->sendConfigReply(CFG_END, c.index, CFG_ERR_CHECKSUM, 0);
                printf("CONFIG: checksum error\n");
                return;
            }

            this->applyConfig();
            this->sendConfigReply(CFG_END, c.index, CFG_OK, 0);
            printf("CONFIG: done, %d module(s), running\n", this->moduleCount);
            this->printOutputs("CONFIG");
            return;
        }
    }
}


void CanNode::applyConfig()
{
    memcpy(this->modules, this->pending, sizeof(this->modules));
    this->moduleCount = this->pendingCount;

    // outputs start in their safe state until the first COMMAND
    this->safe = true;
    this->outputs = 0;
    for (int i = 0; i < this->moduleCount; i++)
    {
        const canNodeModule_t& m = this->modules[i];
        bool output = (m.flags & DPIN_OUTPUT) != 0;
        canIoConfigure(m.pin, output, (m.flags & DPIN_MOD_MASK) >> DPIN_MOD_SHIFT, (m.flags & DPIN_SAFE_HIGH) != 0);
    }

    this->state = NODE_STATE_RUNNING;
    this->lastCommand = this->now;
    this->commandCount = 0;
    this->sendStatus(this->readInputs());
}


void CanNode::releaseModules()
{
    // outputs to their safe level first, then all pins back to high impedance
    this->safe = true;
    this->writeOutputs();
    for (int i = 0; i < this->moduleCount; i++) canIoRelease(this->modules[i].pin);
    this->moduleCount = 0;
}


/***********************************************************************
    Process data (phase 3)
************************************************************************/

void CanNode::writeOutputs()
{
    for (int i = 0; i < this->moduleCount; i++)
    {
        const canNodeModule_t& m = this->modules[i];
        if (!(m.flags & DPIN_OUTPUT)) continue;

        bool level;
        if (this->safe) level = (m.flags & DPIN_SAFE_HIGH) != 0;           // physical level, not inverted
        else level = (((this->outputs >> m.bit) & 1) != 0) != ((m.flags & DPIN_INVERT) != 0);
        canIoWrite(m.pin, level);
    }
}


uint16_t CanNode::readInputs()
{
    uint16_t in = 0;
    for (int i = 0; i < this->moduleCount; i++)
    {
        const canNodeModule_t& m = this->modules[i];
        if (m.flags & DPIN_OUTPUT) continue;
        if (canIoRead(m.pin) != ((m.flags & DPIN_INVERT) != 0)) in |= (uint16_t)(1u << m.bit);
    }
    return in;
}


void CanNode::handleCommand(const canFrame_t& frame)
{
    if (frame.len < sizeof(canCommand_t)) return;
    if (this->state != NODE_STATE_RUNNING && this->state != NODE_STATE_SAFE) return;

    canCommand_t cmd;
    memcpy(&cmd, frame.data, sizeof(cmd));

    bool safe = (cmd.flags & CMD_FLAG_SAFE) != 0;
    bool changed = this->commandCount == 0 || cmd.outputs != this->outputs || safe != this->safe;

    this->lastCommand = this->now;
    this->commandCount++;
    this->outputs = cmd.outputs;
    this->safe = safe;

    if (this->state == NODE_STATE_SAFE)
    {
        this->state = NODE_STATE_RUNNING;
        printf("COMMAND from the hub again, running\n");
    }
    this->writeOutputs();
    if (changed) this->printOutputs("COMMAND");
}


void CanNode::printOutputs(const char* why)
{
    printf("%s: outputs 0x%04x, %s", why, this->outputs,
           this->safe ? "safe (LinuxCNC not connected)" : "LinuxCNC connected");
    char name[6];
    for (int i = 0; i < this->moduleCount; i++)
    {
        const canNodeModule_t& m = this->modules[i];
        if (!(m.flags & DPIN_OUTPUT)) continue;
        printf(", %s %s", canPinName(m.pin, name), canIoRead(m.pin) ? "high" : "low");    // read back from the pin
    }
    printf("\n");
}


void CanNode::service(uint32_t now)
{
    this->now = now;
    this->can->service();

    canFrame_t f;
    int budget = 16;
    while (budget-- && this->can->receive(f))
    {
        if (f.extended) continue;
        if (f.id == CAN_ID_LSS) this->handleLss(f);
        else if (this->node && f.id == (uint32_t)(CAN_ID_CONFIG + this->node)) this->handleConfig(f);
        else if (this->node && f.id == (uint32_t)(CAN_ID_COMMAND + this->node)) this->handleCommand(f);
    }

    if (this->state == NODE_STATE_RUNNING || this->state == NODE_STATE_SAFE)
    {
        // hub gone: outputs to their safe state until COMMAND comes back
        if (this->state == NODE_STATE_RUNNING && (now - this->lastCommand) > CAN_COMMAND_TIMEOUT_MS)
        {
            this->state = NODE_STATE_SAFE;
            this->safe = true;
            this->writeOutputs();
            printf("no COMMAND from the hub, outputs in their safe state\n");
            this->printOutputs("SAFE");
        }

        // TX queue full (nobody acknowledges): try again in the next ms, not on every loop pass
        uint16_t in = this->readInputs();
        if ((in != this->inputs || (now - this->lastStatus) >= CAN_PROCESS_MS) && now != this->lastStatusTry)
        {
            this->lastStatusTry = now;
            this->sendStatus(in);
        }
    }

    if (this->node && (now - this->lastHeartbeat) >= CAN_HEARTBEAT_MS)
    {
        this->lastHeartbeat = now;
        this->sendHeartbeat();
    }
}
