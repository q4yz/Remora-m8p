#include "CanNode.h"
#include "canNodeIo.h"

#include <string.h>

CanNode::CanNode(RemoraCAN* can, uint32_t baseFreq) :
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
    commandCount(0),
    jointCount(0),
    lastJoint(0),
    jointsStopped(false),
    jointFrames(0),
    tmcCount(0),
    analogCount(0),
    analogSamples(0),
    lastAnalogSample(0),
    lastAnalog(0)
{
    memset(this->joints, 0, sizeof(this->joints));
    // as the Remora stepgen: no floating point in the interrupt (the Cortex-M0+ has no FPU)
    this->frequencyScale = ((int64_t)1 << (NODE_STEPBIT + 32)) / baseFreq;

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

// the pins a module uses
static int modulePins(const canNodeModule_t& m, uint8_t pins[3])
{
    pins[0] = m.pin;
    if (m.type != MODULE_STEPGEN) return 1;
    pins[1] = m.dirPin;
    if (m.enablePin == CAN_PIN_NONE) return 2;
    pins[2] = m.enablePin;
    return 3;
}


// on an error m.pin is the pin concerned (CONFIG REPLY detail)
uint8_t CanNode::checkModule(const canFrame_t& frame, canNodeModule_t& m)
{
    m.type = frame.data[2];
    m.pin = frame.data[3];
    m.flags = frame.data[4];
    m.bit = frame.data[5];
    m.dirPin = frame.data[6];
    m.enablePin = frame.data[7];

    if (m.type == MODULE_DIGITAL_PIN)
    {
        if (m.bit > 15) return CFG_ERR_BIT;
    }
    else if (m.type == MODULE_STEPGEN)
    {
        if (m.bit >= CAN_NODE_JOINTS) return CFG_ERR_JOINT;
    }
    else if (m.type == MODULE_TMC2209)
    {
        if ((m.flags >> TMC_MSTEP_SHIFT) > 8 || m.bit == 0 || m.bit > TMC_CURRENT_MAX / 10 || m.dirPin == 0)
        {
            return CFG_ERR_VALUE;
        }
        int drivers = 0;
        for (int i = 0; i < this->pendingCount; i++) if (this->pending[i].type == MODULE_TMC2209) drivers++;
        if (drivers >= CAN_NODE_TMC) return CFG_ERR_FULL;
    }
    else if (m.type == MODULE_ANALOG_IN)
    {
        if (m.bit >= CAN_NODE_ANALOG) return CFG_ERR_VALUE;
        if (canIoPinExists(m.pin) && !canIoPinReserved(m.pin) && !canIoAnalogCapable(m.pin)) return CFG_ERR_NOT_ANALOG;
    }
    else return CFG_ERR_TYPE;

    uint8_t pins[3], used[3];
    int count = modulePins(m, pins);
    for (int k = 0; k < count; k++)
    {
        uint8_t pin = pins[k];
        if (!canIoPinExists(pin)) { m.pin = pin; return CFG_ERR_PIN; }
        if (canIoPinReserved(pin)) { m.pin = pin; return CFG_ERR_RESERVED; }
        for (int l = 0; l < k; l++) if (pins[l] == pin) { m.pin = pin; return CFG_ERR_PIN_USED; }
    }

    for (int i = 0; i < this->pendingCount; i++)
    {
        const canNodeModule_t& p = this->pending[i];
        int n = modulePins(p, used);
        for (int k = 0; k < count; k++)
        {
            for (int l = 0; l < n; l++) if (used[l] == pins[k]) { m.pin = pins[k]; return CFG_ERR_PIN_USED; }
        }
        if (p.type != m.type || p.bit != m.bit || m.type == MODULE_TMC2209) continue;
        if (m.type == MODULE_STEPGEN) return CFG_ERR_JOINT;
        if (m.type == MODULE_ANALOG_IN) return CFG_ERR_VALUE;
        if ((p.flags & DPIN_OUTPUT) == (m.flags & DPIN_OUTPUT)) return CFG_ERR_BIT;
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
            if (m.type == MODULE_STEPGEN)
            {
                char dir[6], enable[6];
                printf("CONFIG: module %d: stepgen joint %d, step %s, direction %s, enable %s\n", c.index + 1, m.bit,
                       canPinName(m.pin, name), canPinName(m.dirPin, dir),
                       m.enablePin == CAN_PIN_NONE ? "none" : canPinName(m.enablePin, enable));
            }
            else if (m.type == MODULE_ANALOG_IN)
            {
                printf("CONFIG: module %d: analog input %d, %s\n", c.index + 1, m.bit, canPinName(m.pin, name));
            }
            else if (m.type == MODULE_TMC2209)
            {
                printf("CONFIG: module %d: TMC2209 UART %s, address %d, %d mA, %d microsteps, %s, stall %d\n",
                       c.index + 1, canPinName(m.pin, name), m.flags & TMC_ADDR_MASK, m.bit * 10,
                       1 << (m.flags >> TMC_MSTEP_SHIFT), (m.flags & TMC_STEALTH) ? "StealthChop" : "SpreadCycle",
                       m.enablePin);
            }
            else
            {
                printf("CONFIG: module %d: digital %s %s, bit %d\n", c.index + 1,
                       (m.flags & DPIN_OUTPUT) ? "output" : "input", canPinName(m.pin, name), m.bit);
            }
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

    // outputs start in their safe state until the first COMMAND, drivers disabled until the first JOINT
    this->safe = true;
    this->outputs = 0;
    this->jointCount = 0;
    this->analogCount = 0;
    for (int i = 0; i < this->moduleCount; i++)
    {
        const canNodeModule_t& m = this->modules[i];
        if (m.type == MODULE_ANALOG_IN)
        {
            canIoAnalogConfigure(m.pin);
            this->analogPin[m.bit] = m.pin;
            this->analogValue[m.bit] = 0;
            if (m.bit + 1 > this->analogCount) this->analogCount = (uint8_t)(m.bit + 1);
            continue;
        }
        if (m.type == MODULE_STEPGEN)
        {
            canIoConfigure(m.pin, true, DPIN_MOD_NONE, false);
            canIoConfigure(m.dirPin, true, DPIN_MOD_NONE, false);
            if (m.enablePin != CAN_PIN_NONE) canIoConfigure(m.enablePin, true, DPIN_MOD_NONE, true);

            canNodeJoint_t& j = this->joints[m.bit];
            j.stepPin = m.pin;
            j.dirPin = m.dirPin;
            j.enablePin = m.enablePin;
            j.enabled = false;
            j.addValue = 0;
            j.accumulator = 0;
            j.count = 0;
            j.stepHigh = false;
            j.forward = false;
            j.active = true;                    // last: the interrupt starts using it now
            if (m.bit + 1 > this->jointCount) this->jointCount = (uint8_t)(m.bit + 1);
            continue;
        }
        if (m.type == MODULE_TMC2209)
        {
            // UART line idle high, the pull up holds it while the driver answers
            canIoConfigure(m.pin, true, DPIN_MOD_PULLUP, true);
            int k = this->tmcCount;
            this->tmc[k].setup(m.pin, m.flags & TMC_ADDR_MASK, m.dirPin, (uint16_t)(m.bit * 10),
                               (uint16_t)(1u << (m.flags >> TMC_MSTEP_SHIFT)), (m.flags & TMC_STEALTH) != 0, m.enablePin);
            this->tmc[k].start();
            this->tmcModule[k] = (uint8_t)i;
            this->lastEmcy[k] = this->now;
            this->tmcCount = (uint8_t)(k + 1);      // last: the interrupt ticks it from now on
            continue;
        }
        bool output = (m.flags & DPIN_OUTPUT) != 0;
        canIoConfigure(m.pin, output, (m.flags & DPIN_MOD_MASK) >> DPIN_MOD_SHIFT, (m.flags & DPIN_SAFE_HIGH) != 0);
    }

    this->state = NODE_STATE_RUNNING;
    this->lastCommand = this->now;
    this->lastJoint = this->now;
    this->jointsStopped = false;
    this->commandCount = 0;
    this->jointFrames = 0;
    memset(this->analogSum, 0, sizeof(this->analogSum));
    this->analogSamples = 0;
    this->lastAnalogSample = this->now;
    this->lastAnalog = this->now;
    this->sendStatus(this->readInputs());
}


void CanNode::releaseModules()
{
    // outputs to their safe level and drivers disabled first, then all pins back to high impedance
    this->safe = true;
    this->writeOutputs();
    this->stopJoints();
    for (int i = 0; i < CAN_NODE_JOINTS; i++) this->joints[i].active = false;
    this->jointCount = 0;
    for (int i = 0; i < this->tmcCount; i++) this->tmc[i].stop();
    this->tmcCount = 0;
    this->analogCount = 0;

    uint8_t pins[3];
    for (int i = 0; i < this->moduleCount; i++)
    {
        int n = modulePins(this->modules[i], pins);
        for (int k = 0; k < n; k++) canIoRelease(pins[k]);
    }
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
        if (m.type != MODULE_DIGITAL_PIN || !(m.flags & DPIN_OUTPUT)) continue;

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
        if (m.type != MODULE_DIGITAL_PIN || (m.flags & DPIN_OUTPUT)) continue;
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


/***********************************************************************
    Joints: step generators, same DDS as the Remora stepgen module
************************************************************************/

void CanNode::setJointEnable(canNodeJoint_t& j, bool enabled)
{
    if (enabled == j.enabled) return;
    j.enabled = enabled;
    if (j.enablePin != CAN_PIN_NONE) canIoWrite(j.enablePin, !enabled);        // active low
}


void CanNode::stopJoints()
{
    for (int i = 0; i < CAN_NODE_JOINTS; i++)
    {
        canNodeJoint_t& j = this->joints[i];
        if (!j.active) continue;
        j.addValue = 0;
        this->setJointEnable(j, false);
    }
}


void CanNode::handleJoint(const canFrame_t& frame)
{
    if (frame.len < 2 || !this->jointCount) return;
    if (this->state != NODE_STATE_RUNNING && this->state != NODE_STATE_SAFE) return;

    uint8_t enable = frame.data[0];
    bool run = this->state == NODE_STATE_RUNNING;       // SAFE: hub gone, drivers stay off

    for (int i = 0; i < this->jointCount; i++)
    {
        canNodeJoint_t& j = this->joints[i];
        if (!j.active) continue;

        bool on = run && ((enable >> i) & 1) && frame.len >= 2 + 3 * (i + 1);
        int32_t freq = on ? canGetInt24(&frame.data[2 + 3 * i]) : 0;

        // scaled here in the main loop, the interrupt only adds
        int64_t add = (int64_t)freq * this->frequencyScale;
        j.addValue = (int32_t)(add >= 0 ? add >> 32 : -((-add) >> 32));       // truncated towards zero
        this->setJointEnable(j, on);
    }

    if (this->jointsStopped)
    {
        this->jointsStopped = false;
        printf("JOINT from the hub again\n");
    }
    this->lastJoint = this->now;
    this->jointFrames++;
    this->sendJointFeedback(frame.data[1]);
}


void CanNode::sendJointFeedback(uint8_t seq)
{
    canFrame_t f;
    f.id = CAN_ID_JOINT_FEEDBACK + this->node;
    f.extended = false;
    f.len = (uint8_t)(2 + 3 * this->jointCount);
    f.data[0] = seq;
    f.data[1] = 0;
    for (int i = 0; i < this->jointCount; i++)
    {
        const canNodeJoint_t& j = this->joints[i];
        if (j.enabled) f.data[1] |= (uint8_t)(1u << i);
        canPutInt24(&f.data[2 + 3 * i], j.count);
    }
    this->can->send(f);                     // a lost one is replaced by the next, every JOINT is answered
}


/***********************************************************************
    TMC2209 drivers
************************************************************************/

bool CanNode::sendTmcStatus(int i)
{
    canFrame_t f;
    f.id = CAN_ID_EMCY + this->node;
    f.extended = false;
    f.len = 4;
    f.data[0] = EMCY_TMC;
    f.data[1] = this->tmcModule[i];
    f.data[2] = this->tmc[i].getStatus();
    f.data[3] = this->tmc[i].getVersion();
    return this->can->send(f);
}


void CanNode::serviceTmc()
{
    char name[6];
    for (int i = 0; i < this->tmcCount; i++)
    {
        NodeTmc2209& t = this->tmc[i];
        bool changed = t.service(this->now);
        if (changed)
        {
            printf("TMC2209 %s: %s", canPinName(t.getPin(), name), tmcStatusText(t.getStatus()));
            if (t.getStatus() == TMC_STATUS_OK) printf(", version 0x%02x", t.getVersion());
            printf("\n");
        }
        // on a change, and regularly: the hub may have missed it
        if ((changed || (this->now - this->lastEmcy[i]) >= CAN_EMCY_REPEAT_MS) && this->sendTmcStatus(i))
        {
            this->lastEmcy[i] = this->now;
        }
    }
}


/***********************************************************************
    Analog inputs
************************************************************************/

void CanNode::serviceAnalog()
{
    if (!this->analogCount) return;

    if ((this->now - this->lastAnalogSample) >= CAN_ANALOG_SAMPLE_MS)
    {
        this->lastAnalogSample = this->now;
        for (int i = 0; i < this->analogCount; i++) this->analogSum[i] += canIoAnalogRead(this->analogPin[i]);
        this->analogSamples++;
    }

    if ((this->now - this->lastAnalog) < CAN_ANALOG_MS || !this->analogSamples) return;

    canFrame_t f;
    f.id = CAN_ID_ANALOG + this->node;
    f.extended = false;
    f.len = (uint8_t)(2 * this->analogCount);
    for (int i = 0; i < this->analogCount; i++)
    {
        uint16_t v = (uint16_t)(this->analogSum[i] / this->analogSamples);
        f.data[2 * i] = (uint8_t)v;
        f.data[2 * i + 1] = (uint8_t)(v >> 8);
    }
    // TX queue full: keep averaging, try again in the next ms
    if (!this->can->send(f)) return;

    for (int i = 0; i < this->analogCount; i++)
    {
        this->analogValue[i] = (uint16_t)(f.data[2 * i] | (f.data[2 * i + 1] << 8));
        this->analogSum[i] = 0;
    }
    this->analogSamples = 0;
    this->lastAnalog = this->now;
}


void CanNode::baseTick()
{
    for (int i = 0; i < this->tmcCount; i++) this->tmc[i].tick();

    for (int i = 0; i < CAN_NODE_JOINTS; i++)
    {
        canNodeJoint_t& j = this->joints[i];
        if (!j.active) continue;

        // the step pulse lasts one base period
        if (j.stepHigh)
        {
            canIoWrite(j.stepPin, false);
            j.stepHigh = false;
        }

        int32_t add = j.addValue;
        if (add == 0) continue;

        uint32_t before = j.accumulator;
        j.accumulator += (uint32_t)add;
        if (!((before ^ j.accumulator) & (1u << NODE_STEPBIT))) continue;

        bool forward = add > 0;
        if (forward != j.forward)
        {
            canIoWrite(j.dirPin, forward);
            j.forward = forward;
        }
        canIoWrite(j.stepPin, true);
        j.stepHigh = true;
        j.count = forward ? j.count + 1 : j.count - 1;
    }
}


void CanNode::printOutputs(const char* why)
{
    printf("%s: outputs 0x%04x, %s", why, this->outputs,
           this->safe ? "safe (LinuxCNC not connected)" : "LinuxCNC connected");
    char name[6];
    for (int i = 0; i < this->moduleCount; i++)
    {
        const canNodeModule_t& m = this->modules[i];
        if (m.type != MODULE_DIGITAL_PIN || !(m.flags & DPIN_OUTPUT)) continue;
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
        else if (this->node && f.id == (uint32_t)(CAN_ID_JOINT + this->node)) this->handleJoint(f);
    }

    if (this->state == NODE_STATE_RUNNING || this->state == NODE_STATE_SAFE)
    {
        // hub gone: outputs to their safe state until COMMAND comes back
        if (this->state == NODE_STATE_RUNNING && (now - this->lastCommand) > CAN_COMMAND_TIMEOUT_MS)
        {
            this->state = NODE_STATE_SAFE;
            this->safe = true;
            this->writeOutputs();
            this->stopJoints();
            printf("no COMMAND from the hub, outputs in their safe state\n");
            this->printOutputs("SAFE");
        }

        // JOINT comes every servo period: without it the motion stops right away, drivers disabled
        if (this->jointCount && !this->jointsStopped && (now - this->lastJoint) > CAN_JOINT_TIMEOUT_MS)
        {
            this->jointsStopped = true;
            this->stopJoints();
            printf("no JOINT from the hub, joints stopped, drivers disabled\n");
        }

        this->serviceTmc();
        this->serviceAnalog();

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
