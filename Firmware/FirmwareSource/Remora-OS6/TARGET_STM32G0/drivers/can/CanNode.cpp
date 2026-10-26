#include "CanNode.h"

#include <string.h>

CanNode::CanNode(RemoraCAN* can) :
    can(can),
    node(0),
    state(NODE_STATE_UNASSIGNED),
    lastHeartbeat(0)
{
    // 96-bit factory unique ID of the STM32 -> 48-bit UUID
    uint32_t w[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
    uint8_t uid[12];
    memcpy(uid, w, sizeof(uid));
    canUuidFromUid(uid, this->uuid);
}


void CanNode::sendLssReply()
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
    this->can->send(f);
}


void CanNode::sendHeartbeat(uint32_t now)
{
    canHeartbeat_t hb;
    hb.state = this->state;
    hb.protocol = CAN_PROTOCOL_VERSION;
    hb.errors = this->can->getErrorCount();
    hb.uptime = now;

    canFrame_t f;
    f.id = CAN_ID_HEARTBEAT + this->node;
    f.extended = false;
    f.len = sizeof(hb);
    memcpy(f.data, &hb, sizeof(hb));
    this->can->send(f);
}


void CanNode::handleLss(const canFrame_t& frame)
{
    if (frame.len < 1) return;

    switch (frame.data[0])
    {
        case LSS_IDENTIFY:
            this->sendLssReply();
            break;

        case LSS_ASSIGN:
            if (frame.len == 8 && !memcmp(&frame.data[1], this->uuid, 6))
            {
                uint8_t n = frame.data[7];
                this->node = (n <= CAN_NODE_MAX) ? n : 0;
                this->state = this->node ? NODE_STATE_PREOP : NODE_STATE_UNASSIGNED;
                this->sendLssReply();                           // confirm
            }
            break;
    }
}


void CanNode::service(uint32_t now)
{
    this->can->service();

    canFrame_t f;
    int budget = 16;
    while (budget-- && this->can->receive(f))
    {
        if (!f.extended && f.id == CAN_ID_LSS) this->handleLss(f);
    }

    if (this->node && (now - this->lastHeartbeat) >= CAN_HEARTBEAT_MS)
    {
        this->lastHeartbeat = now;
        this->sendHeartbeat(now);
    }
}
