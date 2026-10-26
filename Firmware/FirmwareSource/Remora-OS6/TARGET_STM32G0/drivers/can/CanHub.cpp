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
    unknownPending(false),
    unknownPrintedAny(false)
{
    memset(this->nodes, 0, sizeof(this->nodes));
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

    // a node that is not in config.txt: report its UUID once, and make sure it has no number
    if (!this->unknownPrintedAny || memcmp(this->unknownPrinted, reply.uuid, 6))
    {
        memcpy(this->unknownUuid, reply.uuid, 6);
        this->unknownPending = true;
    }
    if (reply.node != 0) this->sendAssign(reply.uuid, 0);
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
        return;
    }
}


void CanHub::service(uint32_t now, bool quiet)
{
    this->can->service();

    canFrame_t f;
    int budget = 16;                                    // bounded work per main loop pass
    while (budget-- && this->can->receive(f))
    {
        if (f.extended && (f.id & CAN_EXTID_LSS_MASK) == CAN_EXTID_LSS_REPLY)
        {
            this->handleLssReply(f, now);
        }
        else if (!f.extended && f.id > CAN_ID_HEARTBEAT && f.id <= CAN_ID_HEARTBEAT + CAN_NODE_MAX)
        {
            this->handleHeartbeat(f, now);
        }
    }

    // heartbeat timeouts, and IDENTIFY while a configured node is missing
    bool missing = false;
    for (int i = 0; i < this->nodeCount; i++)
    {
        canNodeEntry_t* n = &this->nodes[i];
        if (n->alive && (now - n->lastSeen) > CAN_NODE_TIMEOUT_MS)
        {
            n->alive = false;
            n->reported = true;
        }
        if (!n->alive) missing = true;
    }

    if ((missing || this->nodeCount == 0) && (now - this->lastIdentify) >= CAN_IDENTIFY_MS)
    {
        this->lastIdentify = now;
        this->sendIdentify();
    }

    if (!quiet) this->report();
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
        printf("\n");
    }

    if (this->unknownPending)
    {
        this->unknownPending = false;
        printf("CAN: unknown node UUID ");
        printUuid(this->unknownUuid);
        printf(", add it to \"CAN\" \"Nodes\" in config.txt\n");
        memcpy(this->unknownPrinted, this->unknownUuid, 6);
        this->unknownPrintedAny = true;
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
