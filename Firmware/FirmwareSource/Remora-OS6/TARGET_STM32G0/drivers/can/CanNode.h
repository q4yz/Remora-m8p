#ifndef CANNODE_H
#define CANNODE_H

#include "RemoraCAN.h"
#include "canProtocol.h"

/***********************************************************************
    CAN node (expansion board, e.g. BTT EBB42), phase 1: identifies itself
    by UUID, takes the node number the hub assigns and sends heartbeats.
************************************************************************/

class CanNode
{
    private:

        RemoraCAN*  can;
        uint8_t     uuid[6];
        uint8_t     node;           // 0: no number assigned yet
        uint8_t     state;          // NODE_STATE_*
        uint32_t    lastHeartbeat;

        void sendLssReply(void);
        void sendHeartbeat(uint32_t now);
        void handleLss(const canFrame_t& frame);

    public:

        CanNode(RemoraCAN* can);

        void service(uint32_t now);
        const uint8_t* getUuid(void) { return this->uuid; }
        uint8_t getNode(void) { return this->node; }
        uint8_t getState(void) { return this->state; }
};

#endif
