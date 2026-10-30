#include "CanHub.h"
#include "CanNode.h"
#include "canNodeIoSim.h"
#include <string>
#include <cmath>
std::vector<RemoraCAN*> RemoraCAN::bus;
uint32_t sim_uid[3];

static int fails;
#define CHECK(c) do { if (c) printf("  ok   %s\n", #c); else { printf("  FAIL %s\n", #c); fails++; } } while (0)

static std::string hex(const uint8_t u[6]) { char b[13]; for (int i = 0; i < 6; i++) sprintf(b + 2 * i, "%02x", u[i]); return b; }

/* a TMC2209 on the single wire UART of a simulated node, bit by bit at the node base tick (3 ticks per bit) */
struct SimTmc {
    uint8_t pin, addr = 0;
    bool powered = true;
    int dropWrites = 0;                     /* ignore this many writes (corrupted on the line) */
    uint32_t regs[128];
    uint8_t ifcnt;
    int writes = 0, reads = 0;
    int rxBit = -1, rxTick = 0, len = 0, idle = 0; uint16_t rxShift = 0; uint8_t buf[8];
    uint8_t tx[8]; int txLen = 0, txByte = 0, txBit = 0, txTick = 0, delay = 0;
    SimTmc(uint8_t pin) : pin(pin) { reset(); }
    void reset() { memset(regs, 0, sizeof(regs)); regs[0x01] = 1; regs[0x06] = 0x21u << 24; regs[0x6C] = 0x10000053; ifcnt = 0; }
    void datagram() {
        if (len == 4 && !(buf[2] & 0x80)) {                 /* read request */
            len = 0;
            if (!powered || buf[0] != 0x05 || buf[1] != addr || buf[3] != tmcCrc(buf, 3)) return;
            uint32_t v = buf[2] == 0x02 ? ifcnt : regs[buf[2] & 0x7F];
            tx[0] = 0x05; tx[1] = 0xFF; tx[2] = buf[2];
            tx[3] = (uint8_t)(v >> 24); tx[4] = (uint8_t)(v >> 16); tx[5] = (uint8_t)(v >> 8); tx[6] = (uint8_t)v;
            tx[7] = tmcCrc(tx, 7);
            txLen = 8; delay = 24; reads++;                  /* SENDDELAY 8 bit times */
        }
        else if (len == 8) {                                 /* write */
            len = 0;
            if (!powered || buf[0] != 0x05 || buf[1] != addr || buf[7] != tmcCrc(buf, 7)) return;
            if (dropWrites > 0) { dropWrites--; return; }
            uint32_t v = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8) | buf[6];
            uint8_t r = buf[2] & 0x7F;
            if (r == 0x01) regs[r] &= ~v; else regs[r] = v;  /* GSTAT: write 1 to clear */
            ifcnt++; writes++;
        }
    }
    void tick(SimIo& io) {
        bool driven = io.output[pin];
        if (txLen) {                                         /* answering */
            if (delay) { if (--delay == 0) { txByte = 0; txBit = 0; txTick = 1; } return; }
            if (--txTick) return;
            if (txBit == 10) { txBit = 0; if (++txByte >= txLen) { txLen = 0; if (!driven) io.level[pin] = true; return; } }
            uint16_t frame = (uint16_t)((tx[txByte] << 1) | 0x200);
            if (!driven) io.level[pin] = (frame >> txBit) & 1;
            txBit++; txTick = 3;
            return;
        }
        if (!driven) { io.level[pin] = true; rxBit = -1; return; }   /* pull up */
        bool line = io.level[pin];
        if (rxBit < 0) {
            if (!line) { rxBit = 0; rxTick = 4; rxShift = 0; }
            else if (++idle > 60) len = 0;                   /* gap: datagram over */
            return;
        }
        if (--rxTick) return;
        if (rxBit < 8) {
            rxShift = (uint16_t)((rxShift >> 1) | (line ? 0x80 : 0));
            rxTick = 3;
            if (++rxBit == 8) { if (len < 8) buf[len++] = (uint8_t)rxShift; idle = 0; datagram(); }
            return;
        }
        rxBit = -1;
    }
};

struct Node { RemoraCAN can; CanNode* node; SimIo io; SimTmc* tmc = nullptr; Node(uint32_t a, uint32_t b, uint32_t c) { sim_uid[0] = a; sim_uid[1] = b; sim_uid[2] = c; node = new CanNode(&can); } };

/* what the hub gets from LinuxCNC */
static bool simSafe = true;
static uint16_t simOutputs = 0;
static bool hubRuns = true;
static int32_t simFreq[8];                 /* rxData.jointFreqCmd */
static uint8_t simEnable = 0;              /* rxData.jointEnable */
static int32_t simFeedback[8];             /* txData.jointFeedback */
static float simPv[6];                     /* txData.processVariable */

/* one LinuxCNC WRITE per ms, the node base thread at CAN_NODE_BASE_FREQ */
static void run(CanHub& hub, std::vector<Node*> nodes, uint32_t& now, uint32_t ms, bool quiet = false) {
    for (uint32_t t = 0; t < ms; t++, now++) {
        if (hubRuns) {
            hub.service(now, quiet, simSafe, simOutputs);
            hub.serviceJoints(now, !simSafe, simSafe, simFreq, simEnable, simFeedback);
            hub.writeVariables(simPv);
        }
        for (Node* n : nodes) {
            simIo = &n->io;
            n->node->service(now);
            for (int k = 0; k < CAN_NODE_BASE_FREQ / 1000; k++) {
                n->node->baseTick();
                if (n->tmc) n->tmc->tick(n->io);
            }
        }
    }
}

static const uint8_t PB8 = CAN_PIN(1, 8), PB9 = CAN_PIN(1, 9);
static const uint8_t PD0 = CAN_PIN(3, 0), PD1 = CAN_PIN(3, 1), PD2 = CAN_PIN(3, 2);

/* node with an extruder stepgen as LinuxCNC joint 3 (step PD_0, direction PD_1, enable PD_2), running */
static void setupJoint(CanHub& hub, Node& a, uint32_t& now) {
    hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
    CHECK(hub.addStepgen(1, 3, "PD_0", "PD_1", "PD_2"));
    CHECK(hub.jointMask() == (1u << 3));
    run(hub, {&a}, now, 1500);
    CHECK(a.node->getState() == NODE_STATE_RUNNING && a.node->getJointCount() == 1);
}

/* EBB42: stepgen joint 3 and its TMC2209 on PA_15, 800 mA, 16 microsteps, StealthChop, configured */
static const uint8_t PA15 = CAN_PIN(0, 15);
static void setupTmc(CanHub& hub, Node& a, SimTmc& tmc, uint32_t& now) {
    a.tmc = &tmc;
    hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
    CHECK(hub.addStepgen(1, 3, "PD_0", "PD_1", "PD_2"));
    CHECK(hub.addTmc2209(1, "PA_15", 0.11f, 0, 800, 16, true, 0));
    run(hub, {&a}, now, 1500);
    CHECK(a.node->getState() == NODE_STATE_RUNNING && a.node->getTmcCount() == 1);
}

/* node with an output on PB_8 (LinuxCNC output bit 1, safe high) and an inverted, pulled up
   input on PB_9 (LinuxCNC input bit 3, reads 1 while the node is lost), configured and running */
static void setupIo(CanHub& hub, Node& a, uint32_t& now) {
    hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
    CHECK(hub.addDigitalPin(1, "PB_8", true, false, DPIN_MOD_NONE, true, false, 1));
    CHECK(hub.addDigitalPin(1, "PB_9", false, true, DPIN_MOD_PULLUP, false, true, 3));
    CHECK(hub.inputMask() == (1u << 3) && hub.inputValues() == (1u << 3));   /* not running yet: lost value */
    run(hub, {&a}, now, 1500);
}

int main(int argc, char** argv) {
    std::string sc = argc > 1 ? argv[1] : "";
    printf("== %s\n", sc.c_str());
    uint32_t now = 1;

    if (sc == "uuid") {
        uint8_t uid[12] = {1,2,3,4,5,6,7,8,9,10,11,12}, a[6], b[6];
        canUuidFromUid(uid, a); canUuidFromUid(uid, b);
        CHECK(!memcmp(a, b, 6));
        uid[11] ^= 1; canUuidFromUid(uid, b);
        CHECK(memcmp(a, b, 6) != 0);
        uint8_t p[6];
        CHECK(canParseUuid("0e0d81e4210c", p) && p[0] == 0x0e && p[5] == 0x0c);
        CHECK(!canParseUuid("0e0d81e4210", p) && !canParseUuid("0e0d81e4210g", p));
    }
    else if (sc == "assign") {
        Node a(0x11111111, 0x22222222, 0x33333333);
        RemoraCAN hc; CanHub hub(&hc);
        CHECK(hub.addNode(hex(a.node->getUuid()).c_str(), 3, "toolhead"));
        CHECK(a.node->getNode() == 0);
        run(hub, {&a}, now, 1500);
        CHECK(a.node->getNode() == 3 && a.node->getState() == NODE_STATE_RUNNING);     /* configured, no modules */
        CHECK(hub.aliveMask() == (1u << 2));
    }
    else if (sc == "two-nodes") {
        Node a(1, 2, 3), b(4, 5, 6);
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "left");
        hub.addNode(hex(b.node->getUuid()).c_str(), 2, "right");
        run(hub, {&a, &b}, now, 1500);
        CHECK(a.node->getNode() == 1 && b.node->getNode() == 2);
        CHECK(hub.aliveMask() == 0x3);
        int sent = hc.sentLss;
        run(hub, {&a, &b}, now, 3000, true);
        CHECK(hc.sentLss == sent);                             /* no IDENTIFY traffic once all nodes are online */
    }
    else if (sc == "unknown") {
        Node a(1, 2, 3), u(7, 8, 9);
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        run(hub, {&a, &u}, now, 3000);
        CHECK(a.node->getNode() == 1 && u.node->getNode() == 0);
        CHECK(hub.aliveMask() == 0x1);
        printf("  (unknown node UUID %s should be in the boot summary above)\n", hex(u.node->getUuid()).c_str());
    }
    else if (sc == "lost") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        run(hub, {&a}, now, 1500);
        CHECK(hub.aliveMask() == 0x1);
        a.can.alive = false;                                   /* unplugged */
        run(hub, {&a}, now, 240);
        CHECK(hub.aliveMask() == 0x1);                          /* last heartbeat <= 100 + 240 ms ago: not lost yet */
        run(hub, {&a}, now, 200);
        CHECK(hub.aliveMask() == 0x0);
        a.can.alive = true;                                    /* plugged back in: found again */
        run(hub, {&a}, now, 1500);
        CHECK(hub.aliveMask() == 0x1);
    }
    else if (sc == "node-reboot") {
        RemoraCAN hc; CanHub hub(&hc);
        Node* a = new Node(1, 2, 3);
        hub.addNode(hex(a->node->getUuid()).c_str(), 5, "toolhead");
        run(hub, {a}, now, 1500);
        CHECK(a->node->getNode() == 5);
        delete a->node; sim_uid[0] = 1; sim_uid[1] = 2; sim_uid[2] = 3;
        a->node = new CanNode(&a->can);                           /* node resets, forgets its number */
        CHECK(a->node->getNode() == 0);
        run(hub, {a}, now, 2000);
        CHECK(a->node->getNode() == 5 && hub.aliveMask() == (1u << 4));
    }
    else if (sc == "bad-config") {
        RemoraCAN hc; CanHub hub(&hc);
        CHECK(!hub.addNode("xyz", 1, ""));
        CHECK(!hub.addNode("0e0d81e4210c", 0, ""));
        CHECK(!hub.addNode("0e0d81e4210c", 16, ""));
        CHECK(hub.addNode("0e0d81e4210c", 1, ""));
        CHECK(!hub.addNode("0e0d81e4210c", 2, ""));               /* same UUID twice */
        CHECK(!hub.addNode("aaaaaaaaaaaa", 1, ""));               /* same node number twice */
        CHECK(hub.getNodeCount() == 1);
    }
    else if (sc == "pins") {
        uint8_t p; char b[6];
        CHECK(canParsePin("PB_8", &p) && p == PB8 && !strcmp(canPinName(p, b), "PB_8"));
        CHECK(canParsePin("pa15", &p) && p == CAN_PIN(0, 15) && !strcmp(canPinName(p, b), "PA_15"));
        CHECK(!canParsePin("PB_16", &p) && !canParsePin("PG_1", &p) && !canParsePin("B8", &p) && !canParsePin("PB_", &p));
    }
    else if (sc == "config-io") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        a.io.level[PB9] = true;                                 /* input idles high (pull up), inverted: bit 0 */
        setupIo(hub, a, now);
        CHECK(a.node->getState() == NODE_STATE_RUNNING && a.node->getModuleCount() == 2);
        CHECK(hub.runningMask() == 0x1);
        CHECK(a.io.configured[PB8] && a.io.output[PB8] && a.io.configured[PB9] && !a.io.output[PB9]);
        CHECK(a.io.modifier[PB9] == DPIN_MOD_PULLUP);
        CHECK(a.io.level[PB8] == true);                         /* LinuxCNC not there yet: safe state high */
        run(hub, {&a}, now, 50);
        CHECK(hub.inputValues() == 0);                          /* input high, inverted */

        simSafe = false; simOutputs = 0;                        /* LinuxCNC running */
        run(hub, {&a}, now, 5);
        CHECK(a.io.level[PB8] == false);
        simOutputs = 1u << 1;
        run(hub, {&a}, now, 5);
        CHECK(a.io.level[PB8] == true);
        simOutputs = 1u << 0;                                   /* other bits do not reach the node */
        run(hub, {&a}, now, 5);
        CHECK(a.io.level[PB8] == false);

        a.io.level[PB9] = false;                                /* switch closes */
        run(hub, {&a}, now, 5);
        CHECK(hub.inputValues() == (1u << 3));

        simSafe = true;                                         /* LinuxCNC gone */
        run(hub, {&a}, now, 5);
        CHECK(a.io.level[PB8] == true);
        CHECK(a.node->getState() == NODE_STATE_RUNNING);
    }
    else if (sc == "hub-lost") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        setupIo(hub, a, now);
        simSafe = false; simOutputs = 0;
        run(hub, {&a}, now, 50);
        CHECK(a.io.level[PB8] == false);
        hubRuns = false;                                        /* hub resets / hangs */
        run(hub, {&a}, now, 70);                                /* last COMMAND at most 20 ms before */
        CHECK(a.node->getState() == NODE_STATE_RUNNING);
        run(hub, {&a}, now, 60);
        CHECK(a.node->getState() == NODE_STATE_SAFE && a.io.level[PB8] == true);
        hubRuns = true;
        run(hub, {&a}, now, 50);
        CHECK(a.node->getState() == NODE_STATE_RUNNING && a.io.level[PB8] == false);
    }
    else if (sc == "node-reboot-config") {
        RemoraCAN hc; CanHub hub(&hc);
        Node* a = new Node(1, 2, 3);
        a->io.level[PB9] = true;
        setupIo(hub, *a, now);
        simSafe = false; simOutputs = 1u << 1;
        run(hub, {a}, now, 50);
        CHECK(a->io.level[PB8] == true && hub.inputValues() == 0);
        delete a->node; sim_uid[0] = 1; sim_uid[1] = 2; sim_uid[2] = 3;
        a->node = new CanNode(&a->can);                           /* node resets: no number, no modules */
        a->io = SimIo(); a->io.level[PB9] = true;
        run(hub, {a}, now, 2500);
        CHECK(a->node->getState() == NODE_STATE_RUNNING && a->node->getModuleCount() == 2);
        CHECK(a->io.level[PB8] == true && hub.runningMask() == 0x1 && hub.inputValues() == 0);
    }
    else if (sc == "rejected") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        CHECK(hub.addDigitalPin(1, "PB_8", true, false, DPIN_MOD_NONE, false, false, 1));
        CHECK(hub.addDigitalPin(1, "PB_0", true, false, DPIN_MOD_NONE, false, false, 2));    /* the node's CAN RX */
        run(hub, {&a}, now, 3000);
        CHECK(a.node->getState() == NODE_STATE_PREOP && a.node->getModuleCount() == 0);
        CHECK(hub.runningMask() == 0);
        CHECK(!a.io.configured[PB8]);                           /* nothing applied */
        printf("  (\"rejected ... module 2 (PB_0): pin reserved\" should be printed once above)\n");
    }
    else if (sc == "bit-conflict") {
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode("0e0d81e4210c", 1, "");
        hub.reserveBits(1u << 0, 1u << 2);                      /* hub's own pins */
        CHECK(!hub.addDigitalPin(1, "PB_8", true, false, 0, false, false, 0));
        CHECK(hub.addDigitalPin(1, "PB_8", true, false, 0, false, false, 1));
        CHECK(!hub.addDigitalPin(1, "PB_9", true, false, 0, false, false, 1));       /* bit taken */
        CHECK(!hub.addDigitalPin(1, "PB_8", false, false, 0, false, false, 4));      /* pin taken */
        CHECK(!hub.addDigitalPin(1, "PB_9", false, false, 0, false, false, 2));      /* hub's input bit */
        CHECK(hub.addDigitalPin(1, "PB_9", false, false, 0, false, false, 1));       /* input 1 is free */
        CHECK(!hub.addDigitalPin(2, "PB_5", true, false, 0, false, false, 5));       /* no node 2 */
        CHECK(!hub.addDigitalPin(1, "XB_5", true, false, 0, false, false, 5));
        CHECK(!hub.addDigitalPin(1, "PB_5", true, false, 0, false, false, 16));
    }
    else if (sc == "lossy") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        hub.addDigitalPin(1, "PB_8", true, false, DPIN_MOD_NONE, false, false, 1);
        hub.addDigitalPin(1, "PB_9", false, false, DPIN_MOD_NONE, false, false, 0);
        run(hub, {&a}, now, 1200);                              /* number assigned, configuration starts */
        /* lose frames in both directions during the transfer: retries and repeated frames */
        for (int i = 0; i < 6; i++) { a.can.drop = 1; run(hub, {&a}, now, 30); hc.drop = 1; run(hub, {&a}, now, 30); }
        run(hub, {&a}, now, 2000);
        CHECK(a.node->getState() == NODE_STATE_RUNNING && a.node->getModuleCount() == 2);
        CHECK(hub.runningMask() == 0x1);
        simSafe = false; simOutputs = 1u << 1;
        run(hub, {&a}, now, 50);
        CHECK(a.io.level[PB8] == true);
    }
    else if (sc == "mandatory") {
        Node a(1, 2, 3), b(4, 5, 6);
        RemoraCAN hc; CanHub hub(&hc);
        hub.reserveBits(0, 1u << 0);                            /* hub's own input 0 */
        CHECK(!hub.setStatusBit(0) && !hub.setStatusBit(16));
        CHECK(hub.setStatusBit(15));
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");               /* mandatory by default */
        hub.addNode(hex(b.node->getUuid()).c_str(), 2, "camera", false);          /* optional */
        CHECK(!hub.addDigitalPin(1, "PB_9", false, false, 0, false, false, 15));  /* the status bit */
        const uint16_t S = 1u << 15;
        CHECK((hub.inputMask() & S) && !(hub.inputValues() & S));                /* nothing online: E-stop */
        b.can.alive = false;                                                     /* optional node missing */
        run(hub, {&a, &b}, now, 3000);
        CHECK(hub.runningMask() == 0x1 && (hub.inputValues() & S));              /* boot summary printed above */
        a.can.alive = false;                                                     /* mandatory node lost */
        run(hub, {&a, &b}, now, 400);
        CHECK(!(hub.inputValues() & S));
        printf("  (\"LOST, mandatory: status bit 0, E-stop\" above)\n");
        a.can.alive = true;
        run(hub, {&a, &b}, now, 1500);
        CHECK(hub.runningMask() == 0x1 && (hub.inputValues() & S));              /* back and configured */
        b.can.alive = true;
        run(hub, {&a, &b}, now, 1500);
        b.can.alive = false;
        run(hub, {&a, &b}, now, 400);
        CHECK(hub.runningMask() == 0x1 && (hub.inputValues() & S));              /* optional node lost: no E-stop */
    }
    else if (sc == "no-status-bit") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        run(hub, {&a}, now, 3000);
        CHECK(hub.inputMask() == 0);
        printf("  (\"no \\\"Status Bit\\\"\" warning in the boot summary above)\n");
    }
    else if (sc == "joint") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        setupJoint(hub, a, now);
        CHECK(a.io.output[PD0] && a.io.output[PD1] && a.io.output[PD2]);
        CHECK(a.io.level[PD2] == true && !a.node->getJointEnabled(0));           /* LinuxCNC not there: disabled */
        simSafe = false; simEnable = 1u << 3; simFreq[3] = 1000;               /* 1000 steps/s */
        run(hub, {&a}, now, 2);
        CHECK(a.node->getJointEnabled(0) && a.io.level[PD2] == false);         /* enable is active low */
        run(hub, {&a}, now, 500);
        int32_t steps = a.node->getJointPosition(0);
        printf("  node %d steps, %d pulses, hub feedback %d\n", steps, a.io.rises[PD0], simFeedback[3]);
        CHECK(steps >= 495 && steps <= 505);
        CHECK(a.io.rises[PD0] == steps && a.io.level[PD1] == true);            /* forward */
        CHECK(simFeedback[3] >= steps - 2 && simFeedback[3] <= steps);         /* one frame behind at most */
        simFreq[3] = 0;
        run(hub, {&a}, now, 5);
        CHECK(simFeedback[3] == a.node->getJointPosition(0));                  /* stopped: exact */
        simFreq[3] = 20000;                                                     /* max: half the base frequency */
        int32_t before = a.node->getJointPosition(0);
        run(hub, {&a}, now, 100);
        printf("  20 kHz: %d steps in 100 ms\n", a.node->getJointPosition(0) - before);
        CHECK(a.node->getJointPosition(0) - before >= 1990);
        simEnable = 0;                                                          /* LinuxCNC disables the joint */
        run(hub, {&a}, now, 2);
        before = a.node->getJointPosition(0);
        run(hub, {&a}, now, 20);
        CHECK(!a.node->getJointEnabled(0) && a.io.level[PD2] == true && a.node->getJointPosition(0) == before);
        simSafe = true;
    }
    else if (sc == "joint-reverse") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        setupJoint(hub, a, now);
        simSafe = false; simEnable = 1u << 3; simFreq[3] = -2000;              /* backwards through 0: the 24-bit counts wrap */
        run(hub, {&a}, now, 300);
        run(hub, {&a}, now, 0);
        simFreq[3] = 0;
        run(hub, {&a}, now, 5);
        printf("  node %d steps, hub feedback %d\n", a.node->getJointPosition(0), simFeedback[3]);
        CHECK(a.node->getJointPosition(0) <= -595 && a.io.level[PD1] == false);
        CHECK(simFeedback[3] == a.node->getJointPosition(0));
        CHECK(canDelta24(0x000005, 0xFFFFFE) == 7 && canDelta24(0xFFFFFE, 0x000005) == -7);
        simSafe = true;
    }
    else if (sc == "joint-hub-lost") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        setupJoint(hub, a, now);
        simSafe = false; simEnable = 1u << 3; simFreq[3] = 5000;
        run(hub, {&a}, now, 50);
        CHECK(a.node->getJointEnabled(0));
        hubRuns = false;                                                        /* hub hangs */
        run(hub, {&a}, now, 12);                                                /* > CAN_JOINT_TIMEOUT_MS */
        int32_t pos = a.node->getJointPosition(0);
        run(hub, {&a}, now, 20);
        CHECK(!a.node->getJointEnabled(0) && a.io.level[PD2] == true && a.node->getJointPosition(0) == pos);
        hubRuns = true;
        run(hub, {&a}, now, 5);
        CHECK(a.node->getJointEnabled(0) && a.node->getJointPosition(0) > pos);  /* runs again */
        simSafe = true;                                                         /* LinuxCNC gone */
        run(hub, {&a}, now, 5);
        CHECK(!a.node->getJointEnabled(0));
    }
    else if (sc == "joint-config") {
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode("0e0d81e4210c", 1, "");
        hub.reserveJoints(1u << 0);                                             /* hub's own stepgen */
        CHECK(!hub.addStepgen(1, 0, "PD_0", "PD_1", "PD_2"));                   /* joint taken */
        CHECK(!hub.addStepgen(1, 8, "PD_0", "PD_1", "PD_2"));                   /* out of range */
        CHECK(!hub.addStepgen(1, 3, "PD_0", "PD_0", ""));                       /* same pin twice */
        CHECK(!hub.addStepgen(1, 3, "PD_0", "XD_1", ""));
        CHECK(!hub.addStepgen(2, 3, "PD_0", "PD_1", ""));                       /* no node 2 */
        CHECK(hub.addStepgen(1, 3, "PD_0", "PD_1", ""));                        /* no enable pin */
        CHECK(!hub.addStepgen(1, 4, "PD_3", "PD_1", ""));                       /* direction pin taken */
        CHECK(!hub.addDigitalPin(1, "PD_1", true, false, 0, false, false, 1));  /* stepgen pin taken */
        CHECK(hub.addStepgen(1, 4, "PD_3", "PD_4", "PD_5"));
        CHECK(!hub.addStepgen(1, 5, "PD_6", "PD_7", ""));                       /* 2 joints per node */
        CHECK(hub.jointMask() == ((1u << 0) | (1u << 3) | (1u << 4)));
        /* the node rejects a reserved pin (CAN RX) */
        Node a(1, 2, 3);
        RemoraCAN hc2; CanHub hub2(&hc2);
        hub2.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        CHECK(hub2.addStepgen(1, 3, "PD_0", "PB_0", ""));
        run(hub2, {&a}, now, 3000);
        CHECK(a.node->getState() == NODE_STATE_PREOP && hub2.runningMask() == 0);
        CHECK(!a.io.configured[PD0]);
    }
    else if (sc == "joint-reboot") {
        RemoraCAN hc; CanHub hub(&hc);
        Node* a = new Node(1, 2, 3);
        setupJoint(hub, *a, now);
        simSafe = false; simEnable = 1u << 3; simFreq[3] = 1000;
        run(hub, {a}, now, 200);
        simFreq[3] = 0;
        run(hub, {a}, now, 5);
        int32_t pos = simFeedback[3];
        CHECK(pos > 190);
        delete a->node; sim_uid[0] = 1; sim_uid[1] = 2; sim_uid[2] = 3;
        a->node = new CanNode(&a->can);                                         /* node resets: count 0 */
        a->io = SimIo();
        run(hub, {a}, now, 2500);
        CHECK(a->node->getJointCount() == 1 && a->node->getJointPosition(0) == 0);
        CHECK(simFeedback[3] == pos);                                           /* LinuxCNC sees no jump */
        simFreq[3] = 1000;
        run(hub, {a}, now, 100);
        simFreq[3] = 0;
        run(hub, {a}, now, 5);
        CHECK(simFeedback[3] == pos + a->node->getJointPosition(0));
        simSafe = true;
    }
    else if (sc == "tmc") {
        Node a(1, 2, 3); SimTmc tmc(PA15);
        RemoraCAN hc; CanHub hub(&hc);
        setupTmc(hub, a, tmc, now);
        CHECK(a.io.configured[PA15] && a.io.modifier[PA15] == DPIN_MOD_PULLUP);
        CHECK(a.node->getTmc(0).getStatus() == TMC_STATUS_OK && a.node->getTmc(0).getVersion() == 0x21);
        CHECK(hub.getTmcStatus(1, 0) == TMC_STATUS_OK);
        printf("  %d writes, %d reads, GCONF 0x%x CHOPCONF 0x%08x IHOLD_IRUN 0x%06x\n", tmc.writes, tmc.reads,
               tmc.regs[0x00], tmc.regs[0x6C], tmc.regs[0x10]);
        CHECK(tmc.writes == TMC_CONFIG_REGS);
        CHECK(tmc.regs[0x00] == 0x1C0);                                       /* UART, register microsteps, StealthChop */
        CHECK(((tmc.regs[0x6C] >> 24) & 0xF) == 4 && (tmc.regs[0x6C] & (1u << 17)) && (tmc.regs[0x6C] & 0xF) == 3);
        CHECK(tmc.regs[0x10] == (12u | (25u << 8) | (10u << 16)));            /* 800 mA, 0.11 Ohm: CS 25 with vsense */
        CHECK(tmc.regs[0x01] == 0 && tmc.regs[0x70] == 0xC80D0E24 && tmc.regs[0x40] == 0);
        int reads = tmc.reads;
        run(hub, {&a}, now, 3100);                                            /* GSTAT check every second, no writes */
        CHECK(tmc.reads - reads >= 3 && tmc.reads - reads <= 4 && tmc.writes == TMC_CONFIG_REGS && a.node->getTmc(0).getConfigs() == 1);
        simSafe = false; simEnable = 1u << 3; simFreq[3] = 1000;              /* the joint runs alongside */
        run(hub, {&a}, now, 200);
        CHECK(a.node->getJointPosition(0) >= 195 && a.node->getJointPosition(0) <= 201);
        simSafe = true;
    }
    else if (sc == "tmc-reset") {
        Node a(1, 2, 3); SimTmc tmc(PA15);
        RemoraCAN hc; CanHub hub(&hc);
        setupTmc(hub, a, tmc, now);
        CHECK(a.node->getTmc(0).getStatus() == TMC_STATUS_OK);
        tmc.reset();                                                          /* motor power off and on: driver defaults */
        run(hub, {&a}, now, 1300);
        CHECK(a.node->getTmc(0).getStatus() == TMC_STATUS_OK && a.node->getTmc(0).getConfigs() == 2);
        CHECK(tmc.regs[0x00] == 0x1C0 && tmc.regs[0x01] == 0 && tmc.writes == 2 * TMC_CONFIG_REGS);
        CHECK(hub.getTmcStatus(1, 0) == TMC_STATUS_OK);
    }
    else if (sc == "tmc-no-power") {
        Node a(1, 2, 3); SimTmc tmc(PA15);
        tmc.powered = false;                                                  /* no motor power: no answer */
        RemoraCAN hc; CanHub hub(&hc);
        setupTmc(hub, a, tmc, now);
        run(hub, {&a}, now, 500);
        CHECK(a.node->getTmc(0).getStatus() == TMC_STATUS_NO_ANSWER && hub.getTmcStatus(1, 0) == TMC_STATUS_NO_ANSWER);
        CHECK(a.node->getState() == NODE_STATE_RUNNING && hub.runningMask() == 1);  /* the node runs anyway */
        CHECK(a.io.output[PA15] && a.io.level[PA15]);                         /* line back to idle high */
        tmc.powered = true;
        run(hub, {&a}, now, 1300);                                            /* retried every second */
        CHECK(a.node->getTmc(0).getStatus() == TMC_STATUS_OK && hub.getTmcStatus(1, 0) == TMC_STATUS_OK);
        CHECK(tmc.regs[0x00] == 0x1C0);
    }
    else if (sc == "tmc-write-lost") {
        Node a(1, 2, 3); SimTmc tmc(PA15);
        tmc.dropWrites = 1;
        RemoraCAN hc; CanHub hub(&hc);
        a.tmc = &tmc;
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        CHECK(hub.addTmc2209(1, "PA_15", 0.11f, 0, 800, 16, true, 0));
        run(hub, {&a}, now, 1200);
        CHECK(a.node->getTmc(0).getStatus() == TMC_STATUS_WRITE_FAILED);
        run(hub, {&a}, now, 1300);
        CHECK(a.node->getTmc(0).getStatus() == TMC_STATUS_OK && tmc.regs[0x00] == 0x1C0);
    }
    else if (sc == "thermistor") {
        /* EBB42 hotend: 100k NTC, beta 3950, 4.7k pullup on PA_3, LinuxCNC PV[2] */
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        const uint8_t PA3 = CAN_PIN(0, 3);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        CHECK(hub.addThermistor(1, "PA_3", 2, 3950.0f, 100000.0f, 25.0f, 4700.0f));
        CHECK(hub.addDigitalPin(1, "PB_13", true, false, DPIN_MOD_NONE, false, false, 0));   /* heater */
        a.io.analog[PA3] = (uint16_t)(65536.0 * 100000.0 / 104700.0);                       /* 25 C: r = r0 */
        run(hub, {&a}, now, 300);
        CHECK(simPv[2] == CAN_TEMP_ERROR);                                                  /* not configured yet */
        run(hub, {&a}, now, 1500);
        CHECK(a.node->getState() == NODE_STATE_RUNNING && a.node->getAnalogCount() == 1 && a.io.analogOn[PA3]);
        printf("  25 C: PV %.2f\n", simPv[2]);
        CHECK(simPv[2] > 24.8f && simPv[2] < 25.2f);
        double r = 100000.0 * exp(3950.0 * (1.0 / 473.15 - 1.0 / 298.15));                /* 200 C */
        a.io.analog[PA3] = (uint16_t)(65536.0 * r / (r + 4700.0));
        run(hub, {&a}, now, 250);                                                           /* averaged over 100 ms */
        printf("  200 C: PV %.2f\n", simPv[2]);
        CHECK(simPv[2] > 199.0f && simPv[2] < 201.0f);
        a.io.analog[PA3] = 0xFFFF;                                                          /* sensor open */
        run(hub, {&a}, now, 250);
        CHECK(simPv[2] == CAN_TEMP_ERROR);
        a.io.analog[PA3] = 0;                                                               /* shorted */
        run(hub, {&a}, now, 250);
        CHECK(simPv[2] == CAN_TEMP_ERROR);
        CHECK(simPv[0] == 0.0f && simPv[1] == 0.0f);                                        /* other PVs untouched */
    }
    else if (sc == "thermistor-lost") {
        Node a(1, 2, 3);
        RemoraCAN hc; CanHub hub(&hc);
        const uint8_t PA3 = CAN_PIN(0, 3);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        CHECK(hub.addThermistor(1, "PA_3", 2, 3950.0f, 100000.0f, 25.0f, 4700.0f));
        a.io.analog[PA3] = (uint16_t)(65536.0 * 100000.0 / 104700.0);
        run(hub, {&a}, now, 2000);
        CHECK(simPv[2] > 24.8f && simPv[2] < 25.2f);
        run(hub, {}, now, 400);                                                             /* node gone */
        CHECK(simPv[2] == CAN_TEMP_ERROR);
        run(hub, {&a}, now, 2000);                                                          /* back, configured again */
        CHECK(simPv[2] > 24.8f && simPv[2] < 25.2f);
    }
    else if (sc == "thermistor-config") {
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode("0e0d81e4210c", 1, "");
        hub.reserveVariables(1u << 0);                                                      /* the hub's own Temperature */
        CHECK(!hub.addThermistor(1, "PA_3", 0, 3950.0f, 100000.0f, 25.0f, 4700.0f));        /* PV taken */
        CHECK(!hub.addThermistor(1, "PA_3", 6, 3950.0f, 100000.0f, 25.0f, 4700.0f));        /* out of range */
        CHECK(!hub.addThermistor(1, "PA_3", 1, 0.0f, 100000.0f, 25.0f, 4700.0f));           /* no beta */
        CHECK(!hub.addThermistor(1, "XA_3", 1, 3950.0f, 100000.0f, 25.0f, 4700.0f));
        CHECK(hub.addThermistor(1, "PA_3", 1, 3950.0f, 100000.0f, 25.0f, 4700.0f));
        CHECK(!hub.addThermistor(1, "PA_4", 1, 3950.0f, 100000.0f, 25.0f, 4700.0f));        /* PV used twice */
        CHECK(!hub.addThermistor(1, "PA_3", 2, 3950.0f, 100000.0f, 25.0f, 4700.0f));        /* pin used twice */
        /* the node rejects a pin without ADC */
        Node a(1, 2, 3);
        RemoraCAN hc2; CanHub hub2(&hc2);
        hub2.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        CHECK(hub2.addThermistor(1, "PB_13", 2, 3950.0f, 100000.0f, 25.0f, 4700.0f));
        run(hub2, {&a}, now, 3000);
        CHECK(a.node->getState() == NODE_STATE_PREOP && hub2.runningMask() == 0);
        CHECK(simPv[2] == CAN_TEMP_ERROR);
    }
    else if (sc == "tmc-config") {
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode("0e0d81e4210c", 1, "");
        CHECK(hub.addStepgen(1, 3, "PD_0", "PD_1", "PD_2"));
        CHECK(!hub.addTmc2209(1, "PA_15", 0.11f, 0, 800, 3, true, 0));       /* microsteps not a power of 2 */
        CHECK(!hub.addTmc2209(1, "PA_15", 0.11f, 0, 800, 512, true, 0));
        CHECK(!hub.addTmc2209(1, "PA_15", 0.11f, 0, 0, 16, true, 0));        /* current */
        CHECK(!hub.addTmc2209(1, "PA_15", 0.11f, 0, 2500, 16, true, 0));
        CHECK(!hub.addTmc2209(1, "PA_15", 0.11f, 4, 800, 16, true, 0));      /* address */
        CHECK(!hub.addTmc2209(1, "PA_15", 0.0f, 0, 800, 16, true, 0));       /* Rsense */
        CHECK(!hub.addTmc2209(1, "PD_1", 0.11f, 0, 800, 16, true, 0));       /* stepgen pin */
        CHECK(!hub.addTmc2209(1, "XA_15", 0.11f, 0, 800, 16, true, 0));
        CHECK(hub.addTmc2209(1, "PA_15", 0.11f, 0, 800, 16, true, 0));
        CHECK(!hub.addTmc2209(1, "PA_15", 0.11f, 1, 800, 16, true, 0));      /* one driver per UART pin */
        CHECK(!hub.addDigitalPin(1, "PA_15", true, false, 0, false, false, 1));
        CHECK(hub.addTmc2209(1, "PA_14", 0.11f, 0, 800, 1, false, 100));
        CHECK(!hub.addTmc2209(1, "PA_13", 0.11f, 0, 800, 16, true, 0));      /* 2 per node */
        /* the node checks the record too: 32 microsteps as 2^9 */
        Node a(1, 2, 3);
        RemoraCAN hc2; CanHub hub2(&hc2);
        hub2.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        CHECK(hub2.addTmc2209(1, "PB_0", 0.11f, 0, 800, 16, true, 0));       /* CAN RX: rejected by the node */
        run(hub2, {&a}, now, 3000);
        CHECK(a.node->getState() == NODE_STATE_PREOP && hub2.runningMask() == 0 && a.node->getTmcCount() == 0);
    }
    printf("%s\n", fails ? "FAILED" : "passed");
    return fails != 0;
}
