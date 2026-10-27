#include "CanHub.h"
#include "CanNode.h"
#include <string>
std::vector<RemoraCAN*> RemoraCAN::bus;
uint32_t sim_uid[3];

static int fails;
#define CHECK(c) do { if (c) printf("  ok   %s\n", #c); else { printf("  FAIL %s\n", #c); fails++; } } while (0)

static std::string hex(const uint8_t u[6]) { char b[13]; for (int i = 0; i < 6; i++) sprintf(b + 2 * i, "%02x", u[i]); return b; }

struct Node { RemoraCAN can; CanNode* node; Node(uint32_t a, uint32_t b, uint32_t c) { sim_uid[0] = a; sim_uid[1] = b; sim_uid[2] = c; node = new CanNode(&can); } };

static void run(CanHub& hub, std::vector<Node*> nodes, uint32_t& now, uint32_t ms, bool quiet = false) {
    for (uint32_t t = 0; t < ms; t++, now++) {
        hub.service(now, quiet);
        for (Node* n : nodes) n->node->service(now);
    }
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
        CHECK(a.node->getNode() == 3 && a.node->getState() == NODE_STATE_PREOP);
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
        int sent = hc.sent;
        run(hub, {&a, &b}, now, 3000, true);
        CHECK(hc.sent == sent);                                /* no IDENTIFY traffic once all nodes are online */
    }
    else if (sc == "unknown") {
        Node a(1, 2, 3), u(7, 8, 9);
        RemoraCAN hc; CanHub hub(&hc);
        hub.addNode(hex(a.node->getUuid()).c_str(), 1, "toolhead");
        run(hub, {&a, &u}, now, 1500);
        CHECK(a.node->getNode() == 1 && u.node->getNode() == 0);
        CHECK(hub.aliveMask() == 0x1);
        printf("  (unknown node UUID %s should be printed above, again every 5 s)\n", hex(u.node->getUuid()).c_str());
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
    printf("%s\n", fails ? "FAILED" : "passed");
    return fails != 0;
}
