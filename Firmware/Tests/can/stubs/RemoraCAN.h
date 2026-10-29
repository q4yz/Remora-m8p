#pragma once
/* simulated CAN bus: every frame sent by one controller is received by all others */
#include "mbed.h"
#include <vector>
#include <deque>
typedef struct { uint32_t id; bool extended; uint8_t len; uint8_t data[8]; } canFrame_t;
class RemoraCAN {
public:
    static std::vector<RemoraCAN*> bus;
    std::deque<canFrame_t> rx;
    bool alive = true;                 /* false: unplugged, sends and receives nothing */
    int sent = 0;
    int sentLss = 0;                   /* IDENTIFY / ASSIGN frames */
    int drop = 0;                     /* lose the next n frames this controller sends */
    RemoraCAN() { bus.push_back(this); }
    bool start() { return true; }
    bool send(const canFrame_t& f) {
        if (!alive) return false;
        sent++;
        if (!f.extended && f.id == 0x7E5) sentLss++;
        if (drop > 0) { drop--; return true; }
        for (RemoraCAN* c : bus) if (c != this && c->alive) c->rx.push_back(f);
        return true;
    }
    bool receive(canFrame_t& f) { if (!alive || rx.empty()) return false; f = rx.front(); rx.pop_front(); return true; }
    void service() {}
    uint16_t getErrorCount() { return 0; }
    void printStatus() {}
};
