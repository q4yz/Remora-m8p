#pragma once
/* simulated CAN bus: every frame sent by one controller is received by all others */
#include "mbed.h"
#include "CanBus.h"
#include <vector>
#include <deque>
class RemoraCAN : public CanBus {
public:
    static std::vector<RemoraCAN*> bus;
    std::deque<canFrame_t> rx;
    bool alive = true;                 /* false: unplugged, sends and receives nothing */
    int sent = 0;
    int sentLss = 0;                   /* IDENTIFY / ASSIGN frames */
    int drop = 0;                     /* lose the next n frames this controller sends */
    RemoraCAN() { bus.push_back(this); }
    bool start() override { return true; }
    bool send(const canFrame_t& f) override {
        if (!alive) return false;
        sent++;
        if (!f.extended && f.id == 0x7E5) sentLss++;
        if (drop > 0) { drop--; return true; }
        for (RemoraCAN* c : bus) if (c != this && c->alive) c->rx.push_back(f);
        return true;
    }
    bool receive(canFrame_t& f) override { if (!alive || rx.empty()) return false; f = rx.front(); rx.pop_front(); return true; }
    void service() override {}
    uint16_t getErrorCount() override { return 0; }
    void printStatus() override {}
};
