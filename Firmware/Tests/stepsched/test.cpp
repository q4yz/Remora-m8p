// PC test of the event driven step generation (drivers/stepScheduler) against a simulated timer
#include "StepScheduler.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

static int fails;
#define CHECK(c) do { if (c) printf("  ok   %s\n", #c); else { printf("  FAIL %s\n", #c); fails++; } } while (0)

static const uint32_t HZ = 64000000;        // the G0 timer clock
static const uint32_t PULSE_NS = 3000, DIR_NS = 5000;
static const uint32_t PULSE = HZ / 1000000 * 3, DIR_SETUP = HZ / 1000000 * 5;

// the pins of every joint, edges with their time
struct Edge { uint64_t t; bool high; };
struct SimPins : StepPins
{
    uint64_t t = 0;                          // set by the simulation before each onTimer()
    std::vector<Edge> stepLog[STEPSCHED_JOINTS];
    std::vector<std::pair<uint64_t, bool>> dirLog[STEPSCHED_JOINTS];
    bool level[STEPSCHED_JOINTS] = {};
    int bad = 0;                             // a rising edge on a high pin or the other way round
    void step(uint8_t j, bool high) override
    {
        if (!stepLog[j].empty() && high == level[j]) bad++;     // the first write sets the idle level
        level[j] = high;
        stepLog[j].push_back({t, high});
    }
    void dir(uint8_t j, bool forward) override { dirLog[j].push_back({t, forward}); }
};

struct Rig
{
    SimPins pins;
    StepScheduler s;
    volatile int32_t fb[STEPSCHED_JOINTS] = {};
    uint64_t t = 0;
    uint32_t programmed = 1;
    uint64_t nextServo = 0;
    uint32_t block = 0;                      // the next interrupt this many ticks late (a blocking higher priority)

    Rig(uint32_t maxRate = 100000) : s(pins, HZ, PULSE_NS, DIR_NS, maxRate) {}

    // run for us microseconds; servo(ms) gives the commands each servo period
    void run(uint64_t us, std::function<void(StepScheduler&, uint64_t)> servo)
    {
        uint64_t end = t + us * (HZ / 1000000);
        while (t < end)
        {
            if (t >= nextServo)
            {
                servo(s, nextServo / (HZ / 1000));
                nextServo += HZ / 1000;
            }
            pins.t = t;
            uint32_t d = s.onTimer(programmed);
            if (block) { if (d < block) d = block; block = 0; }   // the hardware: the next update not before
            programmed = d;
            t += d;
        }
    }
};

// rising edge times of joint j
static std::vector<uint64_t> rises(const SimPins& p, int j)
{
    std::vector<uint64_t> r;
    for (const Edge& e : p.stepLog[j]) if (e.high) r.push_back(e.t);
    return r;
}

static uint64_t minPulse(const SimPins& p, int j, bool high)
{
    uint64_t m = ~0ull;
    for (size_t i = 1; i < p.stepLog[j].size(); i++)
    {
        if (p.stepLog[j][i - 1].high != high) continue;
        uint64_t w = p.stepLog[j][i].t - p.stepLog[j][i - 1].t;
        if (w < m) m = w;
    }
    return m;
}

static void constantRate()
{
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    r.run(1000000, [](StepScheduler& s, uint64_t) { s.command(0, 1000, true); });     // 1 s at 1 kHz
    std::vector<uint64_t> up = rises(r.pins, 0);
    printf("  1 kHz for 1 s: %zu steps\n", up.size());
    CHECK(up.size() >= 999 && up.size() <= 1001);
    bool exact = true;
    for (size_t i = 1; i < up.size(); i++) if (up[i] - up[i - 1] != HZ / 1000) exact = false;
    CHECK(exact);                                                   // every interval 64000 ticks
    CHECK(r.fb[0] == (int32_t)up.size() && r.s.getCount(0) == (int32_t)up.size());
    CHECK(minPulse(r.pins, 0, true) == PULSE);
    CHECK(r.pins.bad == 0);
}

static void highRate()
{
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    r.run(100000, [](StepScheduler& s, uint64_t) { s.command(0, 20000, true); });     // 100 ms at 20 kHz
    std::vector<uint64_t> up = rises(r.pins, 0);
    bool exact = true;
    for (size_t i = 1; i < up.size(); i++) if (up[i] - up[i - 1] != HZ / 20000) exact = false;
    printf("  20 kHz for 100 ms: %zu steps, %u interrupts\n", up.size(), (unsigned)r.s.events);
    CHECK(up.size() >= 1999 && up.size() <= 2001 && exact);
    CHECK(minPulse(r.pins, 0, true) == PULSE && minPulse(r.pins, 0, false) >= PULSE);
}

static void ramp()
{
    // 0 -> 20 kHz over 100 ms, a new frequency every ms: steps = the integral, 1000
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    r.run(100000, [](StepScheduler& s, uint64_t ms) { s.command(0, (int32_t)(ms * 200 + 100), true); });
    printf("  ramp to 20 kHz in 100 ms: %d steps (integral 1000)\n", (int)r.fb[0]);
    CHECK(std::abs(r.fb[0] - 1000) <= 2);
    CHECK(r.pins.bad == 0 && minPulse(r.pins, 0, false) >= PULSE);
}

static void reverse()
{
    // +5 kHz for 10 ms, -5 kHz for 10 ms: back at the start, a step not before the direction setup
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    r.run(20000, [](StepScheduler& s, uint64_t ms) { s.command(0, ms < 10 ? 5000 : -5000, true); });
    printf("  +5 kHz / -5 kHz 10 ms each: position %d, %zu direction edges\n", (int)r.fb[0], r.pins.dirLog[0].size());
    CHECK(std::abs(r.fb[0]) <= 1);
    CHECK(r.pins.dirLog[0].size() == 2 && r.pins.dirLog[0].back().second == false);   // addJoint's forward, then back
    uint64_t dirAt = r.pins.dirLog[0].back().first, firstAfter = 0;
    for (uint64_t u : rises(r.pins, 0)) if (u >= dirAt) { firstAfter = u; break; }
    CHECK(firstAfter - dirAt >= DIR_SETUP);
    CHECK(r.pins.bad == 0);
}

static void stopAndDisable()
{
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    r.run(10000, [](StepScheduler& s, uint64_t ms) { s.command(0, ms < 5 ? 10000 : 0, true); });
    int32_t at = r.fb[0];
    CHECK(at >= 49 && at <= 51);
    CHECK(!r.pins.level[0]);                                        // the last pulse ended
    size_t n = r.pins.stepLog[0].size();
    r.run(5000, [](StepScheduler& s, uint64_t) { s.command(0, 10000, false); });       // disabled
    CHECK(r.pins.stepLog[0].size() == n && r.fb[0] == at);
    CHECK(r.pins.dirLog[0].size() == 1);                               // stop and disable change no direction
}

static void threeJoints()
{
    Rig r;
    for (int j = 0; j < 3; j++) r.s.addJoint((uint8_t)j, &r.fb[j]);
    const int32_t f[3] = {7000, -13000, 20000};
    r.run(100000, [&](StepScheduler& s, uint64_t) { for (int j = 0; j < 3; j++) s.command((uint8_t)j, f[j], true); });
    bool ok = true;
    for (int j = 0; j < 3; j++)
    {
        std::vector<uint64_t> up = rises(r.pins, j);
        uint32_t want = HZ / (uint32_t)std::abs(f[j]);
        // edges of joints due within the slack come in one interrupt, up to STEPSCHED_SLACK_TICKS early
        // (the rhythm stays on the scheduled times, so the error does not add up)
        for (size_t i = 1; i < up.size(); i++)
        {
            int64_t e = (int64_t)(up[i] - up[i - 1]) - want;
            if (e > STEPSCHED_SLACK_TICKS || e < -STEPSCHED_SLACK_TICKS) ok = false;
        }
        if (up.size() && (int64_t)(up.back() - up.front()) - (int64_t)want * (up.size() - 1) > STEPSCHED_SLACK_TICKS)
            ok = false;
        printf("  joint %d at %d Hz: position %d\n", j, (int)f[j], (int)r.fb[j]);
        if (std::abs(r.fb[j] - f[j] / 10) > 1) ok = false;
    }
    CHECK(ok);                                                      // each its own exact interval
    CHECK(r.pins.bad == 0);
}

static void maxRate()
{
    Rig r(100000);
    r.s.addJoint(0, &r.fb[0]);
    r.run(10000, [](StepScheduler& s, uint64_t) { s.command(0, 300000, true); });       // above the limit
    printf("  300 kHz commanded, 100 kHz limit, 10 ms: %d steps\n", (int)r.fb[0]);
    CHECK(r.s.getMinInterval() == HZ / 100000 && std::abs(r.fb[0] - 1000) <= 1);
}

static void lateInterrupt()
{
    // every 7th interrupt 40 us late (a long higher priority interrupt): no step lost, pulses kept
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    int n = 0;
    r.run(100000, [&](StepScheduler& s, uint64_t) { s.command(0, 15000, true); });
    int32_t ideal = r.fb[0];
    Rig q;
    q.s.addJoint(0, &q.fb[0]);
    uint64_t end = q.t + 100000ull * 64;
    while (q.t < end)
    {
        if (q.t >= q.nextServo) { q.s.command(0, 15000, true); q.nextServo += HZ / 1000; }
        q.pins.t = q.t;
        uint32_t d = q.s.onTimer(q.programmed);
        if (++n % 7 == 0 && d < 40 * 64) d = 40 * 64;
        q.programmed = d;
        q.t += d;
    }
    printf("  15 kHz, every 7th interrupt 40 us late: %d steps (on time %d), latest edge %u ticks\n",
           (int)q.fb[0], (int)ideal, (unsigned)q.s.lateMax);
    CHECK(std::abs(q.fb[0] - ideal) <= 2);
    CHECK(minPulse(q.pins, 0, true) >= PULSE && minPulse(q.pins, 0, false) >= PULSE && q.pins.bad == 0);
}

static void idle()
{
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    r.run(10000, [](StepScheduler& s, uint64_t) { s.command(0, 0, true); });
    printf("  idle 10 ms: %u interrupts\n", (unsigned)r.s.events);
    CHECK(r.s.events >= 99 && r.s.events <= 102);                   // every STEPSCHED_IDLE_US
    CHECK(r.pins.stepLog[0].size() == 1);                              // only addJoint's low
}

int main(int argc, char** argv)
{
    struct { const char* name; void (*fn)(); } tests[] = {
        {"constant", constantRate}, {"high-rate", highRate}, {"ramp", ramp}, {"reverse", reverse},
        {"stop", stopAndDisable}, {"three-joints", threeJoints}, {"max-rate", maxRate},
        {"late", lateInterrupt}, {"idle", idle},
    };
    for (auto& t : tests)
    {
        if (argc > 1 && std::string(argv[1]) != t.name) continue;
        printf("%s\n", t.name);
        t.fn();
    }
    printf("%s\n", fails ? "FAILED" : "PASSED");
    return fails ? 1 : 0;
}
