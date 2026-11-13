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
    void wait(uint32_t ticks) override { t += ticks; }          // the interrupt takes that much longer
};

static const uint32_t MARGIN = 64;          // the timer driver: the next interrupt this long after the last one

struct Rig
{
    SimPins pins;
    StepScheduler s;
    volatile int32_t fb[STEPSCHED_JOINTS] = {};
    uint64_t t = 0;
    uint32_t programmed = 1;
    uint64_t nextServo = 0;
    uint32_t block = 0;                      // the next interrupt this many ticks late (a blocking higher priority)
    uint32_t isr = 0;                        // ticks the interrupt's code takes (the CPU's speed)
    uint64_t isrTotal = 0;                   // ticks spent in the interrupt

    Rig(uint32_t maxRate = 100000, uint32_t pulseNs = PULSE_NS) : s(pins, HZ, pulseNs, DIR_NS, maxRate) {}

    // one timer interrupt at time t; the time to the next one, as the timer driver sets it
    uint32_t interrupt()
    {
        pins.t = t;
        uint32_t d = s.onTimer(programmed);
        uint64_t end = pins.t + isr;          // wait() moved pins.t
        isrTotal += end - t;
        if (t + d < end + MARGIN) d = (uint32_t)(end + MARGIN - t);
        return d;
    }

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
            uint32_t d = interrupt();
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
        // edges of joints due within the slack come in one interrupt, up to STEPSCHED_SLACK_TICKS early,
        // and an edge due while the interrupt holds another joint's pulse comes after it
        // (the rhythm stays on the scheduled times, so the error does not add up)
        const int64_t tol = STEPSCHED_SLACK_TICKS + PULSE + MARGIN;
        for (size_t i = 1; i < up.size(); i++)
        {
            int64_t e = (int64_t)(up[i] - up[i - 1]) - want;
            if (e > tol || e < -tol) ok = false;
        }
        if (up.size() && std::abs((int64_t)(up.back() - up.front()) - (int64_t)want * (int64_t)(up.size() - 1)) > tol)
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
    r.run(100000, [&](StepScheduler& s, uint64_t) { s.command(0, 30000, true); });
    int32_t ideal = r.fb[0];
    Rig q;
    q.s.addJoint(0, &q.fb[0]);
    uint64_t end = q.t + 100000ull * 64;
    while (q.t < end)
    {
        if (q.t >= q.nextServo) { q.s.command(0, 30000, true); q.nextServo += HZ / 1000; }
        uint32_t d = q.interrupt();
        if (++n % 7 == 0 && d < 40 * 64) d = 40 * 64;
        q.programmed = d;
        q.t += d;
    }
    printf("  30 kHz, every 7th interrupt 40 us late: %d steps (on time %d), latest edge %u ticks\n",
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

// a joint standing still longer than 2^31 ticks (33.5 s at 64 MHz), then started with a slow command
// first, as LinuxCNC's homing does: the old last step / direction change must not hold it back
// (it waited up to 33 s: no steps, a following error)
static void standstill()
{
    for (int reverse = 0; reverse < 2; reverse++)
    {
        for (double idleS : {10.0, 40.0, 60.0})
        {
            Rig r;
            r.s.addJoint(0, &r.fb[0]);
            r.run(200000, [](StepScheduler& s, uint64_t) { s.command(0, 1000, true); });     // 0.2 s forward
            r.run((uint64_t)(idleS * 1e6), [](StepScheduler& s, uint64_t) { s.command(0, 0, true); });
            int32_t sign = reverse ? -1 : 1;
            int32_t before = r.s.getCount(0);
            r.run(7000, [&](StepScheduler& s, uint64_t) { s.command(0, sign * 70, true); });
            r.run(300000, [&](StepScheduler& s, uint64_t) { s.command(0, sign * 1000, true); });
            int32_t moved = (r.s.getCount(0) - before) * sign;
            printf("  standing %.0f s, then %s: %d steps (about 300)\n", idleS, reverse ? "reverse" : "forward", moved);
            CHECK(moved >= 295 && moved <= 305);
        }
    }
}


static void decel()
{
    // 20 kHz -> 0 over 100 ms: the steps of the integral, and it stands at the end
    Rig r;
    r.s.addJoint(0, &r.fb[0]);
    r.run(50000, [](StepScheduler& s, uint64_t) { s.command(0, 20000, true); });
    int32_t at = r.fb[0];
    r.run(100000, [](StepScheduler& s, uint64_t ms) { s.command(0, (int32_t)(20000 - (ms - 50) * 200), true); });
    printf("  20 kHz to 0 in 100 ms: %d steps (integral 1010)\n", (int)(r.fb[0] - at));
    CHECK(std::abs(r.fb[0] - at - 1010) <= 2);
    int32_t end = r.fb[0];
    r.run(20000, [](StepScheduler& s, uint64_t) { s.command(0, 0, true); });
    CHECK(r.fb[0] == end && !r.pins.level[0] && r.pins.bad == 0);
}

// a pulse too long to wait for in the interrupt (an external driver, 20 us): an interrupt for each edge
static void longPulse()
{
    Rig r(100000, 20000);
    r.s.addJoint(0, &r.fb[0]);
    CHECK(!r.s.getInlinePulse());
    r.run(100000, [](StepScheduler& s, uint64_t ms) { s.command(0, ms < 50 ? 10000 : -10000, true); });
    std::vector<uint64_t> up = rises(r.pins, 0);
    printf("  20 us pulse, +10 kHz / -10 kHz: %zu steps, position %d, %u interrupts\n", up.size(), (int)r.fb[0],
           (unsigned)r.s.events);
    CHECK(up.size() >= 998 && up.size() <= 1001 && std::abs(r.fb[0]) <= 1);
    CHECK(minPulse(r.pins, 0, true) == 20 * 64 && minPulse(r.pins, 0, false) >= 20 * 64 && r.pins.bad == 0);
    CHECK(r.s.events >= 2 * up.size());
}

// the M8P at a 190 mm/s CoreXY move: two motors at 15.2 kHz, the interrupt's code 6 us on the M0+.
// One interrupt per step (the pulse ends in it), the commands of a deceleration are followed
static void load()
{
    Rig r;
    r.isr = 6 * 64;
    for (int j = 0; j < 5; j++) r.s.addJoint((uint8_t)j, &r.fb[j]);
    CHECK(r.s.getInlinePulse());
    r.run(300, [](StepScheduler& s, uint64_t) { s.command(0, 15170, true); });       // the two out of phase
    r.run(999700, [](StepScheduler& s, uint64_t) { s.command(0, 15170, true); s.command(1, 15170, true); });
    uint32_t steps = r.s.steps;
    unsigned share = (unsigned)(r.isrTotal * 100 / (1000000ull * 64));
    printf("  2 x 15.17 kHz for 1 s: %u steps, %u interrupts, %u %% of the CPU in the interrupt, latest edge %u ticks\n",
           (unsigned)steps, (unsigned)r.s.events, share, (unsigned)r.s.lateMax);
    CHECK(std::abs((int)r.fb[0] - 15170) <= 4 && std::abs((int)r.fb[1] - 15170) <= 20);
    CHECK(r.s.events <= steps + 100);                               // not two per step
    CHECK(share < 35);
    CHECK(minPulse(r.pins, 0, true) == PULSE && minPulse(r.pins, 1, false) >= PULSE && r.pins.bad == 0);
    // LinuxCNC's deceleration, 15170 -> 0 in 60 ms
    int32_t at = r.fb[0];
    uint64_t t0 = r.nextServo / (HZ / 1000);
    r.run(80000, [&](StepScheduler& s, uint64_t ms)
    {
        int32_t f = ms - t0 < 60 ? (int32_t)(15170 - (ms - t0) * 15170 / 60) : 0;
        s.command(0, f, true); s.command(1, f, true);
    });
    printf("  then to 0 in 60 ms: %d steps (integral 463, and up to 1 ms at 15 kHz before the first command)\n",
           (int)(r.fb[0] - at));
    CHECK(r.fb[0] - at >= 460 && r.fb[0] - at <= 480);
    int32_t end = r.fb[0];
    r.run(10000, [](StepScheduler& s, uint64_t) { s.command(0, 0, true); s.command(1, 0, true); });
    CHECK(r.fb[0] == end);
}

int main(int argc, char** argv)
{
    struct { const char* name; void (*fn)(); } tests[] = {
        {"constant", constantRate}, {"high-rate", highRate}, {"ramp", ramp}, {"reverse", reverse},
        {"stop", stopAndDisable}, {"three-joints", threeJoints}, {"max-rate", maxRate},
        {"late", lateInterrupt}, {"idle", idle}, {"standstill", standstill},
        {"decel", decel}, {"long-pulse", longPulse}, {"load", load},
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
