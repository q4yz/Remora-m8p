// PC test of the SPI link logic (drivers/remoraLink/RemoraLink.cpp) against a simulated SPI / DMA
#include "RemoraLink.h"

#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

volatile rxData_t rxData;
volatile txData_t txData;

static int fails;
#define CHECK(c) do { if (c) printf("  ok   %s\n", #c); else { printf("  FAIL %s\n", #c); fails++; } } while (0)

// the SPI / DMA / NSS side of one board
struct FakeHw : LinkHardware
{
    rxData_t dma;                       // RX DMA target
    uint32_t remaining = SPI_BUFF_SIZE; // DMA counter when no sequence is queued
    std::deque<uint32_t> counter;       // DMA counter values, one per read
    bool nss = true;
    uint32_t now = 0;
    int rearms = 0, restarts = 0, stops = 0;

    uint32_t dmaRemaining() override
    {
        if (counter.empty()) return remaining;
        uint32_t v = counter.front();
        counter.pop_front();
        return v;
    }
    bool nssHigh() override { return nss; }
    void rearm() override { rearms++; }
    void restart() override { restarts++; }
    void stop() override { stops++; }
    uint32_t millis() override { return now; }
};

struct Rig
{
    FakeHw hw;
    RemoraLink link;
    Rig() : link(hw, &rxData, hw.dma.rxBuffer) { memset((void*)&rxData, 0, sizeof(rxData)); }

    // the host clocks one complete frame, NSS goes high
    void frame(int32_t header, int32_t cmd0 = 0)
    {
        memset(hw.dma.rxBuffer, 0xA5, SPI_BUFF_SIZE);
        hw.dma.header = header;
        hw.dma.jointFreqCmd[0] = cmd0;
        link.frameEnd();
    }
};

static void writeFrame()
{
    Rig r;
    r.frame(PRU_WRITE, 1234);
    CHECK(rxData.jointFreqCmd[0] == 1234);            // at the frame end, without the main loop
    r.link.servoUpdate();
    CHECK(r.link.getStatus());                        // the link status too
    r.link.tasks();
    CHECK(rxData.header == PRU_WRITE);
    CHECK(r.link.dbgWrites == 1 && r.link.dbgFrames == 1);
    CHECK(r.hw.rearms == 1);                          // re-armed at the frame end, bus idle
    r.link.servoUpdate();
    CHECK(r.link.getStatus());
}

static void readFrame()
{
    Rig r;
    r.frame(PRU_READ, 99);
    r.link.tasks();
    CHECK(r.link.dbgReads == 1);
    CHECK(rxData.jointFreqCmd[0] == 0);              // READ data is not used
    r.link.servoUpdate();
    CHECK(r.link.getStatus());
}

static void tornFrames()
{
    Rig r;
    r.hw.remaining = 10;                              // NSS edge in the middle of a DMA frame
    r.frame(PRU_WRITE, 1);
    r.frame(PRU_WRITE, 2);
    r.link.tasks();
    CHECK(r.link.dbgTorn == 2 && r.link.dbgWrites == 0);
    CHECK(r.hw.restarts == 0);
    r.frame(PRU_WRITE, 3);
    r.link.tasks();
    CHECK(r.link.dbgTorn == 3);
    CHECK(r.hw.restarts == 1 && r.link.dbgTimeouts == 1);   // EDGE_REALIGN torn frames in a row
    CHECK(rxData.jointFreqCmd[0] == 0);

    r.hw.remaining = SPI_BUFF_SIZE;                   // aligned again
    r.frame(PRU_WRITE, 4);
    r.link.tasks();
    CHECK(rxData.jointFreqCmd[0] == 4);
}

static void tornCountResetsOnGoodFrame()
{
    Rig r;
    r.hw.remaining = 10;
    r.frame(PRU_WRITE); r.frame(PRU_WRITE);
    r.link.tasks();
    r.hw.remaining = SPI_BUFF_SIZE;
    r.frame(PRU_WRITE);
    r.link.tasks();
    r.hw.remaining = 10;
    r.frame(PRU_WRITE); r.frame(PRU_WRITE);
    r.link.tasks();
    CHECK(r.link.dbgTorn == 4);
    CHECK(r.hw.restarts == 0);                        // never 3 in a row
}

static void nextFrameDuringCopy()
{
    Rig r;
    r.hw.counter = {SPI_BUFF_SIZE, 60};               // complete at the edge, the next frame started during the copy
    r.frame(PRU_WRITE, 7);
    r.link.tasks();
    CHECK(r.link.dbgTorn == 1 && r.link.dbgWrites == 0);
}

static void dmaSettles()
{
    Rig r;
    r.hw.counter = {1, 1, SPI_BUFF_SIZE};             // the last byte reaches memory a moment after the edge
    r.frame(PRU_WRITE, 8);
    r.link.tasks();
    CHECK(r.link.dbgTorn == 0);
    CHECK(rxData.jointFreqCmd[0] == 8);
}

static void queueFull()
{
    Rig r;
    for (int i = 1; i <= FRAME_QUEUE; i++) r.frame(PRU_WRITE, i);
    CHECK(r.link.dbgLost == 1);                       // FRAME_QUEUE - 1 slots
    CHECK(r.hw.rearms == FRAME_QUEUE);                // the dropped frame re-arms too
    r.link.tasks();
    CHECK(r.link.dbgWrites == FRAME_QUEUE - 1);
    CHECK(rxData.jointFreqCmd[0] == FRAME_QUEUE);     // the data of the frame the main loop lost was used
}

// the main loop does not run at all (the CPU is busy): LinuxCNC's commands still arrive
static void mainLoopStarved()
{
    Rig r;
    for (int i = 1; i <= 50; i++)
    {
        r.frame(PRU_WRITE, i * 100);
        r.link.servoUpdate();
    }
    CHECK(rxData.jointFreqCmd[0] == 5000);
    CHECK(r.link.getStatus());
    CHECK(r.link.dbgLost == 50 - (FRAME_QUEUE - 1));
    r.hw.remaining = 10;                              // a torn frame never reaches rxData
    r.frame(PRU_WRITE, 7);
    CHECK(rxData.jointFreqCmd[0] == 5000);
}

static void lateFrameEnd()
{
    Rig r;
    r.hw.nss = false;                                 // the next frame already started when the interrupt ran
    r.frame(PRU_WRITE, 5);
    CHECK(r.link.dbgLate == 1);
    CHECK(r.hw.rearms == 0);                          // no re-arm in the middle of a frame
    r.link.tasks();
    CHECK(rxData.jointFreqCmd[0] == 5);               // the frame itself is fine
}

static void badHeaders()
{
    Rig r;
    for (int i = 0; i < 5; i++) { r.frame(0x00646174); r.link.tasks(); }
    CHECK(!r.link.getError());
    r.frame(0x00646174);
    r.link.tasks();
    CHECK(r.link.getError());                         // more than 5
    CHECK(r.link.dbgLastBad == 0x00646174);
    for (int i = 0; i < REJECT_REALIGN - 6; i++) { r.frame(0x00646174); r.link.tasks(); }
    CHECK(r.hw.restarts == 1);                        // REJECT_REALIGN bad headers in a row
    for (int i = 0; i < REJECT_REALIGN - 1; i++) { r.frame(0xbcbcbcbc); r.link.tasks(); }
    CHECK(r.hw.restarts == 1);                        // counted from 0 again after the re-align
    r.frame(0xbcbcbcbc); r.link.tasks();
    CHECK(r.hw.restarts == 2);
    CHECK(r.link.dbgRejects == 2 * REJECT_REALIGN);
}

static void goodFrameResetsRejects()
{
    Rig r;
    for (int i = 0; i < REJECT_REALIGN - 1; i++) { r.frame(0x07070707); r.link.tasks(); }
    r.frame(PRU_READ); r.link.tasks();
    for (int i = 0; i < REJECT_REALIGN - 1; i++) { r.frame(0x07070707); r.link.tasks(); }
    CHECK(r.hw.restarts == 0);                        // isolated bad headers never re-align
}

static void idleRearm()
{
    Rig r;
    r.frame(PRU_WRITE);
    r.link.tasks();
    r.hw.now = IDLE_REARM_MS;
    r.link.tasks();
    CHECK(r.hw.restarts == 0);                        // not yet
    r.hw.now = IDLE_REARM_MS + 1;
    r.link.tasks();
    CHECK(r.hw.restarts == 1 && r.link.dbgRearms == 1);
    r.hw.now = 1000;
    r.link.tasks();
    CHECK(r.hw.restarts == 1);                        // once per quiet period
    r.frame(PRU_WRITE);
    r.link.tasks();
    r.hw.now = 1000 + IDLE_REARM_MS + 1;
    r.link.tasks();
    CHECK(r.hw.restarts == 2);                        // the next quiet period
}

static void noIdleRearmAtBoot()
{
    Rig r;
    r.hw.now = 5000;                                  // no frame since boot: already armed by start()
    r.link.tasks();
    CHECK(r.hw.restarts == 0);
}

static void linkStatus()
{
    Rig r;
    r.frame(PRU_WRITE); r.link.tasks();
    r.link.servoUpdate();
    CHECK(r.link.getStatus());
    for (int i = 0; i < DATA_ERR_MAX; i++) r.link.servoUpdate();
    CHECK(r.link.getStatus());                        // DATA_ERR_MAX periods without data are tolerated
    r.link.servoUpdate();
    CHECK(!r.link.getStatus());
    r.frame(PRU_READ); r.link.tasks();
    r.link.servoUpdate();
    CHECK(r.link.getStatus());                        // back with the next good frame
}

static void stopAndRestart()
{
    Rig r;
    r.link.stopLink();
    CHECK(r.hw.stops == 1);
    r.frame(PRU_WRITE, 11);                           // the DMA is stopped, the buffer is not a frame
    CHECK(r.hw.rearms == 0);
    r.link.tasks();
    CHECK(r.link.dbgFrames == 0 && rxData.jointFreqCmd[0] == 0);
    r.link.restartLink();
    CHECK(r.hw.restarts == 1);
    r.frame(PRU_WRITE, 12);
    r.link.tasks();
    CHECK(rxData.jointFreqCmd[0] == 12 && r.hw.rearms == 1);
}

static void realignDropsStaleFrames()
{
    Rig r;
    r.hw.remaining = 10;
    r.frame(PRU_WRITE); r.frame(PRU_WRITE); r.frame(PRU_WRITE);
    r.link.tasks();                                   // the third re-aligns
    CHECK(r.hw.restarts == 1);
    r.hw.remaining = SPI_BUFF_SIZE;
    r.frame(PRU_WRITE, 21);
    r.link.tasks();
    CHECK(r.link.dbgFrames == 4);
    CHECK(rxData.jointFreqCmd[0] == 21);
}

static void uploadFrame()
{
    Rig r;
    uint8_t buf[SPI_BUFF_SIZE];
    CHECK(!r.link.getUploadFrame(buf));
    r.frame(PRU_UPLOAD, 0x11223344);
    r.link.tasks();
    CHECK(r.link.getUploadFrame(buf));
    int32_t header, cmd;
    memcpy(&header, buf, 4);
    memcpy(&cmd, buf + 4, 4);
    CHECK(header == PRU_UPLOAD && cmd == 0x11223344);
    CHECK(!r.link.getUploadFrame(buf));               // taken once
    CHECK(rxData.header == 0);                        // never reaches rxData
}

int main(int argc, char** argv)
{
    struct { const char* name; void (*fn)(); } tests[] = {
        {"write", writeFrame}, {"read", readFrame}, {"torn", tornFrames},
        {"torn-reset", tornCountResetsOnGoodFrame}, {"copy-torn", nextFrameDuringCopy},
        {"dma-settle", dmaSettles}, {"queue-full", queueFull}, {"main-loop-starved", mainLoopStarved}, {"late", lateFrameEnd},
        {"bad-header", badHeaders}, {"reject-reset", goodFrameResetsRejects},
        {"idle-rearm", idleRearm}, {"boot", noIdleRearmAtBoot}, {"status", linkStatus},
        {"stop", stopAndRestart}, {"realign-stale", realignDropsStaleFrames}, {"upload", uploadFrame},
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
