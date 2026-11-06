#ifndef RGB_H
#define RGB_H

/***********************************************************************
    "RGB" module: a WS2812 / SK6812 LED chain on a pin of the board
    (STM32G0: timer channel + DMA, see TARGET_STM32G0/drivers/ws2812)

    Each segment of the chain shows the colour of one LinuxCNC set point,
    0xRRGGBB as a number (drivers/rgb/rgbColor.h). Whole chain, one SP:

        { "Type": "RGB", "Thread": "Servo", "Comment": "case light",
          "Pin": "PA_9", "Chain Count": 10, "Color Order": "GRB", "SP[i]": 4 }

    Segments (LED numbers from 1, like Klipper's INDEX), Stealthburner:

        { "Type": "RGB", "Thread": "Servo", "Pin": "PB_15", "Chain Count": 3,
          "Segments": [ { "First": 1, "Count": 1, "SP[i]": 4 },
                        { "First": 2, "Count": 2, "SP[i]": 5 } ] }

    While LinuxCNC is not connected the LEDs are off. The chain is sent on
    a change and once a second (a glitch on the line does not stay).
************************************************************************/

#include <cstdint>

#include "modules/module.h"
#include "extern.h"

#define RGB_SEGMENTS_MAX    6           // as many as set points

void createRgb(void);
void rgbService(void);                  // main loop: send the chains that changed

#if defined(TARGET_STM32G0)

class Ws2812;

class Rgb : public Module
{
    private:

        Ws2812*         strip;
        uint8_t         order;
        int             leds;
        int             segCount;
        int             segFirst[RGB_SEGMENTS_MAX];     // 0 based
        int             segLeds[RGB_SEGMENTS_MAX];
        volatile float* segSp[RGB_SEGMENTS_MAX];
        volatile uint32_t color[RGB_SEGMENTS_MAX];      // servo thread
        volatile bool   dirty;
        uint32_t        lastSend;
        uint8_t*        bytes;

    public:

        Rgb(Ws2812* strip, int leds, uint8_t order);
        bool addSegment(int first, int count, volatile float* sp);

        virtual void update(void);      // servo thread: colours from the set points
        void service(void);             // main loop
};

#endif

#endif
