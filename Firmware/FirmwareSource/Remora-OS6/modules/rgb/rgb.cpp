#include "rgb.h"

#include "drivers/rgb/rgbColor.h"

#if defined(TARGET_STM32G0)

#include "drivers/ws2812/Ws2812.h"
#include "drivers/pin/pin.h"

#define RGB_CHAINS_MAX      4
#define RGB_REFRESH_MS      1000

static Rgb* chains[RGB_CHAINS_MAX];
static int chainCount = 0;


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

static bool segmentFromJson(Rgb* rgb, int leds, int first, int count, int sp)
{
    if (sp < 0 || sp >= VARIABLES)
    {
        printf("RGB: SP[i] %d out of range 0 ... %d\n", sp, VARIABLES - 1);
        return false;
    }
    if (first < 1 || count < 1 || first - 1 + count > leds)
    {
        printf("RGB: segment First %d Count %d outside the chain of %d LEDs\n", first, count, leds);
        return false;
    }
    ptrSetPoint[sp] = &rxData.setPoint[sp];
    return rgb->addSegment(first - 1, count, ptrSetPoint[sp]);
}


void createRgb()
{
    const char* comment = module["Comment"] | "RGB";
    const char* pinName = module["Pin"] | "";
    int leds = module["Chain Count"] | 1;
    int order = rgbOrderCode(module["Color Order"] | "GRB");
    printf("%s\n", comment);

    if (chainCount >= RGB_CHAINS_MAX)
    {
        printf("RGB: more than %d chains\n", RGB_CHAINS_MAX);
        return;
    }
    if (leds < 1 || leds > RGB_MAX_LEDS)
    {
        printf("RGB: Chain Count %d, 1 ... %d\n", leds, RGB_MAX_LEDS);
        return;
    }
    if (order < 0)
    {
        printf("RGB: Color Order '%s', one of RGB RBG GRB GBR BRG BGR\n", module["Color Order"] | "");
        return;
    }

    Pin pin(pinName, OUTPUT);               // pin name parsing only, the timer takes the pin over
    if (!pin.isValid())
    {
        printf("RGB: invalid pin '%s'\n", pinName);
        return;
    }
    Ws2812* strip = Ws2812::create(pin.pinToPinName(), leds, "RGB");
    if (!strip) return;

    Rgb* rgb = new Rgb(strip, leds, (uint8_t)order);
    JsonArray segments = module["Segments"];
    bool ok = true;
    if (segments.size() == 0)
    {
        ok = segmentFromJson(rgb, leds, 1, leds, module["SP[i]"] | -1);
    }
    for (JsonArray::iterator it = segments.begin(); ok && it != segments.end(); ++it)
    {
        JsonObject s = *it;
        ok = segmentFromJson(rgb, leds, s["First"] | 1, s["Count"] | 1, s["SP[i]"] | -1);
    }
    if (!ok)
    {
        printf("RGB: chain on %s not used\n", pinName);
        strip->release();
        delete strip;
        delete rgb;
        return;
    }

    chains[chainCount++] = rgb;
    servoThread->registerModule(rgb);
    printf("RGB: %s, %d LEDs, %s\n", pinName, leds, rgbOrderNames[order]);
}


void rgbService()
{
    for (int i = 0; i < chainCount; i++) chains[i]->service();
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

Rgb::Rgb(Ws2812* strip, int leds, uint8_t order) :
    strip(strip),
    order(order),
    leds(leds),
    segCount(0),
    dirty(true),
    lastSend(0)
{
    this->bytes = new uint8_t[leds * 3];
    for (int i = 0; i < RGB_SEGMENTS_MAX; i++) this->color[i] = 0;
}


bool Rgb::addSegment(int first, int count, volatile float* sp)
{
    if (this->segCount >= RGB_SEGMENTS_MAX)
    {
        printf("RGB: more than %d segments\n", RGB_SEGMENTS_MAX);
        return false;
    }
    this->segFirst[this->segCount] = first;
    this->segLeds[this->segCount] = count;
    this->segSp[this->segCount] = sp;
    this->segCount++;
    return true;
}


void Rgb::update()
{
    // off while LinuxCNC is not there (as the outputs in their safe state)
    for (int i = 0; i < this->segCount; i++)
    {
        uint32_t c = commsLost ? 0 : rgbFromSetPoint(*this->segSp[i]);
        if (c != this->color[i])
        {
            this->color[i] = c;
            this->dirty = true;
        }
    }
}


void Rgb::service()
{
    uint32_t now = HAL_GetTick();
    if (!this->dirty && (now - this->lastSend) < RGB_REFRESH_MS) return;

    this->dirty = false;                    // first: a change during the build sends again
    memset(this->bytes, 0, this->leds * 3);
    for (int i = 0; i < this->segCount; i++)
    {
        uint8_t b[3];
        rgbBytes(this->color[i], this->order, b);
        for (int k = this->segFirst[i]; k < this->segFirst[i] + this->segLeds[i]; k++)
        {
            memcpy(&this->bytes[3 * k], b, 3);
        }
    }
    if (this->strip->send(this->bytes)) this->lastSend = now;
    else this->dirty = true;                // busy: next loop pass
}

#else

void createRgb()
{
    printf("RGB: not supported on this board yet\n");
}

void rgbService() {}

#endif
