#include "Ws2812.h"

#include "pinmap.h"
#include "PeripheralPins.h"

#define WS2812_DMA_CHANNELS     5           // DMA2 on the G0B1
#define WS2812_BIT_NS           1250
#define WS2812_T0H_NS           344         // WS2812B 250 ... 550, SK6812 150 ... 450
#define WS2812_T1H_NS           719         // WS2812B 650 ... 950, SK6812 450 ... 750
#define WS2812_RESET_MS         2           // > 280 us low before the next transfer (ms tick resolution)

static TIM_TypeDef* claimedTimer[WS2812_DMA_CHANNELS];      // per DMA2 channel, nullptr: free


// the timers with a DMA update request, their DMAMUX request and whether they have the main output enable
static bool timerInfo(TIM_TypeDef* tim, uint32_t* request, bool* moe)
{
    *moe = false;
    if (tim == TIM1) { *request = DMA_REQUEST_TIM1_UP; *moe = true; return true; }
    if (tim == TIM3) { *request = DMA_REQUEST_TIM3_UP; return true; }
#ifdef TIM4
    if (tim == TIM4) { *request = DMA_REQUEST_TIM4_UP; return true; }
#endif
#ifdef TIM15
    if (tim == TIM15) { *request = DMA_REQUEST_TIM15_UP; *moe = true; return true; }
#endif
    if (tim == TIM16) { *request = DMA_REQUEST_TIM16_UP; *moe = true; return true; }
    if (tim == TIM17) { *request = DMA_REQUEST_TIM17_UP; *moe = true; return true; }
    return false;                           // TIM2: the mbed us ticker
}


static void timerClock(TIM_TypeDef* tim, bool on)
{
    if (tim == TIM1) { if (on) __HAL_RCC_TIM1_CLK_ENABLE(); else __HAL_RCC_TIM1_CLK_DISABLE(); }
    else if (tim == TIM3) { if (on) __HAL_RCC_TIM3_CLK_ENABLE(); else __HAL_RCC_TIM3_CLK_DISABLE(); }
#ifdef TIM4
    else if (tim == TIM4) { if (on) __HAL_RCC_TIM4_CLK_ENABLE(); else __HAL_RCC_TIM4_CLK_DISABLE(); }
#endif
#ifdef TIM15
    else if (tim == TIM15) { if (on) __HAL_RCC_TIM15_CLK_ENABLE(); else __HAL_RCC_TIM15_CLK_DISABLE(); }
#endif
    else if (tim == TIM16) { if (on) __HAL_RCC_TIM16_CLK_ENABLE(); else __HAL_RCC_TIM16_CLK_DISABLE(); }
    else if (tim == TIM17) { if (on) __HAL_RCC_TIM17_CLK_ENABLE(); else __HAL_RCC_TIM17_CLK_DISABLE(); }
}


static bool timerClaimed(TIM_TypeDef* tim)
{
    for (int i = 0; i < WS2812_DMA_CHANNELS; i++) if (claimedTimer[i] == tim) return true;
    return false;
}


// the pin's PWM map entries, the ALTx variants too (e.g. PB_15: TIM1_CH3N, TIM15_CH1N, TIM15_CH2)
static const PinMap* findChannel(PinName pin, bool freeOnly)
{
    for (const PinMap* map = PinMap_PWM; map->pin != NC; map++)
    {
        if ((map->pin & 0xFF) != (pin & 0xFF)) continue;
        uint32_t request;
        bool moe;
        TIM_TypeDef* tim = (TIM_TypeDef*)map->peripheral;
        if (!timerInfo(tim, &request, &moe)) continue;
        if (freeOnly && (timerClaimed(tim) || (tim->CR1 & TIM_CR1_CEN))) continue;     // running: a PWM module
        return map;
    }
    return nullptr;
}


bool Ws2812::capable(PinName pin)
{
    return findChannel(pin, false) != nullptr;
}


Ws2812* Ws2812::create(PinName pin, int leds, int bytesPerLed, const char* who)
{
    if (leds < 1 || bytesPerLed < 3 || bytesPerLed > 4) return nullptr;

    const PinMap* map = findChannel(pin, true);
    if (!map)
    {
        printf("%s: pin is no free timer channel (TIM1/3/4/15/16/17, not used by a hardware PWM)\n", who);
        return nullptr;
    }
    int k = 0;
    while (k < WS2812_DMA_CHANNELS && claimedTimer[k]) k++;
    if (k == WS2812_DMA_CHANNELS)
    {
        printf("%s: more than %d LED chains\n", who, WS2812_DMA_CHANNELS);
        return nullptr;
    }

    Ws2812* s = new Ws2812();
    s->tim = (TIM_TypeDef*)map->peripheral;
    s->pin = pin;
    s->leds = leds;
    s->bytesPerLed = bytesPerLed;
    s->length = leds * bytesPerLed * 8 + 2;
    s->buffer = new uint16_t[s->length];
    memset(s->buffer, 0, s->length * sizeof(uint16_t));
    s->running = false;
    s->doneTick = HAL_GetTick() - WS2812_RESET_MS;
    s->conflictReported = false;
    s->dmaIndex = k + 1;
    s->dma = (DMA_Channel_TypeDef*)(DMA2_Channel1_BASE + 0x14 * k);
    claimedTimer[k] = s->tim;

    // timer clock = SystemCoreClock (APB prescaler 1)
    uint32_t clk = SystemCoreClock;
    s->period = (uint16_t)(clk / (1000000000u / WS2812_BIT_NS));
    s->bit0 = (uint16_t)((uint64_t)clk * WS2812_T0H_NS / 1000000000u);
    s->bit1 = (uint16_t)((uint64_t)clk * WS2812_T1H_NS / 1000000000u);

    uint32_t channel = STM_PIN_CHANNEL(map->function);         // 1 ... 4
    bool inverted = STM_PIN_INVERTED(map->function);           // CHxN output
    uint32_t request;
    bool moe;
    timerInfo(s->tim, &request, &moe);
    s->ccr = &s->tim->CCR1 + (channel - 1);

    TIM_TypeDef* t = s->tim;
    timerClock(t, true);
    t->CR1 = 0;
    t->PSC = 0;
    t->ARR = s->period - 1;
    *s->ccr = 0;                            // low until the first transfer
    uint32_t mode = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;   // PWM mode 1, compare preload
    if (channel == 1) t->CCMR1 = (t->CCMR1 & ~0x0001FFFFu) | mode;
    else if (channel == 2) t->CCMR1 = (t->CCMR1 & ~0x01FFFF00u) | (mode << 8);
    else if (channel == 3) t->CCMR2 = (t->CCMR2 & ~0x0001FFFFu) | mode;
    else t->CCMR2 = (t->CCMR2 & ~0x01FFFF00u) | (mode << 8);
    uint32_t shift = 4 * (channel - 1);
    t->CCER = (t->CCER & ~(0xFu << shift)) | ((inverted ? TIM_CCER_CC1NE : TIM_CCER_CC1E) << shift);
    if (moe) t->BDTR |= TIM_BDTR_MOE;
    t->DIER = TIM_DIER_UDE;                 // a DMA request at every update: the next compare value
    t->EGR = TIM_EGR_UG;
    t->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    __HAL_RCC_DMA2_CLK_ENABLE();
    s->dma->CCR = 0;
    (DMAMUX1_Channel0 + 7 + k)->CCR = request;      // DMAMUX channels 7 ... 11 are DMA2 1 ... 5
    s->dma->CPAR = (uint32_t)s->ccr;

    pin_function(pin, map->function);       // alternate function last, the output is low already

    printf("%s: %d LEDs on TIM%s channel %d%s, DMA2 channel %d\n", who, leds,
           t == TIM1 ? "1" : t == TIM3 ? "3" : t == TIM16 ? "16" : t == TIM17 ? "17" :
#ifdef TIM15
           t == TIM15 ? "15" :
#endif
           "4", (int)channel, inverted ? "N" : "", s->dmaIndex);
    return s;
}


bool Ws2812::busy()
{
    if (this->running && this->dma->CNDTR == 0)
    {
        // the last compare value (0) is loaded at the next update: the line is low from now on
        this->running = false;
        this->doneTick = HAL_GetTick();
    }
    return this->running;
}


bool Ws2812::send(const uint8_t* bytes)
{
    if (this->busy() || (HAL_GetTick() - this->doneTick) < WS2812_RESET_MS) return false;

    // a hardware PWM module set the same timer up after us: its frequency, our bits would be wrong
    if (this->tim->ARR != (uint32_t)(this->period - 1))
    {
        if (!this->conflictReported) printf("RGB: the timer of the LED chain is used by another module\n");
        this->conflictReported = true;
        return true;                        // nothing to retry
    }

    uint16_t* b = this->buffer;
    for (int i = 0; i < this->leds * this->bytesPerLed; i++)
    {
        uint8_t v = bytes[i];
        for (int k = 7; k >= 0; k--) *b++ = (v >> k) & 1 ? this->bit1 : this->bit0;
    }
    *b++ = 0;
    *b = 0;

    this->dma->CCR = 0;
    DMA2->IFCR = 0xFu << (4 * (this->dmaIndex - 1));
    this->dma->CMAR = (uint32_t)this->buffer;
    this->dma->CNDTR = (uint32_t)this->length;
    // memory -> timer, 16-bit buffer into the 32-bit compare register, below the SPI link DMA
    this->dma->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_PL_0 | DMA_CCR_EN;
    this->running = true;
    return true;
}


void Ws2812::release()
{
    this->dma->CCR = 0;
    *this->ccr = 0;
    this->tim->CR1 = 0;
    this->tim->DIER = 0;
    this->tim->CCER = 0;
    timerClock(this->tim, false);
    claimedTimer[this->dmaIndex - 1] = nullptr;
    delete[] this->buffer;
    this->buffer = nullptr;
}
