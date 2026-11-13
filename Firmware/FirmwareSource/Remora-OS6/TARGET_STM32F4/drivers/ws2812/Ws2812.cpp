#include "Ws2812.h"

#include "pinmap.h"
#include "PeripheralPins.h"

#define WS2812_BIT_NS           1250
#define WS2812_T0H_NS           344         // WS2812B 250 ... 550, SK6812 150 ... 450
#define WS2812_T1H_NS           719         // WS2812B 650 ... 950, SK6812 450 ... 750
#define WS2812_RESET_MS         2           // > 280 us low before the next transfer (ms tick resolution)
#define WS2812_TIMERS           5

// the timers with a DMA update request: their stream and channel (RM0390 / RM0090 DMA request mapping)
struct TimerDma
{
    TIM_TypeDef*    tim;
    DMA_TypeDef*    dma;
    uint8_t         stream;
    uint8_t         channel;
    bool            apb2;                   // the timer is on APB2 (else APB1)
    bool            moe;                    // advanced timer: main output enable
    const char*     name;
};

static const TimerDma timers[WS2812_TIMERS] = {
    { TIM1, DMA2, 5, 6, true,  true,  "TIM1" },
    { TIM2, DMA1, 1, 3, false, false, "TIM2" },
    { TIM3, DMA1, 2, 5, false, false, "TIM3" },
    { TIM4, DMA1, 6, 2, false, false, "TIM4" },
#ifdef TIM8
    { TIM8, DMA2, 1, 7, true,  true,  "TIM8" },
#else
    { nullptr, nullptr, 0, 0, false, false, "" },
#endif
};

static bool claimed[WS2812_TIMERS];


static const TimerDma* timerDma(TIM_TypeDef* tim)
{
    for (int i = 0; i < WS2812_TIMERS; i++) if (timers[i].tim && timers[i].tim == tim) return &timers[i];
    return nullptr;
}


static DMA_Stream_TypeDef* streamOf(const TimerDma* t)
{
    return (DMA_Stream_TypeDef*)((uint32_t)t->dma + 0x10u + 0x18u * t->stream);
}


static void timerClock(TIM_TypeDef* tim, bool on)
{
    if (tim == TIM1) { if (on) __HAL_RCC_TIM1_CLK_ENABLE(); else __HAL_RCC_TIM1_CLK_DISABLE(); }
    else if (tim == TIM2) { if (on) __HAL_RCC_TIM2_CLK_ENABLE(); else __HAL_RCC_TIM2_CLK_DISABLE(); }
    else if (tim == TIM3) { if (on) __HAL_RCC_TIM3_CLK_ENABLE(); else __HAL_RCC_TIM3_CLK_DISABLE(); }
    else if (tim == TIM4) { if (on) __HAL_RCC_TIM4_CLK_ENABLE(); else __HAL_RCC_TIM4_CLK_DISABLE(); }
#ifdef TIM8
    else if (tim == TIM8) { if (on) __HAL_RCC_TIM8_CLK_ENABLE(); else __HAL_RCC_TIM8_CLK_DISABLE(); }
#endif
}


// the clock of a timer: PCLK, doubled when its APB is divided
static uint32_t timerHz(bool apb2)
{
    if (apb2) return HAL_RCC_GetPCLK2Freq() * ((RCC->CFGR & RCC_CFGR_PPRE2_2) ? 2u : 1u);
    return HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1_2) ? 2u : 1u);
}


// the pin's PWM map entries, the ALTx variants too (e.g. PB_0: TIM1_CH2N, TIM3_CH3, TIM8_CH2N)
static const PinMap* findChannel(PinName pin, bool freeOnly)
{
    for (const PinMap* map = PinMap_PWM; map->pin != NC; map++)
    {
        if ((map->pin & 0xFF) != (pin & 0xFF)) continue;
        TIM_TypeDef* tim = (TIM_TypeDef*)map->peripheral;
        const TimerDma* t = timerDma(tim);
        if (!t) continue;
        if (freeOnly)
        {
            if (claimed[t - timers]) continue;
            if (tim->CR1 & TIM_CR1_CEN) continue;                   // running: a PWM module
            if (streamOf(t)->CR & DMA_SxCR_EN) continue;             // the stream is in use
        }
        return map;
    }
    return nullptr;
}


LedChain* ledChainCreate(PinName pin, int leds, int bytesPerLed, const char* who)
{
    return Ws2812::create(pin, leds, bytesPerLed, who);
}


bool ledChainCapable(PinName pin)
{
    return Ws2812::capable(pin);
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
        printf("%s: pin is no free timer channel (TIM1/2/3/4/8, not used by a hardware PWM)\n", who);
        return nullptr;
    }
    const TimerDma* td = timerDma((TIM_TypeDef*)map->peripheral);

    Ws2812* s = new Ws2812();
    s->tim = td->tim;
    s->dmaController = td->dma;
    s->streamIndex = td->stream;
    s->stream = streamOf(td);
    s->pin = pin;
    s->leds = leds;
    s->bytesPerLed = bytesPerLed;
    s->length = leds * bytesPerLed * 8 + 2;
    s->buffer = new uint32_t[s->length];
    memset(s->buffer, 0, s->length * sizeof(uint32_t));
    s->running = false;
    s->doneTick = HAL_GetTick() - WS2812_RESET_MS;
    s->conflictReported = false;
    claimed[td - timers] = true;

    uint32_t clk = timerHz(td->apb2);
    s->period = clk / (1000000000u / WS2812_BIT_NS);
    s->bit0 = (uint32_t)((uint64_t)clk * WS2812_T0H_NS / 1000000000u);
    s->bit1 = (uint32_t)((uint64_t)clk * WS2812_T1H_NS / 1000000000u);

    uint32_t channel = STM_PIN_CHANNEL(map->function);         // 1 ... 4
    bool inverted = STM_PIN_INVERTED(map->function);           // CHxN output
    s->ccr = &s->tim->CCR1 + (channel - 1);

    TIM_TypeDef* t = s->tim;
    timerClock(t, true);
    t->CR1 = 0;
    t->PSC = 0;
    t->ARR = s->period - 1;
    *s->ccr = 0;                            // low until the first transfer
    uint32_t mode = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;   // PWM mode 1, compare preload
    if (channel == 1) t->CCMR1 = (t->CCMR1 & ~0x00FFu) | mode;
    else if (channel == 2) t->CCMR1 = (t->CCMR1 & ~0xFF00u) | (mode << 8);
    else if (channel == 3) t->CCMR2 = (t->CCMR2 & ~0x00FFu) | mode;
    else t->CCMR2 = (t->CCMR2 & ~0xFF00u) | (mode << 8);
    uint32_t shift = 4 * (channel - 1);
    t->CCER = (t->CCER & ~(0xFu << shift)) | ((inverted ? TIM_CCER_CC1NE : TIM_CCER_CC1E) << shift);
    if (td->moe) t->BDTR |= TIM_BDTR_MOE;
    t->DIER = TIM_DIER_UDE;                 // a DMA request at every update: the next compare value
    t->EGR = TIM_EGR_UG;
    t->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    if (td->dma == DMA1) __HAL_RCC_DMA1_CLK_ENABLE();
    else __HAL_RCC_DMA2_CLK_ENABLE();
    s->stream->CR = 0;
    for (int i = 0; i < 1000 && (s->stream->CR & DMA_SxCR_EN); i++) {}
    s->stream->PAR = (uint32_t)s->ccr;
    s->stream->FCR = 0;                     // direct mode

    pin_function(pin, map->function);       // alternate function last, the output is low already

    printf("%s: %d LEDs on %s channel %d%s, DMA%d stream %d\n", who, leds, td->name, (int)channel,
           inverted ? "N" : "", td->dma == DMA1 ? 1 : 2, s->streamIndex);
    return s;
}


// the stream's flags: LIFCR for streams 0 ... 3, HIFCR for 4 ... 7, 6 bit groups at 0, 6, 16, 22
void Ws2812::clearFlags()
{
    static const uint8_t offset[4] = { 0, 6, 16, 22 };
    uint32_t bits = 0x3Du << offset[this->streamIndex & 3];
    if (this->streamIndex < 4) this->dmaController->LIFCR = bits;
    else this->dmaController->HIFCR = bits;
}


bool Ws2812::busy()
{
    if (this->running && this->stream->NDTR == 0)
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
    if (this->tim->ARR != this->period - 1)
    {
        if (!this->conflictReported) printf("RGB: the timer of the LED chain is used by another module\n");
        this->conflictReported = true;
        return true;                        // nothing to retry
    }

    uint32_t* b = this->buffer;
    for (int i = 0; i < this->leds * this->bytesPerLed; i++)
    {
        uint8_t v = bytes[i];
        for (int k = 7; k >= 0; k--) *b++ = (v >> k) & 1 ? this->bit1 : this->bit0;
    }
    *b++ = 0;
    *b = 0;

    this->stream->CR = 0;
    for (int i = 0; i < 1000 && (this->stream->CR & DMA_SxCR_EN); i++) {}
    this->clearFlags();
    this->stream->M0AR = (uint32_t)this->buffer;
    this->stream->NDTR = (uint32_t)this->length;
    // memory -> timer, 32 bit both, the channel of the timer's update request, below the SPI link DMA
    const TimerDma* td = timerDma(this->tim);
    this->stream->CR = ((uint32_t)td->channel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 | DMA_SxCR_MINC |
                       DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_PL_0 | DMA_SxCR_EN;
    this->running = true;
    return true;
}


void Ws2812::release()
{
    this->stream->CR = 0;
    *this->ccr = 0;
    this->tim->CR1 = 0;
    this->tim->DIER = 0;
    this->tim->CCER = 0;
    timerClock(this->tim, false);
    const TimerDma* td = timerDma(this->tim);
    if (td) claimed[td - timers] = false;
    delete[] this->buffer;
    this->buffer = nullptr;
}
