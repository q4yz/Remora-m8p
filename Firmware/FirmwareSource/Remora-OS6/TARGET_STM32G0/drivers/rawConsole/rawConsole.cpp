#include "configuration.h"

#if REMORA_DEBUG && defined(TARGET_MANTA8)

#include "mbed.h"
#include "rawConsole.h"

#define RAW_BAUD    115200

static uint32_t rawConsoleClock = 0;     // SystemCoreClock the baud rate was set for

static void rawDelay(uint32_t loops)
{
    while (loops--) __NOP();
}

void rawConsoleInit(void)
{
    // (re)configure when the core clock changed, the first message may come before the PLL is up
    if (rawConsoleClock == SystemCoreClock) return;
    if (rawConsoleClock) while (!(USART4->ISR & USART_ISR_TC)) {}

    // PE8 as USART4_TX: alternate function 0 on the STM32G0B1 (AF4 is USART4 on PA0, not PE8)
    RCC->IOPENR |= RCC_IOPENR_GPIOEEN;
    GPIOE->AFR[1] = (GPIOE->AFR[1] & ~(0xFU << ((8 - 8) * 4))) | (0U << ((8 - 8) * 4));
    GPIOE->OSPEEDR |= (3U << (8 * 2));
    GPIOE->MODER = (GPIOE->MODER & ~(3U << (8 * 2))) | (2U << (8 * 2));

    // USART4 on PCLK (= SystemCoreClock, APB prescaler 1), transmitter only. Take it out of
    // reset and clear CR2/CR3 in case the bootloader left it in reset or reconfigured
    // (inverted or swapped TX would also leave the pin silent)
    RCC->APBRSTR1 &= ~RCC_APBRSTR1_USART4RST;
    RCC->APBENR1 |= RCC_APBENR1_USART4EN;
    (void)RCC->APBENR1;
    USART4->CR1 = 0;
    USART4->CR2 = 0;
    USART4->CR3 = 0;
    USART4->PRESC = 0;
    USART4->BRR = (SystemCoreClock + RAW_BAUD / 2) / RAW_BAUD;
    USART4->CR1 = USART_CR1_TE | USART_CR1_UE;

    rawConsoleClock = SystemCoreClock;
}

static void rawPutc(char c)
{
    // bounded wait: if the USART never gets ready, carry on instead of hanging forever
    uint32_t timeout = 1000000;
    while (!(USART4->ISR & USART_ISR_TXE_TXFNF) && --timeout) {}
    USART4->TDR = (uint8_t)c;
}

void rawConsolePuts(const char *s)
{
    rawConsoleInit();
    while (*s)
    {
        if (*s == '\n') rawPutc('\r');
        rawPutc(*s++);
    }
    uint32_t timeout = 1000000;
    while (!(USART4->ISR & USART_ISR_TC) && --timeout) {}
}

void rawHeartbeat(void)
{
    // EXP2 pin 5 (PE7, free) toggles every 0.5 s from the main loop, independent of the UART
    static bool init = false;
    static uint32_t last = 0;

    if (!init)
    {
        RCC->IOPENR |= RCC_IOPENR_GPIOEEN;
        GPIOE->MODER = (GPIOE->MODER & ~(3U << (7 * 2))) | (1U << (7 * 2));
        init = true;
    }

    if ((HAL_GetTick() - last) >= 500)
    {
        last = HAL_GetTick();
        GPIOE->ODR ^= (1U << 7);
    }
}


// stdout / stderr through the raw console instead of the mbed serial driver
class RawConsole : public mbed::FileHandle
{
    public:

        ssize_t write(const void *buffer, size_t size) override
        {
            const char *p = (const char *)buffer;
            rawConsoleInit();
            for (size_t i = 0; i < size; i++)
            {
                if (p[i] == '\n') rawPutc('\r');
                rawPutc(p[i]);
            }
            return size;
        }

        ssize_t read(void *buffer, size_t size) override { (void)buffer; (void)size; return 0; }
        off_t seek(off_t offset, int whence = SEEK_SET) override { (void)offset; (void)whence; return -ESPIPE; }
        int close() override { return 0; }
        int isatty() override { return 1; }
};

namespace mbed
{
    FileHandle *mbed_override_console(int fd)
    {
        (void)fd;
        static RawConsole console;
        return &console;
    }
}

// First signs of life, before any static constructor of the application runs:
//  1. PE8 (EXP2 pin 7) as a plain GPIO, ~0.3 s square wave burst: the core runs and the pin is right
//  2. PE8 as USART4 TX, ~0.1 s of 'U' (0x55 = square wave at half the baud rate), then "[raw] init"
__attribute__((constructor(101))) static void rawConsoleEarly(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOEEN;
    GPIOE->MODER = (GPIOE->MODER & ~(3U << (8 * 2))) | (1U << (8 * 2));
    for (int i = 0; i < 600; i++)
    {
        GPIOE->ODR ^= (1U << 8);
        rawDelay(SystemCoreClock / 8000);       // roughly 0.5 ms per edge, whatever the clock
    }
    GPIOE->BSRR = (1U << 8);                    // leave it high (UART idle level)

    rawConsoleInit();
    for (int i = 0; i < 1100; i++) rawPutc('U');
    rawConsolePuts("\n[raw] init\n");
}

#endif
