#include "configuration.h"

#if REMORA_DEBUG && defined(TARGET_MANTA8)

#include "mbed.h"
#include "rawConsole.h"

#define RAW_BAUD    115200

static uint32_t rawConsoleClock = 0;     // SystemCoreClock the baud rate was set for

void rawConsoleInit(void)
{
    // (re)configure when the core clock changed, the first message may come before the PLL is up
    if (rawConsoleClock == SystemCoreClock) return;
    if (rawConsoleClock) while (!(USART4->ISR & USART_ISR_TC)) {}

    // PE8 as USART4_TX (alternate function 4)
    RCC->IOPENR |= RCC_IOPENR_GPIOEEN;
    GPIOE->MODER = (GPIOE->MODER & ~(3U << (8 * 2))) | (2U << (8 * 2));
    GPIOE->OSPEEDR |= (3U << (8 * 2));
    GPIOE->AFR[1] = (GPIOE->AFR[1] & ~(0xFU << ((8 - 8) * 4))) | (4U << ((8 - 8) * 4));

    // USART4 on PCLK (= SystemCoreClock, APB prescaler 1), transmitter only
    RCC->APBENR1 |= RCC_APBENR1_USART4EN;
    USART4->CR1 = 0;
    USART4->BRR = (SystemCoreClock + RAW_BAUD / 2) / RAW_BAUD;
    USART4->CR1 = USART_CR1_TE | USART_CR1_UE;

    rawConsoleClock = SystemCoreClock;
}

static void rawPutc(char c)
{
    while (!(USART4->ISR & USART_ISR_TXE_TXFNF)) {}
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
    while (!(USART4->ISR & USART_ISR_TC)) {}
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

// first sign of life, before any static constructor of the application runs
__attribute__((constructor(101))) static void rawConsoleEarly(void)
{
    rawConsolePuts("\n[raw] init\n");
}

#endif
