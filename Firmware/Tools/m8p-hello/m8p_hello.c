/*
 * m8p_hello: minimal bring-up test for the BTT Manta M8P V1.x (STM32G0B1VET6) behind the
 * BTT SD card bootloader. No mbed, no libraries, about 1 KB, linked at 0x08002000.
 *
 * After the bootloader jumps to it:
 *   1. EXP2 pin 7 (PE8) as a plain GPIO: ~0.3 s square wave burst (~1 kHz)
 *   2. EXP2 pin 7 (PE8) as USART4 TX (AF0), 115200 8N1: "UUUU..." then "M8P hello N" every 0.5 s
 *   3. EXP2 pin 5 (PE7) toggles every 0.5 s
 * The core stays on the 16 MHz HSI, the bootloader's clock settings are not relied on.
 *
 * Build:  make            Flash: copy m8p_hello.bin to the MCU SD card as firmware.bin, reset
 */

#include <stdint.h>

#define REG(addr)       (*(volatile uint32_t *)(addr))

#define RCC_BASE        0x40021000u
#define RCC_CR          REG(RCC_BASE + 0x00)
#define RCC_CFGR        REG(RCC_BASE + 0x08)
#define RCC_IOPENR      REG(RCC_BASE + 0x34)
#define RCC_APBRSTR1    REG(RCC_BASE + 0x2C)
#define RCC_APBENR1     REG(RCC_BASE + 0x3C)

#define GPIOE_BASE      0x50001000u
#define GPIOE_MODER     REG(GPIOE_BASE + 0x00)
#define GPIOE_OSPEEDR   REG(GPIOE_BASE + 0x08)
#define GPIOE_ODR       REG(GPIOE_BASE + 0x14)
#define GPIOE_BSRR      REG(GPIOE_BASE + 0x18)
#define GPIOE_AFRH      REG(GPIOE_BASE + 0x24)

#define USART4_BASE     0x40004C00u
#define USART4_CR1      REG(USART4_BASE + 0x00)
#define USART4_CR2      REG(USART4_BASE + 0x04)
#define USART4_CR3      REG(USART4_BASE + 0x08)
#define USART4_BRR      REG(USART4_BASE + 0x0C)
#define USART4_ISR      REG(USART4_BASE + 0x1C)
#define USART4_TDR      REG(USART4_BASE + 0x28)
#define USART4_PRESC    REG(USART4_BASE + 0x2C)

#define IWDG_KR         REG(0x40003000u)
#define SCB_VTOR        REG(0xE000ED08u)

#define HSI_HZ          16000000u
#define BAUD            115200u

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
void Reset_Handler(void);
void Default_Handler(void);

__attribute__((section(".isr_vector"), used))
const void *vectors[48] = {
    &_estack,
    (void *)Reset_Handler,
    [2 ... 47] = (void *)Default_Handler,
};

static void delay(uint32_t loops)
{
    while (loops--) __asm volatile ("nop");
}

static void kick(void)
{
    IWDG_KR = 0xAAAA;                   // reload, in case the bootloader started the watchdog
}

static void putc_u4(char c)
{
    uint32_t timeout = 200000;
    while (!(USART4_ISR & (1u << 7)) && --timeout) {}      // TXE
    USART4_TDR = (uint8_t)c;
}

static void puts_u4(const char *s)
{
    while (*s) putc_u4(*s++);
}

static void put_num(uint32_t n)
{
    char buf[11];
    int i = 10;
    buf[i] = 0;
    do { buf[--i] = '0' + n % 10; n /= 10; } while (n && i);
    puts_u4(&buf[i]);
}

int main(void)
{
    // run from the internal 16 MHz oscillator, whatever the bootloader left
    RCC_CR |= (1u << 8);                                    // HSION
    while (!(RCC_CR & (1u << 10))) {}                       // HSIRDY
    RCC_CFGR &= ~7u;                                        // SW = HSISYS
    while (RCC_CFGR & (7u << 3)) {}                         // SWS = HSISYS

    RCC_IOPENR |= (1u << 4);                                // GPIOE

    // 1. PE8 plain GPIO square wave, PE7 output
    GPIOE_MODER = (GPIOE_MODER & ~((3u << 16) | (3u << 14))) | (1u << 16) | (1u << 14);
    for (int i = 0; i < 600; i++)
    {
        GPIOE_ODR ^= (1u << 8);
        delay(2000);                                        // ~0.5 ms at 16 MHz
        kick();
    }
    GPIOE_BSRR = (1u << 8);

    // 2. PE8 as USART4 TX, alternate function 0
    RCC_APBRSTR1 &= ~(1u << 19);                            // USART4 out of reset
    RCC_APBENR1 |= (1u << 19);                              // USART4 clock
    GPIOE_AFRH &= ~0xFu;                                    // PE8 AF0
    GPIOE_OSPEEDR |= (3u << 16);
    GPIOE_MODER = (GPIOE_MODER & ~(3u << 16)) | (2u << 16); // PE8 alternate function
    USART4_CR1 = 0;
    USART4_CR2 = 0;
    USART4_CR3 = 0;
    USART4_PRESC = 0;
    USART4_BRR = (HSI_HZ + BAUD / 2) / BAUD;
    USART4_CR1 = (1u << 3) | (1u << 0);                     // TE, UE

    for (int i = 0; i < 200; i++) putc_u4('U');
    puts_u4("\r\n\r\nm8p_hello: core running at 16 MHz HSI, USART4 PE8 AF0\r\n");

    // 3. heartbeat
    uint32_t n = 0;
    for (;;)
    {
        puts_u4("M8P hello ");
        put_num(n++);
        puts_u4("\r\n");
        GPIOE_ODR ^= (1u << 7);
        for (int i = 0; i < 100; i++) { delay(2000); kick(); }
    }
}

void Reset_Handler(void)
{
    SCB_VTOR = (uint32_t)vectors;

    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;

    main();
    for (;;) {}
}

void Default_Handler(void)
{
    // any fault: hold PE7 high, so the heartbeat stops in a recognisable state
    GPIOE_BSRR = (1u << 7);
    for (;;) kick();
}
