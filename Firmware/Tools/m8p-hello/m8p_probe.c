/*
 * m8p_probe: finds out where the bootloader on a BTT Manta M8P V1.x (STM32G0B1VET6) starts
 * the application, and whether the application survives the jump.
 *
 * mkprobe.py copies this program into the image at 16 file offsets (0x1000 ... 0x10000). The
 * vector at the start of the file always points to 0x08010000, so whatever the bootloader
 * size B (0 ... 60 KB in 4 KB steps), exactly one copy lands at 0x08010000 and runs. Each copy
 * has its file offset stored right behind it (ID word), so it can report B = 64 KB - offset.
 *
 * It first disables all interrupts and SysTick, in case the bootloader leaves them running.
 * Then on EXP2 pin 7 (PE8):
 *   1. GPIO square wave, ~0.3 s, ~1 kHz
 *   2. USART4 TX (AF0) at 115200 8N1 on the 16 MHz HSI: "UUUU..." then repeatedly
 *      "M8P probe: bootloader size = N KB, app at 0x0800xxxx"
 * EXP2 pin 5 (PE7) toggles every 0.5 s.
 */

#include <stdint.h>

#define REG(addr)       (*(volatile uint32_t *)(addr))

#define RCC_CR          REG(0x40021000u)
#define RCC_CFGR        REG(0x40021008u)
#define RCC_APBRSTR1    REG(0x4002102Cu)
#define RCC_IOPENR      REG(0x40021034u)
#define RCC_APBENR1     REG(0x4002103Cu)

#define GPIOE_MODER     REG(0x50001000u)
#define GPIOE_OSPEEDR   REG(0x50001008u)
#define GPIOE_ODR       REG(0x50001014u)
#define GPIOE_BSRR      REG(0x50001018u)
#define GPIOE_AFRH      REG(0x50001024u)

#define USART4_CR1      REG(0x40004C00u)
#define USART4_CR2      REG(0x40004C04u)
#define USART4_CR3      REG(0x40004C08u)
#define USART4_BRR      REG(0x40004C0Cu)
#define USART4_ISR      REG(0x40004C1Cu)
#define USART4_TDR      REG(0x40004C28u)
#define USART4_PRESC    REG(0x40004C2Cu)

#define IWDG_KR         REG(0x40003000u)
#define SYST_CSR        REG(0xE000E010u)
#define NVIC_ICER       REG(0xE000E180u)
#define NVIC_ICPR       REG(0xE000E280u)
#define SCB_ICSR        REG(0xE000ED04u)

#define PROBE_ADDR      0x08010000u
#define ID_OFFSET       0x0FFCu         // file offset of this copy, written by mkprobe.py

void probe_entry(void);
static void probe_main(void);

// must be the first code at 0x08010000 (section .entry first in the linker script)
__attribute__((section(".entry"), naked, used))
void probe_entry(void)
{
    __asm volatile (
        "cpsid i            \n"         // interrupts off before anything can fire
        "ldr r0, =0x20024000\n"
        "mov sp, r0         \n"
        "b probe_main_trampoline \n"
    );
}

__attribute__((used)) void probe_main_trampoline(void)
{
    probe_main();
}

static void delay(uint32_t loops)
{
    while (loops--) __asm volatile ("nop");
}

static void putc_u4(char c)
{
    uint32_t timeout = 200000;
    while (!(USART4_ISR & (1u << 7)) && --timeout) {}
    USART4_TDR = (uint8_t)c;
}

static void puts_u4(const char *s)
{
    while (*s) putc_u4(*s++);
}

static void put_dec(uint32_t n)
{
    char buf[11];
    int i = 10;
    buf[i] = 0;
    do { buf[--i] = '0' + n % 10; n /= 10; } while (n && i);
    puts_u4(&buf[i]);
}

static void put_hex(uint32_t n)
{
    puts_u4("0x");
    for (int s = 28; s >= 0; s -= 4)
    {
        uint32_t d = (n >> s) & 0xF;
        putc_u4(d < 10 ? '0' + d : 'a' + d - 10);
    }
}

static void probe_main(void)
{
    // quiet everything the bootloader may have left running
    SYST_CSR = 0;
    NVIC_ICER = 0xFFFFFFFFu;
    NVIC_ICPR = 0xFFFFFFFFu;
    SCB_ICSR = (1u << 25);                                  // clear a pending SysTick

    uint32_t offset = REG(PROBE_ADDR + ID_OFFSET);          // this copy's file offset
    uint32_t base = PROBE_ADDR - offset;                    // where the bootloader put the file

    // 16 MHz HSI as system clock, whatever the bootloader left
    RCC_CR |= (1u << 8);
    while (!(RCC_CR & (1u << 10))) {}
    RCC_CFGR &= ~7u;
    while (RCC_CFGR & (7u << 3)) {}

    RCC_IOPENR |= (1u << 4);                                // GPIOE

    GPIOE_MODER = (GPIOE_MODER & ~((3u << 16) | (3u << 14))) | (1u << 16) | (1u << 14);
    for (int i = 0; i < 600; i++)
    {
        GPIOE_ODR ^= (1u << 8);
        delay(2000);
        IWDG_KR = 0xAAAA;
    }
    GPIOE_BSRR = (1u << 8);

    RCC_APBRSTR1 &= ~(1u << 19);
    RCC_APBENR1 |= (1u << 19);
    GPIOE_AFRH &= ~0xFu;
    GPIOE_OSPEEDR |= (3u << 16);
    GPIOE_MODER = (GPIOE_MODER & ~(3u << 16)) | (2u << 16);
    USART4_CR1 = 0;
    USART4_CR2 = 0;
    USART4_CR3 = 0;
    USART4_PRESC = 0;
    USART4_BRR = (16000000u + 57600u) / 115200u;
    USART4_CR1 = (1u << 3) | (1u << 0);

    for (int i = 0; i < 200; i++) putc_u4('U');

    for (;;)
    {
        puts_u4("\r\nM8P probe: bootloader size = ");
        put_dec((base - 0x08000000u) / 1024u);
        puts_u4(" KB, app at ");
        put_hex(base);
        puts_u4("\r\n");
        GPIOE_ODR ^= (1u << 7);
        for (int i = 0; i < 100; i++) { delay(2000); IWDG_KR = 0xAAAA; }
    }
}
