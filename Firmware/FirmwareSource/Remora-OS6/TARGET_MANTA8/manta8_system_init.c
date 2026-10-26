/*
 * SystemInit for the BTT Manta M8P (STM32G0B1VET6) behind the BTT SD card bootloader.
 *
 * The bootloader jumps to the application with its own clock setup still active (PLL as
 * system clock) and possibly SysTick / interrupts running. mbed's SystemInit only sets VTOR,
 * and HAL_RCC_OscConfig refuses to reconfigure a PLL that is the system clock unless the
 * settings match exactly, so mbed's SetSysClock fails and the firmware stops before main().
 *
 * This replaces mbed's weak SystemInit: interrupts quiet, clocks back to the reset state
 * (HSI16 system clock, PLL off), then VTOR as mbed sets it. mbed's SetSysClock then brings
 * up 8 MHz HSE -> PLL -> 64 MHz as usual.
 *
 * Runs before .data / .bss are initialised: no global variables here.
 */

#include "stm32g0xx.h"
#include "nvic_addr.h"
#include "configuration.h"

#if REMORA_DEBUG

// minimal USART4 TX on PE8 (AF0), 115200 at 16 MHz HSI, usable before RAM is initialised
static void u4_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOEEN;
    GPIOE->AFR[1] &= ~0xFU;
    GPIOE->MODER = (GPIOE->MODER & ~(3U << 16)) | (2U << 16);
    RCC->APBRSTR1 &= ~RCC_APBRSTR1_USART4RST;
    RCC->APBENR1 |= RCC_APBENR1_USART4EN;
    USART4->CR1 = 0;
    USART4->CR2 = 0;
    USART4->CR3 = 0;
    USART4->PRESC = 0;
    USART4->BRR = (16000000U + 57600U) / 115200U;
    USART4->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void u4_putc(char c)
{
    uint32_t timeout = 200000;
    while (!(USART4->ISR & USART_ISR_TXE_TXFNF) && --timeout) {}
    USART4->TDR = (uint8_t)c;
}

static void u4_puts(const char *s)
{
    while (*s) u4_putc(*s++);
}

static void u4_hex(uint32_t v)
{
    u4_puts("0x");
    for (int s = 28; s >= 0; s -= 4)
    {
        uint32_t d = (v >> s) & 0xFU;
        u4_putc((char)(d < 10 ? '0' + d : 'a' + d - 10));
    }
}

static void u4_flush(void)
{
    uint32_t timeout = 200000;
    while (!(USART4->ISR & USART_ISR_TC) && --timeout) {}
}

#endif

void SystemInit(void)
{
    // what the bootloader left behind
    uint32_t boot_cr = RCC->CR;
    uint32_t boot_cfgr = RCC->CFGR;
    uint32_t boot_pllcfgr = RCC->PLLCFGR;
    uint32_t boot_systick = SysTick->CTRL;
    uint32_t boot_iser = NVIC->ISER[0];

    // nothing may fire before mbed has set up its own handlers
    SysTick->CTRL = 0;
    NVIC->ICER[0] = 0xFFFFFFFFU;
    NVIC->ICPR[0] = 0xFFFFFFFFU;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    // clocks back to the reset state: HSI16 as system clock, PLL off and reset
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR = 0;                                          // SW = HSISYS, all prescalers 1
    while (RCC->CFGR & RCC_CFGR_SWS) {}
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}
    RCC->PLLCFGR = RCC_PLLCFGR_PLLN_4;                      // reset value 0x00001000

#if REMORA_DEBUG
    u4_init();
    u4_puts("\r\n[sysinit] bootloader left RCC_CR=");
    u4_hex(boot_cr);
    u4_puts(" CFGR=");
    u4_hex(boot_cfgr);
    u4_puts(" PLLCFGR=");
    u4_hex(boot_pllcfgr);
    u4_puts(" SysTick=");
    u4_hex(boot_systick);
    u4_puts(" NVIC_ISER=");
    u4_hex(boot_iser);
    u4_puts("\r\n[sysinit] clocks reset to HSI16, starting mbed\r\n");
    u4_flush();
#else
    (void)boot_cr; (void)boot_cfgr; (void)boot_pllcfgr; (void)boot_systick; (void)boot_iser;
#endif

    SCB->VTOR = NVIC_FLASH_VECTOR_ADDRESS;                  // as mbed's SystemInit

    // the bootloader may have jumped with interrupts masked, mbed needs them enabled
    __enable_irq();
}
