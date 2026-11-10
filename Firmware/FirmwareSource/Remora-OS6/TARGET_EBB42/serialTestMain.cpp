/***********************************************************************
    USB serial test firmware for the BTT EBB42 / EBB SB2209 CAN (STM32G0B1)

    Build: Firmware/build.sh EBB_SERIAL_TEST (EBB_SB2209 target with
    -D NODE_SERIAL_TEST=1), flash like the node firmware (DFU, 0x08000000).

    Only the USB serial port: no CAN, no threads, no node logic. Twice a
    second it prints what differs between boards:

    - the clock: whether the 8 MHz crystal started (system clock from
      the PLL on HSE) or mbed fell back to the internal HSI16
    - the reset cause, the watchdog (started here with 4 s, kicked)
    - USB: configured by the host, the HSI48 trim of the clock recovery
    - the option bytes (FLASH OPTR): boot selection, hardware watchdog

    Sign of life without USB on the hotend fan output PA0 (fan port LED):
    3 quick flashes when main() starts, then 1 Hz blinking once USB is
    started. No flashes: the firmware does not reach main(); flashes but
    no blinking: it hangs starting USB; blinking but no COM port: it runs,
    only USB fails.

    No output on a board where it works on another: compare the option
    bytes with dfu-util (see SERIAL_TEST.md).
************************************************************************/

#include "mbed.h"

#if defined(REMORA_CAN_NODE) && defined(NODE_SERIAL_TEST)

#include "UsbCdc.h"

static const char* clockSource(void)
{
    switch ((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos)
    {
        case 0: return "HSISYS";
        case 1: return "HSE";
        case 2: return (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE ? "PLL from HSE (8 MHz crystal)"
                                                                                  : "PLL from HSI16 (crystal NOT used)";
        case 3: return "LSI";
        case 4: return "LSE";
    }
    return "?";
}

static void printResetCause(uint32_t csr)
{
    printf("reset cause:");
    if (csr & RCC_CSR_LPWRRSTF) printf(" low-power");
    if (csr & RCC_CSR_WWDGRSTF) printf(" window-watchdog");
    if (csr & RCC_CSR_IWDGRSTF) printf(" WATCHDOG");
    if (csr & RCC_CSR_SFTRSTF) printf(" software");
    if (csr & RCC_CSR_PWRRSTF) printf(" power-on");
    if (csr & RCC_CSR_PINRSTF) printf(" reset-pin");
    if (csr & RCC_CSR_OBLRSTF) printf(" option-byte-load");
    printf(" (CSR 0x%08lx)\n", (unsigned long)csr);
}

static void printOptionBytes(void)
{
    uint32_t optr = FLASH->OPTR;
    printf("option bytes OPTR 0x%08lx: nBOOT_SEL %lu, nBOOT1 %lu, nBOOT0 %lu, IWDG_SW %lu (%s watchdog), RDP 0x%02lx\n",
           (unsigned long)optr,
           (unsigned long)((optr & FLASH_OPTR_nBOOT_SEL) != 0), (unsigned long)((optr & FLASH_OPTR_nBOOT1) != 0),
           (unsigned long)((optr & FLASH_OPTR_nBOOT0) != 0), (unsigned long)((optr & FLASH_OPTR_IWDG_SW) != 0),
           (optr & FLASH_OPTR_IWDG_SW) ? "software" : "HARDWARE", (unsigned long)(optr & FLASH_OPTR_RDP));
    // boot from main flash needs: nBOOT_SEL 1 and the BOOT0 pin (PA14) low, or nBOOT_SEL 0 and nBOOT0 1
    printf("  boot: %s\n", (optr & FLASH_OPTR_nBOOT_SEL) ? "BOOT0 pin (PA14) decides, low = this firmware"
                           : ((optr & FLASH_OPTR_nBOOT0) ? "option bit nBOOT0 = 1: main flash (this firmware)"
                                                         : "option bit nBOOT0 = 0: SYSTEM BOOTLOADER, never this firmware"));
}

// sign of life without USB: the hotend fan output PA0 (fan port LED, the fan with 24 V)
static void pa0(bool on)
{
    if (on) GPIOA->BSRR = GPIO_PIN_0;
    else GPIOA->BRR = GPIO_PIN_0;
}

int main()
{
    uint32_t csr = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    // stage 1, main() reached: 3 quick flashes on PA0
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    for (int i = 0; i < 3; i++)
    {
        pa0(true);
        HAL_Delay(100);
        pa0(false);
        HAL_Delay(150);
    }
    HAL_Delay(500);

    Watchdog& watchdog = Watchdog::get_instance();
    watchdog.start(4000);

    // the USB serial number: the chip's unique ID, so two boards can be told apart in the device list
    char serial[13];
    snprintf(serial, sizeof(serial), "%08lx%04lx", (unsigned long)HAL_GetUIDw0(), (unsigned long)(HAL_GetUIDw1() & 0xFFFF));
    usbCdc.start(serial);

    uint32_t last = 0, count = 0;
    while (1)
    {
        watchdog.kick();
        usbCdc.service();

        uint32_t now = HAL_GetTick();
        if (now - last < 500) continue;
        last = now;
        pa0((count & 1) == 0);              // stage 2, USB started: PA0 blinks at 1 Hz

        if (count % 10 == 0)
        {
            printf("\n=== Remora USB serial test, " __DATE__ " " __TIME__ " ===\n");
            printf("UID %08lx %08lx %08lx, DEV_ID 0x%03lx rev 0x%04lx, flash %u KB\n",
                   (unsigned long)HAL_GetUIDw0(), (unsigned long)HAL_GetUIDw1(), (unsigned long)HAL_GetUIDw2(),
                   (unsigned long)(DBG->IDCODE & 0xFFF), (unsigned long)(DBG->IDCODE >> 16),
                   (unsigned)(*(volatile uint16_t*)FLASHSIZE_BASE));
            printf("SystemCoreClock %lu Hz, system clock: %s, HSE %s\n", (unsigned long)SystemCoreClock,
                   clockSource(), (RCC->CR & RCC_CR_HSERDY) ? "running" : "not running");
            printResetCause(csr);
            printOptionBytes();
        }
        printf("#%lu uptime %lu.%lu s, USB %s, HSI48 trim %lu, CRS %s\n", (unsigned long)count,
               (unsigned long)(now / 1000), (unsigned long)(now % 1000 / 100),
               usbCdc.isConfigured() ? "configured" : "not configured",
               (unsigned long)((CRS->CR & CRS_CR_TRIM) >> CRS_CR_TRIM_Pos),
               (CRS->ISR & CRS_ISR_SYNCOKF) ? "in sync" : ((CRS->ISR & CRS_ISR_SYNCMISS) ? "SYNC MISSED" : "waiting"));
        count++;
    }
}

#endif
