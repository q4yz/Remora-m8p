#include "bootInfo.h"

#include "mbed.h"
#include "stm32g0xx_hal.h"
#include "pruThread.h"

void familyDebugBootInfo(void)
{
    printf("[debug] reset cause:");
#ifdef RCC_FLAG_PORRST
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) printf(" power-on");
#endif
#ifdef RCC_FLAG_PWRRST
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST)) printf(" power-on");
#endif
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) printf(" reset-pin");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) printf(" software");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) printf(" WATCHDOG");
    printf("\n");
    __HAL_RCC_CLEAR_RESET_FLAGS();

    // what the threads did before this reset (TAMP backup registers survive it, not a power cycle)
    uint32_t bMax, bOvr, sMax, sOvr;
    pruThread::bkpEnable();
    pruThread::bkpLoad(0, &bMax, &bOvr);
    pruThread::bkpLoad(1, &sMax, &sOvr);
    uint32_t tpu = SystemCoreClock / 1000000;
    printf("[debug] before this reset: base thread max %lu us, %lu overruns; servo thread max %lu us, %lu overruns\n",
           (unsigned long)(bMax / tpu), (unsigned long)bOvr, (unsigned long)(sMax / tpu), (unsigned long)sOvr);
    pruThread::bkpClear();
}
