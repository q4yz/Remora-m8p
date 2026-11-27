#include "bootInfo.h"

#include "mbed.h"
#include "stm32f4xx_hal.h"

void familyDebugBootInfo(void)
{
    printf("[debug] reset cause:");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) printf(" brown-out");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) printf(" power-on");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) printf(" reset-pin");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) printf(" software");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) printf(" WATCHDOG");
    printf("\n");
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
