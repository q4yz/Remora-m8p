#ifndef BOOTINFO_H
#define BOOTINFO_H

/***********************************************************************
    Debug builds (REMORA_DEBUG): what the MCU family knows about the
    last reset, printed once at boot - the reset cause and, where the
    family keeps them over a reset (REMORA_HAS_THREAD_STATS), the
    longest thread runs and overruns before it.

    A family with REMORA_HAS_BOOT_INFO implements it
    (STM32G0: TARGET_STM32G0/drivers/debug).
************************************************************************/

void familyDebugBootInfo(void);

#endif
