/**
    \file lpc_systick.c
    \brief Functions related to the SysTick peripheral

*/

#include "hal/lpc/lpc11c14.h"

/**
    \fn void EnableSysTickException(void)
    \brief Enables the SysTick Cortex-M0 Module exception generation

    By default, the SysTick exception generation is disabled, so the SysTick
    handler is not called. The SysTick exception handler will be called after
    this function is called.
*/
void EnableSysTickException(void)
{
    cpu.core->armv6_systick->csr |= COREM0_SYSTCSR_TICKINT;
}
