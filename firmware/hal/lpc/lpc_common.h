/**
 *
 * LPC Common definitions
 *
 * (c) 2011 Brian Sidebotham
 *
 * Part of the opencar ( http://www.valvers.com/opencar ) project!
 *
 *
 *
 *
 */

#ifndef LPC_COMMON_H
#define LPC_COMMON_H 1

/* Require the standard integer definitions */
#include <stdint.h>

/** Read and write registers are still marked as volatile! */
#ifndef rw
#define rw  volatile
#endif

/** Read only registers */
#ifndef ro
#define ro  const volatile
#endif

/** Write only registers - same as read/write as we cannot limit reading! */
#ifndef wo
#define wo  rw
#endif

/** LPC11xx Base addresses */
#define APB_BASE    0x40000000UL
#define AHB_BASE    0x50000000UL

/* All functions below must be provided by the HAL */
extern void Delayms(uint32_t time);
extern void CpuInit(void);


#endif
