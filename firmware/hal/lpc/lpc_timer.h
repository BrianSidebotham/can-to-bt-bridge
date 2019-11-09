/**
    \file lpc_timer.h
    \brief LPC Timer (16-bit and 32-bit) header support file

    Support for the LPC Cortex-M0 Timers. Most of this work
    is derived from the LPC11C14 user manual (Rev 4. 4th March 2011)
    Chapter 15

    The ARMv6-M Architecture Reference Manual should also be consulted

    This file is provided under the modified BSD licence. This licence is
    compatible with the GNU GPL licence. A copy of the modified BSD licence
    is included below:

    --- Modified BSD Licence -------------------------------------------------

    Copyright (c) 2011, Brian Sidebotham
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the opencar project nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS

*/


#ifndef LPC_TIMER_H
#define LPC_TIMER_H 1

#include "hal/lpc/lpc_common.h"

/** LPC Timer structure */
typedef struct {
    rw uint32_t ir;
    rw uint32_t tcr;
    rw uint32_t tc;
    rw uint32_t pr;
    rw uint32_t pc;
    rw uint32_t mcr;
    rw uint32_t mr0;
    rw uint32_t mr1;
    rw uint32_t mr2;
    rw uint32_t mr3;
    rw uint32_t ccr;
    ro uint32_t cr0;
    rw uint32_t emr;
    ro uint32_t reserved[12];
    rw uint32_t ctcr;
    rw uint32_t pwmc;
    } lpc_timer_t;

/** Definitions for the timer Interrupt Register (IR) */
#define LPCTIMER_IR_MR0I    (1 << 0)
#define LPCTIMER_IR_MR1I    (1 << 1)
#define LPCTIMER_IR_MR2I    (1 << 2)
#define LPCTIMER_IR_MR3I    (1 << 3)
#define LPCTIMER_IR_CR0I    (1 << 4)

/** Definitions for the Timer Control Register (TCR) */
#define LPCTIMER_TCR_CE     (1 << 0)
#define LPCTIMER_TCR_CRST   (1 << 1)

/** Definitions for the timer Match Control Register */
#define LPCTIMER_MCR_MR0I   (1 << 0)
#define LPCTIMER_MCR_MR0R   (1 << 1)
#define LPCTIMER_MCR_MR0S   (1 << 2)

#define LPCTIMER_MCR_MR1I   (1 << 3)
#define LPCTIMER_MCR_MR1R   (1 << 4)
#define LPCTIMER_MCR_MR1S   (1 << 5)

#define LPCTIMER_MCR_MR2I   (1 << 6)
#define LPCTIMER_MCR_MR2R   (1 << 7)
#define LPCTIMER_MCR_MR2S   (1 << 8)

#define LPCTIMER_MCR_MR3I   (1 << 9)
#define LPCTIMER_MCR_MR3R   (1 << 10)
#define LPCTIMER_MCR_MR3S   (1 << 11)

/** Definitions for the timer Capture Control Register */
#define LPCTIMER_CCR_CAP0RE (1 << 0)
#define LPCTIMER_CCR_CAP0FE (1 << 1)
#define LPCTIMER_CCR_CAP0I  (1 << 2)

/** Definitions for the timer External Match Register */
#define LPCTIMER_EMR_EM0    (1 << 0)
#define LPCTIMER_EMR_EM1    (1 << 1)
#define LPCTIMER_EMR_EM2    (1 << 2)
#define LPCTIMER_EMR_EM3    (1 << 3)
#define LPCTIMER_EMR_EMC0   (3 << 4)
#define LPCTIMER_EMR_EMC1   (3 << 6)
#define LPCTIMER_EMR_EMC2   (3 << 8)
#define LPCTIMER_EMR_EMC3   (3 << 10)

/** Definitions of the various external match control options */
#define LPCTIMER_EMC_NOTHING    0
#define LPCTIMER_EMC_CLEAR      1
#define LPCTIMER_EMC_SET        2
#define LPCTIMER_EMC_TOGGLE     3

/** Definitions for the Count Control Register */
#define LPCTIMER_CTCR_CTM       (3 << 0)
#define LPCTIMER_CTCR_CIS       (3 << 2)

/** Definitions for bits in the PWM register */
#define LPCTIMER_PWMC_PWMEN0    (1 << 0)
#define LPCTIMER_PWMC_PWMEN1    (1 << 1)
#define LPCTIMER_PWMC_PWMEN2    (1 << 2)
#define LPCTIMER_PWMC_PWMEN3    (1 << 3)

/** A definition of the base address of the 16-bit timer 0 */
#define LPC_TIMER16B0_BASE_ADDRESS  ((lpc_timer_t*)(APB_BASE + 0x0C000))

/** A definition of the base address of the 16-bit timer 1 */
#define LPC_TIMER16B1_BASE_ADDRESS  ((lpc_timer_t*)(APB_BASE + 0x10000))

/** A definition of the base address of the 32-bit timer 0 */
#define LPC_TIMER32B0_BASE_ADDRESS  ((lpc_timer_t*)(APB_BASE + 0x14000))

/** A definition of the base address of the 32-bit timer 1 */
#define LPC_TIMER32B1_BASE_ADDRESS  ((lpc_timer_t*)(APB_BASE + 0x18000))

#endif
